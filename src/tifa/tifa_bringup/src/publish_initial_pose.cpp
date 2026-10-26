// src/publish_initial_pose.cpp
//
// Redesign dari versi sebelumnya. Perubahan utama:
//   1. Tunggu AMCL lifecycle service 'amcl/get_state' available sebelum publish
//   2. Poll state sampai AMCL benar-benar 'active' (bukan hanya 'launched')
//   3. Publish loop tidak fixed-count — terus publish sampai /amcl_pose confirm diterima
//   4. Interval publish lebih lambat (500ms) supaya TF buffer AMCL punya waktu settle
//   5. Graceful shutdown — tidak crash dengan SIGABRT saat context sudah invalid

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <lifecycle_msgs/srv/get_state.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <chrono>
#include <cmath>
#include <string>
#include <atomic>
#include <thread>

using namespace std::chrono_literals;

// ─────────────────────────────────────────────────────────────────────────────
// Helper: tunggu AMCL lifecycle state == active
// Return true jika berhasil, false jika timeout
// ─────────────────────────────────────────────────────────────────────────────
static bool wait_for_amcl_active(
  rclcpp::Node::SharedPtr node,
  std::chrono::seconds timeout)
{
  auto client = node->create_client<lifecycle_msgs::srv::GetState>("amcl/get_state");

  RCLCPP_INFO(node->get_logger(), "[initpose] Waiting for amcl/get_state service...");

  // Tunggu service available
  auto deadline = std::chrono::steady_clock::now() + timeout;
  while (rclcpp::ok()) {
    if (client->wait_for_service(1s)) break;
    if (std::chrono::steady_clock::now() >= deadline) {
      RCLCPP_ERROR(node->get_logger(),
        "[initpose] amcl/get_state service not available after %lds — proceeding anyway",
        timeout.count());
      return false;
    }
    RCLCPP_INFO(node->get_logger(), "[initpose] Still waiting for amcl/get_state...");
  }

  RCLCPP_INFO(node->get_logger(), "[initpose] amcl/get_state available. Polling AMCL state...");

  // Poll sampai state == active (id=3) atau timeout
  while (rclcpp::ok()) {
    auto req = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
    auto future = client->async_send_request(req);

    // Spin dengan timeout 2s per request
    if (rclcpp::spin_until_future_complete(node, future, 2s)
        != rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_WARN(node->get_logger(), "[initpose] get_state request timed out, retrying...");
    } else {
      auto result = future.get();
      uint8_t state_id = result->current_state.id;
      RCLCPP_INFO(node->get_logger(),
        "[initpose] AMCL state: %s (id=%d)",
        result->current_state.label.c_str(), state_id);

      // lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE = 3
      if (state_id == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
        RCLCPP_INFO(node->get_logger(), "[initpose] AMCL is ACTIVE. Proceeding to publish.");
        return true;
      }
    }

    if (std::chrono::steady_clock::now() >= deadline) {
      RCLCPP_ERROR(node->get_logger(),
        "[initpose] AMCL did not reach active state within %lds — proceeding anyway",
        timeout.count());
      return false;
    }

    std::this_thread::sleep_for(500ms);
    rclcpp::spin_some(node);
  }

  return false;
}

// ─────────────────────────────────────────────────────────────────────────────
// Main
// ─────────────────────────────────────────────────────────────────────────────
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // Parse args
  double x = 0.0, y = 0.0, yaw = 0.0;
  if (argc >= 4) {
    try {
      x   = std::stod(argv[1]);
      y   = std::stod(argv[2]);
      yaw = std::stod(argv[3]);
    } catch (...) {
      RCLCPP_WARN(rclcpp::get_logger("initpose"), "Invalid args, using defaults 0,0,0");
    }
  }

  auto node = rclcpp::Node::make_shared("one_shot_initialpose");

  RCLCPP_INFO(node->get_logger(),
    "[initpose] Starting (x=%.3f y=%.3f yaw=%.3f)", x, y, yaw);

  // ── Step 1: Tunggu AMCL active ──────────────────────────────────────────
  // Timeout 60s cukup untuk kondisi boot paling lambat sekalipun
  wait_for_amcl_active(node, 60s);

  // ── Step 2: Bangun publisher dan message ────────────────────────────────
  // QoS: transient_local depth 1 — AMCL subscribe dengan QoS ini,
  // memastikan message tidak hilang meski subscriber sedikit terlambat.
  auto qos = rclcpp::QoS(1).transient_local();
  auto pub = node->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "initialpose", qos);

  geometry_msgs::msg::PoseWithCovarianceStamped msg;
  msg.header.frame_id = "map";
  msg.pose.pose.position.x = x;
  msg.pose.pose.position.y = y;
  msg.pose.pose.position.z = 0.0;
  msg.pose.pose.orientation.z = std::sin(yaw * 0.5);
  msg.pose.pose.orientation.w = std::cos(yaw * 0.5);

  // ── Step 3: Subscribe /amcl_pose untuk konfirmasi ───────────────────────
  // Node ini tidak shutdown sampai konfirmasi diterima, atau timeout.
  std::atomic<bool> amcl_pose_received{false};

  auto amcl_sub = node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "amcl_pose", rclcpp::QoS(1),
    [&](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr) {
      if (!amcl_pose_received.exchange(true)) {
        RCLCPP_INFO(node->get_logger(),
          "[initpose] /amcl_pose confirmed received. AMCL has accepted the pose.");
      }
    }
  );

  // ── Step 4: Publish loop sampai konfirmasi diterima ─────────────────────
  // Interval 500ms — lebih lambat dari versi lama (200ms).
  // Alasan: TF buffer EKF di Raspi jitter ~100-200ms, interval 500ms
  // memberi cukup waktu TF settle sebelum AMCL proses transform.
  // Max 30 attempt = 15 detik total window setelah AMCL active.
  const int max_attempts  = 30;
  const auto pub_interval = 500ms;

  for (int i = 0; i < max_attempts && rclcpp::ok(); ++i) {
    msg.header.stamp = node->now();
    pub->publish(msg);

    RCLCPP_INFO(node->get_logger(),
      "[initpose] Published attempt %d/%d", i + 1, max_attempts);

    // Spin sebentar untuk proses callback subscriber
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(pub_interval);
    rclcpp::spin_some(node);

    if (amcl_pose_received.load()) {
      RCLCPP_INFO(node->get_logger(),
        "[initpose] Done after %d attempt(s). Shutting down.", i + 1);
      break;
    }
  }

  if (!amcl_pose_received.load()) {
    RCLCPP_WARN(node->get_logger(),
      "[initpose] /amcl_pose never confirmed after %d attempts. "
      "AMCL may need manual pose reset via RViz.",
      max_attempts);
  }

  // ── Step 5: Graceful shutdown ────────────────────────────────────────────
  // Spin sekali lagi untuk flush callback pending sebelum shutdown.
  // Ini mencegah SIGABRT dari 'rcl node context invalid' saat launch
  // mengirim SIGINT bersamaan.
  rclcpp::spin_some(node);
  std::this_thread::sleep_for(100ms);

  rclcpp::shutdown();
  return 0;
}