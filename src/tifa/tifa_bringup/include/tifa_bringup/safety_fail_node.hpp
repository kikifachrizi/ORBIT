#pragma once

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

class SafetyFailNode : public rclcpp::Node
{
public:
  SafetyFailNode();

private:
  void cmdVelCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
  void watchdogLoop();

  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr safe_cmd_pub_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;

  rclcpp::Time last_cmd_time_;
  rclcpp::Duration cmd_timeout_;

  geometry_msgs::msg::TwistStamped last_cmd_;
  bool received_cmd_;
};
