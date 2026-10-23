# tifa_bringup

## Ringkasan

`tifa_bringup` adalah **paket runtime robot inti** untuk workspace ROS2 TIFA - Robot Pengantar Makanan. Paket ini menyediakan bringup, navigasi, lokalisasi, keamanan, dan abstraksi hardware lengkap untuk robot layanan indoor differential-drive yang dirancang untuk ber navigasi di lingkungan dinamis seperti Coffee Shop Universitas Telkom.

Paket ini menangani segala sesuatu mulai dari deskripsi robot URDF dan antarmuka hardware ros2_control, hingga pemrosesan sensor dan estimasi status, hingga orkestrasi stack navigasi Nav2 penuh dengan urutan launch multi-layer.

---

## Arsitektur

![Arsitektur](Documentation/architecture.png)

![Pipeline Kecepatan](Documentation/velocity_pipeline.png)

---

## Struktur Paket

```
tifa_bringup/
├── config/                         # Semua file parameter
│   ├── DWB_params.yaml             # Konfigurasi Nav2 produksi (planner DWB)
│   ├── MPPI_params.yaml            # Konfigurasi Nav2 alternatif (MPPI, simulasi)
│   ├── ekf.yaml                    # Konfigurasi sensor fusion EKF
│   ├── robot_controllers.yaml      # ros2_control diff_drive + joint_state (fisik & sim)
│   ├── bridge_gazebo.yaml          # ros_gz_bridge: /clock, /scan, /imu (sim)
│   ├── twist_mux.yaml              # Prioritas multiplexer kecepatan
│   ├── lidar_filter.yaml           # Filter poligon tubuh LaserScan
│   └── mapper_params_online_async.yaml  # Konfigurasi SLAM Toolbox
├── description/                    # File URDF/Xacro robot
│   ├── robot.urdf.xacro            # Level atas, arg sim_mode (default false = robot fisik)
│   ├── robot_core.xacro            # Struktur mekanis & kinematic chain
│   ├── ros2_control.xacro          # Hardware: DiffDriveArduino (fisik) / gz_ros2_control (sim)
│   ├── lidar.xacro                 # Frame RPLidar C1 + sensor gpu_lidar Gazebo
│   ├── imu.xacro                   # Frame IMU + sensor IMU Gazebo
│   └── inertial_macros.xacro       # Macro properti inersia
├── include/tifa_bringup/               # Header C++
│   ├── safety_fail_node.hpp
│   └── (file header)
├── launch/                         # File launch
│   ├── launch_robot.launch.py      # Bringup utama (mode navigasi)
│   ├── tifa_sim.launch.py          # Simulasi Gazebo Harmonic (rantai cmd_vel sama dgn fisik)
│   ├── manual_mapping.launch.py    # Bringup mode pemetaan SLAM
│   ├── rsp.launch.py               # Robot State Publisher
│   ├── localization_launch.py      # Lokalisasi Nav2 (AMCL + map_server)
│   ├── navigation_launch.py        # Stack navigasi Nav2
│   └── laser_filter.launch.py      # Filter laser standalone
├── maps/                           # Peta occupancy grid yang sudah dibuat
├── worlds/                         # World Gazebo (telu_coffee.sdf + model telu_coffee/, industrial-warehouse.sdf)
├── meshes/                         # Mesh STL (disimpan dalam mm, skala ×0.001)
├── src/                            # File sumber C++
│   ├── safety_fail_node.cpp
│   └── publish_initial_pose.cpp
├── CMakeLists.txt
└── package.xml
```

---

## Node

### `safety_fail_node`

**Tipe:** Watchdog kecepatan / interlock fail-safe

Berada di antara `twist_mux` dan pengontrol diff_drive. Timer watchdog yang berjalan pada 50 Hz memeriksa apakah perintah kecepatan terakhir yang diterima masih segar (dalam timeout yang dapat dikonfigurasi). Jika aliran perintah sudah usang atau belum pernah diterima, TwistStamped dengan kecepatan nol akan dipublikasikan, menghentikan robot dalam jangka waktu timeout.

| Parameter | Tipe | Default | Deskripsi |
|---|---|---|---|
| `cmd_timeout_ms` | `int` | `500` | Timeout dalam ms sebelum menyatakan aliran perintah mati |

**Topik:**
- Berlangganan: `/diff_cont/cmd_vel_raw` (`geometry_msgs/TwistStamped`, output `twist_mux`)
- Mempublikasikan: `/diff_cont/cmd_vel` (`geometry_msgs/TwistStamped`, di-stamp ulang setiap siklus)

> **Jazzy:** `diff_drive_controller` Jazzy hanya subscribe `~/cmd_vel` bertipe `TwistStamped`;
> topic `cmd_vel_unstamped` versi Humble sudah tidak ada. Diagram `velocity_pipeline.png` masih
> memakai nama topic lama.

---

### `publish_initial_pose`

**Tipe:** Penerbit pose awal satu kali yang sadar lifecycle

Mempublikasikan pose awal ke `/initialpose` untuk lokalisasi AMCL. Menunggu AMCL mencapai status lifecycle `ACTIVE` sebelum mempublikasikan, kemudian mencoba ulang hingga 30 kali dengan interval 500 ms hingga AMCL mengonfirmasi penerimaan. Dirancang untuk startup yang andal pada hardware dengan sumber daya terbatas (misalnya Raspberry Pi).

**Argumen:** `x` `y` `yaw` (semua default ke `0.0`)

**Topik:**
- Mempublikasikan: `/initialpose` (PoseWithCovarianceStamped, frame `map`)
- Berlangganan: `/amcl_pose` (untuk mengonfirmasi penerimaan)

---

## File Launch

### `launch_robot.launch.py` — Bringup Utama (Mode Navigasi)

Titik masuk utama untuk mengoperasikan robot. Meluncurkan segalanya dalam urutan 4-layer bertahap untuk mengelola inisialisasi hardware dan penundaan konvergensi TF.

**Argumen:**

| Argumen | Default | Deskripsi |
|---|---|---|
| `robot_id` | `TFRB1` | Identifikasi robot |
| `ui_id` | `TFUI1` | Identifikasi UI |
| `ws_uri_primary` | `wss://tifa-ws.forgixrobotic.com` | Endpoint cloud WebSocket |
| `map` | dari `~/tifa_selected_map.txt` atau `abie_022526.yaml` | Peta yang akan dimuat |

**Layer startup:**
1. **Layer 0:** Robot State Publisher, RPLidar C1, WebSocket bridge, diagnostik
2. **Layer 1:** Filter laser, odometri laser RF2O, controller_manager
3. **Layer 2:** Pengontrol diff drive, broadcaster state joint, EKF, twist_mux
4. **Layer 3:** Lokalisasi (+12s), Navigasi (+17s), safety_fail_node (+14s)
5. **Layer 4:** Pose awal (+20s), robot API (+22s), manajer sensor (+25s)

**Jalankan:**
```bash
ros2 launch tifa_bringup launch_robot.launch.py map:=abie_022526.yaml
```

---

### `manual_mapping.launch.py` — Mode Pemetaan SLAM

Setara dengan `launch_robot.launch.py` tetapi menggunakan **SLAM Toolbox** sebagai pengganti AMCL + map_server. Termasuk `slam_supervisor_node` dan `position_sensor_node` untuk manajemen sesi pemetaan.

**Jalankan:**
```bash
ros2 launch tifa_bringup manual_mapping.launch.py
```

---

### `rsp.launch.py` — Robot State Publisher

Mempublikasikan deskripsi robot URDF dengan memproses `robot.urdf.xacro` melalui xacro.
Argumen `sim_mode` (default `false`) memilih plugin hardware: `false` = DiffDriveArduino (robot fisik), `true` = `gz_ros2_control` (Gazebo).

---

### `tifa_sim.launch.py` — Simulasi Gazebo Harmonic

Menjalankan Gazebo, spawn robot (`sim_mode:=true`), bridge `/clock` `/scan` `/imu`, controller `diff_cont` + `joint_state_broadcaster` (di dalam Gazebo), lalu rantai yang **sama** dengan robot fisik: laser filter, rf2o, EKF, twist_mux, `safety_fail_node`. Nav2 belum diikutkan.

| Argumen | Default | Keterangan |
|---------|---------|------------|
| `world` | `worlds/telu_coffee.sdf` | World SDF (model lokal `worlds/telu_coffee/`). Alternatif: `worlds/industrial-warehouse.sdf` (model Gazebo Fuel, butuh cache/internet saat pertama) |
| `gz_args` | `-r -v 1` | `-r -s --headless-rendering -v 1` untuk tanpa GUI |
| `x`, `y`, `yaw` | `0.0` | Posisi spawn |
| `use_sim_time` | `true` | |

---

### `localization_launch.py` — Lokalisasi Nav2

Meluncurkan `map_server` dan `amcl` dengan `lifecycle_manager_localization`. Parameter dimuat dari `DWB_params.yaml`.

---

### `navigation_launch.py` — Stack Navigasi Nav2

Meluncurkan stack Nav2 lengkap: controller_server (DWB), smoother_server, planner_server, behavior_server, bt_navigator, waypoint_follower, dan velocity_smoother.

---

## Konfigurasi

### Parameter Navigasi (`DWB_params.yaml`)

Tuning produksi untuk coffee shop dengan rintangan manusia dinamis.

| Subsistem | Pengaturan Utama |
|---|---|
| **AMCL** | `max_particles: 3000`, `laser_min_range: 0.46`, `max_beams: 90`, `scan_topic: scan_filtered` |
| **DWB Controller** | `max_vel_x: 0.35 m/s`, `acc_lim_x: 0.3 m/s²`, `decel_lim_x: -0.5 m/s²`, `sim_time: 1.7s` |
| **Global Costmap** | Hanya static layer (tanpa obstacle layer — mencegah rintangan dinamis memblokir perencanaan global) |
| **Local Costmap** | Voxel + inflation layers, `robot_radius: 0.36`, rolling window 3×3m |
| **Planner** | NavFn dengan A* (`use_astar: true`), `tolerance: 0.5` |
| **Velocity Smoother** | Max 0.35 m/s, max accel 0.3 m/s², deadband 0.01 |

### Konfigurasi EKF (`ekf.yaml`

Menggabungkan tiga sumber:
- **odom0** (`/diff_cont/odom`): Encoder roda — hanya kecepatan (vx, vyaw)
- **odom1** (`/odom_rf2o`): Odometri laser RF2O — hanya pose (differential=true)
- **imu0** (`/imu`): Gyro — hanya kecepatan sudut yaw. Saat ini hanya ada di simulasi; robot fisik belum punya IMU sehingga input ini kosong

Mempublikasikan transformasi `odom → base_link`, menjaga pohon TF lengkap: `map → odom → base_link → base_footprint / chassis / laser_frame`.

### Twist Mux (`twist_mux.yaml`)

| Sumber | Topik | Prioritas | Timeout |
|---|---|---|---|
| Navigasi | `/cmd_vel_smoothed` (output `velocity_smoother`) | 10 | 0.5s |
| Teleop | `/diff_cont` | 20 | 0.5s |

`controller_server` dan `behavior_server` (spin, backup, dll.) publish ke `/cmd_vel_nav`; `velocity_smoother` meneruskannya ke `/cmd_vel_smoothed`.

Semua input dan output `twist_mux` bertipe `geometry_msgs/TwistStamped` (`use_stamped: true`). Nav2 publish TwistStamped lewat `enable_stamped_cmd_vel: true` di params Nav2.

### Filter Laser (`lidar_filter.yaml`)

Filter poligon 12-sisi dalam frame `base_link` (radius 0.38m) untuk menghapus titik scan yang mengenai tubuh robot.

---

## Deskripsi Robot

### Struktur Mekanis

Platform **differential-drive** dengan:
- Dua roda bertenaga (wheel_separation: 0.425 m, wheel_radius: 0.0635 m)
- Dua roda caster (depan dan belakang)
- RPLidar C1 dipasang pada chassis dengan ketinggian 0.27 m

### Kinematic Chain

```
base_link (root)
├── base_footprint (fixed, ground level)
├── chassis (fixed, main body)
│   ├── rear_caster_wheel
│   └── back_caster_wheel
├── left_wheel (continuous)
└── right_wheel (continuous)
```

### Antarmuka Hardware

- **Arduino Nano** melalui serial (`/dev/arduino_nano`, 57600 baud)
- Plugin **DiffDriveArduinoHardware**
- Encoder: 2000 counts/rev, loop rate 30 Hz
- ros2_control update rate: 100 Hz
- Simulasi: plugin **gz_ros2_control/GazeboSimSystem** dengan joint, interface, dan `robot_controllers.yaml` yang sama

### Mesh (disimpan dalam mm, skala ×0.001)

| Mesh | Digunakan Untuk |
|---|---|
| `base.stl` | Tubuh chassis utama |
| `left_wheel.stl` | Roda kiri bertenaga |
| `right_wheel.stl` | Roda kanan bertenaga |
| `front_caster.stl` | Roda caster depan/belakang |
| `back_caster.stl` | Roda caster belakang |
| `rplidar.stl` | Housing RPLidar C1 |
| `body_case.stl` | Tutup atas (saat ini dikomentari) |

---

## Peta

Peta occupancy grid yang sudah dibuat disimpan di `maps/`. Setiap peta terdiri dari gambar `.pgm` dan file metadata `.yaml`.

Parameter umum di semua peta:
- Resolusi: 0.05 m/pixel
- Mode: trinary
- Threshold occupied: 0.65
- Threshold free: 0.25

Peta fallback default adalah `abie_022526.yaml`.

---

## Dependensi

### Paket ROS 2
- `rclcpp` — Pustaka klien C++
- `rclcpp_action` — Dukungan aksi
- `geometry_msgs` — Primitif geometri
- `nav2_msgs` — Pesan Navigation2
- `std_msgs` — Pesan standar
- `lifecycle_msgs` — Manajemen lifecycle
- `sensor_msgs` — Pesan sensor
- `rcl_interfaces` — Antarmuka parameter ROS 2

### Pustaka Eksternal
- `nlohmann_json` — JSON untuk Modern C++
- `PostgreSQL` (`libpq-dev`) — Klien database

### Dependensi Runtime (paket saudara)
- `tifa_ws_bridge` — Konektivitas cloud WebSocket
- `tifa_diagnostic_node` — Pemantauan kesehatan
- `tifa_robot_api` — Kontrol robot tingkat aplikasi
- `rplidar_ros` — Driver RPLidar
- `rf2o_laser_odometry` — Odometri berbasis laser
- `laser_filters` — Pemindaian filter
- `slam_toolbox` — Pemetaan SLAM online
- `nav2_bringup` — Stack Navigation2
- `robot_localization` — Sensor fusion EKF/UKF

---

## Build & Jalankan

### Build

```bash
cd ~/work/abie/tifa
colcon build --packages-select tifa_bringup
source install/setup.bash
```

### Jalankan (Mode Navigasi)

```bash
ros2 launch tifa_bringup launch_robot.launch.py map:=abie_022526.yaml
```

### Jalankan (Simulasi + Teleop)

```bash
ros2 launch tifa_bringup tifa_sim.launch.py
# terminal lain: teleop masuk ke input teleop twist_mux (/diff_cont, TwistStamped)
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args \
    -p stamped:=true -p frame_id:=base_link -r cmd_vel:=/diff_cont
```

### Jalankan (Mode Pemetaan)

```bash
ros2 launch tifa_bringup manual_mapping.launch.py
```

### Jalankan (Filter Laser Standalone)

```bash
ros2 launch tifa_bringup laser_filter.launch.py
```

---

## Catatan Keamanan

- `safety_fail_node` adalah **interlock keamanan kritis**. Node ini harus berjalan setiap kali robot bergerak. Node ini secara otomatis menghentikan robot jika aliran perintah kecepatan sudah usang (timeout default: 500 ms).
- Global costmap secara sengaja mengecualikan obstacle layer untuk mencegah rintangan dinamis (manusia) memblokir jalur global secara permanen dan menyebabkan robot membatalkan navigasi.
- EKF menggunakan mode differensial untuk odometri laser untuk mencegah drift pose absolut bertentangan dengan odometri encoder roda.
