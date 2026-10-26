# tifa_bringup

## Ringkasan

`tifa_bringup` adalah **definisi satu robot TIFA untuk ORBIT** (ADR-004): URDF, hardware, sensor, rantai `cmd_vel`, EKF, dan Nav2, untuk robot fisik maupun simulasi. Package ini tidak bergantung pada RMF, jadi robot bisa dijalankan dan di-teleop tanpa ORBIT. TIFA adalah robot pengantar makanan. Paket ini menyediakan bringup, navigasi, lokalisasi, keamanan, dan abstraksi hardware lengkap untuk robot layanan indoor differential-drive yang dirancang untuk ber navigasi di lingkungan dinamis seperti Coffee Shop Universitas Telkom.

Paket ini menangani segala sesuatu mulai dari deskripsi robot URDF dan antarmuka hardware ros2_control, hingga pemrosesan sensor dan estimasi status, hingga orkestrasi stack navigasi Nav2 penuh dengan urutan launch multi-layer.

---

## Arsitektur

Diagram arsitektur dan pipeline kecepatan ada di repo `tifa` (`src/tifa_bot/Documentation/`). Alur kecepatan:

```
teleop (TwistStamped) ---------> diff_cont ------------------+
Nav2 -> cmd_vel_nav -> velocity_smoother -> cmd_vel_smoothed -+-> twist_mux
  -> diff_cont/cmd_vel_raw -> safety_fail_node -> diff_cont/cmd_vel -> diff_cont
```

Semua nama di atas relatif: dengan `namespace:=tifa1` menjadi `/tifa1/...`.

---

## Struktur Paket

```
tifa_bringup/
├── config/
│   ├── DWB_params.yaml             # Nav2 (controller DWB), dipakai robot fisik
│   ├── MPPI_params.yaml            # Nav2 alternatif (MPPI)
│   ├── ekf.yaml                    # EKF: odom roda + rf2o + IMU
│   ├── robot_controllers.yaml      # diff_cont + joint_state_broadcaster (fisik & sim sama)
│   ├── bridge_gazebo.yaml          # ros_gz_bridge sim: scan, imu (per robot; /clock per world)
│   ├── twist_mux.yaml              # Prioritas teleop > navigasi
│   ├── lidar_filter.yaml           # Filter poligon badan robot
│   └── mapper_params_online_async.yaml  # SLAM Toolbox
├── description/
│   ├── robot.urdf.xacro            # Arg sim_mode (default false = fisik) dan namespace (sim)
│   ├── robot_core.xacro
│   ├── ros2_control.xacro          # Plugin: DiffDriveArduino (fisik) / gz_ros2_control (sim)
│   ├── lidar.xacro                 # RPLidar C1 + sensor gpu_lidar Gazebo
│   ├── imu.xacro                   # IMU + sensor IMU Gazebo
│   └── inertial_macros.xacro
├── include/tifa_bringup/safety_fail_node.hpp
├── launch/
│   ├── robot.launch.py             # PINTU MASUK satu robot, fisik dan sim
│   ├── launch_robot.launch.py      # Kompatibilitas: = robot.launch.py sim:=false
│   ├── manual_mapping.launch.py    # Mode pemetaan SLAM (belum mendukung namespace)
│   ├── rsp.launch.py               # robot_state_publisher (xacro sim_mode, namespace)
│   ├── localization_launch.py      # map_server + AMCL
│   ├── navigation_launch.py        # Stack Nav2
│   └── laser_filter.launch.py      # Filter laser standalone
├── maps/                           # Peta robot fisik (abie_022526, telcoff_map)
├── meshes/                         # Mesh STL (mm, skala x0.001)
├── src/                            # safety_fail_node.cpp, publish_initial_pose.cpp
├── CMakeLists.txt
└── package.xml
```

World simulasi tidak ada di sini, tetapi di package `orbit_sim`.

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

### `robot.launch.py` — Pintu masuk satu robot

Sama untuk robot fisik dan simulasi. Semua node, topic, dan tf berada di bawah `/<namespace>`; frame tetap `base_link`, `odom`, `map` (pola multi-robot Nav2). File ini tidak pernah menyalakan Gazebo.

| Argumen | Default | Keterangan |
|---|---|---|
| `namespace` | kosong | Mis. `tifa1`. Kosong = tanpa namespace |
| `sim` | `false` | `true` = spawn ke Gazebo yang sudah jalan (gz_ros2_control), `false` = Arduino + RPLidar |
| `use_sim_time` | = `sim` | |
| `map` | `maps/abie_022526.yaml` | Peta Nav2 |
| `nav` | `true` | AMCL + Nav2 |
| `params_file` | `config/DWB_params.yaml` | Parameter Nav2 |
| `x`, `y`, `yaw` | `0.0` | Posisi spawn (sim) sekaligus initial pose AMCL |

| Bagian | Fisik (`sim:=false`) | Sim (`sim:=true`) |
|---|---|---|
| Hardware | `ros2_control_node` + `diffdrive_arduino` (jeda 5 s), `rplidar_ros` | `ros_gz_sim create` (entity = namespace), `ros_gz_bridge` |
| Controller | spawner `joint_state_broadcaster` + `diff_cont` | sama (controller_manager di dalam Gazebo) |
| Rantai | filter, rf2o, EKF, twist_mux, safety | sama |
| Nav2 (`nav:=true`) | localization +17 s, navigation +22 s, initial pose +25 s | sama |

Urutan nyala memakai `TimerAction`: aksi dari event handler (`OnProcessStart`/`OnProcessExit`) berjalan di luar `PushRosNamespace` dan node-nya kehilangan namespace.

---

### `launch_robot.launch.py` — Kompatibilitas

Sama dengan `robot.launch.py sim:=false`, untuk perintah lama di robot. Peta tidak lagi dibaca dari `~/tifa_selected_map.txt`; berikan lewat argumen `map`.

---

### `manual_mapping.launch.py` — Mode Pemetaan SLAM

Robot fisik dengan **SLAM Toolbox** sebagai pengganti AMCL + map_server, untuk membuat peta baru. Belum mendukung `namespace` (pemetaan dilakukan satu robot).

**Jalankan:**
```bash
ros2 launch tifa_bringup manual_mapping.launch.py
```

---

### `rsp.launch.py` — Robot State Publisher

Mempublikasikan deskripsi robot URDF dengan memproses `robot.urdf.xacro` melalui xacro.
Argumen `sim_mode` (default `false`) memilih plugin hardware: `false` = DiffDriveArduino (robot fisik), `true` = `gz_ros2_control` (Gazebo).

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

- Node C++: `rclcpp`, `geometry_msgs`, `lifecycle_msgs`.
- Runtime: `robot_state_publisher`, `xacro`, `controller_manager`, `diff_drive_controller`, `joint_state_broadcaster`, `twist_mux`, `laser_filters`, `robot_localization`, `slam_toolbox`, `navigation2`.
- Vendored di `src/tifa/`: `diffdrive_arduino`, `rf2o_laser_odometry`, `rplidar_ros` (sementara, lihat ADR-004).
- Simulasi: `ros_gz_sim`, `ros_gz_bridge`, `gz_ros2_control`. World ada di `orbit_sim`.
- Tidak bergantung pada RMF maupun package ORBIT lain.

---

## Build & Jalankan

```bash
cd ~/ORBIT
colcon build --packages-up-to tifa_bringup orbit_sim
source install/setup.zsh     # setup.bash bila memakai bash
```

### Robot fisik

```bash
ros2 launch tifa_bringup robot.launch.py namespace:=tifa1 map:=/path/peta.yaml
```

### Simulasi: satu atau dua robot dalam satu world

```bash
ros2 launch orbit_sim world.launch.py site:=test_room
MAP=$(ros2 pkg prefix orbit_sim)/share/orbit_sim/sites/test_room/maps/test_room.yaml
ros2 launch tifa_bringup robot.launch.py sim:=true namespace:=tifa1 x:=-2.5 y:=0.5 map:=$MAP
ros2 launch tifa_bringup robot.launch.py sim:=true namespace:=tifa2 x:=0.5 y:=-0.5 map:=$MAP
# teleop ke satu robot (input teleop twist_mux, TwistStamped)
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args \
    -p stamped:=true -p frame_id:=base_link -r cmd_vel:=/tifa1/diff_cont
```

### Mode pemetaan

```bash
ros2 launch tifa_bringup manual_mapping.launch.py
```

---

## Catatan Keamanan

- `safety_fail_node` adalah **interlock keamanan kritis**. Node ini harus berjalan setiap kali robot bergerak. Node ini secara otomatis menghentikan robot jika aliran perintah kecepatan sudah usang (timeout default: 500 ms).
- Global costmap secara sengaja mengecualikan obstacle layer untuk mencegah rintangan dinamis (manusia) memblokir jalur global secara permanen dan menyebabkan robot membatalkan navigasi.
- EKF menggunakan mode differensial untuk odometri laser untuk mencegah drift pose absolut bertentangan dengan odometri encoder roda.
