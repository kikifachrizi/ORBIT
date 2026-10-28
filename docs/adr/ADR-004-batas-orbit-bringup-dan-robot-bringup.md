# ADR-004: Batas `orbit_bringup` dan bringup robot (`tifa_bringup`, `nevo_bringup`)

**Status:** Proposed  
**Tanggal:** 23 Oktober 2026 (WIB)  
**Owner:** _diisi saat review_ · **Reviewer:** _diisi saat review_  
**Requirements:** FR-10, NFR-02, G1, G3 · **PR / issue:** _diisi_

## Konteks

Gambar Package Stack awal menaruh "adapter" (Python + YAML) di dalam tiap bringup robot. ADR-002 sudah memutuskan satu kode adapter (`orbit_adapter`, turunan pola `fleet_adapter_template`) untuk semua robot. Yang belum jelas: apa isi `orbit_bringup` dibanding `tifa_bringup` / `nevo_bringup`, dan di mana config fleet RMF disimpan.

Isi config fleet `fleet_adapter_template` ternyata sebagian besar bergantung pada **site**, bukan pada robot:

| Isi config fleet | Bergantung pada |
| --- | --- |
| Nama dan jumlah robot (`tifa1`, `tifa2`) | site / deployment |
| Waypoint charger (`tifa_charger`) | nav graph site |
| `reference_coordinates` (pasangan titik nav graph ↔ peta SLAM) | nav graph site **dan** peta SLAM site |
| `limits`, `footprint`, `reversible` | robot (duplikat parameter Nav2) |

## Alternatif

| Alternatif | Kelebihan | Kekurangan | Keputusan |
| --- | --- | --- | --- |
| A. Adapter (py + yaml) di tiap bringup | Robot "lengkap" sendiri | Logika transform/health dobel; robot wajib memasang RMF | Ditolak (ADR-002) |
| B. Kode di `orbit_adapter`, YAML fleet di tiap bringup robot | Profil robot dekat dengan URDF/Nav2 | YAML berisi data site (charger, reference point) sehingga bringup robot harus diubah tiap site baru; bringup robot tahu RMF | Ditolak |
| **C. Kode di `orbit_adapter`, YAML fleet per site di `orbit_bringup/config/sites/<site>/`** | Bringup robot bebas RMF; site baru tidak menyentuh package robot | `limits`/`footprint` terduplikasi dengan param Nav2 | **Dipilih** |

## Keputusan

1. **Bringup robot** (`tifa_bringup`, `nevo_bringup`, dan bringup robot lain seperti AGV PLC) = definisi satu robot. Untuk robot Nav2: URDF, hardware (`sim:=true|false`), sensor, rantai `cmd_vel`, EKF, Nav2. Untuk robot PLC: `plc_agent`, `plc_map.yaml`, dan `plc_emulator` untuk sim. Tidak mengandung kode adapter atau config fleet, dan tidak bergantung pada RMF core maupun package ORBIT lain. Robot non-Nav2 hanya bergantung pada definisi message `rmf_fleet_msgs`. Harus bisa dijalankan dan dikendalikan tanpa ORBIT (robot Nav2: teleop atau goal RViz; robot PLC: `PathRequest` manual).
2. Kontrak bringup robot yang dipakai ORBIT:
   - pintu masuk `launch/robot.launch.py` dengan argumen `namespace`, `sim`, `use_sim_time`, `map`, `nav`, dan `x`/`y`/`yaw` (posisi spawn sim sekaligus initial pose AMCL);
   - `robot.launch.py` hanya menyalakan robot, tidak pernah menyalakan Gazebo. World dan bridge `/clock` dijalankan sekali per world oleh `orbit_sim/launch/world.launch.py` (dipanggil manual untuk uji, atau oleh `orbit_bringup/sim.launch.py`);
   - semua world, termasuk ruang uji `test_room`, ada di `orbit_sim/sites/<site>/`; bringup robot tidak menyimpan world. Mesh robot ditemukan Gazebo lewat export `<gazebo_ros gazebo_model_path="${prefix}/.."/>` di `package.xml` bringup robot;
   - semua topic, action, dan tf relatif (di bawah `/<namespace>`);
   - robot Nav2 (`backend: nav2`): menyediakan action `navigate_to_pose`, pose robot di frame `map`, dan status baterai;
   - robot non-Nav2, mis. PLC (`backend: graph`): menyediakan kontrak `rmf_fleet_msgs` (`PathRequest`, `ModeRequest` masuk; `RobotState` keluar) lewat node penerjemah di bringup-nya sendiri (ADR-002).
3. **`orbit_bringup`** = konfigurasi dan peluncuran satu site, berjalan di server FMS: node inti RMF, satu `orbit_adapter` per fleet, dan (hanya `mode:=sim`) Gazebo beserta `robot.launch.py` tiap robot.
4. Config fleet disimpan per site: `orbit_bringup/config/sites/<site>/fleet_<robot>.yaml`, di samping `site.yaml`.
5. Arah dependency: `orbit_bringup` → `orbit_adapter`, `orbit_sim`, dan bringup robot (`exec_depend`, hanya untuk mode sim). Bringup robot tidak bergantung pada package ORBIT mana pun.

## Konsekuensi

- Komputer robot tidak perlu memasang paket RMF.
- Menambah site baru hanya menyentuh `orbit_sim` (building.yaml, peta) dan `orbit_bringup` (config site).
- Nilai `limits`/`footprint` di YAML fleet harus konsisten dengan param Nav2 robot. Tambahkan tes yang membandingkan keduanya.

## Isi awal `tifa_bringup` (branch `feat/tifa_bringup`, 23 Okt 2026)

Sumber: `tifa` repo commit `2be8b4e` (`src/tifa_bot` + driver). Diperiksa 23 Okt 2026.

| Package | Sumber di ORBIT | Alasan |
| --- | --- | --- |
| `tifa_bringup` | salinan `tifa_bot`, diganti nama | Definisi robot TIFA |
| `diffdrive_arduino` | vendored (`src/tifa/`) | Tidak ada di apt; sudah di-patch untuk ros2_control Jazzy |
| `rf2o_laser_odometry` | vendored (`src/tifa/`) | Tidak ada rosdep key di Jazzy. Lisensi GPLv3: source wajib ikut dirilis |
| `rplidar_ros` | vendored **sementara** | apt Jazzy (2.1.0) tidak punya `rplidar_c1_launch.py` maupun executable `rplidar_node`; salinan tifa 2.1.4. Namanya sama dengan paket apt, jadi tidak boleh ikut dirilis sebagai `.deb` (ADR-003). Ganti ke apt setelah diuji di robot, atau ganti nama |

Tidak ikut: `tifa_ws_bridge`, `tifa_diagnostic_node`, `tifa_robot_api`, `tifa_data_logger`, `tifa_gsheet_logger_v2` (di luar lingkup robot untuk RMF). `worlds/` dan peta site pindah ke `orbit_sim` (PR terpisah). Hanya peta yang dipakai yang disalin.

Perbaikan wajib sebelum merge: semua topic relatif (termasuk yang hardcoded di `safety_fail_node.cpp` dan `publish_initial_pose.cpp`), key YAML `/**/` untuk node non-Nav2, Nav2 lewat `RewrittenYaml(root_key=namespace)`, hapus dependency PostgreSQL/nlohmann_json yang tidak dipakai, hapus path peta hardcoded `/home/tifav1/...`.

## Mekanisme namespace (diterapkan di `tifa_bringup`, 26 Okt 2026)

- Namespace topic, bukan prefix frame: tf tiap robot di `/<ns>/tf` (remap `/tf`→`tf`), nama frame tetap `base_link`, `odom`, `map`. Pola multi-robot Nav2.
- Topic dan nama service relatif di config, launch, dan C++; key YAML node non-Nav2 memakai `/**/`; Nav2 lewat `RewrittenYaml(root_key=namespace)`.
- Sim: argumen xacro `namespace` untuk topic sensor Gazebo dan `<ros><namespace>` plugin `gz_ros2_control` (controller_manager per robot di dalam Gazebo); bridge memakai `expand_gz_topic_names`.
- Urutan nyala memakai `TimerAction`, bukan event handler: aksi dari `RegisterEventHandler` berjalan di luar `PushRosNamespace` sehingga node kehilangan namespace.
- Terverifikasi di sim: dua TIFA (`tifa1`, `tifa2`) dalam satu world, controller dan Nav2 masing-masing aktif, teleop terisolasi, goal Nav2 keduanya SUCCEEDED.

## Verifikasi

| Tes | Expected | Status |
| --- | --- | --- |
| `ros2 launch tifa_bringup robot.launch.py namespace:=tifa1 sim:=true` tanpa RMF ter-source | Robot bisa di-teleop dan menerima goal Nav2 | NOT RUN |
| `rosdep keys --from-paths src/tifa` | Tidak ada key `rmf_*` | NOT RUN |
| `orbit.launch.py mode:=sim` dengan dua robot | Semua node robot di bawah `/tifa1`, `/nevo1`; adapter menerima status keduanya | NOT RUN |
| `orbit_sim world.launch.py site:=test_room` + 2× `tifa_bringup robot.launch.py sim:=true` (`tifa1`, `tifa2`), `nav:=true` | Dua entity, 2×2 controller active, scan 10 Hz per robot, satu publisher `/clock`, teleop terisolasi, goal Nav2 keduanya SUCCEEDED | PASS (26 Okt 2026). Catatan: satu kali dari tiga run goal `tifa2` ABORTED karena `compute_path_to_pose` timeout saat CPU penuh (load 11) |
| `orbit_sim world.launch.py site:=telu_coffee` dan `site:=gedung_P` + satu TIFA `sim:=true` | World dan semua `model://` termuat tanpa error, controller active, scan 10 Hz | PASS (28 Okt 2026). Step fisika 4 ms + lidar sim 360 sampel: RTF 1.0, di `test_room` perintah 0.3 m/s menghasilkan EKF 0.31 m/s |
| Tes konsistensi limits YAML fleet vs param Nav2 | Sama | NOT RUN |
