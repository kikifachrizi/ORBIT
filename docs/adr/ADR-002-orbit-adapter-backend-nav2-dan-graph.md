# ADR-002: `orbit_adapter` dengan dua backend navigasi (`nav2` dan `graph`)

**Status:** Proposed  
**Tanggal:** 21 Oktober 2026 (WIB)  
**Owner:** _diisi saat review_ · **Reviewer:** _diisi saat review_  
**Requirements:** FR-02, FR-04, FR-05, FR-06, FR-07, FR-09, NFR-01, NFR-03 · **PR / issue:** _diisi_

## Konteks

Berdasarkan ADR-001, Open-RMF menjadi fleet core. RMF memerlukan **fleet adapter** untuk setiap jenis robot: komponen yang menerima perintah RMF ("pergi ke waypoint X", "berhenti", "jalankan aksi") dan melaporkan posisi, baterai, serta status robot.

Kebutuhan ORBIT:

1. TIFA dan NEVO menggunakan ROS 2 + Nav2 (free navigation).
2. FMS juga harus dapat dipakai untuk robot dengan navigasi custom yang mengikuti nav graph traffic editor secara langsung, yaitu: (a) robot dengan lokalisasi sendiri dan controller titik-ke-titik, (b) line follower yang dikendalikan PLC, dan (c) slotcar untuk simulasi cepat.
3. NFR-01: logika fleet terpisah dari detail platform robot; menambah atau mengganti robot tidak boleh mengubah scheduler.
4. FR-04: kontrak adapter yang sama untuk simulator, TIFA, dan NEVO.

Fakta yang sudah diverifikasi dari source code:

- Slotcar (`rmf_simulation`) menerima rute melalui `rmf_fleet_msgs/PathRequest` pada topic `/robot_path_requests`, menerima mode (pause/resume/charging) melalui `rmf_fleet_msgs/ModeRequest` pada `/robot_mode_requests`, dan melapor melalui `rmf_fleet_msgs/RobotState` pada `/robot_state`.
- `rmf_fleet_msgs` tersedia sebagai binary resmi Jazzy dan tidak bergantung pada Nav2.
- `PathRequest` = `fleet_name`, `robot_name`, `path` (`Location[]`: `t`, `x`, `y`, `yaw`, `level_name`, `index`, ...), `task_id`.
- `RobotState` = `name`, `model`, `task_id`, `seq`, `mode` (IDLE, CHARGING, MOVING, PAUSED, WAITING, EMERGENCY, ..., ADAPTER_ERROR, PERFORMING_ACTION, ACTION_COMPLETED), `battery_percent`, `location`, `path`.
- Referensi adapter RMF ↔ Nav2: `free_fleet/free_fleet_adapter` (`nav2_robot_adapter.py`) dan `ionic_demo/fleet_adapter_nav2`. Keduanya memakai `nudged` untuk transformasi koordinat; `python3-nudged` tidak memiliki rosdep key di Jazzy.

## Alternatif

| Alternatif | Kelebihan | Kekurangan | Keputusan |
| --- | --- | --- | --- |
| **A. Satu `orbit_adapter` dengan backend yang dapat dipilih per robot: `nav2` dan `graph`** | Satu kode adapter untuk semua robot. Robot custom cukup mengikuti kontrak `rmf_fleet_msgs`. Backend `graph` dapat diuji dengan slotcar tanpa hardware | Perlu desain interface backend yang rapi | **Dipilih** |
| B. Satu adapter terpisah per jenis robot (pola `fleet_adapter_template`) | Paling sederhana untuk satu robot | Duplikasi logika (health, transform, dedup); melanggar semangat NFR-01 | Ditolak |
| C. Memakai `free_fleet_adapter` langsung | Sudah jalan dengan Nav2 | Tidak dirilis binary, dependency pip dan Zenoh bridge, hanya Nav1/Nav2 | Ditolak untuk runtime; dipakai sebagai referensi |
| D. Satu backend per jenis robot custom (line follower, slotcar, controller sendiri) | Fleksibel | Jumlah backend bertambah setiap ada robot baru | Ditolak; diganti kontrak `graph` bersama |

## Keputusan dan konsekuensi

### Arsitektur

```
RMF core (task, traffic, nav graph)
          │  EasyFullControl callbacks: navigate / stop / execute_action
          ▼
orbit_adapter
  ├── core (tanpa ROS): RobotBackend interface, transform koordinat (numpy),
  │                     health/stale detector, dedup task, pelaporan status
  ├── backend "nav2":  NavigateToPose / NavigateThroughPoses ke Nav2 robot
  │                     → TIFA, NEVO
  └── backend "graph": publish PathRequest + ModeRequest, subscribe RobotState
                        → slotcar (sim), robot custom, PLC agent (line follower)
```

### Interface backend (kontrak di dalam `orbit_adapter`)

| Operasi | Arti | Dipanggil saat |
| --- | --- | --- |
| `navigate(target, path)` | Bergerak ke target (Nav2) atau mengikuti path nav graph (`graph`) | RMF `navigate` |
| `stop()` / `pause()` / `resume()` | Hentikan atau jeda gerakan | RMF `stop`, traffic negotiation |
| `execute_action(name, params)` | Aksi khusus (mis. konfirmasi muat/bongkar sintetis) | RMF `execute_action` |
| `status()` | Posisi, baterai, mode, error, progres perintah, timestamp terakhir | Update loop adapter |

### Kontrak backend `graph` (untuk robot non-Nav2)

Robot custom wajib mengimplementasikan tiga message standar RMF berikut, dengan namespace atau `robot_name` sesuai konfigurasi:

| Arah | Message | Kewajiban robot |
| --- | --- | --- |
| Adapter → robot | `rmf_fleet_msgs/PathRequest` | Mengikuti urutan titik; berhenti di titik terakhir |
| Adapter → robot | `rmf_fleet_msgs/ModeRequest` | Pause, resume, dan mode lain yang didukung |
| Robot → adapter | `rmf_fleet_msgs/RobotState` | Publish periodik: posisi di frame nav graph, baterai, mode, `task_id`, `seq` naik |

Implementasi per jenis robot:

| Jenis | Implementasi di sisi robot | Status sprint |
| --- | --- | --- |
| Slotcar (simulasi) | Sudah tersedia di `rmf_robot_sim_gz_plugins` | Target sprint (bukti NFR-01) |
| Robot custom dengan lokalisasi sendiri | Node kecil: `PathRequest` → controller titik-ke-titik; publish `RobotState` | Opsional jika waktu cukup |
| Line follower + PLC | PLC agent: `PathRequest` → antrian cabang PLC (Modbus TCP); marker terakhir → `RobotState`; fail-safe di PLC | Pasca-sprint; kontrak sudah siap |

### Robot dengan navigasi penuh di PLC (rincian backend `graph`)

Didiskusikan 23 Okt 2026. Status: Proposed.

- **Pita = nav graph.** Fleet PLC memakai `graph_idx` sendiri (mis. 2). Satu waypoint per tag RFID/marker, satu lane per ruas pita. Tidak ada peta SLAM dan tidak ada transform: posisi dihitung di frame nav graph dari marker terakhir ditambah jarak encoder.
- **Tidak ada kode Modbus di `orbit_adapter`.** Adapter memakai backend `graph` (rmf_fleet_msgs). Penerjemah ke PLC (`plc_agent`) berada di package bringup robot tersebut (mis. `agv_bringup`), setara posisinya dengan driver Arduino di `tifa_bringup`.
- **`plc_agent`**: `PathRequest.path[].index` → nomor RFID target + antrian belokan (dihitung dari nav graph) → register; register status → `RobotState` 10 Hz; heartbeat ke PLC tiap 200 ms.
- **Protokol: Modbus TCP via `python3-pymodbus`.** Diverifikasi 23 Okt 2026: tersedia di apt Ubuntu 24.04 (3.6.9) dan punya rosdep key. Library OPC UA (`python3-opcua`, `python3-asyncua`) tidak punya rosdep key, sehingga tidak boleh dipakai sebagai dependency runtime (ADR-003).
- **Safety di PLC**: scanner menghentikan motor; watchdog heartbeat (mis. 1 s) menghentikan robot bila `plc_agent` atau jaringan mati. Adapter menandai robot stale bila `RobotState` berhenti datang.
- Robot tidak bisa memutari halangan. RMF hanya melihat keterlambatan lalu menggeser jadwal atau merutekan ulang.
- Mode sim: `plc_emulator` (server Modbus tiruan) menggantikan PLC, jadi `plc_agent` dan backend `graph` teruji di CI tanpa hardware.
- Program PLC (ST/ladder) disimpan berversi di repo dengan `COLCON_IGNORE`, dibangun dengan software vendor.
- Kontrak register (`plc_map.yaml`) disepakati bersama programmer PLC sebelum implementasi.

### Konfigurasi robot

Menambah robot dengan jenis yang sudah didukung dilakukan melalui konfigurasi, tanpa mengubah kode adapter. File config fleet disimpan per site di `orbit_bringup/config/sites/<site>/fleet_<robot>.yaml`, bukan di bringup robot (ADR-004):

```yaml
robots:
  tifa:
    backend: nav2
    namespace: /tifa
    charger: tifa_charger            # waypoint di nav graph
    reference_points: ...            # pasangan titik RMF ↔ peta robot (khusus nav2)
  slotcar_1:
    backend: graph
    charger: slotcar_1_charger
```

Selain YAML, robot baru juga membutuhkan: waypoint charger/parkir di nav graph, profil fisik (footprint, kecepatan), dan untuk `nav2` kalibrasi titik referensi peta. Jenis robot baru yang tidak cocok dengan `nav2` maupun kontrak `graph` memerlukan backend baru.

### Implementasi awal (branch `feat/orbit_adapter`, 28 Okt 2026)

- Scope dipersempit karena jadwal (D7): hanya backend `nav2` dan site `test_room`. Backend `graph`, NEVO, dan PLC ditunda; interface `RobotBackend` sudah disiapkan agar backend baru tidak mengubah core.
- Struktur: `backends/base.py` (`RobotBackend`: `pose`, `battery_soc`, `navigate`, `stop`, `nav_status`), `backends/nav2.py`, `robot_adapter.py` (callback EasyFullControl ↔ backend), `fleet_adapter.py` (satu proses per fleet), `dispatch_task.py` (kirim task lewat `/task_api_requests`, karena `rmf_demos_tasks` tidak dirilis sebagai dependency ORBIT).
- Config memakai format `FleetConfiguration.from_config_files` milik RMF. Key ORBIT (`backend`, `namespace`, `map_frame`, `base_frame`, `battery_topic`, `pose_timeout`) ditaruh di `rmf_fleet.robots.<nama>` dan diabaikan parser RMF.
- **Transformasi koordinat memakai `reference_coordinates` bawaan EasyFullControl**, bukan kode `numpy` sendiri: RMF 2.7 sudah mengubah posisi robot ↔ nav graph dari pasangan titik di config fleet. Tanpa `reference_coordinates`, transformasinya identitas (dipakai `test_room`, karena frame nav graph = frame map Nav2).
- Backend `nav2`: satu node rclpy per robot di namespace robot. Listener tf di-remap ke `/<ns>/tf`; pose = tf `map → base_link` dan dianggap basi setelah `pose_timeout` (default 2 s). Goal dikirim ke action `navigate_to_pose`.
- Robot baru didaftarkan ke RMF setelah pose pertama tersedia, yaitu setelah AMCL aktif. Bila pose basi, update ke RMF dihentikan.
- Bila Nav2 gagal, goal dicoba ulang hingga 3 kali dengan jeda 2 s. Setelah itu perintah diakhiri dan RMF merencanakan ulang dari posisi aktual.
- Baterai: dari `battery_topic` (`sensor_msgs/BatteryState`) bila diisi; selain itu 100%.

### Konsekuensi

- Transformasi koordinat RMF ↔ peta robot memakai `reference_coordinates` EasyFullControl (lihat implementasi awal), jadi tidak butuh `nudged` maupun kode transformasi sendiri (ADR-003).
- Untuk robot Nav2 diperlukan dua peta yang disejajarkan: nav graph (traffic editor) untuk RMF, dan occupancy map (`.pgm/.yaml`) untuk Nav2.
- Safety existing robot tidak dioverride: adapter hanya mengirim tujuan dan perintah stop/pause; pengereman darurat, scanner, dan e-stop tetap di robot/PLC (NFR-03).
- Health detector menandai robot stale bila `status()` tidak diperbarui melewati batas waktu yang ditetapkan sebelum tes; robot stale tidak menerima task baru (G1-06).

## Verifikasi

| Tes | Expected | Status |
| --- | --- | --- |
| Contract test backend dengan `FakeBackend` (tanpa ROS/Gazebo) | Navigate, stop, timeout, dan stale terdeteksi sesuai kontrak | PASS (28 Okt 2026): `orbit_adapter/test/test_robot_adapter.py`, 5 tes: registrasi setelah pose pertama, navigate sukses, retry lalu menyerah, stop hanya untuk aktivitas yang cocok, pose basi menghentikan update |
| Backend `nav2` dengan dua robot Nav2 di Gazebo Harmonic | Task RMF dieksekusi, feedback dan hasil terminal tercatat | PASS (28 Okt 2026, `test_room`, Cyclone DDS): `tifa1` patrol north→east dan `tifa2` go_to south bersamaan, RMF menahan robot di titik tunggu untuk menghindari konflik di `center`, keduanya kembali parkir ke charger. Dengan Fast DDS gagal 2/4 run (ADR-005) |
| Backend `graph` dengan slotcar | Task yang sama dieksekusi tanpa mengubah kode core (bukti NFR-01) | NOT RUN |
| Fault injection: robot berhenti melapor | Robot ditandai stale, task tidak menggantung tanpa status | NOT RUN |
| TIFA dan NEVO fisik dengan backend `nav2` | G2-01 s.d. G2-05 per robot | NOT RUN |

Trigger peninjauan ulang: backend `graph` tidak dapat memenuhi kebutuhan line follower tanpa mengubah kontrak, atau interface backend memerlukan perubahan saat menambah TIFA/NEVO.
