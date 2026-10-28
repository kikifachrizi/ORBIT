# ADR-001: Open-RMF sebagai fleet core ORBIT

**Status:** Proposed  
**Tanggal:** 21 Oktober 2026 (WIB)  
**Owner:** _diisi saat review_ · **Reviewer:** _diisi saat review_  
**Requirements:** FR-02, FR-03, FR-04, FR-05, FR-06, FR-08, NFR-01, NFR-02 · **PR / issue:** _diisi_

## Konteks

ORBIT harus menghasilkan prototype FMS multi-robot dalam 9 hari (36 jam efektif per orang), berjalan di Ubuntu 24.04 + ROS 2 Jazzy + Gazebo Harmonic, mengoordinasikan minimal dua robot di simulasi dan TIFA/NEVO di dunia nyata, lalu dirilis sebagai binary yang bisa dipasang di environment bersih (G3).

Fungsi inti FMS yang dibutuhkan:

| Fungsi | Contoh kebutuhan ORBIT |
| --- | --- |
| Registrasi robot dan status | FR-02, FR-06 |
| Penerimaan task dan pemilihan robot | FR-01, FR-03 |
| Lifecycle task dan hasil | FR-05 |
| Koordinasi lalu lintas multi-robot (mencegah tabrakan dan deadlock di jalur bersama) | G2-06 |
| Peta jalur bersama (titik, lane, stasiun, charger) | FR-08, G1 |

Fakta yang sudah diverifikasi (21 Okt 2026, dari `ros/rosdistro` cabang Jazzy dan source code):

- Open-RMF tersedia sebagai binary resmi Jazzy: `rmf_ros2` 2.7.2 (`rmf_fleet_adapter`, `rmf_fleet_adapter_python`, `rmf_task_ros2`, `rmf_traffic_ros2`), `rmf_traffic_editor` 1.9.2 (termasuk `rmf_building_map_tools`), `rmf_simulation` 2.3.3, `rmf_visualization` 2.3.2, `rmf_demos` 2.3.0 (`rmf_demos_fleet_adapter`, `rmf_demos_tasks`, `rmf_demos_assets`).
- `rmf_demos` menggunakan robot simulasi "slotcar" (plugin Gazebo dari `rmf_simulation`), bukan Nav2.
- `free_fleet` (open-rmf) menyediakan integrasi RMF + Nav2 yang diuji CI di Jazzy, tetapi tidak dirilis sebagai binary dan bergantung pada paket pip (`eclipse-zenoh`, `nudged`, `pycdr2`, `rosbags`).

## Alternatif

| Alternatif | Kelebihan | Kekurangan | Keputusan |
| --- | --- | --- | --- |
| **A. Open-RMF sebagai fleet core** | Traffic schedule, negotiation, task allocation (bidding), charging, door/lift, dan visualisasi sudah matang. Binary resmi Jazzy tersedia. Ekosistem adapter dan contoh luas | Kurva belajar tinggi. Asumsi robot mengikuti lane/jadwal. Debug lintas banyak node | **Dipilih** |
| B. FMS buatan sendiri (dispatcher + traffic zone/reservation) | Kontrol penuh, ringan, mudah dijelaskan | Traffic multi-robot dan deadlock harus dibuktikan sendiri. Tidak realistis untuk 36 jam bersama G2 dan G3 | Ditolak untuk sprint; dapat ditinjau pasca-sprint |
| C. openTCS (Fraunhofer IML) | GUI lengkap, model block area, fokus AGV | Java dan bukan ROS. Integrasi Nav2 memerlukan bridge (VDA5050 v1.1/v2.0). Tidak sesuai baseline ROS 2 | Ditolak |
| D. `free_fleet` apa adanya sebagai solusi akhir | Paling cepat jalan dengan Nav2 di Jazzy | Tidak dirilis sebagai binary, dependency pip, ketergantungan Zenoh bridge. Bertentangan dengan G3 | Dipakai sebagai **baseline/referensi**, bukan runtime release |

## Keputusan dan konsekuensi

ORBIT memakai **Open-RMF (binary resmi Jazzy) sebagai fleet core** tanpa memodifikasi kode RMF. Pekerjaan ORBIT difokuskan pada bagian yang tidak disediakan RMF:

| Disediakan Open-RMF (dipakai apa adanya) | Dibangun ORBIT |
| --- | --- |
| Traffic schedule dan negotiation | `orbit_adapter`: penerjemah RMF ↔ robot (lihat ADR-002) |
| Task dispatcher dan bidding | `orbit_sim`: peta fasilitas fiktif (traffic editor), world Gazebo, model TIFA/NEVO |
| Building map server dan nav graph tools | `orbit_bringup`: launch dan konfigurasi sim/fisik |
| Visualisasi RViz dan API task (JSON) | `orbit`: metapackage untuk binary release (lihat ADR-003) |

Konsekuensi:

- Semua anggota tim mempelajari RMF bersama pada D1–D2 (menjalankan `rmf_demos` dan baseline `free_fleet`) sebelum membagi pekerjaan.
- Kontrak bersama yang wajib disepakati lebih awal: nav graph (titik, lane, nama stasiun) dan konfigurasi fleet (nama robot, namespace, kecepatan, footprint).
- Perilaku RMF pada kondisi gagal tidak diasumsikan, tetapi diuji (lihat Verifikasi).
- Batas dukungan: ORBIT hanya mendukung versi RMF yang dirilis untuk Jazzy. Fitur RMF di luar scope sprint (door, lift, workcell nyata, rmf-web produksi) dicatat sebagai known limitation.

Risiko dan mitigasi:

| Risiko | Mitigasi |
| --- | --- |
| Kurva belajar RMF menghabiskan waktu sprint | Go/no-go di akhir D2: dua robot simulasi menerima task dari RMF. Bila gagal, gunakan baseline `free_fleet` untuk G1 dan catat gap |
| Perilaku RMF saat adapter/robot hilang tidak transparan | Fault injection wajib (G1-06) dan health check di `orbit_adapter` |
| Ketidaksesuaian versi contoh komunitas (mis. `ionic_demo` menargetkan Rolling + Gazebo Ionic) | Hanya gunakan contoh yang terverifikasi di Jazzy; contoh lain dipakai sebagai referensi kode |

## Verifikasi

| Item                                                                                           | Expected                                                                  | Status / evidence                                                                                                                           |
| ---------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------- |
| Baseline RMF + Nav2 dua robot di Jazzy (`free_fleet`, `unique_multi_tb3_simulation_launch.py`) | RMF core, fleet adapter, dan dua robot aktif; task patrol diterima        | Berjalan 21 Okt 2026 setelah perbaikan environment (zsh `setup.zsh`, `ROS_DOMAIN_ID`); evidence log/video perlu dilampirkan                 |
| Temuan awal: fleet adapter mati saat task dikirim                                              | Task ditolak dengan alasan yang jelas                                     | **Actual:** `dispatch_patrol` hanya mengembalikan "Did not get a response" (timeout 5 detik). Input untuk desain health check ORBIT (FR-07) |
| Go/no-go RMF (akhir D2)                                                                        | Dua robot simulasi ORBIT menerima task dari RMF                           | NOT RUN                                                                                                                                     |
| Fault injection: adapter mati, Zenoh/jaringan putus, robot berhenti                            | Status task terminal tercatat, robot tidak sehat tidak menerima task baru | NOT RUN                                                                                                                                     |

Trigger peninjauan ulang: go/no-go D2 gagal, RMF tidak mampu memenuhi G1-06/G1-07 tanpa modifikasi core, atau kebutuhan deployment tanpa ROS 2.
