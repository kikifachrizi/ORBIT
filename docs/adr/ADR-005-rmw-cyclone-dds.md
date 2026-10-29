# ADR-005: Cyclone DDS sebagai RMW ORBIT

**Status:** Proposed  
**Tanggal:** 28 Oktober 2026 (WIB)  
**Owner:** _diisi saat review_ · **Reviewer:** _diisi saat review_  
**Requirements:** G1, NFR-02 · **PR / issue:** _diisi_

## Konteks

Uji end-to-end `orbit.launch.py site:=test_room mode:=sim` (inti RMF, `orbit_adapter`, dua TIFA dengan Nav2) dijalankan di laptop dev (Jazzy, RTX 5050, GZ_PARTITION dan ROS_DOMAIN_ID terisolasi). Total sekitar 50 proses ROS.

Dengan RMW default Jazzy (Fast DDS, `rmw_fastrtps_cpp`), pada 2 dari 4 run satu robot tidak pernah terdaftar di RMF:

- Node lifecycle Nav2 (`map_server` atau `amcl`) mencatat `failed to send response to /<ns>/<node>/change_state (timeout): client will not receive response`.
- `lifecycle_manager_localization` menunggu respons itu tanpa batas waktu (Jazzy tidak punya timeout `change_state`). AMCL tertahan di `unconfigured`/`inactive`, robot tidak punya pose `map`, dan adapter tidak mendaftarkannya.
- Kejadian ini tidak terkait beban CPU: tetap muncul setelah robot dinyalakan berjarak 25 s dan load rendah. Server service mengirim respons sebelum discovery klien selesai, dan peluangnya naik seiring jumlah participant.

## Alternatif

| Alternatif | Hasil / pertimbangan | Keputusan |
| --- | --- | --- |
| A. Tetap Fast DDS, nyalakan robot bergiliran (`robot_stagger`) | Tetap gagal 1 dari 2 run setelah jeda 25 s | Tidak cukup |
| B. Nav2 composition (satu `component_container_isolated` per robot) | Race hilang, tetapi container segfault begitu controller DWB menerima goal; robot tidak bergerak. Perlu investigasi terpisah | Ditolak untuk sprint ini |
| C. Pemulihan otomatis (paksa `change_state` AMCL dari node lain) | Menambal gejala; lifecycle manager tetap menggantung | Ditolak |
| **D. Cyclone DDS (`rmw_cyclonedds_cpp`, binary resmi Jazzy)** | 3 dari 3 run: kedua robot terdaftar dan kedua task selesai, 0 kejadian `failed to send response`. Perlu `MaxAutoParticipantIndex` dinaikkan (default kehabisan index sekitar 50 proses) | **Diusulkan** |
| E. Zenoh (`rmw_zenoh_cpp`) | Tersedia di apt, tetapi butuh router dan belum diuji | Cadangan |

## Keputusan dan konsekuensi

- Semua proses ORBIT, yaitu server FMS, robot, dan terminal operator (`ros2 ...`, `dispatch_task`), memakai:

  ```bash
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  export CYCLONEDDS_URI=file://$(ros2 pkg prefix orbit_bringup)/share/orbit_bringup/config/cyclonedds.xml
  ```

- `orbit_bringup/config/cyclonedds.xml` menaikkan `MaxAutoParticipantIndex` ke 200.
- RMW tidak dipaksa dari dalam launch file, karena robot fisik dan terminal lain harus memakai RMW yang sama. Pengaturannya didokumentasikan di README dan runbook.
- Komputer robot (TIFA, NEVO) perlu `ros-jazzy-rmw-cyclonedds-cpp`. Paket ini ada di apt Jazzy, jadi sesuai ADR-003.
- Tim lain yang belum pindah tetap bisa berkomunikasi lewat interoperabilitas DDS, tetapi kombinasi campuran tidak diuji dan tidak didukung.

## Verifikasi

| Tes | Expected | Status |
| --- | --- | --- |
| Uji G1 `test_room` 2 TIFA, Fast DDS (4 run) | Kedua robot terdaftar, task selesai | FAIL 2/4 (28 Okt 2026): race lifecycle Nav2 |
| Uji G1 `test_room` 2 TIFA, Cyclone DDS + `cyclonedds.xml` (3 run) | Kedua robot terdaftar, task selesai | PASS 3/3 untuk registrasi. Run ke-2 kehilangan request `dispatch_task`; penyebabnya di `dispatch_task` (keluar sebelum fleet adapter ter-match), sudah diperbaiki dan run ke-3 PASS penuh |
| Robot TIFA fisik dengan Cyclone DDS terhubung ke server FMS | Topic dan tf robot terlihat dari server | NOT RUN |

Trigger peninjauan ulang: Fast DDS Jazzy merilis perbaikan race respons service, atau Cyclone gagal di jaringan robot fisik (multicast, Wi-Fi).
