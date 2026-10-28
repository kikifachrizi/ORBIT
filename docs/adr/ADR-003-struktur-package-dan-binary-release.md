# ADR-003: Struktur package dan batas dependency untuk binary release

**Status:** Proposed  
**Tanggal:** 21 Oktober 2026 (WIB)  
**Owner:** _diisi saat review_ · **Reviewer:** _diisi saat review_  
**Requirements:** FR-10, NFR-02, NFR-04, G3-01 s.d. G3-05 · **PR / issue:** _diisi_

## Konteks

G3 mensyaratkan binary berversi yang dapat dipasang pada environment bersih **tanpa source workspace dan tanpa build manual**. Ubuntu 24.04 + ROS 2 Jazzy adalah baseline. Clean install akan gagal bila ORBIT bergantung pada paket yang hanya tersedia dari source atau pip.

Status dependency yang sudah diverifikasi di `ros/rosdistro` (Jazzy) pada 21 Okt 2026:

| Dependency | Tersedia sebagai `.deb` resmi? |
| --- | --- |
| `rmf_ros2`, `rmf_traffic_editor`, `rmf_simulation`, `rmf_visualization`, `rmf_demos` (sebagian) | Ya |
| `navigation2` / `nav2_bringup` (1.3.14), `ros_gz` (1.0.25), `rmw_zenoh` (0.2.12) | Ya |
| `rmf_demos_gz`, `rmf_demos_maps` | Tidak (source saja) |
| `free_fleet` | Tidak (source + pip) |
| `nudged` | Tidak ada rosdep key (pip saja) |
| `python3-numpy` | Ya |

## Alternatif

| Alternatif | Kelebihan | Kekurangan | Keputusan |
| --- | --- | --- | --- |
| **A. Empat package ROS 2 + metapackage, dirilis sebagai `.deb` via `bloom-generate rosdebian`; dependency hanya dari apt/rosdep** | Sesuai praktik ROS; apt menarik RMF/Nav2 otomatis; clean install dapat diuji | Perlu disiplin dependency sejak awal | **Dipilih** |
| B. Docker image sebagai release | Environment terkunci | G3 meminta installable package di environment target; GUI/Gazebo di container lebih rumit | Opsional untuk demo, bukan artifact utama |
| C. Source archive + skrip build | Mudah dibuat | Ditolak oleh G3-02 ("source ZIP saja tidak cukup") | Ditolak |
| D. Git submodule untuk dependency luar | Versi terkunci | Bukan konvensi ROS; menyulitkan bloom dan CI | Ditolak; gunakan `orbit.repos` + `vcstool` hanya untuk workspace dev |

## Keputusan dan konsekuensi

### Struktur repository

```text
ORBIT/
├── orbit.repos                # source dependency khusus dev/referensi (bukan runtime)
├── src/
│   ├── orbit_adapter/         # fleet adapter RMF: core + backend nav2 + backend graph (ADR-002)
│   ├── orbit_sim/             # building.yaml (traffic editor), nav graph & world hasil generate,
│   │                          # occupancy map Nav2, model simulasi TIFA/NEVO
│   ├── orbit_bringup/         # launch site (RMF + adapter, + Gazebo saat sim), config fleet per site (ADR-004)
│   ├── tifa/tifa_bringup/     # definisi robot TIFA: URDF, hardware, Nav2; tanpa RMF (ADR-004)
│   ├── nevo/nevo_bringup/     # definisi robot NEVO, kontrak sama
│   └── orbit/                 # metapackage: exec_depend seluruh package ORBIT
├── tests/                     # contract test & launch_testing
├── docs/  templates/  evidence/
└── .github/workflows/         # build, test, bloom → .deb, checksum
```

### Aturan dependency

1. Dependency runtime hanya boleh berasal dari apt/rosdep Jazzy dan dideklarasikan di `package.xml`.
2. Tidak ada dependency runtime ke paket pip atau paket yang hanya tersedia dari source (`free_fleet`, `nudged`, `rmf_demos_gz`, `rmf_demos_maps`). Transformasi koordinat memakai `numpy`.
3. Model dan world simulasi di-bundle di `orbit_sim`; tidak bergantung pada unduhan Gazebo Fuel saat runtime.
4. Contoh komunitas (`free_fleet`, `ionic_demo`, `rmf_demos_gz`) hanya boleh muncul di `orbit.repos` untuk workspace dev.

### Artifact release

- Satu `.deb` per package (`ros-jazzy-orbit-*`), dibangun dari commit release candidate, disertai SHA-256.
- Pemasangan: `sudo apt install ./ros-jazzy-orbit*_<arch>.deb` dengan repository ROS 2 Jazzy terkonfigurasi.
- Arsitektur yang diklaim hanya yang memiliki artifact dan bukti clean install sendiri (minimal satu, mis. `amd64`).

### Konsekuensi

- Pemilihan library baru wajib dicek terhadap rosdistro Jazzy sebelum dipakai.
- Pipeline CI membangun `.deb` sejak D2 agar masalah dependency ditemukan lebih awal, bukan pada D8.
- Pemisahan metapackage runtime dan simulasi dapat ditambahkan bila server produksi tidak boleh memasang Gazebo.

## Verifikasi

| Tes | Expected | Status |
| --- | --- | --- |
| `rosdep check --from-paths src --ignore-src` pada environment bersih | Semua key terselesaikan dari apt | NOT RUN |
| `bloom-generate rosdebian` + build `.deb` di CI | Artifact `.deb` untuk setiap package + SHA-256 | NOT RUN |
| Clean install di VM/container Ubuntu 24.04 + Jazzy oleh reviewer independen | Install berhasil tanpa source workspace; smoke test simulasi berjalan | NOT RUN |

Trigger peninjauan ulang: dependency wajib yang tidak tersedia di apt, atau kebutuhan dukungan arsitektur tambahan.
