# orbit_sim

Data site untuk simulasi ORBIT. Satu folder per site di `sites/<site>/`:

```
models/            # perabot dipakai bersama semua site (model://<Nama>), lihat ATTRIBUTION.md
sites/<site>/
├── <site>.sdf     # world Gazebo
├── models/<site>/ # bangunan site (model://<site>), bila ada
├── maps/          # peta Nav2 (yaml + pgm) yang cocok dengan world
└── denah.svg      # denah sumber bangunan, bila ada
```

`world.launch.py` menambahkan `models/` dan `sites/<site>/models/` ke `GZ_SIM_RESOURCE_PATH`.

Package ini tidak berisi robot. Robot di-spawn oleh `<robot>_bringup/launch/robot.launch.py sim:=true` (ADR-004).

| Site | Isi | Peta Nav2 |
| --- | --- | --- |
| `test_room` | Ruang uji 8 x 6 m dengan dua kotak, untuk menguji satu atau beberapa robot | `maps/test_room.yaml` (frame map = world) |
| `telu_coffee` | TelU Coffee, dengan perabot | `maps/telu_coffee.yaml`, hasil SLAM robot fisik. Belum dicocokkan dengan frame world |
| `gedung_P` | Gedung P lantai 3, 97 x 34.2 m | Belum ada |

Model pihak ketiga wajib tercatat di `models/ATTRIBUTION.md` beserta lisensinya.

## Uji satu atau dua TIFA

```bash
ros2 launch orbit_sim world.launch.py site:=test_room
MAP=$(ros2 pkg prefix orbit_sim)/share/orbit_sim/sites/test_room/maps/test_room.yaml
ros2 launch tifa_bringup robot.launch.py sim:=true namespace:=tifa1 x:=-2.5 y:=0.5 nav:=true map:=$MAP
ros2 launch tifa_bringup robot.launch.py sim:=true namespace:=tifa2 x:=0.5 y:=-0.5 nav:=true map:=$MAP
```
