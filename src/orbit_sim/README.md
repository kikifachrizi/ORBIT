# orbit_sim

Data site untuk simulasi ORBIT. Satu folder per site di `sites/<site>/`:

```
sites/<site>/
├── <site>.sdf     # world Gazebo
├── models/        # model yang dipakai world (bila ada)
└── maps/          # peta Nav2 (yaml + pgm) yang cocok dengan world
```

Package ini tidak berisi robot. Robot di-spawn oleh `<robot>_bringup/launch/robot.launch.py sim:=true` (ADR-004).

| Site | Isi |
| --- | --- |
| `test_room` | Ruang uji 8 x 6 m dengan dua kotak, untuk menguji satu atau beberapa robot |

## Uji satu atau dua TIFA

```bash
ros2 launch orbit_sim world.launch.py site:=test_room
MAP=$(ros2 pkg prefix orbit_sim)/share/orbit_sim/sites/test_room/maps/test_room.yaml
ros2 launch tifa_bringup robot.launch.py sim:=true namespace:=tifa1 x:=-2.5 y:=0.5 nav:=true map:=$MAP
ros2 launch tifa_bringup robot.launch.py sim:=true namespace:=tifa2 x:=0.5 y:=-0.5 nav:=true map:=$MAP
```
