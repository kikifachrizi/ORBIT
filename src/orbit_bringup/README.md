# orbit_bringup

Konfigurasi dan peluncuran satu site ORBIT (ADR-004): inti RMF, satu `orbit_adapter` per fleet, dan pada `mode:=sim` juga world Gazebo serta `robot.launch.py` tiap robot.

```
config/cyclonedds.xml            # konfigurasi RMW (ADR-005)
config/sites/<site>/site.yaml    # fleet dan robot sim site ini
config/sites/<site>/fleet_*.yaml # config fleet EasyFullControl + key ORBIT per robot
launch/orbit.launch.py           # pintu masuk: site:=<site> mode:=sim|real
launch/rmf_core.launch.py        # schedule, blockade, building map, dispatcher, visualisasi
```

World, peta Nav2, `building.yaml`, dan nav graph diambil dari `orbit_sim/sites/<site>/`.

## Menjalankan site test_room (sim, 2 TIFA)

Di setiap terminal (ADR-005):

```bash
source install/setup.zsh
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$(ros2 pkg prefix orbit_bringup)/share/orbit_bringup/config/cyclonedds.xml
```

```bash
ros2 launch orbit_bringup orbit.launch.py site:=test_room mode:=sim
# tunggu log "tifa1 terdaftar di RMF" dan "tifa2 terdaftar di RMF" (sekitar 60 s)
ros2 run orbit_adapter dispatch_task --sim --robot tifa1 patrol north east
ros2 run orbit_adapter dispatch_task --sim --robot tifa2 go_to south
ros2 run orbit_adapter dispatch_task --sim patrol east south     # RMF memilih robot
```

Argumen: `rviz:=false` (tanpa RViz RMF), `gz_args:='-r -s --headless-rendering -v 1'` (Gazebo tanpa GUI), `robot_stagger:=25.0` (jeda antar robot sim, detik).

Waypoint `test_room`: `tifa1_charger`, `tifa2_charger`, `west`, `center`, `north`, `east`, `south`.
