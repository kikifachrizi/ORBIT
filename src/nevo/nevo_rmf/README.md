# nevo_rmf

Open-RMF fleet adapter for NEVO (Project ORBIT). It uses EasyFullControl: RMF sends one
waypoint at a time, and the adapter drives NEVO there through Nav2. Owner: Arya.

Each lane (previous waypoint → next one) is sent to `/<ns>/follow_path` as a straight path
for the robot's `LaneFollow` (Regulated Pure Pursuit) controller. NEVO stays on the drawn
lane and turns on the spot at waypoints. With `lane_following:=false` the adapter sends
`/<ns>/navigate_to_pose` instead, and Nav2 plans its own path. The robot pose comes from
TF `map → base_link` on `/<ns>/tf`.

## Contents

| Path | What |
|---|---|
| `nevo_rmf/fleet_adapter.py` | the adapter |
| `config/fleet_config.yaml` | sim: nevo1 + nevo2 (orbit_nevo_bringup), house map |
| `config/fleet_config_real.yaml` | real NEVO: nevo1 only, real speed limits |
| `launch/nevo_rmf.launch.xml` | RMF core (schedule, blockade, building map, dispatcher, RViz) + adapter |
| `maps/house/` | sim building, graphs and robot map (turtlebot3_house) |

## Requirements

- ROS 2 Jazzy + Open-RMF: `rmf_fleet_adapter_python`, `rmf_traffic_ros2`, `rmf_task_ros2`,
  `rmf_building_map_tools`, `rmf_visualization`, `nav2_msgs`, `tf2_ros`.
- Zenoh in every terminal (ORBIT and NEVO both run on it):
  ```bash
  export RMW_IMPLEMENTATION=rmw_zenoh_cpp
  export ROS_DOMAIN_ID=<same as the robot>
  ros2 run rmw_zenoh_cpp rmw_zenohd      # one router per PC, start it first
  ros2 daemon stop                       # after switching RMW
  ```
  To reach NEVO, this PC's router must connect to the robot's router (`tcp/<robot ip>:7447`).
  Over Tailscale that needs an explicit connect endpoint, because multicast discovery
  doesn't cross it.

## Real NEVO

Robot side: `nevo_bot` branch `feat/hw-namespace`, launched with
```bash
ros2 launch nevo_bot launch_robot.launch.py stack_mode:=nav cloud:=false namespace:=nevo1 \
    nav_params:=$(ros2 pkg prefix nevo_bot)/share/nevo_bot/config/hw_nav_rmf.yaml
```
Check `ros2 action list | grep nevo1` shows `/nevo1/navigate_to_pose` and `/nevo1/follow_path`.

### Map and graph (traffic-editor)

1. Use the **same map the robot localizes on** (its `.yaml` + `.pgm`) and copy both into a
   folder, e.g. `~/rmf_maps/lab/`.
2. In traffic-editor, create a building in that folder. Add a level whose floor plan image is
   the robot map, with **scale = the map's resolution** (m/px, the `resolution:` value in the yaml)
   and **no rotation or offset**. The adapter converts RMF → robot coordinates from the map
   yaml alone, so this is what makes the two frames line up.
3. Draw lanes on graph 0 (or note the number you used). Add a waypoint named `nevo1_charger`
   and mark it as a charger. The robot must start on or near the graph, or RMF won't accept it.
4. Export / save, so the folder has `<name>.building.yaml` and `nav_graphs/<N>.yaml`. If the
   level isn't named `L1`, set `rmf_map_name` in `fleet_config_real.yaml` and `level:=` below.

### Run

```bash
ros2 launch nevo_rmf nevo_rmf.launch.xml use_sim_time:=false \
    config_file:=$(ros2 pkg prefix nevo_rmf)/share/nevo_rmf/config/fleet_config_real.yaml \
    map_dir:=$HOME/rmf_maps/lab building:=lab nav_graph:=0 robot_map:=lab_map
```
- `rmf_core:=false`: start only the adapter when another launch already runs the RMF
  schedule + dispatcher (only one RMF core per system, e.g. if TIFA's adapter starts it).
- `server_uri:=ws://localhost:8000/_internal`: report to rmf-web.
- `headless:=true`: no RViz.

The log should show `registered nevo1 with RMF at rmf=(…)`. If it keeps printing
`RMF did not accept nevo1`, the robot is too far from the graph, the charger waypoint is
missing, or the map/graph frames don't line up (check step 2).

Test task:
```bash
ros2 run rmf_demos_tasks dispatch_patrol -p <waypoint_a> <waypoint_b> -n 1   # add --use_sim_time in sim
```

## Sim (two NEVOs, house world)

Robot bringups live in each robot's own repo, not in ORBIT. The NEVO sim needs
`diagonalrobotics/nevo` (private; `nevo_bot` + `orbit_nevo_bringup`, sim branch) in the same
workspace.

```bash
ros2 launch orbit_nevo_bringup sim_fleet.launch.py
ros2 launch nevo_rmf nevo_rmf.launch.xml        # defaults: sim time, house map, graph 1
```

## Behaviour notes

- If Nav2 fails a goal (e.g. an obstacle on the lane: RPP stops instead of swerving), or the
  action server isn't reachable, the adapter asks RMF to replan after `replan_delay` (5 s).
- The battery is a fixed `battery_soc` (no battery reading yet). Custom actions (docking etc.)
  are not implemented: they finish immediately.
- Safety: the adapter only sends Nav2 goals. Robot motion still goes through the robot's
  twist_mux (Nav2 has the lowest priority) and its cmd_vel watchdog.
