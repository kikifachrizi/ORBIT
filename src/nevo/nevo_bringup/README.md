# nevo_bringup

NEVO robot definition for ORBIT (ADR-004): URDF, ros2_control (real and Gazebo), sensors,
cmd_vel chain, EKF and Nav2. Owner: Arya. Does not depend on RMF or any other ORBIT package.
It can be run and driven without ORBIT: teleop, or a Nav2 goal from RViz.

Source: the NEVO repo (`diagonalrobotics/nevo`, branch `thesis-hw` 7239e5d + the namespace work
of `feat/hw-namespace`), package `nevo_bot`, trimmed to what the robot needs for navigation.
Not included: docking (rear camera, AprilTag, opennav_docking), the cloud dashboard
(`nevo_ws_bridge`, `nevo_robot_api`), `nevo_diagnostic_node`, the gsheet loggers, mapping mode,
thesis scripts and worlds. Site worlds and maps live in `orbit_sim`.

## Entry point

```bash
# Real robot (on the robot's Pi, with nevo.service stopped: sudo systemctl stop nevo.service)
ros2 launch nevo_bringup robot.launch.py namespace:=nevo1 map:=/path/map.yaml

# Simulation (world first, from orbit_sim)
ros2 launch orbit_sim world.launch.py site:=test_room
ros2 launch nevo_bringup robot.launch.py sim:=true namespace:=nevo1 x:=-2.5 y:=0.6 \
    map:=$(ros2 pkg prefix orbit_sim)/share/orbit_sim/sites/test_room/maps/test_room.yaml
```

| Argument | Default | Meaning |
|---|---|---|
| `namespace` | `''` | everything (nodes, topics, tf) under `/<namespace>` |
| `sim` | `false` | `true` = Gazebo (gz_ros2_control), `false` = real hardware |
| `use_sim_time` | = `sim` | |
| `map` | `maps/abie_lab_save.yaml` | Nav2 map |
| `nav` | `true` | AMCL + Nav2 |
| `params_file` | `''` | Nav2 params; empty = `config/nav2_sim.yaml` or `config/nav2_real.yaml` |
| `camera` | real `true`, sim `false` | Kinect (real) / depth camera (sim) for the STVL layer |
| `x`, `y`, `yaw` | `0` | sim spawn pose and AMCL initial pose (site maps: map frame = world frame) |

Interfaces ORBIT uses (all relative, under `/<namespace>`): action `navigate_to_pose` and
`follow_path` (controllers `FollowPath` = MPPI/DWB and `LaneFollow` = Regulated Pure Pursuit for
RMF lanes), pose `map → base_link` on `<ns>/tf`, teleop on `<ns>/cmd_vel`.
Battery status is not published yet (the INA226 bridge is disabled on the robot).

## Real vs sim

| | Real | Sim |
|---|---|---|
| Wheels | `ros2_control_node` + `nevo_diffdrive_arduino` (`/dev/ttyMOTOR`), `robot_controllers.yaml` (calibrated 0.401 / 0.0660) | controller_manager inside Gazebo, `robot_controllers_sim.yaml` (URDF geometry 0.425 / 0.0635) |
| Lidar | RPLIDAR A1 (`rplidar_ros`, `/dev/ttyLIDAR`) | gpu_lidar → `ros_gz_bridge` |
| IMU | Arduino Nano + MPU6050 (`imu_publisher.py`) | none (the EKF runs on wheels + rf2o) |
| Depth | Kinect (`kinect_ros2` → `points`) | depth camera with `camera:=true` |
| Nav2 | `nav2_real.yaml` (MPPI, STVL), NEVO behaviour tree with AMCL re-localization | `nav2_sim.yaml` (DWB), Nav2 stock behaviour tree |

Same for both: laser filter (box around the body), rf2o lidar odometry, EKF
(`odom → base_link`), twist_mux (`cmd_vel_smoothed` from Nav2 < `cmd_vel` teleop) →
`safety_fail_node` (500 ms freshness watchdog) → `diff_cont`.

## Packages in `src/nevo`

| Package | Why |
|---|---|
| `nevo_bringup` | robot definition |
| `nevo_diffdrive_arduino` | NEVO's Arduino hardware interface (joshnewans/diffdrive_arduino, Jazzy port + motor deadband compensation). Renamed: TIFA's `diffdrive_arduino` is a different driver with the same name |
| `serial` | needed by `nevo_diffdrive_arduino`, not in apt |
| `kinect_ros2` | Kinect driver with `full_depth_every_n`, not in apt; needs `libfreenect-dev` |

Shared with TIFA (in `src/tifa`): `rplidar_ros` (same 2.1.4; NEVO passes `serial_port` and
`frame_id`) and `rf2o_laser_odometry` (NEVO's twist covariance patch added there, off by default
so TIFA is unchanged; NEVO sets `twist_cov_vx` / `twist_cov_vyaw`).

## Notes

- Frames are not prefixed (`base_link`, `odom`, `map`): robots are separated by `/<ns>/tf`.
  Jazzy's `diff_drive_controller` prefixes its frames with the namespace by default, so both
  controller yamls set `tf_frame_prefix_enable: false`.
- Costmap layers resolve a relative `topic` inside the costmap namespace, so `robot.launch.py`
  rewrites the Nav2 `topic` / `scan_topic` / `odom_topic` values to `/<ns>/...` at launch.
- The EKF starts 5 s after the rest of the stack. Started together, its output went NaN at sim
  startup in 4 of 9 runs (random robot, clean wheel/rf2o input; root cause not pinned down);
  with the delay 0 of 6. Once NaN, the robot's `odom → base_link` is lost until a restart.
- AMCL and map_server are activated by the lifecycle script in `localization_launch.py`
  (retries, because lifecycle replies can get lost under load with rmw_zenoh), which then
  publishes the x/y/yaw initial pose once.
