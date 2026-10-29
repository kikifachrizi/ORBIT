# orbit_adapter

Fleet adapter RMF ORBIT (EasyFullControl), satu proses per fleet dengan backend navigasi per robot (ADR-002). Hanya kode: config fleet ada di `orbit_bringup/config/sites/<site>/` (ADR-004).

| Modul | Isi |
| --- | --- |
| `fleet_adapter.py` | Entry point `fleet_adapter -c <fleet.yaml> -n <nav_graph.yaml> [--sim]` |
| `robot_adapter.py` | Callback RMF (`navigate`, `stop`, aksi) ke backend, retry goal gagal, update status |
| `backends/base.py` | Interface `RobotBackend` |
| `backends/nav2.py` | Backend `nav2`: action `navigate_to_pose` dan tf `map → base_link` di `/<namespace>` |
| `dispatch_task.py` | Kirim task (`patrol`, `go_to`) ke `/task_api_requests` dan tunggu balasan RMF |

Key ORBIT per robot di `rmf_fleet.robots.<nama>`: `backend` (`nav2`), `namespace`, `map_frame`, `base_frame`, `battery_topic`, `pose_timeout`. Transformasi nav graph ke peta robot memakai `reference_coordinates` bawaan EasyFullControl.

Contract test tanpa ROS dan Gazebo:

```bash
cd src/orbit_adapter && python3 -m pytest test
```
