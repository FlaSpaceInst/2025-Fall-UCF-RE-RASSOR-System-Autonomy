# Summary

[RE-RASSOR](https://fsi.ucf.edu/robotics/) (Research and Education – Regolith Advanced Surface Systems Operational Robot) is the initial FSI robotic offering. It combines the NASA Mini-RASSOR rover design (non-commercial license) with UCF Senior Design team developed EZ-RASSOR software to create a robust, 1R Class rover capable of Exploration Level tasks.

This specific repository is a continuation of 2024 AutoNav Senior Design Project

---

## Nav2 Costmap Setup

### Overview

RE-RASSOR uses two Nav2 costmaps:

| Costmap | Frame | Purpose |
|---|---|---|
| **Local costmap** | `odom` | Real-time obstacle avoidance from depth camera; 6×6 m rolling window around the robot |
| **Global costmap** | `map` | Long-range path planning; static layer from a pre-built map + live obstacle overlay |

Both costmaps consume `/camera/depth/points` (PointCloud2) from the Astra Pro depth camera.

### Requirements

Before launching, ensure the following are available:

1. **Astra Pro camera** plugged in (provides `/camera/depth/points`)
2. **Motor controller server** reachable at `server_ip:8000`
3. **A pre-built map** at `src/re_rassor_bringup/maps/my_map.yaml` (default) — replace with your own or use RTAB-Map to build one

### Launching

```bash
# Full Nav2 stack (map + costmaps + navigation)
ros2 launch re_rassor_bringup re_rassor_full.launch.py server_ip:=<IP> rviz:=true

# Sensor pipeline only (depth camera, no Nav2)
ros2 launch re_rassor_bringup sensors.launch.py

# Autonomy stack (no pre-built map required)
ros2 launch re_rassor_bringup autonomy.launch.py server_ip:=<IP> rviz:=true
```

### RViz2 — Viewing the Costmaps

After launch, add the following displays in RViz2:

1. **Local costmap**: `Add → By topic → /local_costmap/costmap → Map`
2. **Global costmap**: `Add → By topic → /global_costmap/costmap → Map`
3. **Point cloud** (raw sensor): `Add → By topic → /camera/depth/points → PointCloud2`

> **No map received / costmap not updating?** See the troubleshooting section below.

### Providing a Map

The `re_rassor_full.launch.py` launch file automatically loads `src/re_rassor_bringup/maps/my_map.yaml`.
To use a different map:

```bash
# Replace the default map files
cp /path/to/your_map.pgm src/re_rassor_bringup/maps/my_map.pgm
cp /path/to/your_map.yaml src/re_rassor_bringup/maps/my_map.yaml
# Edit my_map.yaml so `image:` points to my_map.pgm
```

Or build a new map live with RTAB-Map, then save:

```bash
# After mapping session, save from RTAB-Map database
ros2 run rtabmap_ros map_saver --ros-args -p output_file:=src/re_rassor_bringup/maps/my_map
```

### Key Parameters (`src/re_rassor_bringup/config/nav2_params.yaml`)

| Parameter | Value | Notes |
|---|---|---|
| Local costmap size | 6×6 m rolling window | Follows the robot in `odom` frame |
| Global costmap size | 20×20 m | Centered on map origin |
| Resolution | 0.05 m/cell | Both costmaps |
| Robot radius | 0.35 m | Used for inflation |
| Inflation radius | 0.55 m | Around obstacles |
| Point cloud topic | `/camera/depth/points` | Both local VoxelLayer and global ObstacleLayer |
| Min obstacle height | 0.10 m (local) / 0.05 m (global) | Filters ground returns |
| Max obstacle height | 1.5 m (local) / 2.0 m (global) | Filters above-robot objects |

### Troubleshooting

**"No map received" on local or global costmap in RViz2**

- Check that `map_server` is active: `ros2 lifecycle get /map_server` — should be `active`
- Check the `/map` topic is being published: `ros2 topic echo /map --once`
- If the map file path is wrong, edit `maps/my_map.yaml` and verify `image:` points to a valid `.pgm`
- Verify the lifecycle manager started all nodes: `ros2 topic echo /lifecycle_manager_navigation/transition_event`

**Local costmap not updating (no obstacles visible)**

- Check the depth camera is publishing: `ros2 topic hz /camera/depth/points`
- Verify TF chain `map → odom → base_link → camera_link` is complete: `ros2 run tf2_ros tf2_echo map camera_link`
- Check costmap node logs: `ros2 node info /local_costmap/local_costmap`

**Global costmap static layer empty**

- Confirm `map_server` loaded the map: `ros2 service call /map_server/load_map nav2_msgs/srv/LoadMap "{map_url: 'src/re_rassor_bringup/maps/my_map.yaml'}"`
- The map must exist before launching; Nav2 lifecycle manager starts `map_server` first

**Nav2 not activating**

- Check the lifecycle manager log for which node failed to transition
- Make sure `odom → base_link` TF is being published (requires `mission_control` to be running)
- Nav2 starts at t=6 s after launch; wait for all nodes before diagnosing

---

