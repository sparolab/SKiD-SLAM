# Launch guide — ROS 2 (Humble)

## Quick start

```bash
ros2 launch skid_slam run.launch.py config:=majang
ros2 bag play majang --clock
```

Save everything to `~/results` without RViz:

```bash
ros2 launch skid_slam run.launch.py config:=majang rviz:=false \
  save_poses:=true save_map:=true save_whole_map:=true save_dir:=$HOME/results
```

## Arguments

| Argument | Default | Description |
| --- | --- | --- |
| `config` | `majang` | Dataset config in [`config/params/`](../config/params): `majang`, `planetary_analog` |
| `rviz` | `true` | Open RViz |
| `use_sim_time` | `true` | Use the bag clock |
| `fusion` | `true` | Start the inter-robot map fusion nodes |
| `robot0` `robot1` `robot2` | `jackal0` `jackal1` `jackal2` | Robot namespaces; `robot0` anchors the global map |
| `save_poses` | `false` | Keyframe poses in the global frame |
| `save_map` | `false` | Downsampled feature map in the global frame |
| `save_whole_map` | `false` | Raw scans stacked at the global poses |
| `save_scans` | `false` | Raw keyframe scans in the body frame |
| `save_dir` | `/tmp/skid_slam` | Output directory |

## Outputs

Written on shutdown (Ctrl-C) for each robot:

```
<save_dir>/<robot>/
├── <robot>_global_path.txt   # save_poses: TUM (time x y z qx qy qz qw), global frame
├── trajectory.pcd            # save_poses: keyframe positions, local odom frame
├── transformations.pcd       # save_poses: keyframe 6-DoF poses, local odom frame
├── <robot>_global_map.pcd    # save_map
├── <robot>_whole_map.pcd     # save_whole_map
└── scans/%06d.pcd            # save_scans, index matches the path file
```

## Your own robots

- Topics are namespaced per robot: `/<robot>/<pointCloudTopic>` and `/<robot>/<imuTopic>`.
- Robot names must end with a single digit (`jackal0`, `jackal1`, ...). The digit is the robot index, and robot `0` anchors the global map.
- Copy a file in [`ros2/config/params/`](../config/params), then set the topics, `sensor`, `N_SCAN`/`Horizon_SCAN`, and the IMU extrinsics. Map-fusion parameters live under `mapfusion:`.
