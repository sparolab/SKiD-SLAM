# Configuration guide — ROS 1 (Noetic)

Each file in [`params/`](params) is the full parameter set for one dataset (`majang`, `planetary_analog`). It is selected with `config:=<name>`, and every value is read at runtime, so tuning never needs a rebuild.

```yaml
skid_slam:      # single-robot LiDAR-inertial odometry (LIO-SAM based)
  ...
mapfusion:      # inter-robot loop closure and map fusion
  ...
```

## `skid_slam` — odometry

| Group | Parameters | Notes |
| --- | --- | --- |
| Topics | `pointCloudTopic`, `imuTopic`, `odomTopic` | Relative to the robot namespace (`/<robot>/...`) |
| Frames | `lidarFrame`, `baselinkFrame`, `odometryFrame`, `mapFrame` | TF frames are prefixed with the robot name |
| Sensor | `sensor`, `N_SCAN`, `Horizon_SCAN`, `downsampleRate`, `lidarMinRange`, `lidarMaxRange` | `sensor`: `velodyne` or `ouster` |
| IMU | `imuType`, `imuAccNoise`, `imuGyrNoise`, `imuAccBiasN`, `imuGyrBiasN`, `imuGravity`, `imuRPYWeight` | `imuType`: `0` 6-axis, `1` 9-axis |
| Extrinsics | `extrinsicTrans`, `extrinsicRot`, `extrinsicRPY` | LiDAR → IMU |
| Features / mapping | `edgeThreshold`, `surfThreshold`, `*LeafSize`, `surrounding*`, `numberOfCores`, `mappingProcessInterval` | Same meaning as in LIO-SAM |
| Loop closure (intra-robot) | `loopClosureEnableFlag`, `historyKeyframe*` | |
| Visualization | `globalMapVisualization*` | `...Raw: true` draws the map from raw scans; `...RawRange` limits each scan so the map grows along the path |
| Export | `saveMap`, `savePoses`, `saveScans`, `saveWholeMap`, `saveWholeMapLeafSize`, `saveDirectory` | Usually set from the launch file |

## `mapfusion` — inter-robot fusion

| Group | Parameters | Notes |
| --- | --- | --- |
| General | `robot_initial`, `topic`, `frame`, `local_topic`, `loop_frame_threshold` | `robot_initial` anchors the global map |
| `solid` | `num_range`, `num_angle`, `num_height`, `min_range`, `max_range`, `fov_up`, `fov_down`, `voxel_size` | SOLiD descriptor; match `fov_*` to the LiDAR |
| `retrieval` | `num_nearest`, `num_candidates`, `loop_threshold`, `use_position_search`, `position_search_radius_*` | `loop_threshold` = max SOLiD distance (1 − cosine similarity) |
| `registration` | `voxel_size`, `use_kiss_matcher`, `kiss_matcher.*`, `gicp.*`, `fitness_threshold`, `fitness_max_distance` | KISS-Matcher (global) → small_gicp (local) |
| `pcm` | `enable`, `start_threshold`, `threshold` | Pairwise consistency maximization |

## Tuning tips

- **No inter-robot loops found** → raise `retrieval.loop_threshold` (more candidates), then `registration.fitness_threshold` (accept looser alignments).
- **Wrong loops accepted** → lower `fitness_threshold`, raise `pcm.start_threshold` (5 for accuracy, 2 for harsh scenes), or lower `pcm.threshold`.
- **Sparse or open scenes** → larger `registration.voxel_size` and `solid.max_range`; small or cluttered scenes → smaller.
- **Registration too slow** → larger `registration.voxel_size`; `use_kiss_matcher: false` skips global registration (only if the initial guesses are already close).
