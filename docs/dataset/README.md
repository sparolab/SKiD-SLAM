---
# Hugging Face dataset card (YAML front matter is read by the Hub).
# TODO before upload: confirm the licence (see "Before publishing" below).
license: cc-by-4.0
pretty_name: SKiD-SLAM Field Datasets (Majang lava tube, planetary analog site)
tags:
  - robotics
  - slam
  - lidar
  - multi-robot
  - point-cloud
  - planetary-analog
  - lava-tube
  - rosbag
task_categories:
  - robotics
size_categories:
  - n<1K
arxiv: 2610.38141
---

<!--
BEFORE PUBLISHING (maintainers only; delete this block from the copy you upload)
1. Licence. CC BY 4.0 is a placeholder. No licence, agreement or permit text was found next to the data.
   Confirm with the PI which licence applies (CC BY 4.0, or CC BY-NC 4.0 if a partner requires non-commercial use).
2. Partners / permits. Confirm that KICT (Yeoncheon planetary analog test site) agrees to public release of data
   recorded on its site, and that the access permission for the Majang lava tube (Jeju, part of a protected
   lava-tube system) allows publishing LiDAR scans of the cave. Nothing in the data folders documents either.
3. Ground truth. No survey/TLS reference or GT trajectory was found for either sequence. Either add the reference
   used in the paper (if one exists) or keep the "no ground truth" statement below.
4. Provenance. The published bags were assembled from the raw recordings (Majang 2024-06, Yeoncheon 2024-12-23)
   and re-stamped (bag start times 2025-03). Fill in the true recording dates and how the robots were assembled.
5. Personal data. The bags contain only LiDAR and IMU (no images). Do NOT add the raw cave bags with RGB/thermal
   cameras, or the trip photos/videos, without checking for people.
-->

# SKiD-SLAM Field Datasets

Multi-robot LiDAR–inertial recordings from two field deployments of
**[SKiD-SLAM](https://github.com/sparolab/SKiD-SLAM)** (IEEE Transactions on Field Robotics, 2026):
a **lava tube** (Majang, Jeju, Korea) and a **planetary analog site** (KICT, Yeoncheon, Korea).
They are the sequences used in the paper's field experiments, packaged so that SKiD-SLAM runs on them directly.

- Paper: [doi:10.1109/TFR.2026.3721725](https://doi.org/10.1109/TFR.2026.3721725) · [arXiv:2610.38141](https://arxiv.org/abs/2610.38141)
- Code: <https://github.com/sparolab/SKiD-SLAM> · Project page: <https://sparolab.github.io/research/skid_slam/>

## Sequences

| Sequence | Site | Robots | Duration | LiDAR (per robot) | IMU | Files |
|---|---|---|---|---|---|---|
| `majang` | Majang lava tube, Jeju (underground, GNSS-denied) | 3 (`jackal0`–`jackal2`) | 344.9 s | Ouster, 64 beams × 1800 | Ouster built-in (`/jackalN/ouster/imu`) | `majang.bag` (ROS 1, 23.1 GB), `majang/` (ROS 2 db3, 23.2 GB) |
| `planetary_analog` | KICT planetary analog test site, Yeoncheon | 2 (`jackal0`, `jackal1`) | 572.3 s | Ouster, 64 beams × 1024 | external IMU ≈ 430–500 Hz (`/jackalN/imu/data`) | `planetary_analog.bag` (ROS 1, 67.0 GB) |

Topics per robot: `/jackalN/ouster/points` (`sensor_msgs/PointCloud2`) and the IMU topic above. Robots start in
their own frames; there is no shared frame, clock offset correction or GNSS. No camera topics are included.

<!-- TODO: robot platforms per sequence (the planetary_analog config names a legged robot), exact Ouster models, and recording dates -->

## Layout

```
majang/
  ros1/majang.bag
  ros2/majang/{majang_ros2.db3, metadata.yaml}
  calib/                 # lidar-IMU extrinsics as used in ros{1,2}/config of SKiD-SLAM (identity for Majang)
planetary_analog/
  ros1/planetary_analog.bag
  calib/                 # lidar-IMU extrinsic (90 deg yaw) as used in the SKiD-SLAM config
results/                 # optional: SKiD-SLAM outputs (per-robot global paths, fused maps) for reference
LICENSE
README.md
```

## Ground truth

No ground-truth trajectory or survey map is included (there is no GNSS in the lava tube).
<!-- TODO: state how the paper evaluated these sequences, and add the reference data if one exists. -->
The `results/` folder holds SKiD-SLAM's own output for comparison, not ground truth.

## Running SKiD-SLAM on it

```bash
# ROS 2
ros2 launch skid_slam run.launch.py config:=majang
ros2 bag play majang/ros2/majang --clock
# ROS 1
roslaunch skid_slam run.launch config:=planetary_analog
rosbag play --clock planetary_analog/ros1/planetary_analog.bag
```

To convert a ROS 1 bag to ROS 2: `rosbags-convert --src majang.bag --dst majang --dst-typestore ros2_humble --dst-version 5`.

## Licence

Data: Creative Commons Attribution 4.0 (CC BY 4.0) <!-- placeholder, see BEFORE PUBLISHING -->. Code: BSD 3-Clause (see the GitHub repository).

## Citation

```bibtex
@article{kim2026skidslam,
  author  = {Kim, Hogyun and Kim, Juwon and Cho, Dongjin and Choi, Jiwon and Yang, Geonmo and Jeong, Seokhwan and Lee, Jiyun and Park, Miryeong and Lim, Hyungtae and Cho, Younggun},
  title   = {{SKiD-SLAM}: Resource-Aware Distributed Multi-Robot {LiDAR} {SLAM} for Planetary Missions with Field Deployments at Analog Sites},
  journal = {IEEE Transactions on Field Robotics},
  volume  = {3},
  pages   = {813--841},
  year    = {2026},
  doi     = {10.1109/TFR.2026.3721725},
  eprint  = {2610.38141},
  archivePrefix = {arXiv},
}
```

## Acknowledgements

<!-- TODO: KICT and the lava-tube access authority, funding (e.g. the STEAM project), as the PI wants them credited -->
