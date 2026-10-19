<div align="center">
  <h1>SKiD-SLAM</h1>
  <a href="https://github.com/sparolab/SKiD-SLAM"><img src="https://img.shields.io/badge/-C++-blue?logo=cplusplus" /></a>
  <a href="https://github.com/sparolab/SKiD-SLAM"><img src="https://img.shields.io/badge/ROS-Noetic-blue" /></a>
  <a href="https://github.com/sparolab/SKiD-SLAM"><img src="https://img.shields.io/badge/ROS2-Humble-blue" /></a>
  <a href="#docker"><img src="https://img.shields.io/badge/Docker-2496ED?logo=docker&logoColor=white" /></a>
  <br />
  <a href="https://sparolab.github.io/research/skid_slam/"><img src="https://img.shields.io/badge/Project-Page-blue" /></a>
  <a href="https://arxiv.org/abs/2505.08230"><img src="https://img.shields.io/badge/arXiv-b33737?logo=arXiv" /></a>
  <a href="https://www.dropbox.com/scl/fo/ll5mjft2barlq24ab69a7/ADdh6meCHSzOHKcVhbpSOzE?rlkey=opjpicgu3sb1wfun81o2qbf8f&st=8aziykzl&dl=0"><img src="https://img.shields.io/badge/Dataset-Dropbox-0061FF?logo=dropbox&logoColor=white" /></a>
  <br />
  <br />
  <p align="center"><img src="assets/pipeline.png" alt="SKiD-SLAM" width="95%" /></p>
  <p><strong><em>Robust, lightweight, and distributed multi-robot LiDAR SLAM.</em></strong></p>
  <p><strong>[IEEE T-FR 2026]</strong> Resource-Aware Distributed Multi-Robot LiDAR SLAM for Planetary Missions with Field Deployments at Analog Sites</p>
</div>

______________________________________________________________________

**SKiD-SLAM** = **S**OLiD + **Ki**SS-Matcher + **D**istributed SLAM.
Each robot runs its own LiDAR-inertial odometry. The robots share compact [SOLiD](https://github.com/sparolab/SOLiD) descriptors and find inter-robot loops with coarse-to-fine registration ([KISS-Matcher](https://github.com/MIT-SPARK/KISS-Matcher) → [small_gicp](https://github.com/koide3/small_gicp)). Pairwise consistency maximization rejects outlier loops before the maps are fused.

## :floppy_disk: Datasets

Multi-robot bags used in the paper (`majang.bag`, `planetary_analog.bag`) are available on **[Dropbox](https://www.dropbox.com/scl/fo/ll5mjft2barlq24ab69a7/ADdh6meCHSzOHKcVhbpSOzE?rlkey=opjpicgu3sb1wfun81o2qbf8f&st=8aziykzl&dl=0)**.

______________________________________________________________________

## :package: Installation

```bash
git clone https://github.com/sparolab/SKiD-SLAM.git
cd SKiD-SLAM && git submodule update --init
```

### Docker

```bash
docker build -f docker/Dockerfile.noetic -t skid-slam:noetic .   # ROS 1
docker build -f docker/Dockerfile.humble -t skid-slam:humble .   # ROS 2

xhost +local:docker
docker run -it --rm --net=host -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix -v /path/to/bags:/data skid-slam:humble
```

### From source

<details open>
  <summary><strong>ROS 1 (Noetic)</strong></summary>

```bash
sudo add-apt-repository ppa:borglab/gtsam-release-4.0
sudo apt install libgtsam-dev libgtsam-unstable-dev libflann-dev liblz4-dev libtbb-dev

# clone into ~/catkin_ws/src
cd ~/catkin_ws && catkin_make -DCMAKE_BUILD_TYPE=Release
```

</details>

<details open>
  <summary><strong>ROS 2 (Humble)</strong></summary>

```bash
sudo apt install ros-humble-gtsam libflann-dev liblz4-dev libtbb-dev

# clone into ~/ros2_ws/src
cd ~/ros2_ws && colcon build --base-paths src/SKiD-SLAM/ros2 --cmake-args -DCMAKE_BUILD_TYPE=Release
```

</details>

______________________________________________________________________

## :rocket: Run

```bash
# ROS 1
roslaunch skid_slam run.launch config:=majang
rosbag play --clock majang.bag

# ROS 2
ros2 launch skid_slam run.launch.py config:=majang
ros2 bag play majang --clock
```

| Argument | Default | Description |
| --- | --- | --- |
| `config` | `majang` | `majang` or `planetary_analog` |
| `rviz` | `true` | Launch RViz |
| `save_poses` | `false` | Save trajectories (TUM) |
| `save_map` | `false` | Save the feature map |
| `save_whole_map` | `false` | Save the dense map |
| `save_scans` | `false` | Save keyframe scans |
| `save_dir` | `/tmp/skid_slam` | Output directory |

______________________________________________________________________

## :gear: Configuration

All parameters live in [`ros1/config/params/`](ros1/config/params) and [`ros2/config/params/`](ros2/config/params) and are read at runtime, so tuning never requires a rebuild.

______________________________________________________________________

## Citation

If you use SKiD-SLAM in your research, please cite our [paper](https://arxiv.org/abs/2505.08230).

<details>
  <summary><strong>See bibtex</strong></summary>

```bibtex
@article{kim2026skidslam,
  author    = {Kim, Hogyun and Kim, Juwon and Cho, Dongjin and Choi, Jiwon and Yang, Geonmo and Jeong, Seokhwan and Lee, Jiyun and Park, Miryeong and Lim, Hyungtae and Cho, Younggun},
  title     = {SKiD-SLAM: Resource-Aware Distributed Multi-Robot LiDAR SLAM for Planetary Missions with Field Deployments at Analog Sites},
  journal   = {IEEE Transactions on Field Robotics (T-FR)},
  year      = {2026},
}
```

</details>

## Contributing

We envision SKiD-SLAM as a community-driven project and love to see it grow through contributions from the community. We would love to see your face in the list below, just open a Pull Request!

<a href="https://github.com/sparolab/SKiD-SLAM/graphs/contributors">
  <img src="https://contrib.rocks/image?repo=sparolab/SKiD-SLAM" />
</a>

## Acknowledgements

SKiD-SLAM builds on [LIO-SAM](https://github.com/TixiaoShan/LIO-SAM), [DiSCo-SLAM](https://github.com/RobustFieldAutonomyLab/DiSCo-SLAM), [SOLiD](https://github.com/sparolab/SOLiD), [KISS-Matcher](https://github.com/MIT-SPARK/KISS-Matcher), and [small_gicp](https://github.com/koide3/small_gicp). We thank the authors for open-sourcing their work.

## Special Thanks

We sincerely thank **[Yewei Huang](https://yeweihuang.github.io/)** for open-sourcing [DiSCo-SLAM](https://github.com/RobustFieldAutonomyLab/DiSCo-SLAM), which serves as the backbone of SKiD-SLAM.
