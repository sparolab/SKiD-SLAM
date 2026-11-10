# SKiD-SLAM v1.0 — IEEE T-FR 2026

First tagged release of **SKiD-SLAM**, the distributed multi-robot LiDAR SLAM system of our IEEE Transactions on Field Robotics paper:

> **SKiD-SLAM: Resource-Aware Distributed Multi-Robot LiDAR SLAM for Planetary Missions with Field Deployments at Analog Sites**
> H. Kim, J. Kim, D. Cho, J. Choi, G. Yang, S. Jeong, J. Lee, M. Park, H. Lim, Y. Cho
> IEEE T-FR, vol. 3, pp. 813–841, 2026 · [doi:10.1109/TFR.2026.3721725](https://doi.org/10.1109/TFR.2026.3721725) · [arXiv:2610.38141](https://arxiv.org/abs/2610.38141) · [Project page](https://sparolab.github.io/research/skid_slam/)

## What's included

- **ROS 1 (Noetic)** and **ROS 2 (Humble)** packages, both named `skid_slam`, on a shared ROS-independent fusion core (`core/`).
- Per-robot LiDAR-inertial odometry (LIO-SAM based), **SOLiD** place recognition with compact descriptors exchanged between robots, coarse-to-fine registration (**KISS-Matcher → small_gicp**), **PCM** outlier rejection and distributed map fusion into a common frame.
- Configurations for the paper's field datasets: `majang` (lava tube) and `planetary_analog`.
- Dockerfiles for both ROS versions (`docker/Dockerfile.noetic`, `docker/Dockerfile.humble`).
- `AGENTS.md`: a step-by-step guide that lets a coding agent (Claude Code, Codex) build, run and verify SKiD-SLAM on a fresh machine.
- Launch and configuration guides: `ros{1,2}/launch/GUIDE.md`, `ros{1,2}/config/GUIDE.md`.
- BSD 3-Clause license.

## Quick start (Docker, ROS 2)

```bash
git clone https://github.com/sparolab/SKiD-SLAM.git
cd SKiD-SLAM && git submodule update --init
docker build -f docker/Dockerfile.humble -t skid-slam:humble .

xhost +local:docker
docker run -it --rm --net=host -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix -v /path/to/bags:/data skid-slam:humble
# inside the container
ros2 launch skid_slam run.launch.py config:=majang
ros2 bag play /data/majang --clock      # second shell
```

ROS 1: use `docker/Dockerfile.noetic`, `roslaunch skid_slam run.launch config:=majang` and `rosbag play --clock majang.bag`. Datasets: see the README.

## How to cite

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

GitHub's **Cite this repository** button (from `CITATION.cff`) gives the same entry.
