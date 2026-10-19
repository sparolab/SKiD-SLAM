# SKiD-SLAM — setup guide for coding agents

You are setting up and running SKiD-SLAM (distributed multi-robot LiDAR SLAM) for a user. Take them from a fresh machine to a live RViz view of the robots building and merging their maps, and then a picture of the fused result. Follow the steps in order. Run every command yourself, check its output before moving on, and stop to ask the user only where a step says **Ask**.

## 0. Ask the user first

**Ask**, in a single message:

1. ROS version: **ROS 1 (Noetic)** or **ROS 2 (Humble)**.
2. Build method: **Docker** (recommended; works on any Linux host with Docker) or **from source** (needs a native Noetic install on Ubuntu 20.04, or Humble on Ubuntu 22.04).
3. Dataset: whether they already have `majang.bag` / `planetary_analog.bag` (and where), or whether you should download it (step 4). The bags are large (`majang.bag` ≈ 23 GB, `planetary_analog.bag` ≈ 67 GB), so start with `majang`.
4. Where to save results (default `~/skid_slam_results`).

## 1. Check the machine

- `docker --version` (Docker path), or `ls /opt/ros` (source path: must show `noetic` or `humble`).
- Free disk space with `df -h`. Plan for the bag size, plus ~25 GB more on ROS 2 (the converted bag), plus ~10 GB for images and builds.
- If Docker needs `sudo`, tell the user and use `sudo docker` consistently.

## 2. Get the code

```bash
git clone https://github.com/sparolab/SKiD-SLAM.git
cd SKiD-SLAM
git submodule update --init
```

Check that `thirdparty/SOLiD`, `thirdparty/KISS-Matcher` and `thirdparty/small_gicp` are not empty. Use a plain `--init`, not `--recursive`: SOLiD has optional nested submodules that are not needed.

Layout: `core/` holds the ROS-independent fusion library, and `ros1/` and `ros2/` hold the two packages (both named `skid_slam`). `thirdparty/` holds the git submodules.

## 3. Build

### Docker

```bash
# ROS 1
docker build -f docker/Dockerfile.noetic -t skid-slam:noetic .
# ROS 2
docker build -f docker/Dockerfile.humble -t skid-slam:humble .
```

The first build downloads ROBIN through CMake, so it needs internet access and takes a while.

### From source — ROS 1 (Noetic)

```bash
sudo add-apt-repository -y ppa:borglab/gtsam-release-4.0
sudo apt install -y libgtsam-dev libgtsam-unstable-dev libflann-dev liblz4-dev libtbb-dev
mkdir -p ~/catkin_ws/src && ln -s "$PWD" ~/catkin_ws/src/SKiD-SLAM
cd ~/catkin_ws && source /opt/ros/noetic/setup.bash && catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```

### From source — ROS 2 (Humble)

```bash
sudo apt install -y ros-humble-gtsam libflann-dev liblz4-dev libtbb-dev
mkdir -p ~/ros2_ws/src && ln -s "$PWD" ~/ros2_ws/src/SKiD-SLAM
cd ~/ros2_ws && source /opt/ros/humble/setup.bash
colcon build --base-paths src/SKiD-SLAM/ros2 --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

`--base-paths .../ros2` is required. Without it, colcon also finds the ROS 1 package.

A successful build produces 5 executables: `skid_slam_imageProjection`, `skid_slam_featureExtraction`, `skid_slam_imuPreintegration`, `skid_slam_mapOptmization`, `skid_slam_mapFusion`.

## 4. Prepare the data

Download the bag unless the user already has it. All bags are in the [Dropbox folder](https://www.dropbox.com/scl/fo/ll5mjft2barlq24ab69a7/ADdh6meCHSzOHKcVhbpSOzE?rlkey=opjpicgu3sb1wfun81o2qbf8f&st=8aziykzl&dl=0):

```bash
mkdir -p ~/skid_slam_data && cd ~/skid_slam_data
curl -L -o majang.bag "MAJANG_BAG_URL"                     # ≈ 23 GB
# curl -L -o planetary_analog.bag "PLANETARY_ANALOG_BAG_URL" # ≈ 67 GB
```

Check the file size once the download finishes. If a direct link fails, ask the user to download the file from the Dropbox folder in a browser.

- ROS 1: use the `.bag` file as is.
- ROS 2: the bags are ROS 1 bags. Convert them once (`pip install rosbags`):

  ```bash
  rosbags-convert --src majang.bag --dst majang --dst-typestore ros2_humble
  ```

## 5. Run

Pick the config that matches the bag: `majang` or `planetary_analog`. Start the launch file first, with RViz on, so the user can watch the robots map and merge. Then play the bag from a second shell. Use `rviz:=false` only when there is no display.

```bash
# ROS 1
roslaunch skid_slam run.launch config:=majang \
  save_poses:=true save_map:=true save_whole_map:=true save_dir:=$HOME/skid_slam_results
rosbag play --clock majang.bag

# ROS 2
ros2 launch skid_slam run.launch.py config:=majang \
  save_poses:=true save_map:=true save_whole_map:=true save_dir:=$HOME/skid_slam_results
ros2 bag play majang --clock
```

With Docker, allow the container to open RViz on the host display, then run the launch in a named container and the bag player through `docker exec`:

```bash
xhost +local:docker
docker run -d --name skid --net=host -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v $HOME/skid_slam_data:/data -v $HOME/skid_slam_results:/results skid-slam:humble \
  bash -c "source /ros2_ws/install/setup.bash && exec ros2 launch skid_slam run.launch.py config:=majang \
           save_poses:=true save_map:=true save_whole_map:=true save_dir:=/results"
docker exec -it skid bash -c "source /ros2_ws/install/setup.bash && ros2 bag play /data/majang --clock"
docker stop -t 120 skid   # SIGTERM to the launch; results are saved before it exits
```

For ROS 1, use `skid-slam:noetic`, `source /catkin_ws/devel/setup.bash`, `exec roslaunch skid_slam run.launch ...`, and `rosbag play --clock /data/majang.bag`. (`roslaunch` starts the `roscore` itself.)

In RViz, each robot has its own color. The robots first map in their own frames. As inter-robot loops are verified, their maps snap into one global map.

When the bag has finished, stop the launch with **Ctrl-C** (SIGINT) and wait. Results are written on shutdown, and this can take a minute.

## 6. Verify and report

- Logs contain `loop jackalX <-> jackalY verified` and `PCM: N / M loops ... are consistent`. These mean inter-robot loops were found and accepted.
- `<save_dir>/<robot>/` exists for every robot and contains `<robot>_global_path.txt` (TUM format), `<robot>_global_map.pcd` and `<robot>_whole_map.pcd`.
- Show the fused result right away. Render the merged maps and trajectories from above, save the picture as `fused_map.png`, and open it for the user:

  ```python
  import glob, numpy as np, matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
  def read_pcd(path):  # binary PCD with float32 x y z ...
      with open(path, "rb") as f:
          header = {}
          while True:
              key, *val = f.readline().decode().split()
              header[key] = val
              if key == "DATA": break
          n, fields = int(header["POINTS"][0]), header["FIELDS"]
          data = np.frombuffer(f.read(), dtype=np.float32, count=n * len(fields)).reshape(n, len(fields))
      return data[:, :2]
  results = "RESULTS_DIR"
  colors = ["#5C6BC0", "#EC407A", "#26A69A"]
  robots = sorted(d.rstrip("/").split("/")[-1] for d in glob.glob(f"{results}/*/"))
  pts = [read_pcd(f"{results}/{r}/{r}_whole_map.pcd")[::5] for r in robots]
  xy = np.concatenate(pts); col = np.concatenate([[colors[i]] * len(p) for i, p in enumerate(pts)])
  order = np.random.default_rng(0).permutation(len(xy))  # mix robots so none hides the others
  fig, ax = plt.subplots(figsize=(10, 10))
  ax.scatter(xy[order, 0], xy[order, 1], s=0.15, c=col[order], alpha=0.4, linewidths=0)
  for i, r in enumerate(robots):
      path = np.loadtxt(f"{results}/{r}/{r}_global_path.txt")
      ax.plot(path[:, 1], path[:, 2], c=colors[i], lw=2, label=r)
  lo, hi = np.percentile(xy, [1, 99], axis=0); pad = 0.05 * (hi - lo).max()
  ax.set_xlim(lo[0] - pad, hi[0] + pad); ax.set_ylim(lo[1] - pad, hi[1] + pad)
  ax.set_aspect("equal"); ax.legend(loc="upper right"); ax.axis("off")
  fig.savefig(f"{results}/fused_map.png", dpi=150, bbox_inches="tight")
  ```

- Report to the user: the number of verified loops per robot pair, the number of keyframes per robot (lines in each path file), the output folder, and `fused_map.png`. The maps of different robots should overlap where they shared the environment.

## 7. Troubleshooting

| Symptom | Fix |
| --- | --- |
| CMake: `thirdparty/... is empty` | `git submodule update --init` |
| colcon: duplicate package `skid_slam` or a catkin error | Build with `--base-paths <repo>/ros2` |
| Nodes start but nothing happens | The bag must be played with `--clock`, and the topics are `/<robot>/<pointCloudTopic>` and `/<robot>/<imuTopic>` from the config |
| Robots never align | Check the logs for `verified`. If there are none, see the tuning tips in `ros1/config/GUIDE.md` (`loop_threshold`, `fitness_threshold`) |
| No results written | Stop the launch with Ctrl-C (SIGINT), not SIGKILL, and wait for the save to finish |

More detail: `ros1/launch/GUIDE.md`, `ros2/launch/GUIDE.md` (arguments and outputs), and `ros1/config/GUIDE.md`, `ros2/config/GUIDE.md` (parameters).
