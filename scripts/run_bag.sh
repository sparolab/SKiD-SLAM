#!/bin/bash
# Runs SKiD-SLAM on a multi-robot bag, saves the results and (optionally) records an RViz video.
#
#   scripts/run_bag.sh -c majang -b /data/majang_multi.bag -o /results [-v] [-s 4]
#
#   -c  config name in config/params/ (required)
#   -b  rosbag with /<robot>/... topics (required)
#   -o  output directory (default: /tmp/skid_slam)
#   -v  record an mp4 of RViz (headless, needs Xvfb + ffmpeg)
#   -s  speed-up of the final video relative to bag time (default: 4)
#   -r  rosbag play rate (default: 1.0)
#   -m  closest camera distance in meters; the camera orbits the robots and zooms out as they spread (default: 25)
#   -f  length of the closing shot orbiting the fused map, in seconds (default: 8, 0 disables)
set -e

CONFIG=""; BAG=""; OUT=/tmp/skid_slam; VIDEO=false; SPEEDUP=4; RATE=1.0; MIN_DIST=25; FINALE=8
while getopts "c:b:o:vs:r:m:f:" opt; do
  case $opt in
    c) CONFIG=$OPTARG ;; b) BAG=$OPTARG ;; o) OUT=$OPTARG ;; v) VIDEO=true ;;
    s) SPEEDUP=$OPTARG ;; r) RATE=$OPTARG ;; m) MIN_DIST=$OPTARG ;; f) FINALE=$OPTARG ;;
    *) sed -n '2,14p' "$0"; exit 1 ;;
  esac
done
[[ -z $CONFIG || -z $BAG ]] && { sed -n '2,14p' "$0"; exit 1; }

PKG=$(rospack find skid_slam)
RES=$OUT/$CONFIG
mkdir -p "$RES"
PIDS=()
cleanup() { for p in "${PIDS[@]}"; do kill -INT "$p" 2>/dev/null || true; done; }
trap cleanup EXIT

if ! rostopic list >/dev/null 2>&1; then
  roscore >/dev/null 2>&1 & PIDS+=($!)
  until rostopic list >/dev/null 2>&1; do sleep 0.5; done
fi

roslaunch skid_slam run.launch config:=$CONFIG rviz:=false \
  save_map:=true save_poses:=true save_whole_map:=true save_dir:="$RES" > "$RES/launch.log" 2>&1 &
LAUNCH_PID=$!

if $VIDEO; then
  RVIZ_CFG=$RES/record.rviz
  # render-only window (no panels, 1920x1170 -> 1888x1080 3D view), camera driven by camera_orbit.py
  sed -e '/^Panels:/,/^Preferences:/{/^Preferences:/!d}' -e 's/^Preferences:/Panels: []\nPreferences:/' \
      -e 's/^  Height: .*/  Height: 1170/' \
      -e '/^    Current:/,/^    Saved:/{/^    Saved:/!d}' \
      -e 's#^    Saved:#    Current:\n      Class: rviz_animated_view_controller/Animated\n      Name: Current View\n      Target Frame: map\n      Placement Topic: /rviz/camera_placement\n    Saved:#' \
      "$PKG/config/rviz/skid_slam.rviz" > "$RVIZ_CFG"

  export DISPLAY=:99 DISABLE_ROS1_EOL_WARNINGS=1
  Xvfb :99 -screen 0 1920x1170x24 >/dev/null 2>&1 & PIDS+=($!)
  sleep 2
  rosrun skid_slam camera_orbit.py __name:=skid_slam_camera_orbit _min_distance:=$MIN_DIST >/dev/null 2>&1 & PIDS+=($!)
  rviz -d "$RVIZ_CFG" >/dev/null 2>&1 & PIDS+=($!)
  sleep 8
  ffmpeg -y -loglevel error -f x11grab -draw_mouse 0 -framerate 30 -video_size 1920x1170 -i :99 \
    -vf "crop=1888:1062:16:70" -c:v libx264 -preset ultrafast -crf 16 -pix_fmt yuv420p "$RES/raw_capture.mp4" &
  FFMPEG_PID=$!
fi

sleep 5
rosbag play --clock -q -r "$RATE" "$BAG" < /dev/null
if $VIDEO; then
  # the bag stopped /clock; keep sim time running so RViz keeps animating the closing shot
  python3 - <<'PY' >/dev/null 2>&1 &
import time, rospy
from rosgraph_msgs.msg import Clock
rospy.init_node("skid_slam_clock", disable_signals=True)
t0 = rospy.get_rostime(); w0 = time.time()
pub = rospy.Publisher("/clock", Clock, queue_size=1)
while True:
    pub.publish(Clock(t0 + rospy.Duration.from_sec(time.time() - w0)))
    time.sleep(0.01)
PY
  PIDS+=($!)
fi
sleep 10

if $VIDEO; then
  kill -INT $FFMPEG_PID; wait $FFMPEG_PID || true
  BAG_SEC=$(rosbag info -y -k duration "$BAG")
  # captured in wall time -> spread the captured frames evenly over (bag time / SPEEDUP)
  NFRAMES=$(ffprobe -v error -count_packets -select_streams v -show_entries stream=nb_read_packets -of csv=p=0 "$RES/raw_capture.mp4")
  RETIME="setpts=N*$(printf "%.6f" "$(echo "$BAG_SEC / $SPEEDUP / $NFRAMES" | bc -l)")/TB"
  GRADE="scale=1920:1080:flags=lanczos,vignette=PI/5,format=yuv420p"
  if (( $(echo "$FINALE > 0" | bc) )); then
    # closing shot: pull back and orbit the fused map in real time
    rosparam set /skid_slam_camera_orbit/finale true
    ffmpeg -y -loglevel error -f x11grab -draw_mouse 0 -framerate 30 -video_size 1920x1170 -t "$FINALE" -i :99 \
      -vf "crop=1888:1062:16:70" -c:v libx264 -preset ultrafast -crf 16 -pix_fmt yuv420p "$RES/finale_capture.mp4"
    ffmpeg -y -loglevel error -i "$RES/raw_capture.mp4" -i "$RES/finale_capture.mp4" -filter_complex \
      "[0:v]$RETIME,fps=30,$GRADE[a];[1:v]setpts=N/30/TB,fps=30,$GRADE[b];[a][b]concat=n=2:v=1[v]" \
      -map "[v]" -c:v libx264 -preset slow -crf 18 -movflags +faststart "$RES/${CONFIG}.mp4"
  else
    ffmpeg -y -loglevel error -i "$RES/raw_capture.mp4" -filter:v "$RETIME,fps=30,$GRADE" \
      -c:v libx264 -preset slow -crf 18 -movflags +faststart "$RES/${CONFIG}.mp4"
  fi
  rm -f "$RES/raw_capture.mp4" "$RES/finale_capture.mp4"
  echo "Video: $RES/${CONFIG}.mp4"
fi

kill -INT $LAUNCH_PID; wait $LAUNCH_PID || true
echo "Results: $RES"
