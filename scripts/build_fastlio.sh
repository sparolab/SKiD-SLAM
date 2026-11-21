#!/usr/bin/env bash
# Build SKiD-SLAM (ROS 2) with the FAST-LIO front end (frontend:=fastlio): SOLiD's FAST-LIO SLAM from thirdparty/SOLiD.
# Run from the colcon workspace root, with SKiD-SLAM cloned (--recursive) into src/:
#   src/SKiD-SLAM/scripts/build_fastlio.sh
# Installs Livox-SDK2 (FAST-LIO depends on livox_ros_driver2) unless present, puts livox_ros_driver2 into the
# workspace with its ROS 2 manifest, switches FAST-LIO to C++17 on ROS 2 distributions newer than Humble, and builds.
set -euo pipefail
: "${ROS_DISTRO:?source /opt/ros/<distro>/setup.bash first}"
SKID="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SOLID="$SKID/thirdparty/SOLiD"
[ -n "$(ls -A "$SOLID/thirdparty/FAST_LIO_ROS2" 2>/dev/null)" ] ||
  git -C "$SOLID" submodule update --init --recursive thirdparty/FAST_LIO_ROS2 thirdparty/livox_ros_driver2

if [ ! -e /usr/local/lib/liblivox_lidar_sdk_shared.so ]; then
  tmp=$(mktemp -d)
  git clone -q --depth 1 https://github.com/Livox-SDK/Livox-SDK2.git "$tmp/Livox-SDK2"
  cmake -S "$tmp/Livox-SDK2" -B "$tmp/build" -DCMAKE_BUILD_TYPE=Release >/dev/null
  cmake --build "$tmp/build" -j"$(nproc)" >/dev/null
  $([ "$(id -u)" = 0 ] || echo sudo) cmake --install "$tmp/build" >/dev/null
  rm -rf "$tmp"
fi

mkdir -p src
rm -rf src/livox_ros_driver2 && cp -r "$SOLID/thirdparty/livox_ros_driver2" src/livox_ros_driver2
cp src/livox_ros_driver2/package_ROS2.xml src/livox_ros_driver2/package.xml

# FAST-LIO's ROS 2 branch builds as C++14; rclcpp after Humble needs C++17.
if [ "$ROS_DISTRO" != humble ]; then
  sed -i 's/-std=c++14/-std=c++17/g; s/-std=c++0x //; s/set(CMAKE_CXX_STANDARD 14)/set(CMAKE_CXX_STANDARD 17)/' \
    "$SOLID/thirdparty/FAST_LIO_ROS2/CMakeLists.txt"
fi

colcon build --base-paths src/livox_ros_driver2 "$SOLID/thirdparty/FAST_LIO_ROS2" "$SOLID/slam/common/ros2" "$SKID/ros2" \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DROS_EDITION=ROS2 -DDISTRO_ROS="$ROS_DISTRO" \
  -DCMAKE_FIND_USE_PACKAGE_REGISTRY=OFF "$@"   # no stale exports from other builds (KISS-Matcher registers robin)
