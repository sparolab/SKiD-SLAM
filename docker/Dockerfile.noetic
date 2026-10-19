FROM ros:noetic-perception AS deps

ENV DEBIAN_FRONTEND=noninteractive
ENV DISABLE_ROS1_EOL_WARNINGS=1

RUN apt-get update && apt-get install -y --no-install-recommends \
      software-properties-common git wget \
      libflann-dev liblz4-dev libeigen3-dev libtbb-dev \
      ros-noetic-rviz \
    && add-apt-repository -y ppa:borglab/gtsam-release-4.0 \
    && apt-get update && apt-get install -y --no-install-recommends \
      libgtsam-dev libgtsam-unstable-dev \
    && rm -rf /var/lib/apt/lists/*

FROM deps
COPY . /catkin_ws/src/SKiD-SLAM

WORKDIR /catkin_ws
RUN . /opt/ros/noetic/setup.sh && \
    catkin_make -DCMAKE_BUILD_TYPE=Release -j$(nproc)

RUN echo "source /catkin_ws/devel/setup.bash" >> /root/.bashrc
CMD ["bash"]
