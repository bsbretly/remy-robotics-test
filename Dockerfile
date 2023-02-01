FROM ros:noetic-ros-core-focal

RUN apt-get update && apt-get install -y --no-install-recommends \
    gazebo11 \
    build-essential \
    python3-catkin-tools \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/* \
    && rosdep init \
    && rosdep update

COPY catkin_ws/src /home/catkin_ws/src
COPY catkin_ws/src/simple_scene/worlds /usr/share/gazebo-11/worlds
COPY catkin_ws/src/simple_scene/models /root/.gazebo/models

RUN . /opt/ros/noetic/setup.sh \
    && apt-get update && apt install -y --no-install-recommends \
    ros-noetic-moveit \
    && cd /home/catkin_ws \
    && rosdep install --rosdistro=noetic --from-path . -y --ignore-src \
    && catkin build

WORKDIR /home/catkin_ws/