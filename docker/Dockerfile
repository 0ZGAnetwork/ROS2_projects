FROM ros:jazzy

WORKDIR /ros2_ws

COPY ros2_ws/src ./src 
# container name: ros2_ws/src

RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    ros-jazzy-rviz2 \
    ros-jazzy-urdf-launch \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-xacro \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-geometry-msgs \
    ros-jazzy-sensor-msgs \
    ros-jazzy-nav-msgs \
    ros-jazzy-tf2 \
    ros-jazzy-tf2-ros \