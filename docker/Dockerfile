FROM ros:jazzy

WORKDIR /ros2_ws

COPY ros2_ws/src ./src 
# container name: ros2_ws/src

RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    ros-jazzy-rviz2 \
    ros-jazzy-urdf-launch \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-xacro
