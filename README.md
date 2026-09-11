# Differential-Drive Robot Simulation
## Goal
Develop a modular ROS2 robot simulation of a differential-drive mobile robot, running in Docker and visualzed in RViz.  
## Technologies
C++ \ ROS2 \ Docker/Compose \ GITHUB\Git \ CMAKE \ Colcon \ VS CODE

## Development progress
### Phase 1 — Project Setup
- [x] Define project architecture
- [x] Define deadline
- [x] Initialize Git repository
- [x] Initialize Docker environment
- [x] Initialize ROS 2 workspace
- [x] Create initial ROS 2 packages

### Phase 2 — Robot Description

- [x] Create robot model
- [x] Add URDF/Xacro
- [x] Add launch files
- [x] Visualize robot in RViz

### Phase 3 — Control
- [x] add odometry, and wheel rotation
- [x] Implement wheel_odom control via Terminal
- [x] Implement differential-drive control
- [ ] add teleoperation
- [ ] Test movement

### Phase 4 — Simulation
- [ ] Configure simulator
- [ ] Add sensors
- [ ] Connect simulation with ROS 2

### Phase 5 — Navigation & Perception
- [ ] Implement navigation
- [ ] Implement perception

## How to run
```bash
docker build -t differential_robot .
docker run -it diff_robot_con
docker compose up
