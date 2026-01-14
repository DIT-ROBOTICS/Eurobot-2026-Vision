# Eurobot-2026-Vision
This repository contains the vision system for the Eurobot 2026 competition.
## Start with Docker
### Build image and container
```bash
cd docker/
docker compose up -d
```
### Build workspace
Attach to the running container:
```bash
docker exec -it vision-ws bash
```
Inside the container, build the workspace:
```bash
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```
## Realsense
### Launch realsense node
Make sure the RealSense camera is connected via USB to your computer, then run:
```bash
ros2 launch realsense2_camera rs_launch.py
```
## Aruco
### Run aruco detection for robot
```bash
ros2 run aruco_robot robot_detector
```
### Run aruco detection for sima
```bash
ros2 run aruco_sima sima_detector
```
## GUI
### Run rivz
Open RViz to visualize camera images and TF frames:
```bash
ros2 run rviz2 rviz2
```
