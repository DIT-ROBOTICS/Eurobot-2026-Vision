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
### Launch realsense node
Make sure the RealSense camera is connected via USB to your computer, then run:
```bash
ros2 launch realsense2_camera rs_launch.py
```
### Run aruco detect node
```bash
ros2 run aruco_ros aruco_detector_node
```
### Run rivz
Open RViz to visualize camera images and TF frames:
```bash
ros2 run rviz2 rviz2
```
### Save picture
Save picture for yolo datasets
```bash
ros2 run yolo save_aruco_training_data.py
```
### Launch hazelnut detect
```bash
ros2 launch yolo hazelnut_detect_launch.py
```
### Set detect mode
```bash
ros2 param set /yolo_node mode "obb_mix"
```
mode:['bb', 'obb_dep', 'obb_pnp', 'obb_mix']