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

### Launch Robot detect node （For: 定位組）
```bash
ros2 launch aruco_robot robot_launch.py
```
Problem: There is a 1-3cm error in pose in the X and Y directions. As for orientation, it has not been tested yet. As for location function, sometimes the screen flickers, causing it to fail to detect the location.
Limit: Make sure that at least one of the four Arucos on the field is in the frame, and that the Aruco on the robot is also in the frame.

### Launch Sima detect node （For: SIMA組）
```bash
ros2 launch aruco_sima sima_launch.py
```