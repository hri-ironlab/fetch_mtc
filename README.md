# PRogramAR Demo

![Screenshot from 2023-03-30 15-09-45](https://user-images.githubusercontent.com/56240638/228939524-eb4a8f77-3fb8-40f4-9ee8-d9a2a8dba431.png)

# Ubuntu Setup
### Setup workspace ROS Noetic
```sh
mkdir ros_workspace
cd ros_workspace/
```
### Clone repository into src directory
```sh
git clone --branch PRogramAR https://github.com/hri-ironlab/fetch_mtc.git src
```
### Clone fetch_ros and mtc into src directory
```sh
cd src/
git clone https://github.com/ros-planning/moveit_task_constructor.git
git clone --branch ros1 https://github.com/bryceikeda/fetch_ros.git
git clone https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git
```
# Docker Setup
Alternatively, you may use a docker container to develop in ROS, found here: https://github.com/bryceikeda/ros-docker.git

Build the image as described in the repository README, then run the container. Then, in the overlay_ws/src/ directory, run the following commands:
```sh
git clone --branch PRogramAR https://github.com/hri-ironlab/fetch_mtc.git
```
# Build workspace
```sh
cd ..
rosdep install --from-paths src --ignore-src -r -y
catkin build
source devel/setup.bash
```

# Run PRogramAR 
Get your IP address
```sh
hostname -I
```
Launch PRogramAR:
```sh
roslaunch task_planner PRogramAR_demo.launch tcp_ip:=xxx.xxx.x.x tcp_port:=10000
```
