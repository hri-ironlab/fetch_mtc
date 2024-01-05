# MoveItTaskConstructor for the Fetch robot

![Screenshot from 2023-03-30 15-09-45](https://user-images.githubusercontent.com/56240638/228939524-eb4a8f77-3fb8-40f4-9ee8-d9a2a8dba431.png)

# Setup
## Setup workspace ROS Noetic
```sh
mkdir ros_workspace
cd ros_workspace/
```
## Clone repository into src directory
```sh
git clone https://github.com/hri-ironlab/fetch_mtc.git src
```
## Clone fetch_ros and mtc into src directory
```sh
cd src/
git clone https://github.com/ros-planning/moveit_task_constructor.git
git clone --branch ros1 git@github.com:bryceikeda/fetch_ros.git 
```

## Build workspace
```sh
cd ..
rosdep install --from-paths src --ignore-src -r -y
catkin build
source devel/setup.bash
```
# Run demo
In three different terminals run the following commands in order
```sh
roslaunch task_planner fetch_mtc_demo.launch
roslaunch scene_handler sim_scene.launch
roslaunch task_planner pick_and_place_demo.launch
```
# Other Examples
In two different terminals run the following commands in order
```sh
roslaunch task_planner fetch_mtc_demo.launch
roslaunch task_planner task_planner_examples.launch
```
