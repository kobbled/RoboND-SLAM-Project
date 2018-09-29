# RoboND-SLAM-Project
This project uses RTAB-Map to create a 2D occupancy grid and 3D octomap of simulated environments within gazebo.

# Project Setup
Create a Catkin repository and download this repository into the src folder:
```sh
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src
$ git clone https://github.com/kobbled/RoboND-SLAM-Project.git
```

build the project:
```sh
$ cd ~/catkin_ws
$ catkin_make
```

Source the setup file or add to .bashrc
```sh
$ source ~/catkin_ws/devel/setup.bash
```

launch RTAB-Map

```sh
$ cd ~/catkin_ws/src/RoboND-SLAM-Project/slam_project/scripts
$ chmod +x rtab_run
$ ./rtab_run
```

Alternatively run each node seperately:

```sh
$ cd ~/catkin_ws
$ roslaunch slam_project world.launch
$ roslaunch slam_project teleop.launch
$ roslaunch slam_project mapping.launch
$ roslaunch slam_project rviz.launch
```

Changing environments can either be done by updating the world_name argument in the world.launch file, or specifying the .world file when prompted in the rtab_run script.
