# mobile-manipulator-ros

<p align="center">
  <picture>
    <img alt="image" src="![ros](https://github.com/user-attachments/assets/04389c93-ca73-4bd1-a877-f3ca6cde3a40)
"" width="600" height="350" style="max-width: 100%;">
  </picture>
</p>


# ROS Environment Setup

If you do not already have a ROS workspace, you need to create a catkin_ws before cloning this repository:

    mkdir -p ~/catkin_ws/src
    cd ~/catkin_ws/src

Clone this repository and build the project:

    git clone https://github.com/TranThanhTuan2509/mobile-manipulator-ros.git
    cd ~/catkin_ws
    catkin_make

# Running the Robot

This project provides 4 types of control methods.
Before controlling the robot, launch it in Rviz and Gazebo.
## 1) Differential Drive Control (Normal)
    
    roslaunch midterm controller_diff.launch

### 1.1) Control with a Specific Velocity or Angular Command

    rosrun midterm free_control.py
Then,

    rostopic pub /meca/diff_drive_controller/cmd_vel geometry_msgs/Twist "linear:
      x: 0.8
      y: 0.0
      z: 0.0
    angular:
      x: 0.0
      y: 0.0
      z: 0.2"

### 1.2) Control via Teleoperation (Keyboard Control)

    rosrun midterm teleoperation_all.py

## 2) Independent Control (Dynamic Control of the Robot)

       roslaunch midterm controller.launch

### 2.1) Automatic Control with Obstacle Avoidance

    rosrun midterm main.py

Warning: It is recommended not to modify this file or manually enter CLI commands for control due to its waiting-time property.
### 2.2) Teleoperation (Keyboard Control)
    
    rosrun midterm teleoperation_4_wheels.py
