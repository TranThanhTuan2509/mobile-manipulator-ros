# mobile-manipulator-ros

<p align="center">
  <picture>
    <img alt="image" src="https://github.com/TranThanhTuan2509/mobile-manipulator-ros/blob/main/asset/ros.png "video2command"" width="600" height="350" style="max-width: 100%;">
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
    source devel/setup.bash

# Running the Robot

This project provides 3 types of control methods.
Before controlling the robot, move your current path to my github repo you just cloned:

    cd ~/catkin_ws/src/mobile-manipulator-ros

Then, launch main ros package as `midterm` in Rviz and Gazebo.( Everytime running, keeps your terminal path pointing to `catkin_ws/src/mobile-manipulator-ros` )
## 1) Differential Drive Control (Normal)
    
    roslaunch midterm controller_diff.launch

### 1.1) Control with a Specific Velocity or Angular Command

    rosrun midterm free_control.py
    
Once the program starts, the terminal will prompt you to enter two values, one for v linear x and the other for angular z:
`Enter 'v z' (velocities) or 'stop' to stop: 1 0`
Enters `stop` to stop the robot or `value1 value2` to control the robot car

### 1.2) Control via Teleoperation (Keyboard Control)

    rosrun midterm teleoperation_all.py

## 2) Independent Control (Dynamic Control of the Robot)

       roslaunch midterm controller.launch

### 2.1) Automatic Control with Obstacle Avoidance
To manually enter velocity values and control the robot, run:

    rosrun midterm main.py

Once the program starts, the terminal will prompt you to enter three values, one for vx and the others for vy and wz:
`Enter 'vx vy wz' (velocities) or 'stop' to stop: 1 -1 0`
Enters `stop` to stop the robot or `value1 value2 value3` to control the robot car

### 2.2) Teleoperation (Keyboard Control)
    
    rosrun midterm teleoperation_4_wheels.py
## 3) Control the arm only
To control only the robotic arm, run:

    rosrun midterm control_arm_only.py
Once the program starts, the terminal will prompt you to enter two values, one for link 1 and the other for link 2:
`Enter 's1 s2' (servo1 servo2) or 'exit' to quit: 0.5 0.3`
Enters `exit` to exit the program or `value1 value2` to control the arm
