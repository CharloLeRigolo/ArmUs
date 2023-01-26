# ArmUs

## Material
- OpenCR board
- 2 Dynamixel XL430 motors
- 1 PS4 Controller

## Dependencies

- joy
- dynamixel_sdk
- dynamixel_interface

run theses commands to install packages available in rosdep:
```
sudo apt install ros-noetic-joy
```

Some packages needs to be cloned directly from Github as they aren't maintained in rosdep,
To clone them, first make */catkin_ws/src/* your working directory:
```
cd ~/catkin_ws/src
```
Then, enter these commands to clone the packages:
```
git clone https://github.com/csiro-robotics/dynamixel_interface.git
git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git
```
Once all depedencies are installed, don't forget to compile your workspace
```
cd ~/catkin_ws/
catkin_make
```

## Running the pkg
Open a terminal and run:
```
roslaunch arm_us master_control.launch
```
This will start the communication between the controller and the motor controller.

In an other terminal, run:
```
roslaunch arm_us motor_controller.launch
```
This will start the motor controller which communicates thru the OpenCR in bridge mode to the dynamixel motors.

You should now be able to controls the motors with the ps4 controller





