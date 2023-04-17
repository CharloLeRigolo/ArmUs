# ArmUs

## Material
- 1 OpenCR board
- 2 Dynamixel XL430 motors
- 1 PS4 Controller

## Dependencies

- joy
- dynamixel_sdk
- dynamixel_interface

run these commands to install packages available in rosdep:
```
sudo apt install ros-noetic-joy
```

Some packages need to be cloned directly from Github as they aren't maintained in rosdep,
To clone them, first make */catkin_ws/src/* your working directory:
```
cd ~/catkin_ws/src
```
Then, enter these commands to clone the packages:
```
git clone https://github.com/csiro-robotics/dynamixel_interface.git
git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git
```
Once all dependencies are installed, don't forget to compile your workspace
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

In another terminal, run:
```
roslaunch arm_us motor_controller.launch
```
This will start the motor controller which communicates thru the OpenCR in bridge mode to the Dynamixel motors.

You should now be able to control the motors with the ps4 controller

## Sleeve
The accelerometer used is the [BNO055](bosch-sensortec.com/products/smart-sensors/bno055/). They are a lot better than any other accelero we could find.
![](https://github.com/CharloLeRigolo/arm_us/blob/sleeve/arm_us_sleeve/Screenshot%20from%202023-04-17%2017-54-37.png)
