# ArmUs

## Setting up your environement (software)
1.  Start by installing [ubuntu 20.04 desktop Focal Fossa](https://releases.ubuntu.com/focal/) on your machine (VM or dualboot), dualboot is highly recomended.
2.  Install ROS-Noetic by following [this tutorial](http://wiki.ros.org/noetic/Installation/Ubuntu)
3.  Edit your bashrc
    - Open your .bashrc (if you're using bash) 
      ```
      sudo nano ~/.bashrc
      ```
    - Then add these lines at the end
      ```
      source /opt/ros/noetic/setup.bash
      source ~/catkin_ws/devel/setup.bash
      ```
    - Save and exit
      ```
      ctrl+s
      ctrl+x
      ```
4.  Setup your catkin_ws by following [this tutorial](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment)
5.  Clone all the arm_us packages from this git repo to your catkin_ws/src folder
    - arm_us
    - arm_us_graph
    - gui_arm_us
    - arm_us_msg

6.  Install all the dependencies:
    - Dependencies
      - joy
      - dynamixel_sdk
      - dynamixel_interface

    - Run these commands to install packages available in rosdep:
      ```
      sudo apt install ros-noetic-joy
      ```

    - Some packages need to be cloned directly from GitHub as they aren't maintained in rosdep, to clone them,
      - first make */catkin_ws/src/* your working directory:
        ```
        cd ~/catkin_ws/src
        ```
      - Then, enter these commands to clone the packages:
        ```
        git clone https://github.com/csiro-robotics/dynamixel_interface.git
        git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git
        ```
    - Once all dependencies are installed, don't forget to compile your workspace
      ```
      cd ~/catkin_ws/
      catkin_make
      ```

## Material
- 1 OpenCR board
- 2 Dynamixel XL430 motors
- 3 Dynamixel XM430 motors
- 1 XBOX controller (any controller compatible with [joy](http://wiki.ros.org/joy) will work, you'll probably need to remap the keybindings if you're not using an XBOX controller)

## 

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

You should now be able to control the motors with your controller

# Setup
## OpenCR setup
1. Install [ArduinoIDE](https://www.arduino.cc/en/software) on a computer
2. Follow the steps in section 4.X (depending on your operating system) of the [ROBOTIS e-Manual for OpenCR](https://emanual.robotis.com/docs/en/parts/controller/opencr10/)
3. Select the OpenCR board in ArduinoIDE
4. Select "usb_to_dxl" In *File -> Examples -> OpenCR -> 10. Etc -> usb_to_dxl*
5. Upload the example to the OpenCr board, make sure your on the correct port and the device in /dev/ttyXXX as read/write access
```
sudo chmod a+rw /dev/ttyACM0
```
The OpenCR board is now in bridge mode. We will be able to control directly from the Serial Port

## ROS Setup
This project is developped for ROS noetic, this means you'll need to install ROS on a machine running Ubuntu 20
1. Install [Ubuntu 20.04](https://releases.ubuntu.com/focal/) on a machine
2. Install ROS by following the [ROS website](https://www.ros.org/blog/getting-started/) instructions
3. Install [dependencies](##Dependencies)
4. Install the arm_us package
In your ROS's workspace "src" folder:
``` 
git clone https://github.com/CharloLeRigolo/arm_us.git
cd ..
catkin_make
```
