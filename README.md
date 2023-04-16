# Table of content
- [ArmUs](#armus)
- [Accessing documentation](#accessing-documentation)
- [Setup](#setup)
  * [Setting up your environement (software)](#setting-up-your-environement-software)
  * [Material (Software side)](#material-software-side)
  * [Material and assembly](https://github.com/CharloLeRigolo/arm_us/tree/main/Mechanics)
- [Running the robot](#running-the-robot)
- [ROS information](#ros-information)
  * [General picture](#general-picture)
  * [Packages](#packages)
  * [Launch files](#launch-files)
  * [Config files](#config-files)
  * [Msg, srv and action files](#msg-srv-and-action-files)
- [License](#license)

# ArmUs
ArmUs is a 5 axis robot made to reproduce movement and goemetry of a real human arm. It's made by a team of 6 sherbrooke university undergraduates. The goal is to eventually add a robotic hand at the end and be able to control the whole arm-hand assembly by moving your arm. 

![ARM US](PHOTO D'ARMUS)

At this stage, the arm can be controled either in joint or in cartisian mode and can be visualized and calibrated in real time, it also supports software protection, torque limiters to reduce risks and a simulation mode for test purposes.

Control interface (HMI)
![HMI](https://github.com/CharloLeRigolo/arm_us/blob/main/Photos/HMI%20ScreenShot.png)

# Accessing documentation
To open a package's documentation, open the index.html file found in it's "/doc" folder in your browser.
Ex:
```
firefox ~/catkin_ws/src/arm_us/arm_us/doc/html/index.html
```

# Setup
## Setting up your environement (software)
1.  Start by installing [ubuntu 20.04 desktop Focal Fossa](https://releases.ubuntu.com/focal/) on your machine (VM or dualboot), dualboot is highly recommended.
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
7. Setup your OpenCR in bridge mode
	1. Install [ArduinoIDE](https://www.arduino.cc/en/software) on a computer
	2. Follow the steps in section 4.X (depending on your operating system) of the [ROBOTIS e-Manual for OpenCR](https://emanual.robotis.com/docs/en/parts/controller/opencr10/)
	3. Select the OpenCR board in ArduinoIDE
	4. Select "usb_to_dxl" In *File -> Examples -> OpenCR -> 10. Etc -> usb_to_dxl*
	5. Upload the example to the OpenCr board, make sure you're on the correct port and the device in /dev/ttyXXX has read/write access
		```
		sudo chmod a+rw /dev/ttyACM0
		```
	The OpenCR board is now in bridge mode. We will be able to control it directly from the Serial Port

## Material (Software side)
For the robot's assembly, follow the [README.md](https://github.com/CharloLeRigolo/arm_us/blob/main/Mechanics/README.md) in [/Mechanics](https://github.com/CharloLeRigolo/arm_us/tree/main/Mechanics).

- 1 OpenCR board
- 1 OpenCR 120VAC to 12VDC powersupply
- 2 Dynamixel XL430 motors
- 3 Dynamixel XM430 motors
- 1 USB micro B to USB A
- 1 Generic controller (any controller compatible with [joy](http://wiki.ros.org/joy) will work, you'll probably need to remap the keybindings if you're not using a logitech controller, in [/ArmUs/arm_us/config/controller_config.yaml])

# Running the robot
Open a terminal and launch the first launchfile:
```
roslaunch arm_us 1_interface.launch
```
This will start the HMI and the communication with the motors.
Calibrate the robot by following this [guide](notdone.sorry).

Once ArmUs is calibrated, launch the 2nd launchfile
```
roslaunch arm_us 2_control.launch
```
This will enable the torque on the motors and enable the control with the controller, you should now be able to control the motors with your controller in joint or cartesian as you wish

![Joint control](https://github.com/CharloLeRigolo/arm_us/blob/main/Photos/Keybindings%20Cartesian%20(En).PNG)

![Cartesian control](https://github.com/CharloLeRigolo/arm_us/blob/main/Photos/Keybindings%20Joint%20(En).PNG)

Before killing your node, if you want to keep your calibration values, run this command
```
rosparam dump **yourpath**/calib.yaml
```
and copy the lines under "motor_translator:" in the config file arm_us/config/joint_limit.yaml.

# ROS information
## General picture
![rqt_graph](https://github.com/CharloLeRigolo/arm_us/blob/main/Photos/RQT_ScreenShot.png)
## Packages
There are 4 packages in ArmUs:
 - [arm_us](https://github.com/CharloLeRigolo/arm_us/blob/main/arm_us/doc/manifest.yaml)
 - [arm_us_graph](https://github.com/CharloLeRigolo/arm_us/blob/main/arm_us_graph/doc/manifest.yaml)
 - [gui_arm_us](https://github.com/CharloLeRigolo/arm_us/blob/main/gui_arm_us/doc/manifest.yaml)
 - [arm_us_msg](https://github.com/CharloLeRigolo/arm_us/blob/main/arm_us_msg/doc/manifest.yaml)

## Launch files
- 1_interface.launch
	- Nodes:
		- motor_controller
		- motor_translator
		- rqt
		
- 2_control.launch
	- Param:
		- Verbose
		- Simulation
		- Controller
		
	- Nodes:
		- master_node
		- joy_node
		- arm_us_graphic
		
	- Service:
		- inv_kin_calc_service

## Config files
- controller_config.yaml
	- Keybindings for joy
- joint_limit.yaml
	- Used for angle calibration and joint limit angles
- motor_config.yaml
	- Dynamixel configuration; torque limit, speed limit, motor id, etc

## Msg, srv and action files
Information for msg, srv and action files can be found directly in the [arm_us_msg](https://github.com/CharloLeRigolo/arm_us/tree/main/arm_us_msg) package

- Messages
  - [GraphInfo.msg](https://github.com/CharloLeRigolo/arm_us/blob/main/arm_us_msg/msg/GraphInfo.msg)
    - Consists of an array of float64s of length 5 named angle that represents the angles of the joints in degrees
    - It is used to send the joint angles in degrees from the motor translator to the master node, where it is then sent back to the inverse kinematic service node to control the arm in cartesian mode, and to the graph node to visualize the position of the arm in real time in Rviz.

  - [GuiInfo.msg](https://github.com/CharloLeRigolo/arm_us/blob/main/arm_us_msg/msg/GuiInfo.msg)
    - Consists of an int8 named current_joint that represents the current joint controlled, and another int8 named current_mode, which represents the current movement mode.
    - It is used to send the current joint controlled and the current movement mode to the Gui node so that the user can see in real time the information.

  - [JointLimits.msg](https://github.com/CharloLeRigolo/arm_us/blob/main/arm_us_msg/msg/JointLimits.msg)
    - Consists of an array of booleans of length 5 named joint_limits that indicates if any of the joints reached their limits.
    - It is used by the motor translator to send information to the Gui to indicate if any joint limits are reached.

- Services
  - [InverseKinematicCalc.srv](https://github.com/CharloLeRigolo/arm_us/tree/main/arm_us_msg/srv)
    - Request : 
      - angles : An array of float64s of length 3 named angles that represents the current angles of the first 3 joints used in the inverse kinematic equations.
      - commands : An array of float64s of length 4 named commands that represents the velocities of the end effector of the arm in the X, Y and Z axis.
    - Response :
      - velocities : An array of float64s of length 3 that represents the velocities of the first 3 joints needed to do the cartesian movement requested.
      - singularMatrix : A boolean name singularMatrix which indicates if a singularMatrix was encountered when calculating the joint velocities.



# License
MIT [Licence](https://github.com/CharloLeRigolo/arm_us/blob/main/LICENSE)
