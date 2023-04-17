# Calibration Guide

It is necessary to calibrate the arm for accurate cartesian movements

1. Start the 1_interface.launch launchfile to have access to the GUI

2. Place the arm in a position (by hand, the motor torques are disabled)

3. Write the angle in degrees to which the current joint position will be linked in the corresponding location

4. Press either the "Calibrate min angle" or "Calibrate max angle" button on the GUI

5. The process is done when the position shown in Rviz after launching 2_control.launch matches the position of the arm

5. It is possible to dump the current calibration values with the terminal command
```
rosparam dump **your_path**/**your_calibration_name**.yaml
```

Notes :

It is not currently possible to see the position in real time of the arm while only the first launchfile is running. The code could be modified by sending the joint angles to the 3d graph node directly from the motor translator node, instead of passing by the master node.

Another workaround is to disable the joint limits in the motor translator node, and also start the 2_control.launch launchfile. It is then possible to see the position the program thinks the arm is in, while also moving the arm freely with the controller. If the joint limits are not disable, and some joint limits are off, then it might be impossible to move the arm to the desired positions to do the proper calibration.