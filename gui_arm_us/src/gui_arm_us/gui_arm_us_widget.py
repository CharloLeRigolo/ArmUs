from __future__ import print_function, absolute_import, division, unicode_literals

import os
import rospkg
import rospy
import roslaunch
import os
from rospkg import RosPack
from python_qt_binding import loadUi
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import QObject
from PyQt5.QtGui import QKeySequence
from PyQt5.QtWidgets import (
    QDoubleSpinBox,
    QSpinBox,
    QCheckBox,
)

from threading import Lock

from sensor_msgs.msg import JointState

from arm_us_msg.msg import GuiInfo
from arm_us_msg.msg import JointLimits

ROS_PARAM_HEADING: str = "/motor_translator/j"

FEEDBACK_CALLBACK_FREQUENCY = 10  # Hz (Replaces Ros Rate)


class GuiArmUsWidget(QtWidgets.QWidget):
    def __init__(self):
        self.lock = Lock()

        super(GuiArmUsWidget, self).__init__()

        ui_file = os.path.join(rospkg.RosPack().get_path("gui_arm_us"), "resource", "gui_arm_us.ui")
        loadUi(ui_file, self)
        self.setObjectName("GuiArmUsWidget")

        self.launch: roslaunch.ROSLaunch = roslaunch.scriptapi.ROSLaunch()
        self.launch.start()

        # Publisher to send callibration messages
        self.pub_angles = rospy.Publisher("gui_angles_joint_state", JointState, queue_size=1)

        # Callbacks
        # Receives the current angles in degrees and velocities of each joint
        self.sub_gui_info = rospy.Subscriber("angles_joint_state", JointState, self.update_gui)
        # Receives the raw positions from the dynamixels
        self.sub_joint_state = rospy.Subscriber("joint_states", JointState, self.getRawAngle)
        # Receives the current movement mode and current joint controlled from the master node
        self.sub_gui_info_control = rospy.Subscriber("gui_info", GuiInfo, self.update_gui_info_control)
        # Receives if any joints reached their limits
        self.sub_joint_limits = rospy.Subscriber("joint_limits", JointLimits, self.update_joint_limits)

        self.rate = rospy.Rate(10)  # 10Hz

        # Minimum calibration buttons
        self.calib_min_objects = (
            self.calib_min_1,
            self.calib_min_2,
            self.calib_min_3,
            self.calib_min_4,
            self.calib_min_5,
        )
        # Maximum calibration buttons
        self.calib_max_objects = (
            self.calib_max_1,
            self.calib_max_2,
            self.calib_max_3,
            self.calib_max_4,
            self.calib_max_5,
        )
        # Curent velocties of joints fields
        self.curr_vel_objects: QDoubleSpinBox = (
            self.curr_vel_1,
            self.curr_vel_2,
            self.curr_vel_3,
            self.curr_vel_4,
            self.curr_vel_5,
        )  
        # Current angles in degrees of joints fields
        self.curr_angle_objects: QDoubleSpinBox = (
            self.curr_angle_1,
            self.curr_angle_2,
            self.curr_angle_3,
            self.curr_angle_4,
            self.curr_angle_5,
        )
        # Current joint limits checkboxes
        self.joint_limits_objects: QCheckBox = (
            self.joint_1_checkbox,
            self.joint_2_checkbox,
            self.joint_3_checkbox,
            self.joint_4_checkbox,
            self.joint_5_checkbox,
        )

        # Current movement mode is joint checkbox
        self.mode_checkbox_joint: QCheckBox
        # Current movement mode is cartesian checkbox
        self.mode_checkbox_cartesian: QCheckBox

        # Current joint controlled field
        self.curr_joint: QSpinBox

        self.raw_angles: float = [0.0, 0.0, 0.0, 0.0, 0.0]

        # Intialize minimum and maximum joint limits from arm_us/config/joint_limits.yaml
        self.init_param(self.calib_min_objects, "/min_limit")
        self.init_param(self.calib_max_objects, "/max_limit")

        # Button callbacks
        self.calib_min_b_1.released.connect(lambda: self.calib_btn_callback(1, "min"))
        self.calib_max_b_1.released.connect(lambda: self.calib_btn_callback(1, "max"))

        self.calib_min_b_2.released.connect(lambda: self.calib_btn_callback(2, "min"))
        self.calib_max_b_2.released.connect(lambda: self.calib_btn_callback(2, "max"))

        self.calib_min_b_3.released.connect(lambda: self.calib_btn_callback(3, "min"))
        self.calib_max_b_3.released.connect(lambda: self.calib_btn_callback(3, "max"))

        self.calib_min_b_4.released.connect(lambda: self.calib_btn_callback(4, "min"))
        self.calib_max_b_4.released.connect(lambda: self.calib_btn_callback(4, "max"))

        self.calib_min_b_5.released.connect(lambda: self.calib_btn_callback(5, "min"))
        self.calib_max_b_5.released.connect(lambda: self.calib_btn_callback(5, "max"))

        self.lastTimeFeedback = rospy.Time.now()

    def update_gui(self, data: JointState):
        """Update loop for the GUI, linked with the motor controller's translated topic messages

        Args:
            data (JointState): current joint angles in degrees and velocities
        """
        # Force lower refresh rate
        if (self.lastTimeFeedback + rospy.Duration(10 / FEEDBACK_CALLBACK_FREQUENCY) > rospy.Time.now()):
            return

        self.lastTimeFeedback = rospy.Time.now()

        # Update joint velocities
        index: int = 0
        for qbutton in self.curr_vel_objects:
            # rospy.loginfo("Index:" + str(index))
            qbutton.setValue(data.velocity[index])
            index += 1

        # Update joint angles in degrees
        index = 0
        for qbutton in self.curr_angle_objects:
            # rospy.loginfo("Index " + str(index) + " : " + str(data.position[index]))
            qbutton.setValue(data.position[index])
            index += 1

    
    def update_gui_info_control(self, data: GuiInfo):
        """Update the current movement mode and current joint controlled to the GUI

        Args:
            data (GuiInfo): current movement mode and current joint controlled
        """
        # Joint movement mode
        if data.current_mode == 0:
            self.mode_checkbox_joint.setChecked(False)
            self.mode_checkbox_cartesian.setChecked(True)
            self.curr_joint.setDisabled(False)
        # Cartesian movement mode
        elif data.current_mode == 1:
            self.mode_checkbox_joint.setChecked(True)
            self.mode_checkbox_cartesian.setChecked(False)
            self.curr_joint.setDisabled(True)
        
        # Joint controlled
        self.curr_joint.setValue(data.current_joint)

    def update_joint_limits(self, data: JointLimits):
        """Update the GUI to indicate if any joints reached their limits

        Args:
            data (JointLimits): array of booleans to indicate if any joints reached their limits, sent from the motor translator
        """
        index = 0
        for qcheckbox in self.joint_limits_objects:
            qcheckbox.setChecked(data.joint_limits[index])
            index += 1

    def calib_btn_callback(self, joint_index: int, limit_type: str):
        """Change the joint limits for the new value entered in the GUI

        Args:
            joint_index (int): Which joint's limits are changed
            limit_type (str): If the min limit or max limit is changed
        """
        raw_angle: float = 0.0
        # The position of joint 1 is the difference in position between the first two motors
        if joint_index == 1:
            raw_angle = self.raw_angles[0] - self.raw_angles[1]
        # The position of joint 2 is the average in position of the first two motors
        elif joint_index == 2:
            raw_angle = (self.raw_angles[0] + self.raw_angles[1]) / 2
        # The position of the motor can be directly taken for the remaining joints
        else:
            raw_angle = self.raw_angles[joint_index - 1]

        # Mapping a new joint position to an angle
        if limit_type == "min":
            rospy.set_param(
                ROS_PARAM_HEADING + str(joint_index) + "/min_limit",
                self.calib_min_objects[joint_index - 1].value(),
            )
            rospy.set_param(
                ROS_PARAM_HEADING + str(joint_index) + "/pos_min_angle",
                raw_angle,
            )
        elif limit_type == "max":
            rospy.set_param(
                ROS_PARAM_HEADING + str(joint_index) + "/max_limit",
                self.calib_max_objects[joint_index - 1].value(),
            )
            rospy.set_param(
                ROS_PARAM_HEADING + str(joint_index) + "/pos_max_angle",
                raw_angle,
            )
        else:
            rospy.loginfo("Wrong limit type in calibration callback call")

    def getRawAngle(self, data: JointState):
        """Get the raw positions of the motors from the dynamixels

        Args:
            data (JointState): raw position of the motors
        """
        if (
            self.lastTimeFeedback + rospy.Duration(10 / FEEDBACK_CALLBACK_FREQUENCY)
            > rospy.Time.now()
        ):
            return
        self.lastTimeFeedback = rospy.Time.now()

        #Reordering positions array
        motor_dict = {
                    'motor1': 0,
                    'motor2': 1,
                    'motor3': 2,
                    'motor4': 3,
                    'motor5': 4
                    }       
        
        for i in range(len(data.name)-1):
            self.raw_angles[motor_dict[data.name[i]]] = data.position[i]

    def init_param(self, object_list, param_name: str):
        """Intialize minimum and maximum joint limits from arm_us/config/joint_limits.yaml and visualizing them on the GUI

        Args:
            object_list (QDoubleSpinBox): Fields where the min and max joint limits are displayed
            param_name (str): If current joint limit is min or max
        """
        index = 1
        for qbutton in object_list:
            qbutton.setValue(
                rospy.get_param(ROS_PARAM_HEADING + str(index) + param_name)
            )
            index += 1
