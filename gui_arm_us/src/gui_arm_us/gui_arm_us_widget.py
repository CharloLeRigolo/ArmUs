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
    QShortcut,
    QSlider,
    QLCDNumber,
    QLabel,
    QPushButton,
    QFrame,
    QDoubleSpinBox,
    QSpinBox,
    QCheckBox,
)
from threading import Lock
import rosservice

from sensor_msgs.msg import JointState

from arm_us_msg.msg import GuiInfo
from arm_us_msg.msg import JointLimits

ROS_PARAM_HEADING: str = "/motor_translator/j"

FEEDBACK_CALLBACK_FREQUENCY = 10  # Hz (Replaces Ros Rate)


class GuiArmUsWidget(QtWidgets.QWidget):
    def __init__(self):
        self.lock = Lock()

        super(GuiArmUsWidget, self).__init__()

        ui_file = os.path.join(
            rospkg.RosPack().get_path("gui_arm_us"), "resource", "gui_arm_us.ui"
        )
        loadUi(ui_file, self)
        self.setObjectName("GuiArmUsWidget")

        self.launch: roslaunch.ROSLaunch = roslaunch.scriptapi.ROSLaunch()
        self.launch.start()

        self.pub_angles = rospy.Publisher(
            "gui_angles_joint_state", JointState, queue_size=1
        )

        # Callbacks
        self.sub_gui_info = rospy.Subscriber(
            "angles_joint_state", JointState, self.update_gui
        )

        self.sub_joint_state = rospy.Subscriber("joint_states", JointState, self.getRawAngle)

        self.sub_gui_info_control = rospy.Subscriber(
            "gui_info", GuiInfo, self.update_gui_info_control
        )

        self.sub_joint_limits = rospy.Subscriber(
            "joint_limits", JointLimits, self.update_joint_limits
        )

        self.rate = rospy.Rate(10)  # 10Hz

        self.calib_min_objects = (
            self.calib_min_1,
            self.calib_min_2,
            self.calib_min_3,
            self.calib_min_4,
            self.calib_min_5,
        )
        self.calib_max_objects = (
            self.calib_max_1,
            self.calib_max_2,
            self.calib_max_3,
            self.calib_max_4,
            self.calib_max_5,
        )

        self.curr_vel_objects: QDoubleSpinBox = (
            self.curr_vel_1,
            self.curr_vel_2,
            self.curr_vel_3,
            self.curr_vel_4,
            self.curr_vel_5,
        )

        self.curr_angle_objects: QDoubleSpinBox = (
            self.curr_angle_1,
            self.curr_angle_2,
            self.curr_angle_3,
            self.curr_angle_4,
            self.curr_angle_5,
        )

        self.joint_limits_objects: QCheckBox = (
            self.joint_1_checkbox,
            self.joint_2_checkbox,
            self.joint_3_checkbox,
            self.joint_4_checkbox,
            self.joint_5_checkbox,
        )

        self.mode_checkbox_joint: QCheckBox
        self.mode_checkbox_cartesian: QCheckBox

        self.curr_joint: QSpinBox

        self.raw_angles: float = [0.0, 0.0, 0.0, 0.0, 0.0]

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

    # Update loop for GUI, linked with motor controller's translated topic messages
    def update_gui(self, data: JointState):
            if (
                self.lastTimeFeedback + rospy.Duration(10 / FEEDBACK_CALLBACK_FREQUENCY)
                > rospy.Time.now()
            ):
                return

            self.lsastTimeFeedback = rospy.Time.now()

            index: int = 0
            for qbutton in self.curr_vel_objects:
                # rospy.loginfo("Index:" + str(index))
                qbutton.setValue(data.velocity[index])
                index += 1

            index = 0
            for qbutton in self.curr_angle_objects:
                # rospy.loginfo("Index " + str(index) + " : " + str(data.position[index]))
                qbutton.setValue(data.position[index])
                index += 1

    def update_gui_info_control(self, data: GuiInfo):
        if data.current_mode == 0:
            self.mode_checkbox_joint.setChecked(False)
            self.mode_checkbox_cartesian.setChecked(True)
            self.curr_joint.setDisabled(False)
        elif data.current_mode == 1:
            self.mode_checkbox_joint.setChecked(True)
            self.mode_checkbox_cartesian.setChecked(False)
            self.curr_joint.setDisabled(True)

        self.curr_joint.setValue(data.current_joint)

    def update_joint_limits(self, data: JointLimits):
        index = 0
        for qcheckbox in self.joint_limits_objects:
            qcheckbox.setChecked(data.joint_limits[index])
            index += 1

    def calib_btn_callback(self, joint_index: int, limit_type: str):
        raw_angle: float = 0.0
        if joint_index == 1:
            raw_angle = self.raw_angles[0] - self.raw_angles[1];
        elif joint_index == 2:
            raw_angle = (self.raw_angles[0] + self.raw_angles[1]) / 2;
        else:
            raw_angle = self.raw_angles[joint_index - 1];
    
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
                    'motor5': 4}
        
        for i in range(len(data.name)-1):
            self.raw_angles[motor_dict[data.name[i]]] = data.position[i]

    def init_param(self, object_list, param_name: str):
        index = 1
        for qbutton in object_list:
            qbutton.setValue(
                rospy.get_param(ROS_PARAM_HEADING + str(index) + param_name)
            )
            index += 1
