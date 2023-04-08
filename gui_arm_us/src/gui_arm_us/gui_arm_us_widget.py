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
)
import rosservice

from sensor_msgs.msg import JointState

ROS_PARAM_HEADING: str = "/motor_translator/j"


class GuiArmUsWidget(QtWidgets.QWidget):
    def __init__(self):
        super(GuiArmUsWidget, self).__init__()

        ui_file = os.path.join(
            rospkg.RosPack().get_path("gui_arm_us"), "resource", "gui_arm_us.ui"
        )
        loadUi(ui_file, self)
        self.setObjectName("GuiArmUsWidget")

        self.launch: roslaunch.ROSLaunch = roslaunch.scriptapi.ROSLaunch()
        self.launch.start()

        # Callbacks
        self.sub_gui_info = rospy.Subscriber(
            "angles_joint_state", JointState, self.update_gui
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

        self.raw_angles: float = (0.0, 0.0, 0.0, 0.0, 0.0)

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

    # Update loop for GUI, linked with motor controller's translated topic messages
    def update_gui(self, data: JointState):
        index: int = 0
        for qbutton in self.curr_vel_objects:
            rospy.loginfo("Index:" + str(index))
            qbutton.setValue(data.velocity[index])
            index += 1

        index = 0
        for qbutton in self.curr_angle_objects:
            qbutton.setValue(data.position[index])
            index += 1

    def calib_btn_callback(self, joint_index: int, limit_type: str):
        if limit_type == "min":
            rospy.set_param(
                ROS_PARAM_HEADING + str(joint_index) + "/min_limit",
                self.calib_min_objects[joint_index - 1].value(),
            )
            rospy.set_param(
                ROS_PARAM_HEADING + str(joint_index) + "/pos_min_angle",
                self.raw_angles[joint_index - 1],
            )

        elif limit_type == "max":
            rospy.set_param(
                ROS_PARAM_HEADING + str(joint_index) + "/max_limit",
                self.calib_max_objects[joint_index - 1].value(),
            )
            rospy.set_param(
                ROS_PARAM_HEADING + str(joint_index) + "/pos_max_angle",
                self.raw_angles[joint_index - 1],
            )
        else:
            rospy.loginfo("Wrong limit type in calibration callback call")

    def init_param(self, object_list, param_name: str):
        index = 1
        for qbutton in object_list:
            qbutton.setValue(
                rospy.get_param(ROS_PARAM_HEADING + str(index) + param_name)
            )
            index += 1

    # TODO add topic for angle and current velocity from angle_joint_state and link raw motor pos
