from __future__ import (print_function, absolute_import, division, unicode_literals)

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
from PyQt5.QtWidgets import QShortcut, QSlider, QLCDNumber, QLabel, QPushButton, QFrame
import rosservice

from ArmUs.msg import MotorControl

class GuiArmUsWidget(QtWidgets.QWidget):

    motor1_state = motor_control()
    motor2_state = motor_control()
    
    motor1_state.motor_id = 1
    motor1_state.enabled = false
    motor1_state.velocity = 0
    
    motor2_state.motor_id = 2
    motor2_state.enabled = false
    motor2_state.velocity = 0

    def __init__(self):
    
        super(GuiArmUsWidget, self).__init__()

        ui_file = os.path.join(rospkg.RosPack().get_path('gui_arm_us'), 'resource', 'gui_arm_us.ui')
        loadUi(ui_file, self)
        self.setObjectName('GuiArmUsWidget')

        self.launch: roslaunch.ROSLaunch = roslaunch.scriptapi.ROSLaunch()
        self.launch.start()

        self.pub_gui_arm_us_cmd = rospy.Publisher('gui_arm_us_chatter', motor_control)
        self.rate = rospy.Rate(10) # 10Hz

        self.motor1_button.released.connect(self.motor1_button_callback)
        self.motor2_button.released.connect(self.motor2_button_callback)

    def motor1_button_callback(self):
        if self.motor1_state.enabled:
            self.motor1_state.enabled = False
        else:
            self.motor1_state.enabled = True
        rospy.loginfo("Motor 1 : %s", self.motor1_state.enabled)
        self.pub_gui_arm_us_cmd.publish(self.motor1_state.enabled)

    def motor2_button_callback(self):
        if self.motor2_state.enabled:
            self.motor2_state.enabled = False
        else:
            self.motor2_state.enabled = True
        rospy.loginfo("Motor 2 : %s", self.motor2_state.enabled)
        self.pub_gui_arm_us_cmd.publish(self.motor2_state.enabled)
