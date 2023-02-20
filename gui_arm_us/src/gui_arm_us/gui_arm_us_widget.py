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

from arm_us.msg import MotorControl


class GuiArmUsWidget(QtWidgets.QWidget):

    max_velocity = 4.8

    motor_statuses = []

    motor_statuses.append(MotorControl(0, False, 0.0))
    motor_statuses.append(MotorControl(1, False, 0.0))

    def __init__(self):
    
        super(GuiArmUsWidget, self).__init__()

        ui_file = os.path.join(rospkg.RosPack().get_path('gui_arm_us'), 'resource', 'gui_arm_us.ui')
        loadUi(ui_file, self)
        self.setObjectName('GuiArmUsWidget')

        self.launch: roslaunch.ROSLaunch = roslaunch.scriptapi.ROSLaunch()
        self.launch.start()

        self.pub_gui_arm_us_cmd = rospy.Publisher('gui_arm_us_chatter', MotorControl)
        self.rate = rospy.Rate(10) # 10Hz

        self.motor1_button.released.connect(self.motor1_button_callback)
        self.motor2_button.released.connect(self.motor2_button_callback)

    def motor1_button_callback(self):
        if self.motor_statuses[0].enabled:
            self.motor_statuses[0].enabled = False
            self.motor_statuses[0].velocity = 0
        else:
            self.motor_statuses[0].enabled = True
            self.motor_statuses[0].velocity = self.max_velocity

        rospy.loginfo("Motor 1 : %s, velocity : %f", self.motor_statuses[0].enabled, self.motor_statuses[0].velocity)
        self.pub_gui_arm_us_cmd.publish(self.motor_statuses[0])


    def motor2_button_callback(self):
        if self.motor_statuses[1].enabled:
            self.motor_statuses[1].enabled = False
            self.motor_statuses[1].velocity = 0
        else:
            self.motor_statuses[1].enabled = True
            self.motor_statuses[1].velocity = self.max_velocity

        rospy.loginfo("Motor 1 : %s, velocity : %f", self.motor_statuses[1].enabled, self.motor_statuses[1].velocity)
        self.pub_gui_arm_us_cmd.publish(self.motor_statuses[1])

