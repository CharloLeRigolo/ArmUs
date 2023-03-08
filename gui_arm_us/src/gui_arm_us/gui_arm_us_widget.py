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

from arm_us.msg import GuiInfo


class GuiArmUsWidget(QtWidgets.QWidget):

    def __init__(self):
    
        super(GuiArmUsWidget, self).__init__()

        ui_file = os.path.join(rospkg.RosPack().get_path('gui_arm_us'), 'resource', 'gui_arm_us.ui')
        loadUi(ui_file, self)
        self.setObjectName('GuiArmUsWidget')

        self.launch: roslaunch.ROSLaunch = roslaunch.scriptapi.ROSLaunch()
        self.launch.start()

        self.sub_gui_info = rospy.Subscriber("gui_info", GuiInfo, self.update_gui)

        self.rate = rospy.Rate(10) # 10Hz

    def update_gui(self, data):
        self.motor1_position.display(data.position[0])
        self.motor2_position.display(data.position[1])
        self.motor3_position.display(data.position[2])
        self.motor4_position.display(data.position[3])
        self.motor5_position.display(data.position[4])

        self.motor1_velocity.display(data.velocity[0])
        self.motor2_velocity.display(data.velocity[1])
        self.motor3_velocity.display(data.velocity[2])
        self.motor4_velocity.display(data.velocity[3])
        self.motor5_velocity.display(data.velocity[4])

        if data.connected[0]:
            self.motor1_connection.setText("Connected")
        else:
            self.motor1_connection.setText("Not connected")

        if data.connected[1]:
            self.motor2_connection.setText("Connected")
        else:
            self.motor2_connection.setText("Not connected")

        if data.connected[2]:
            self.motor3_connection.setText("Connected")
        else:
            self.motor3_connection.setText("Not connected")

        if data.connected[3]:
            self.motor4_connection.setText("Connected")
        else:
            self.motor4_connection.setText("Not connected")

        if data.connected[4]:
            self.motor5_connection.setText("Connected")
        else:
            self.motor5_connection.setText("Not connected")


        if data.limit_reached[0]:
            self.motor1_limit.setText("Connected")
        else:
            self.motor1_limit.setText("Not connected")

        if data.limit_reached[1]:
            self.motor2_limit.setText("Connected")
        else:
            self.motor2_limit.setText("Not connected")

        if data.limit_reached[2]:
            self.motor3_limit.setText("Connected")
        else:
            self.motor3_limit.setText("Not connected")

        if data.limit_reached[3]:
            self.motor4_limit.setText("Connected")
        else:
            self.motor4_limit.setText("Not connected")

        if data.limit_reached[4]:
            self.motor5_limit.setText("Connected")
        else:
            self.motor5_limit.setText("Not connected")