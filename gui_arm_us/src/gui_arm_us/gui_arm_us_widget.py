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
from sensor_msgs.msg import JointState


class GuiArmUsWidget(QtWidgets.QWidget):

    max_velocity = 4.8

    motor_statuses = []

    motor_statuses.append(MotorControl(0, False, 0.0))
    motor_statuses.append(MotorControl(1, False, 0.0))

    motor_velocities = []

    motor_velocities.append(max_velocity)
    motor_velocities.append(max_velocity)

    def __init__(self):
    
        super(GuiArmUsWidget, self).__init__()

        ui_file = os.path.join(rospkg.RosPack().get_path('gui_arm_us'), 'resource', 'gui_arm_us.ui')
        loadUi(ui_file, self)
        self.setObjectName('GuiArmUsWidget')

        self.launch: roslaunch.ROSLaunch = roslaunch.scriptapi.ROSLaunch()
        self.launch.start()

        self.sub_motor_positions = rospy.Subscriber("joint_states", JointState, self.motor_positions_callback)

        self.pub_gui_arm_us_cmd = rospy.Publisher('gui_arm_us_chatter', MotorControl, queue_size=1)
        self.rate = rospy.Rate(10) # 10Hz

        self.motor1_button.released.connect(self.motor1_button_callback)
        self.motor2_button.released.connect(self.motor2_button_callback)

        self.motor1_velocity.setValue(self.max_velocity)
        self.motor2_velocity.setValue(self.max_velocity)


        self.motor1_velocity.setRange(0, self.max_velocity)
        self.motor2_velocity.setRange(0, self.max_velocity)

        self.motor1_velocity.valueChanged.connect(self.motor1_velocity_callback)
        self.motor2_velocity.valueChanged.connect(self.motor2_velocity_callback)



    def motor1_button_callback(self):
        if self.motor_statuses[0].enabled:
            self.motor_statuses[0].enabled = False
            self.motor_statuses[0].velocity = 0
        else:
            self.motor_statuses[0].enabled = True
            self.motor_statuses[0].velocity = self.motor_velocities[0]

        rospy.loginfo("Motor 1 : %s", self.motor_statuses[0].enabled)
        self.pub_gui_arm_us_cmd.publish(self.motor_statuses[0])



    def motor2_button_callback(self):
        if self.motor_statuses[1].enabled:
            self.motor_statuses[1].enabled = False
            self.motor_statuses[1].velocity = 0
        else:
            self.motor_statuses[1].enabled = True
            self.motor_statuses[1].velocity = self.motor_velocities[1]

        rospy.loginfo("Motor 2 : %s", self.motor_statuses[1].enabled)
        self.pub_gui_arm_us_cmd.publish(self.motor_statuses[1])



    def motor1_velocity_callback(self, velocity):
        self.motor_velocities[0] = velocity
        rospy.loginfo("Motor 1 velocity : %f", self.motor_velocities[0])



    def motor2_velocity_callback(self, velocity):
        self.motor_velocities[1] = velocity
        rospy.loginfo("Motor 1 velocity : %f", self.motor_velocities[1])


    
    def motor_positions_callback(self, positions):
        self.motor1_position.setNum(positions[0].position)
        self.motor2_position.setNum(positions[1].position)
        rospy.loginfo("Motor 1 position : %f, motor 2 position : %f", positions[0].position, positions[1].position)
