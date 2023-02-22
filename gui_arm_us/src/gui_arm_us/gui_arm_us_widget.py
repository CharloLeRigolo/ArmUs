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

    motor1_enabled = False
    motor2_enabled = False

    motor1_velocity = max_velocity
    motor2_velocity = max_velocity

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

        self.motor1_velocity.setSingleStep(0.2)
        self.motor2_velocity.setSingleStep(0.2)

        self.motor1_velocity.valueChanged.connect(self.motor1_velocity_callback)
        self.motor2_velocity.valueChanged.connect(self.motor2_velocity_callback)



    def motor1_button_callback(self):
        msg = MotorControl()
        msg.motor_id = 0

        if self.motor1_enabled:
            self.motor1_enabled = False
            msg.enabled = self.motor1_enabled
            msg.velocity = 0
        else:
            self.motor1_enabled = True
            msg.enabled = self.motor1_enabled
            msg.velocity = float(self.motor1_velocity.value())

        rospy.loginfo("Motor 1 : %f", self.motor1_enabled)
        self.pub_gui_arm_us_cmd.publish(msg)



    def motor2_button_callback(self):
        msg = MotorControl()
        msg.motor_id = 1

        if self.motor2_enabled:
            self.motor2_enabled = False
            msg.enabled = self.motor2_enabled
            msg.velocity = 0
        else:
            self.motor2_enabled = True
            msg.enabled = self.motor2_enabled
            msg.velocity = float(self.motor2_velocity.value())

        rospy.loginfo("Motor 2 : %f", self.motor2_enabled)
        self.pub_gui_arm_us_cmd.publish(msg)



    def motor1_velocity_callback(self, velocity):
        self.motor1_velocity = velocity
        rospy.loginfo("Motor 1 velocity : %f", self.motor1_velocity)



    def motor2_velocity_callback(self, velocity):
        self.motor2_velocity = velocity
        rospy.loginfo("Motor 2 velocity : %f", self.motor2_velocity)


    
    def motor_positions_callback(self, msg):
        self.motor1_position.setNum(msg.position[0])
        self.motor2_position.setNum(msg.position[1])
        #rospy.loginfo("Motor 1 position : %f, motor 2 position : %f", msg.position[1], msg.position[0])