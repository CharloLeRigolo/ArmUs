#!/usr/bin/env python

import rospy
import numpy as np
from numpy.linalg import inv
from arm_us.srv import InverseKinematicCalc, InverseKinematicCalcResponse
import math as m
from math import pi as PI

def build_jacbienne(q1, q2, q3, q4, q5):

    # Grandeurs physiques
    J1x = 0
    J1y = 0
    J1z = 1

    J2x = 1
    J2y = 0
    J2z = 0
    
    J3x = 1
    J3y = 0
    J3z = 0
    
    J4x = 1
    J4y = 0
    J4z = 0

    # Initialisation de la Jacobienne

    x_q1 = -J1x*(-(-(-m.sin(q3)*m.sin(q5)*m.cos(q4) + m.cos(q3)*m.cos(q5))*m.cos(q2) + m.sin(q2)*m.sin(q4)*m.sin(q5))*m.sin(q1) + (m.sin(q3)*m.cos(q5) + m.sin(q5)*m.cos(q3)*m.cos(q4))*m.cos(q1)) - J1x*(m.sin(q1)*m.cos(q2)*m.cos(q3) + m.sin(q3)*m.cos(q1)) - J1x*m.sin(q1)*m.cos(q2) - J1x*m.sin(q1) + J1y*(-(-m.sin(q2)*m.cos(q4) + m.sin(q3)*m.sin(q4)*m.cos(q2))*m.sin(q1) + m.sin(q4)*m.cos(q1)*m.cos(q3)) + 2*J1y*m.sin(q1)*m.sin(q2) - J1z*(-(-(m.sin(q3)*m.cos(q4)*m.cos(q5) + m.sin(q5)*m.cos(q3))*m.cos(q2) - m.sin(q2)*m.sin(q4)*m.cos(q5))*m.sin(q1) + (m.sin(q3)*m.sin(q5) - m.cos(q3)*m.cos(q4)*m.cos(q5))*m.cos(q1)) + J1z*(-m.sin(q1)*m.sin(q3)*m.cos(q2) + m.cos(q1)*m.cos(q3)) + 2*J1z*m.cos(q1) 
    x_q2 = -J1x*(-(m.sin(q3)*m.sin(q5)*m.cos(q4) - m.cos(q3)*m.cos(q5))*m.sin(q2) + m.sin(q4)*m.sin(q5)*m.cos(q2))*m.cos(q1) - J1x*m.sin(q2)*m.cos(q1)*m.cos(q3) - J1x*m.sin(q2)*m.cos(q1) + J1y*(-m.sin(q2)*m.sin(q3)*m.sin(q4) - m.cos(q2)*m.cos(q4))*m.cos(q1) - 2*J1y*m.cos(q1)*m.cos(q2) - J1z*((m.sin(q3)*m.cos(q4)*m.cos(q5) + m.sin(q5)*m.cos(q3))*m.sin(q2) - m.sin(q4)*m.cos(q2)*m.cos(q5))*m.cos(q1) - J1z*m.sin(q2)*m.sin(q3)*m.cos(q1) 
    x_q3 = -J1x*(m.sin(q1)*m.cos(q3) + m.sin(q3)*m.cos(q1)*m.cos(q2)) - J1x*((m.sin(q3)*m.cos(q5) + m.sin(q5)*m.cos(q3)*m.cos(q4))*m.cos(q1)*m.cos(q2) + (-m.sin(q3)*m.sin(q5)*m.cos(q4) + m.cos(q3)*m.cos(q5))*m.sin(q1)) + J1y*(-m.sin(q1)*m.sin(q3)*m.sin(q4) + m.sin(q4)*m.cos(q1)*m.cos(q2)*m.cos(q3)) + J1z*(-m.sin(q1)*m.sin(q3) + m.cos(q1)*m.cos(q2)*m.cos(q3)) - J1z*(-(-m.sin(q3)*m.sin(q5) + m.cos(q3)*m.cos(q4)*m.cos(q5))*m.cos(q1)*m.cos(q2) + (m.sin(q3)*m.cos(q4)*m.cos(q5) + m.sin(q5)*m.cos(q3))*m.sin(q1)) 
    x_q4 = -J1x*((m.sin(q2)*m.sin(q5)*m.cos(q4) - m.sin(q3)*m.sin(q4)*m.sin(q5)*m.cos(q2))*m.cos(q1) - m.sin(q1)*m.sin(q4)*m.sin(q5)*m.cos(q3)) + J1y*((m.sin(q2)*m.sin(q4) + m.sin(q3)*m.cos(q2)*m.cos(q4))*m.cos(q1) + m.sin(q1)*m.cos(q3)*m.cos(q4)) - J1z*((-m.sin(q2)*m.cos(q4)*m.cos(q5) + m.sin(q3)*m.sin(q4)*m.cos(q2)*m.cos(q5))*m.cos(q1) + m.sin(q1)*m.sin(q4)*m.cos(q3)*m.cos(q5)) 
    x_q5 = -J1x*(((m.sin(q3)*m.cos(q4)*m.cos(q5) + m.sin(q5)*m.cos(q3))*m.cos(q2) + m.sin(q2)*m.sin(q4)*m.cos(q5))*m.cos(q1) + (-m.sin(q3)*m.sin(q5) + m.cos(q3)*m.cos(q4)*m.cos(q5))*m.sin(q1)) - J1z*((-(-m.sin(q3)*m.sin(q5)*m.cos(q4) + m.cos(q3)*m.cos(q5))*m.cos(q2) + m.sin(q2)*m.sin(q4)*m.sin(q5))*m.cos(q1) + (m.sin(q3)*m.cos(q5) + m.sin(q5)*m.cos(q3)*m.cos(q4))*m.sin(q1)) 

    y_q1 = 0 
    y_q2 = J1x*((-m.sin(q3)*m.sin(q5)*m.cos(q4) + m.cos(q3)*m.cos(q5))*m.cos(q2) - m.sin(q2)*m.sin(q4)*m.sin(q5)) + J1x*m.cos(q2)*m.cos(q3) + J1x*m.cos(q2) + J1y*(-m.sin(q2)*m.cos(q4) + m.sin(q3)*m.sin(q4)*m.cos(q2)) - 2*J1y*m.sin(q2) - J1z*((-m.sin(q3)*m.cos(q4)*m.cos(q5) - m.sin(q5)*m.cos(q3))*m.cos(q2) - m.sin(q2)*m.sin(q4)*m.cos(q5)) + J1z*m.sin(q3)*m.cos(q2) 
    y_q3 = J1x*(-m.sin(q3)*m.cos(q5) - m.sin(q5)*m.cos(q3)*m.cos(q4))*m.sin(q2) - J1x*m.sin(q2)*m.sin(q3) + J1y*m.sin(q2)*m.sin(q4)*m.cos(q3) - J1z*(m.sin(q3)*m.sin(q5) - m.cos(q3)*m.cos(q4)*m.cos(q5))*m.sin(q2) + J1z*m.sin(q2)*m.cos(q3) 
    y_q4 = J1x*(m.sin(q2)*m.sin(q3)*m.sin(q4)*m.sin(q5) + m.sin(q5)*m.cos(q2)*m.cos(q4)) + J1y*(m.sin(q2)*m.sin(q3)*m.cos(q4) - m.sin(q4)*m.cos(q2)) - J1z*(m.sin(q2)*m.sin(q3)*m.sin(q4)*m.cos(q5) + m.cos(q2)*m.cos(q4)*m.cos(q5)) 
    y_q5 = J1x*((-m.sin(q3)*m.cos(q4)*m.cos(q5) - m.sin(q5)*m.cos(q3))*m.sin(q2) + m.sin(q4)*m.cos(q2)*m.cos(q5)) - J1z*((m.sin(q3)*m.sin(q5)*m.cos(q4) - m.cos(q3)*m.cos(q5))*m.sin(q2) - m.sin(q4)*m.sin(q5)*m.cos(q2)) 

    z_q1 = -J1x*(((-m.sin(q3)*m.sin(q5)*m.cos(q4) + m.cos(q3)*m.cos(q5))*m.cos(q2) - m.sin(q2)*m.sin(q4)*m.sin(q5))*m.cos(q1) - (m.sin(q3)*m.cos(q5) + m.sin(q5)*m.cos(q3)*m.cos(q4))*m.sin(q1)) - J1x*(-m.sin(q1)*m.sin(q3) + m.cos(q1)*m.cos(q2)*m.cos(q3)) - J1x*m.cos(q1)*m.cos(q2) - J1x*m.cos(q1) + J1y*((m.sin(q2)*m.cos(q4) - m.sin(q3)*m.sin(q4)*m.cos(q2))*m.cos(q1) - m.sin(q1)*m.sin(q4)*m.cos(q3)) + 2*J1y*m.sin(q2)*m.cos(q1) - J1z*(((m.sin(q3)*m.cos(q4)*m.cos(q5) + m.sin(q5)*m.cos(q3))*m.cos(q2) + m.sin(q2)*m.sin(q4)*m.cos(q5))*m.cos(q1) - (m.sin(q3)*m.sin(q5) - m.cos(q3)*m.cos(q4)*m.cos(q5))*m.sin(q1)) + J1z*(-m.sin(q1)*m.cos(q3) - m.sin(q3)*m.cos(q1)*m.cos(q2)) - 2*J1z*m.sin(q1) 
    z_q2 = -J1x*((m.sin(q3)*m.sin(q5)*m.cos(q4) - m.cos(q3)*m.cos(q5))*m.sin(q2) - m.sin(q4)*m.sin(q5)*m.cos(q2))*m.sin(q1) + J1x*m.sin(q1)*m.sin(q2)*m.cos(q3) + J1x*m.sin(q1)*m.sin(q2) + J1y*(m.sin(q2)*m.sin(q3)*m.sin(q4) + m.cos(q2)*m.cos(q4))*m.sin(q1) + 2*J1y*m.sin(q1)*m.cos(q2) - J1z*(-(m.sin(q3)*m.cos(q4)*m.cos(q5) + m.sin(q5)*m.cos(q3))*m.sin(q2) + m.sin(q4)*m.cos(q2)*m.cos(q5))*m.sin(q1) + J1z*m.sin(q1)*m.sin(q2)*m.sin(q3) 
    z_q3 = -J1x*(-(m.sin(q3)*m.cos(q5) + m.sin(q5)*m.cos(q3)*m.cos(q4))*m.sin(q1)*m.cos(q2) + (-m.sin(q3)*m.sin(q5)*m.cos(q4) + m.cos(q3)*m.cos(q5))*m.cos(q1)) - J1x*(-m.sin(q1)*m.sin(q3)*m.cos(q2) + m.cos(q1)*m.cos(q3)) + J1y*(-m.sin(q1)*m.sin(q4)*m.cos(q2)*m.cos(q3) - m.sin(q3)*m.sin(q4)*m.cos(q1)) - J1z*((-m.sin(q3)*m.sin(q5) + m.cos(q3)*m.cos(q4)*m.cos(q5))*m.sin(q1)*m.cos(q2) + (m.sin(q3)*m.cos(q4)*m.cos(q5) + m.sin(q5)*m.cos(q3))*m.cos(q1)) + J1z*(-m.sin(q1)*m.cos(q2)*m.cos(q3) - m.sin(q3)*m.cos(q1)) 
    z_q4 = -J1x*((-m.sin(q2)*m.sin(q5)*m.cos(q4) + m.sin(q3)*m.sin(q4)*m.sin(q5)*m.cos(q2))*m.sin(q1) - m.sin(q4)*m.sin(q5)*m.cos(q1)*m.cos(q3)) + J1y*((-m.sin(q2)*m.sin(q4) - m.sin(q3)*m.cos(q2)*m.cos(q4))*m.sin(q1) + m.cos(q1)*m.cos(q3)*m.cos(q4)) - J1z*((m.sin(q2)*m.cos(q4)*m.cos(q5) - m.sin(q3)*m.sin(q4)*m.cos(q2)*m.cos(q5))*m.sin(q1) + m.sin(q4)*m.cos(q1)*m.cos(q3)*m.cos(q5)) 
    z_q5 = -J1x*((-(m.sin(q3)*m.cos(q4)*m.cos(q5) + m.sin(q5)*m.cos(q3))*m.cos(q2) - m.sin(q2)*m.sin(q4)*m.cos(q5))*m.sin(q1) + (-m.sin(q3)*m.sin(q5) + m.cos(q3)*m.cos(q4)*m.cos(q5))*m.cos(q1)) - J1z*(((-m.sin(q3)*m.sin(q5)*m.cos(q4) + m.cos(q3)*m.cos(q5))*m.cos(q2) - m.sin(q2)*m.sin(q4)*m.sin(q5))*m.sin(q1) + (m.sin(q3)*m.cos(q5) + m.sin(q5)*m.cos(q3)*m.cos(q4))*m.cos(q1))

    # Create matrix
    j = np.zeros(5, 5)
    j[0][0] = x_q1
    j[0][1] = x_q2
    j[0][2] = x_q3
    j[0][3] = x_q4
    j[0][4] = x_q5

    j[1][0] = x_q1
    j[1][1] = x_q2
    j[1][2] = x_q3
    j[1][3] = x_q4
    j[1][4] = x_q5

    j[2][0] = x_q1
    j[2][1] = x_q2
    j[2][2] = x_q3
    j[2][3] = x_q4
    j[2][4] = x_q5
    
    return j

def handle_inv_kin_calc(req):

    rospy.loginfo("Service called from python node")

    resp = InverseKinematicCalcResponse()

    command = np.array([0.0] * 5).T

    try:
        # Angles initiaux
        q1 = angle_rad(req.angles[0])
        q2 = angle_rad(req.angles[1])
        q3 = angle_rad(req.angles[2])
        q4 = angle_rad(req.angles[3])
        q5 = angle_rad(req.angles[4])

        # Command
        for i in range(5):
            command[i] = req.commands[i]

        j = build_jacbienne(q1, q2, q3, q4, q5)

        resp.velocities = np.matmul(inv(j), command)

        resp.singularMatrix = 0

    except:
        rospy.WARN("Singular matrix, encountered a column of 0 (can't divide by 0), jog in joiny yo a less critical position")
        resp.velocities = [0, 0, 0, 0, 0]
        resp.singularMatrix = 1

    return resp

def angle_deg(angle : float):
    return angle * 180 / PI

def angle_rad(angle : float):
    return angle * PI / 180

def inv_kin_calc_service():
    rospy.init_node("inv_kin_calc_service")
    inv_calc_service = rospy.Service('inverse_kinematic_calc_service', InverseKinematicCalc, handle_inv_kin_calc)
    rospy.spin()

if __name__ == "__main__":
    inv_kin_calc_service()
