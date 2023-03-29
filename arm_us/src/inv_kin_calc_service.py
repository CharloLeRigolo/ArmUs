#!/usr/bin/env python

import rospy
import numpy as np
from numpy.linalg import inv, det
from arm_us.srv import InverseKinematicCalc, InverseKinematicCalcResponse
import math as m
from math import pi as PI

def build_jacbienne(q1, q2, q3, q4, q5):

    # Grandeurs physiques
    J1x : float = 0
    J1y : float = 0
    J1z : float = 1

    J2x : float = 1
    J2y : float = 0
    J2z : float = 0
    
    J3x : float = 1
    J3y : float = 0
    J3z : float = 0
    
    J4x : float = 1
    J4y : float = 0
    J4z : float = 0

    # Initialisation de la Jacobienne
    x_q1 = -J1x*m.sin(q1) - J2x*m.sin(q1)*m.cos(q2) - J3x*(m.sin(q1)*m.cos(q2)*m.cos(q3) + m.sin(q3)*m.cos(q1)) + J3z*(-m.sin(q1)*m.sin(q3)*m.cos(q2) + m.cos(q1)*m.cos(q3)) - J4x*(-(-(-m.sin(q3)*m.sin(q5)*m.cos(q4) + m.cos(q3)*m.cos(q5))*m.cos(q2) + m.sin(q2)*m.sin(q4)*m.sin(q5))*m.sin(q1) + (m.sin(q3)*m.cos(q5) + m.sin(q5)*m.cos(q3)*m.cos(q4))*m.cos(q1)) + J4y*(-(-m.sin(q2)*m.cos(q4) + m.sin(q3)*m.sin(q4)*m.cos(q2))*m.sin(q1) + m.sin(q4)*m.cos(q1)*m.cos(q3)) - J4z*(-(-(m.sin(q3)*m.cos(q4)*m.cos(q5) + m.sin(q5)*m.cos(q3))*m.cos(q2) - m.sin(q2)*m.sin(q4)*m.cos(q5))*m.sin(q1) + (m.sin(q3)*m.sin(q5) - m.cos(q3)*m.cos(q4)*m.cos(q5))*m.cos(q1)) + (J1z + J2z)*m.cos(q1) - (-J2y - J3y)*m.sin(q1)*m.sin(q2) 
    x_q2 = -J2x*m.sin(q2)*m.cos(q1) - J3x*m.sin(q2)*m.cos(q1)*m.cos(q3) - J3z*m.sin(q2)*m.sin(q3)*m.cos(q1) - J4x*(-(m.sin(q3)*m.sin(q5)*m.cos(q4) - m.cos(q3)*m.cos(q5))*m.sin(q2) + m.sin(q4)*m.sin(q5)*m.cos(q2))*m.cos(q1) + J4y*(-m.sin(q2)*m.sin(q3)*m.sin(q4) - m.cos(q2)*m.cos(q4))*m.cos(q1) - J4z*((m.sin(q3)*m.cos(q4)*m.cos(q5) + m.sin(q5)*m.cos(q3))*m.sin(q2) - m.sin(q4)*m.cos(q2)*m.cos(q5))*m.cos(q1) + (-J2y - J3y)*m.cos(q1)*m.cos(q2) 
    x_q3 = -J3x*(m.sin(q1)*m.cos(q3) + m.sin(q3)*m.cos(q1)*m.cos(q2)) + J3z*(-m.sin(q1)*m.sin(q3) + m.cos(q1)*m.cos(q2)*m.cos(q3)) - J4x*((m.sin(q3)*m.cos(q5) + m.sin(q5)*m.cos(q3)*m.cos(q4))*m.cos(q1)*m.cos(q2) + (-m.sin(q3)*m.sin(q5)*m.cos(q4) + m.cos(q3)*m.cos(q5))*m.sin(q1)) + J4y*(-m.sin(q1)*m.sin(q3)*m.sin(q4) + m.sin(q4)*m.cos(q1)*m.cos(q2)*m.cos(q3)) - J4z*(-(-m.sin(q3)*m.sin(q5) + m.cos(q3)*m.cos(q4)*m.cos(q5))*m.cos(q1)*m.cos(q2) + (m.sin(q3)*m.cos(q4)*m.cos(q5) + m.sin(q5)*m.cos(q3))*m.sin(q1)) 
    x_q4 = -J4x*((m.sin(q2)*m.sin(q5)*m.cos(q4) - m.sin(q3)*m.sin(q4)*m.sin(q5)*m.cos(q2))*m.cos(q1) - m.sin(q1)*m.sin(q4)*m.sin(q5)*m.cos(q3)) + J4y*((m.sin(q2)*m.sin(q4) + m.sin(q3)*m.cos(q2)*m.cos(q4))*m.cos(q1) + m.sin(q1)*m.cos(q3)*m.cos(q4)) - J4z*((-m.sin(q2)*m.cos(q4)*m.cos(q5) + m.sin(q3)*m.sin(q4)*m.cos(q2)*m.cos(q5))*m.cos(q1) + m.sin(q1)*m.sin(q4)*m.cos(q3)*m.cos(q5)) 
    x_q5 = -J4x*(((m.sin(q3)*m.cos(q4)*m.cos(q5) + m.sin(q5)*m.cos(q3))*m.cos(q2) + m.sin(q2)*m.sin(q4)*m.cos(q5))*m.cos(q1) + (-m.sin(q3)*m.sin(q5) + m.cos(q3)*m.cos(q4)*m.cos(q5))*m.sin(q1)) - J4z*((-(-m.sin(q3)*m.sin(q5)*m.cos(q4) + m.cos(q3)*m.cos(q5))*m.cos(q2) + m.sin(q2)*m.sin(q4)*m.sin(q5))*m.cos(q1) + (m.sin(q3)*m.cos(q5) + m.sin(q5)*m.cos(q3)*m.cos(q4))*m.sin(q1)) 

    y_q1 = 0 
    y_q2 = J2x*m.cos(q2) + J3x*m.cos(q2)*m.cos(q3) + J3z*m.sin(q3)*m.cos(q2) + J4x*((-m.sin(q3)*m.sin(q5)*m.cos(q4) + m.cos(q3)*m.cos(q5))*m.cos(q2) - m.sin(q2)*m.sin(q4)*m.sin(q5)) + J4y*(-m.sin(q2)*m.cos(q4) + m.sin(q3)*m.sin(q4)*m.cos(q2)) - J4z*((-m.sin(q3)*m.cos(q4)*m.cos(q5) - m.sin(q5)*m.cos(q3))*m.cos(q2) - m.sin(q2)*m.sin(q4)*m.cos(q5)) - (J2y + J3y)*m.sin(q2) 
    y_q3 = -J3x*m.sin(q2)*m.sin(q3) + J3z*m.sin(q2)*m.cos(q3) + J4x*(-m.sin(q3)*m.cos(q5) - m.sin(q5)*m.cos(q3)*m.cos(q4))*m.sin(q2) + J4y*m.sin(q2)*m.sin(q4)*m.cos(q3) - J4z*(m.sin(q3)*m.sin(q5) - m.cos(q3)*m.cos(q4)*m.cos(q5))*m.sin(q2) 
    y_q4 = J4x*(m.sin(q2)*m.sin(q3)*m.sin(q4)*m.sin(q5) + m.sin(q5)*m.cos(q2)*m.cos(q4)) + J4y*(m.sin(q2)*m.sin(q3)*m.cos(q4) - m.sin(q4)*m.cos(q2)) - J4z*(m.sin(q2)*m.sin(q3)*m.sin(q4)*m.cos(q5) + m.cos(q2)*m.cos(q4)*m.cos(q5)) 
    y_q5 = J4x*((-m.sin(q3)*m.cos(q4)*m.cos(q5) - m.sin(q5)*m.cos(q3))*m.sin(q2) + m.sin(q4)*m.cos(q2)*m.cos(q5)) - J4z*((m.sin(q3)*m.sin(q5)*m.cos(q4) - m.cos(q3)*m.cos(q5))*m.sin(q2) - m.sin(q4)*m.sin(q5)*m.cos(q2)) 

    z_q1 = -J1x*m.cos(q1) - J2x*m.cos(q1)*m.cos(q2) - J3x*(-m.sin(q1)*m.sin(q3) + m.cos(q1)*m.cos(q2)*m.cos(q3)) + J3z*(-m.sin(q1)*m.cos(q3) - m.sin(q3)*m.cos(q1)*m.cos(q2)) - J4x*(((-m.sin(q3)*m.sin(q5)*m.cos(q4) + m.cos(q3)*m.cos(q5))*m.cos(q2) - m.sin(q2)*m.sin(q4)*m.sin(q5))*m.cos(q1) - (m.sin(q3)*m.cos(q5) + m.sin(q5)*m.cos(q3)*m.cos(q4))*m.sin(q1)) + J4y*((m.sin(q2)*m.cos(q4) - m.sin(q3)*m.sin(q4)*m.cos(q2))*m.cos(q1) - m.sin(q1)*m.sin(q4)*m.cos(q3)) - J4z*(((m.sin(q3)*m.cos(q4)*m.cos(q5) + m.sin(q5)*m.cos(q3))*m.cos(q2) + m.sin(q2)*m.sin(q4)*m.cos(q5))*m.cos(q1) - (m.sin(q3)*m.sin(q5) - m.cos(q3)*m.cos(q4)*m.cos(q5))*m.sin(q1)) - (J1z + J2z)*m.sin(q1) + (J2y + J3y)*m.sin(q2)*m.cos(q1) 
    z_q2 = J2x*m.sin(q1)*m.sin(q2) + J3x*m.sin(q1)*m.sin(q2)*m.cos(q3) + J3z*m.sin(q1)*m.sin(q2)*m.sin(q3) - J4x*((m.sin(q3)*m.sin(q5)*m.cos(q4) - m.cos(q3)*m.cos(q5))*m.sin(q2) - m.sin(q4)*m.sin(q5)*m.cos(q2))*m.sin(q1) + J4y*(m.sin(q2)*m.sin(q3)*m.sin(q4) + m.cos(q2)*m.cos(q4))*m.sin(q1) - J4z*(-(m.sin(q3)*m.cos(q4)*m.cos(q5) + m.sin(q5)*m.cos(q3))*m.sin(q2) + m.sin(q4)*m.cos(q2)*m.cos(q5))*m.sin(q1) + (J2y + J3y)*m.sin(q1)*m.cos(q2) 
    z_q3 = -J3x*(-m.sin(q1)*m.sin(q3)*m.cos(q2) + m.cos(q1)*m.cos(q3)) + J3z*(-m.sin(q1)*m.cos(q2)*m.cos(q3) - m.sin(q3)*m.cos(q1)) - J4x*(-(m.sin(q3)*m.cos(q5) + m.sin(q5)*m.cos(q3)*m.cos(q4))*m.sin(q1)*m.cos(q2) + (-m.sin(q3)*m.sin(q5)*m.cos(q4) + m.cos(q3)*m.cos(q5))*m.cos(q1)) + J4y*(-m.sin(q1)*m.sin(q4)*m.cos(q2)*m.cos(q3) - m.sin(q3)*m.sin(q4)*m.cos(q1)) - J4z*((-m.sin(q3)*m.sin(q5) + m.cos(q3)*m.cos(q4)*m.cos(q5))*m.sin(q1)*m.cos(q2) + (m.sin(q3)*m.cos(q4)*m.cos(q5) + m.sin(q5)*m.cos(q3))*m.cos(q1)) 
    z_q4 = -J4x*((-m.sin(q2)*m.sin(q5)*m.cos(q4) + m.sin(q3)*m.sin(q4)*m.sin(q5)*m.cos(q2))*m.sin(q1) - m.sin(q4)*m.sin(q5)*m.cos(q1)*m.cos(q3)) + J4y*((-m.sin(q2)*m.sin(q4) - m.sin(q3)*m.cos(q2)*m.cos(q4))*m.sin(q1) + m.cos(q1)*m.cos(q3)*m.cos(q4)) - J4z*((m.sin(q2)*m.cos(q4)*m.cos(q5) - m.sin(q3)*m.sin(q4)*m.cos(q2)*m.cos(q5))*m.sin(q1) + m.sin(q4)*m.cos(q1)*m.cos(q3)*m.cos(q5)) 
    z_q5 = -J4x*((-(m.sin(q3)*m.cos(q4)*m.cos(q5) + m.sin(q5)*m.cos(q3))*m.cos(q2) - m.sin(q2)*m.sin(q4)*m.cos(q5))*m.sin(q1) + (-m.sin(q3)*m.sin(q5) + m.cos(q3)*m.cos(q4)*m.cos(q5))*m.cos(q1)) - J4z*(((-m.sin(q3)*m.sin(q5)*m.cos(q4) + m.cos(q3)*m.cos(q5))*m.cos(q2) - m.sin(q2)*m.sin(q4)*m.sin(q5))*m.sin(q1) + (m.sin(q3)*m.cos(q5) + m.sin(q5)*m.cos(q3)*m.cos(q4))*m.cos(q1)) 

    a_q1 = 0
    a_q2 = 0
    a_q3 = 0
    a_q4 = 0
    a_q5 = 0

    b_q1 = 0
    b_q2 = 0
    b_q3 = 0
    b_q4 = 0
    b_q5 = 0

    #rospy.logwarn("X = %f, %f, %f, %f, %f", x_q1, x_q2, x_q3, x_q4, x_q5)
    #rospy.logwarn("Y = %f, %f, %f, %f, %f", y_q1, y_q2, y_q3, y_q4, y_q5)
    #rospy.logwarn("Z = %f, %f, %f, %f, %f", z_q1, z_q2, z_q3, z_q4, z_q5)

    # Create matrix
    j = np.zeros((5, 3), dtype=float)

    j[0][0] = x_q1
    j[0][1] = x_q2
    j[0][2] = x_q3
    j[0][3] = x_q4
    j[0][4] = x_q5

    j[1][0] = y_q1
    j[1][1] = y_q2
    j[1][2] = y_q3
    j[1][3] = y_q4
    j[1][4] = y_q5

    j[2][0] = z_q1
    j[2][1] = z_q2
    j[2][2] = z_q3
    j[2][3] = z_q4
    j[2][4] = z_q5

    #j[3][0] = a_q1
    #j[3][1] = a_q2
    #j[3][2] = a_q3
    #j[3][3] = a_q4
    #j[3][4] = a_q5

    #j[4][0] = b_q1
    #j[4][1] = b_q2
    #j[4][2] = b_q3
    #j[4][3] = b_q4
    #j[4][4] = b_q5

    return j

def handle_inv_kin_calc(req):

    resp = InverseKinematicCalcResponse()

    command = np.array([0.0] * 5).T

    try:
        # Angles initiaux
        q1 = angle_rad(req.angles[0])
        q2 = angle_rad(req.angles[1])
        q3 = angle_rad(req.angles[2])
        q4 = angle_rad(req.angles[3])
        q5 = angle_rad(req.angles[4])

        # Commande
        command[0] = req.commands[0]
        command[1] = req.commands[1]
        command[2] = req.commands[2]
        #command[3] = req.commands[3]
        #command[4] = req.commands[4]

        #rospy.logwarn("Angles = %f, %f, %f, %f, %f", q1, q2, q3, q4, q5)
        #rospy.logwarn("Commande = %f, %f, %f, %f, %f", command[0], command[1], command[2], command[3], command[4])

        j = build_jacbienne(q1, q2, q3, q4, q5)

        rospy.logwarn("Jacobienne :")   
        rospy.logwarn(j[0])
        rospy.logwarn(j[1])
        rospy.logwarn(j[2])
        #rospy.logwarn(j[3])
        #rospy.logwarn(j[4])
        rospy.logwarn("Déterminant de la jacobienne = %f", det(j))

        if det(j) != 0:
            resp.velocities = np.matmul(inv(j), command)
            resp.singularMatrix = 0
        else:
            rospy.logwarn("Can't create inverse matrix, jacobian determinant is %f", det(j))
            rospy.logwarn("Jog in joint to a less critical position")
            resp.velocities = [0, 0, 0, 0, 0]
            resp.singularMatrix = 1
        
    except:
        rospy.logwarn("Singular matrix : encountered a column of 0 (can't divide by 0)")
        rospy.logwarn("Jog in joint to a less critical position")
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
    #rospy.logwarn("Service initialized")
    rospy.spin()

if __name__=='__main__':
    inv_kin_calc_service()