#!/usr/bin/env python

import rospy
import numpy as np
from numpy.linalg import inv, det
from arm_us.srv import InverseKinematicCalc, InverseKinematicCalcResponse
import math as m
from math import pi as PI

def angle_deg(angle : float):
    return angle * 180 / PI

def angle_rad(angle : float):
    return angle * PI / 180

def handle_inv_kin_calc(req):

    resp = InverseKinematicCalcResponse()

    # Angles initiaux
    q1 = angle_rad(req.angles[0])
    q2 = angle_rad(req.angles[1])
    q3 = angle_rad(req.angles[2])
    q4 = angle_rad(req.angles[3])

    # Commande
    command = np.array([req.commands[0], req.commands[1], req.commands[2], req.commands[3]]).T

    rospy.loginfo("--------------------------------------------------")
    rospy.loginfo("Angles: %f, %f, %f, %f", q1, q2, q3, q4)
    rospy.loginfo("Commandes: {}".format(command))

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
    x_q1 = -J1x*m.sin(q1) - J2x*m.sin(q1)*m.cos(q2) - J3x*(m.sin(q1)*m.cos(q2)*m.cos(q3) + m.sin(q3)*m.cos(q1)) + J3z*(-m.sin(q1)*m.sin(q3)*m.cos(q2) + m.cos(q1)*m.cos(q3)) - J4x*(m.sin(q1)*m.cos(q2)*m.cos(q3 + q4) + m.sin(q3 + q4)*m.cos(q1)) + J4y*m.sin(q1)*m.sin(q2) + J4z*(-m.sin(q1)*m.sin(q3 + q4)*m.cos(q2) + m.cos(q1)*m.cos(q3 + q4)) + (J1z + J2z)*m.cos(q1) - (-J2y - J3y)*m.sin(q1)*m.sin(q2) 
    x_q2 = -J2x*m.sin(q2)*m.cos(q1) - J3x*m.sin(q2)*m.cos(q1)*m.cos(q3) - J3z*m.sin(q2)*m.sin(q3)*m.cos(q1) - J4x*m.sin(q2)*m.cos(q1)*m.cos(q3 + q4) - J4y*m.cos(q1)*m.cos(q2) - J4z*m.sin(q2)*m.sin(q3 + q4)*m.cos(q1) + (-J2y - J3y)*m.cos(q1)*m.cos(q2) 
    x_q3 = -J3x*(m.sin(q1)*m.cos(q3) + m.sin(q3)*m.cos(q1)*m.cos(q2)) + J3z*(-m.sin(q1)*m.sin(q3) + m.cos(q1)*m.cos(q2)*m.cos(q3)) - J4x*(m.sin(q1)*m.cos(q3 + q4) + m.sin(q3 + q4)*m.cos(q1)*m.cos(q2)) + J4z*(-m.sin(q1)*m.sin(q3 + q4) + m.cos(q1)*m.cos(q2)*m.cos(q3 + q4)) 
    x_q4 = -J4x*(m.sin(q1)*m.cos(q3 + q4) + m.sin(q3 + q4)*m.cos(q1)*m.cos(q2)) + J4z*(-m.sin(q1)*m.sin(q3 + q4) + m.cos(q1)*m.cos(q2)*m.cos(q3 + q4)) 

    y_q1 = 0 
    y_q2 = J2x*m.cos(q2) + J3x*m.cos(q2)*m.cos(q3) + J3z*m.sin(q3)*m.cos(q2) + J4x*m.cos(q2)*m.cos(q3 + q4) - J4y*m.sin(q2) + J4z*m.sin(q3 + q4)*m.cos(q2) - (J2y + J3y)*m.sin(q2) 
    y_q3 = -J3x*m.sin(q2)*m.sin(q3) + J3z*m.sin(q2)*m.cos(q3) - J4x*m.sin(q2)*m.sin(q3 + q4) + J4z*m.sin(q2)*m.cos(q3 + q4) 
    y_q4 = -J4x*m.sin(q2)*m.sin(q3 + q4) + J4z*m.sin(q2)*m.cos(q3 + q4) 

    z_q1 = -J1x*m.cos(q1) - J2x*m.cos(q1)*m.cos(q2) - J3x*(-m.sin(q1)*m.sin(q3) + m.cos(q1)*m.cos(q2)*m.cos(q3)) + J3z*(-m.sin(q1)*m.cos(q3) - m.sin(q3)*m.cos(q1)*m.cos(q2)) - J4x*(-m.sin(q1)*m.sin(q3 + q4) + m.cos(q1)*m.cos(q2)*m.cos(q3 + q4)) + J4y*m.sin(q2)*m.cos(q1) + J4z*(-m.sin(q1)*m.cos(q3 + q4) - m.sin(q3 + q4)*m.cos(q1)*m.cos(q2)) - (J1z + J2z)*m.sin(q1) + (J2y + J3y)*m.sin(q2)*m.cos(q1) 
    z_q2 = J2x*m.sin(q1)*m.sin(q2) + J3x*m.sin(q1)*m.sin(q2)*m.cos(q3) + J3z*m.sin(q1)*m.sin(q2)*m.sin(q3) + J4x*m.sin(q1)*m.sin(q2)*m.cos(q3 + q4) + J4y*m.sin(q1)*m.cos(q2) + J4z*m.sin(q1)*m.sin(q2)*m.sin(q3 + q4) + (J2y + J3y)*m.sin(q1)*m.cos(q2) 
    z_q3 = -J3x*(-m.sin(q1)*m.sin(q3)*m.cos(q2) + m.cos(q1)*m.cos(q3)) + J3z*(-m.sin(q1)*m.cos(q2)*m.cos(q3) - m.sin(q3)*m.cos(q1)) - J4x*(-m.sin(q1)*m.sin(q3 + q4)*m.cos(q2) + m.cos(q1)*m.cos(q3 + q4)) + J4z*(-m.sin(q1)*m.cos(q2)*m.cos(q3 + q4) - m.sin(q3 + q4)*m.cos(q1))
    z_q4 = -J4x*(-m.sin(q1)*m.sin(q3 + q4)*m.cos(q2) + m.cos(q1)*m.cos(q3 + q4)) + J4z*(-m.sin(q1)*m.cos(q2)*m.cos(q3 + q4) - m.sin(q3 + q4)*m.cos(q1)) 

    a_q1 = 0.1 #((-m.sin(q1)*m.sin(q3)*m.cos(q2) + m.cos(q1)*m.cos(q3))*m.sin(q4) + (m.sin(q1)*m.cos(q2)*m.cos(q3) + m.sin(q3)*m.cos(q1))*m.cos(q4))/m.sqrt(1 - ((m.sin(q1)*m.sin(q3) - m.cos(q1)*m.cos(q2)*m.cos(q3))*m.cos(q4) + (m.sin(q1)*m.cos(q3) + m.sin(q3)*m.cos(q1)*m.cos(q2))*m.sin(q4))**2)
    a_q2 = 0.1 #(-m.sin(q2)*m.sin(q3)*m.sin(q4)*m.cos(q1) + m.sin(q2)*m.cos(q1)*m.cos(q3)*m.cos(q4))/m.sqrt(1 - ((m.sin(q1)*m.sin(q3) - m.cos(q1)*m.cos(q2)*m.cos(q3))*m.cos(q4) + (m.sin(q1)*m.cos(q3) + m.sin(q3)*m.cos(q1)*m.cos(q2))*m.sin(q4))**2)
    a_q3 = 0.1 #((-m.sin(q1)*m.sin(q3) + m.cos(q1)*m.cos(q2)*m.cos(q3))*m.sin(q4) + (m.sin(q1)*m.cos(q3) + m.sin(q3)*m.cos(q1)*m.cos(q2))*m.cos(q4))/m.sqrt(1 - ((m.sin(q1)*m.sin(q3) - m.cos(q1)*m.cos(q2)*m.cos(q3))*m.cos(q4) + (m.sin(q1)*m.cos(q3) + m.sin(q3)*m.cos(q1)*m.cos(q2))*m.sin(q4))**2)
    a_q4 = 0.1 #(-(m.sin(q1)*m.sin(q3) - m.cos(q1)*m.cos(q2)*m.cos(q3))*m.sin(q4) + (m.sin(q1)*m.cos(q3) + m.sin(q3)*m.cos(q1)*m.cos(q2))*m.cos(q4))/m.sqrt(1 - ((m.sin(q1)*m.sin(q3) - m.cos(q1)*m.cos(q2)*m.cos(q3))*m.cos(q4) + (m.sin(q1)*m.cos(q3) + m.sin(q3)*m.cos(q1)*m.cos(q2))*m.sin(q4))**2)

    # Create matrix
    j = np.zeros((4, 4), dtype=float)

    j[0][0] = x_q1
    j[0][1] = x_q2
    j[0][2] = x_q3
    j[0][3] = x_q4

    j[1][0] = y_q1
    j[1][1] = y_q2
    j[1][2] = y_q3
    j[1][3] = y_q4

    j[2][0] = z_q1
    j[2][1] = z_q2
    j[2][2] = z_q3
    j[2][3] = z_q4

    j[3][0] = a_q1
    j[3][1] = a_q2
    j[3][2] = a_q3
    j[3][3] = a_q4

    # rospy.loginfo("Jacobienne")
    # rospy.loginfo(j[0])
    # rospy.loginfo(j[1])
    # rospy.loginfo(j[2])
    # rospy.loginfo(j[3])

    # rospy.loginfo("Déterminant de la jacobienne = %f", det(j))

    try:
        if det(j) != 0:
            resp.velocities = np.matmul(inv(j), command)
            resp.singularMatrix = 0
        else:
            rospy.logerr("Can't create inverse matrix, jacobian determinant is %f", det(j))
            # rospy.logerr("Jog in joint to a less critical position")
            resp.velocities = [0, 0, 0, 0]
            resp.singularMatrix = 1
        
    except:
        rospy.logerr("Singular matrix : encountered a column of 0 (can't divide by 0)")
        # rospy.logerr("Jog in joint to a less critical position")
        resp.velocities = [0, 0, 0, 0]
        resp.singularMatrix = 1

    rospy.loginfo("-------------------------")
    rospy.loginfo("Velocities: {}".format(resp.velocities))
    rospy.loginfo("Singular Matrix: {}".format(resp.singularMatrix))
    rospy.loginfo("--------------------------------------------------")

    return resp

def inv_kin_calc_service():
    rospy.init_node("inv_kin_calc_service", anonymous=True, log_level=rospy.INFO)
    s = rospy.Service('inverse_kinematic_calc_service', InverseKinematicCalc, handle_inv_kin_calc)
    rospy.loginfo("Service initialized")
    rospy.spin()

if __name__=='__main__':
    inv_kin_calc_service()