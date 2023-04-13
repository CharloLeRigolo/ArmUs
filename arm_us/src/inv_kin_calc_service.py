#!/usr/bin/env python

import rospy
import numpy as np
from numpy.linalg import inv, det
from arm_us_msg.srv import InverseKinematicCalc, InverseKinematicCalcResponse
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

    # Commande
    command = np.array([req.commands[0], req.commands[1], req.commands[2]]).T

    if (command[0] == 0.0 and command[1] == 0.0 and command[2] == 0.0):
        resp.velocities = [0.0, 0.0, 0.0]
        resp.singularMatrix = 0
        return resp

    rospy.loginfo("--------------------------------------------------")
    rospy.loginfo("Angles: %f, %f, %f", q1, q2, q3)
    rospy.loginfo("Commandes: {}".format(command))

    # Grandeurs physiques
    J1x : float = 2.0548
    J1y : float = 0
    J1z : float = 0

    J2x : float = 0
    J2y : float = -2.25
    J2z : float = 0
    
    J3x : float = 0
    J3y : float = 1.73
    J3z : float = 0
    
    J4x : float = 0
    J4y : float = 0.40
    J4z : float = 0

    # Initialisation de la Jacobienne
    x_q1 = J1z*m.cos(q1) + J2y*m.sin(q2)*m.cos(q1) + J4z*m.cos(q1)*m.cos(q2) - (J1x + J2x)*m.sin(q1) + (J2z + J3z)*m.cos(q1)*m.cos(q2) + (J3x + J4x)*(-m.sin(q1)*m.cos(q3) + m.sin(q2)*m.sin(q3)*m.cos(q1)) - (J3y + J4y)*(-m.sin(q1)*m.sin(q3) - m.sin(q2)*m.cos(q1)*m.cos(q3))
    x_q2 = J2y*m.sin(q1)*m.cos(q2) - J4z*m.sin(q1)*m.sin(q2) - (J2z + J3z)*m.sin(q1)*m.sin(q2) + (J3x + J4x)*m.sin(q1)*m.sin(q3)*m.cos(q2) + (J3y + J4y)*m.sin(q1)*m.cos(q2)*m.cos(q3)
    x_q3 = (J3x + J4x)*(m.sin(q1)*m.sin(q2)*m.cos(q3) - m.sin(q3)*m.cos(q1)) - (J3y + J4y)*(m.sin(q1)*m.sin(q2)*m.sin(q3) + m.cos(q1)*m.cos(q3))

    y_q1 = 0
    y_q2 = -J2y*m.sin(q2) - J4z*m.cos(q2) - (J2z + J3z)*m.cos(q2) - (J3x + J4x)*m.sin(q2)*m.sin(q3) - (J3y + J4y)*m.sin(q2)*m.cos(q3)
    y_q3 = (J3x + J4x)*m.cos(q2)*m.cos(q3) - (J3y + J4y)*m.sin(q3)*m.cos(q2)

    z_q1 = -J1z*m.sin(q1) - J2y*m.sin(q1)*m.sin(q2) - J4z*m.sin(q1)*m.cos(q2) - (J1x + J2x)*m.cos(q1) - (J2z + J3z)*m.sin(q1)*m.cos(q2) - (J3x + J4x)*(m.sin(q1)*m.sin(q2)*m.sin(q3) + m.cos(q1)*m.cos(q3)) + (J3y + J4y)*(-m.sin(q1)*m.sin(q2)*m.cos(q3) + m.sin(q3)*m.cos(q1))
    z_q2 = J2y*m.cos(q1)*m.cos(q2) - J4z*m.sin(q2)*m.cos(q1) - (J2z + J3z)*m.sin(q2)*m.cos(q1) + (J3x + J4x)*m.sin(q3)*m.cos(q1)*m.cos(q2) + (J3y + J4y)*m.cos(q1)*m.cos(q2)*m.cos(q3)
    z_q3 = -(J3x + J4x)*(-m.sin(q1)*m.sin(q3) - m.sin(q2)*m.cos(q1)*m.cos(q3)) + (J3y + J4y)*(m.sin(q1)*m.cos(q3) - m.sin(q2)*m.sin(q3)*m.cos(q1))
    
    # Create matrix
    j = np.zeros((3, 3), dtype=float)

    j[0][0] = x_q1
    j[0][1] = x_q2
    j[0][2] = x_q3

    j[1][0] = y_q1
    j[1][1] = y_q2
    j[1][2] = y_q3

    j[2][0] = z_q1
    j[2][1] = z_q2
    j[2][2] = z_q3

    # rospy.loginfo("Jacobienne")
    # rospy.loginfo(j[0])
    # rospy.loginfo(j[1])
    # rospy.loginfo(j[2])

    # rospy.loginfo("Déterminant de la jacobienne = %f", det(j))

    try:
        if det(j) != 0:
            resp.velocities = np.matmul(inv(j), command)
            resp.velocities[0] *= (45/14)
            resp.velocities[1] *= (30/14)/2
            resp.velocities[2] *= 32/24
            resp.singularMatrix = 0
        else:
            rospy.logerr("Can't create inverse matrix, jacobian determinant is %f", det(j))
            # rospy.logerr("Jog in joint to a less critical position")
            resp.velocities = [0.0, 0.0, 0.0]
            resp.singularMatrix = 1
        
    except:
        rospy.logerr("Singular matrix : encountered a column of 0 (can't divide by 0)")
        # rospy.logerr("Jog in joint to a less critical position")
        resp.velocities = [0.0, 0.0, 0.0]
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
