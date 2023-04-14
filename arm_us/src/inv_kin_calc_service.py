#!/usr/bin/env python

import rospy
import numpy as np
from numpy.linalg import inv, det
from arm_us_msg.srv import InverseKinematicCalc, InverseKinematicCalcResponse
import math as m
from math import pi as PI

def angle_deg(angle : float):
    """Takes an angle in radians and returns it in degrees

    Args:
        angle (float): Angle in radians

    Returns:
        float: Angle in degrees
    """
    return angle * 180 / PI

def angle_rad(angle : float):
    """Takes an angle in degrees and returns it in radians

    Args:
        angle (float): Angle in degrees

    Returns:
        float: Angle in radians
    """
    return angle * PI / 180

def handle_inv_kin_calc(req):
    """ROS service that calculates the inverse kinematic equations needed to find the joint velocities
    needed to make a movement in cartesian coordinates

    Args:
        req (InverseKinematicCalcRequest): The request sent by the service's client
        Contains an array of 3 float64s representing the current angles, req.angles, and
        an array of 3 floats64s representing the cartesian movement wanted (X, Y, Z), req.commands

    Returns:
        resp (InverseKinematicCalcResponse): The response calculated by the service, which is the joint velocities
        to do the movement requested in req.commands
        Contains an array of 3 float64s representing the joint velocities of the first 3 robot joints, resp.velocities, and
        a boolean that expresses if a singular matrix was encountered
    """

    # Create the service's response
    resp = InverseKinematicCalcResponse()

    # Current angles of the arm, changed from degrees to radians
    q1 = angle_rad(req.angles[0])
    q2 = angle_rad(req.angles[1])
    q3 = angle_rad(req.angles[2])

    # Cartesian command transposed in a Numpy array
    command = np.array([req.commands[0], req.commands[1], req.commands[2]]).T

    # If all commands are 0.0, return
    if (command[0] == 0.0 and command[1] == 0.0 and command[2] == 0.0):
        resp.velocities = [0.0, 0.0, 0.0]
        resp.singularMatrix = 0
        return resp

    # Physical dimensions of the arm, used in the inverse kinematic equations
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

    # Initialization of the Jacobian matrix
    # Those are the partial derivatives of the direct kinematic equations of the arm for the first 3 joints
    x_q1 = J1z*m.cos(q1) + J2y*m.sin(q2)*m.cos(q1) + J4z*m.cos(q1)*m.cos(q2) - (J1x + J2x)*m.sin(q1) + (J2z + J3z)*m.cos(q1)*m.cos(q2) + (J3x + J4x)*(-m.sin(q1)*m.cos(q3) + m.sin(q2)*m.sin(q3)*m.cos(q1)) - (J3y + J4y)*(-m.sin(q1)*m.sin(q3) - m.sin(q2)*m.cos(q1)*m.cos(q3))
    x_q2 = J2y*m.sin(q1)*m.cos(q2) - J4z*m.sin(q1)*m.sin(q2) - (J2z + J3z)*m.sin(q1)*m.sin(q2) + (J3x + J4x)*m.sin(q1)*m.sin(q3)*m.cos(q2) + (J3y + J4y)*m.sin(q1)*m.cos(q2)*m.cos(q3)
    x_q3 = (J3x + J4x)*(m.sin(q1)*m.sin(q2)*m.cos(q3) - m.sin(q3)*m.cos(q1)) - (J3y + J4y)*(m.sin(q1)*m.sin(q2)*m.sin(q3) + m.cos(q1)*m.cos(q3))

    y_q1 = 0
    y_q2 = -J2y*m.sin(q2) - J4z*m.cos(q2) - (J2z + J3z)*m.cos(q2) - (J3x + J4x)*m.sin(q2)*m.sin(q3) - (J3y + J4y)*m.sin(q2)*m.cos(q3)
    y_q3 = (J3x + J4x)*m.cos(q2)*m.cos(q3) - (J3y + J4y)*m.sin(q3)*m.cos(q2)

    z_q1 = -J1z*m.sin(q1) - J2y*m.sin(q1)*m.sin(q2) - J4z*m.sin(q1)*m.cos(q2) - (J1x + J2x)*m.cos(q1) - (J2z + J3z)*m.sin(q1)*m.cos(q2) - (J3x + J4x)*(m.sin(q1)*m.sin(q2)*m.sin(q3) + m.cos(q1)*m.cos(q3)) + (J3y + J4y)*(-m.sin(q1)*m.sin(q2)*m.cos(q3) + m.sin(q3)*m.cos(q1))
    z_q2 = J2y*m.cos(q1)*m.cos(q2) - J4z*m.sin(q2)*m.cos(q1) - (J2z + J3z)*m.sin(q2)*m.cos(q1) + (J3x + J4x)*m.sin(q3)*m.cos(q1)*m.cos(q2) + (J3y + J4y)*m.cos(q1)*m.cos(q2)*m.cos(q3)
    z_q3 = -(J3x + J4x)*(-m.sin(q1)*m.sin(q3) - m.sin(q2)*m.cos(q1)*m.cos(q3)) + (J3y + J4y)*(m.sin(q1)*m.cos(q3) - m.sin(q2)*m.sin(q3)*m.cos(q1))
    
    # Create Jacobian matrix
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

    try:
        # If the Jacobian's determinant is not 0
        if det(j) != 0:

            # Multiply the inverse of the Jacobian matrix with the command matrix
            resp.velocities = np.matmul(inv(j), command)

            # Multiply the output by demultiplication factors because of the gear ratios of the assembly
            J1_demultpilcatin_factor = 45/14
            J2_demultpilcatin_factor = (30/14) / 2
            J3_demultpilcatin_factor = 32/24

            resp.velocities[0] *= J1_demultpilcatin_factor
            resp.velocities[1] *= J2_demultpilcatin_factor
            resp.velocities[2] *= J3_demultpilcatin_factor
            resp.singularMatrix = 0
        else:
            # rospy.logerr("Can't create inverse matrix, jacobian determinant is %f", det(j))
            # rospy.logerr("Jog in joint to a less critical position")
            resp.velocities = [0.0, 0.0, 0.0]
            resp.singularMatrix = 1
    
    # A singular matrix was encountered, cannot calculate the joint velocities
    # Return singularMatrix = 1
    except:
        # rospy.logerr("Singular matrix : encountered a column of 0 (can't divide by 0)")
        # rospy.logerr("Jog in joint to a less critical position")
        resp.velocities = [0.0, 0.0, 0.0]
        resp.singularMatrix = 1
        
    return resp

def inv_kin_calc_service():
    rospy.init_node("inv_kin_calc_service", anonymous=True, log_level=rospy.INFO)
    s = rospy.Service('inverse_kinematic_calc_service', InverseKinematicCalc, handle_inv_kin_calc)
    rospy.spin()

if __name__=='__main__':
    inv_kin_calc_service()
