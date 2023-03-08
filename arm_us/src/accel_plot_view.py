#!/usr/bin/env python

import numpy as np
import rospy
import matplotlib as mpl
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style
from mpl_toolkits import mplot3d
from std_msgs.msg import Int16MultiArray

pi = 3.1416

style.use('default')
ARM_COLOR = 'b'
JOINTCOLOR = 'o'
mpl.rcParams['lines.linewidth'] = 1

ax, ay, az = 0, 0, 0


def draw_loop():
    
    plt.ion()
    fig = plt.figure(figsize=(8, 8))
    ax_3d = fig.add_subplot(1, 1, 1, projection='3d')
    
    while (not rospy.is_shutdown()):
        xs = []
        ys = []
        zs = []

        xs, ys, zs = calculateCoord()

        ax_3d.clear()
        ax_3d.set_title("3D view")
        ax_3d.plot3D(xs, ys, zs)
        ax_3d.scatter(xs, ys, zs)
        ax_3d.set_xlabel('z')
        ax_3d.set_ylabel('x')
        ax_3d.set_zlabel('y')
        ax_3d.set_xlim3d(-1.5, 1.5)
        ax_3d.set_ylim3d(-1.5, 1.5)
        ax_3d.set_zlim3d(-1.5, 1.5)
        ax_3d.grid(True)
        plt.draw()
        plt.pause(.01)

    rospy.spin()
    
def calculateCoord():
    #TODO Make simple cinematic calculation
    return [0, np.cos(map(ax, -16536, 16536, -pi/2, pi/2))], [0, np.cos(map(ay, -16536, 16536, -pi/2, pi/2))], [0, np.cos(map(az, -16536, 16536, -pi/2, pi/2))]

def sub_accel_angle_callback(data: Int16MultiArray):
    global ax, ay, az
    ax, ay, az = data.data

def map(val, min_input, max_input, min_val, max_val):
    return ((val/(max_input - min_input)) * (max_val - min_val)) + min_val

if __name__ == '__main__':
    rospy.init_node('accel_plot_view')
    sub_accel_angle = rospy.Subscriber('accel_pos', Int16MultiArray, sub_accel_angle_callback)
    draw_loop()
