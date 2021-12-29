#!/usr/bin/env python3
"""
Moving the articulated tractor-trailer system.
Author: Guido Sanchez
Version: 17/08/2021
"""

import rospy
import math
import numpy as np
import matplotlib.pyplot as plt
import sys
import time
import trajectory_generation as trajectory_generation

from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float64

from export_articulated_tractor_model import *

# Global
REAR_P3D = None
TRAILER_P3D = None
FRONT_P3D = None
FR_STEER_ANGLE = None
FL_STEER_ANGLE = None
ARTICULATION_ANGLE = None

def toEulerAngle(odom_msg):
    qx = odom_msg.pose.pose.orientation.x
    qy = odom_msg.pose.pose.orientation.y
    qz = odom_msg.pose.pose.orientation.z
    qw = odom_msg.pose.pose.orientation.w

    ysqr = qy * qy
    t0 = -2.0 * (ysqr + qz * qz) + 1.0
    t1 = +2.0 * (qx * qy - qw * qz)
    t2 = -2.0 * (qx * qz + qw * qy)
    t3 = +2.0 * (qy * qz - qw * qx)
    t4 = -2.0 * (qx * qx + ysqr) + 1.0

    t2 = 1.0 if t2>1.0 else t2
    t2 = -1.0 if t2<-1.0 else t2

    pitch = -math.asin(t2)
    roll = -math.atan2(t3, t4)
    yaw = -math.atan2(t1, t0)
    return (roll, pitch, yaw)

def rear_p3d_callback(odom_msg):
    # Save a global reference to the most recent sensor state so that
    # it can be accessed in the main control loop.
    # (The global keyword prevents the creation of a local variable here.)
    global REAR_P3D
    REAR_P3D = odom_msg

def trailer_p3d_callback(odom_msg):
    global TRAILER_P3D
    TRAILER_P3D = odom_msg

def front_p3d_callback(odom_msg):
    global FRONT_P3D
    FRONT_P3D = odom_msg

def articulation_callback(pos):
    global ARTICULATION_ANGLE
    ARTICULATION_ANGLE = pos

def fla_callback(pos):
    global FL_STEER_ANGLE
    FL_STEER_ANGLE = pos

def fra_callback(pos):
    global FR_STEER_ANGLE
    FR_STEER_ANGLE = pos

def magical_unwrap(yaw, yaw_prev):
    yaw += (2*np.pi) * np.ceil(((yaw_prev - yaw) // np.pi) / 2.)
    return yaw

def linearVeltoAngular(lin_vel, wheel_radius):
    return float(lin_vel)/float(wheel_radius)

# This is the 'main'
def start():
    rear_prev_yaw = 0.0
    rear_curr_yaw = 0.0
    front_prev_yaw = 0.0
    front_curr_yaw = 0.0
    trailer_prev_yaw = 0.0
    trailer_curr_yaw = 0.0
    steering_left_angle = 0.0
    steering_right_angle = 0.0
    articulation_angle = 0.0

    wheel_radius = 0.2

    # Turn this into an official ROS node named approach
    rospy.init_node('tractor_genzelis_mpctools_trajectory', anonymous=True)

    # Subscribe to the /scan topic.  From now on
    # scan_callback will be called every time a new scan message is
    # published.
    rospy.Subscriber("/ground_truth/rear_axle_state", Odometry, rear_p3d_callback)
    rospy.Subscriber("/ground_truth/front_axle_state", Odometry, front_p3d_callback)
    rospy.Subscriber("/ground_truth/trailer_axle_state", Odometry, trailer_p3d_callback)

    rospy.Subscriber("/base_link__front_cradle_joint_position", Float64, articulation_callback)
    rospy.Subscriber("/front_left_ackermann_steering_joint_position", Float64, fla_callback)
    rospy.Subscriber("/front_right_ackermann_steering_joint_position", Float64, fra_callback)

    vel_pub = rospy.Publisher('/velocity_joint_controller/command', Float64MultiArray, queue_size=10)
    steering_pub = rospy.Publisher('/ackerman_steering_joint_controller/command', Float64MultiArray, queue_size=10)
    articulation_pub = rospy.Publisher('/articulation_joint_controller/command', Float64, queue_size=10)

    current_trajectory = Odometry()
    current_trajectory_pub = rospy.Publisher("/nmpc_trajectory", Odometry, queue_size=0)

    curr_state_pub = rospy.Publisher('/current_state', Float64MultiArray, queue_size=10)
    curr_setpoint_pub = rospy.Publisher('/current_setpoint', Float64MultiArray, queue_size=10)
    curr_control_pub = rospy.Publisher('/current_control', Float64MultiArray, queue_size=10)
    curr_trailer_pub = rospy.Publisher('/current_trailer', Float64MultiArray, queue_size=10)

# Wait until the first odometry message is available.
    while ((REAR_P3D is None) or (TRAILER_P3D is None) or (FRONT_P3D is None) or (ARTICULATION_ANGLE is None) or (FL_STEER_ANGLE is None) or (FR_STEER_ANGLE is None)) and not rospy.is_shutdown():
        rospy.sleep(.1)

    (_, _, rear_prev_yaw) = toEulerAngle(REAR_P3D)
    (_, _, front_prev_yaw) = toEulerAngle(FRONT_P3D)
    (_, _, trailer_prev_yaw) = toEulerAngle(TRAILER_P3D)

    # Rate object used to make the main loop execute at 10hz.
    rate = rospy.Rate(10)

    (controller, Nt, Delta) = export_articulated_tractor_model()

    # Trajectory related stuff
    # generate xy trajectory
    tray_xy_sp_ = trajectory_generation.gen_tray_arado_horizontal(Delta)
    tray_xy_tail = np.tile(tray_xy_sp_[-1, :], (2 * Nt + 1, 1))  # add the N points of the prediction horizon
    tray_xy_sp = np.concatenate((tray_xy_sp_, tray_xy_tail), axis=0)

    tray_dif = np.diff(tray_xy_sp, axis=0)
    tray_dif = np.concatenate((tray_dif, np.array([tray_dif[-1]])), axis=0)
    yaw = np.arctan2(tray_dif[:, 1],tray_dif[:, 0])

    x_ref = np.array([10, 10, 0])
    t = 0

    vel_cmd = Float64MultiArray()
    articulation_cmd = Float64()
    steering_cmd = Float64MultiArray()

    vel_cmd.data = [0, 0, 0, 0]
    articulation_cmd.data = 0
    steering_cmd.data = [0, 0]

    vel_pub.publish(vel_cmd)
    articulation_pub.publish(articulation_cmd)
    steering_pub.publish(steering_cmd)

    Nsim = tray_xy_sp.shape[0] - Nt

    # set to zero uprev before the loop
    controller.par["u_prev",0] = np.array([0, 0, 0])


    while not rospy.is_shutdown() and t<Nsim:
        (_,_,rear_curr_yaw) = toEulerAngle(REAR_P3D)
        rear_curr_yaw = magical_unwrap(rear_curr_yaw, rear_prev_yaw)
        rear_prev_yaw = rear_curr_yaw

        (_,_,trailer_curr_yaw) = toEulerAngle(TRAILER_P3D)
        trailer_curr_yaw = magical_unwrap(trailer_curr_yaw, trailer_prev_yaw)
        trailer_prev_yaw = trailer_curr_yaw

        (_, _, front_curr_yaw) = toEulerAngle(FRONT_P3D)
        front_curr_yaw = magical_unwrap(front_curr_yaw, front_prev_yaw)
        front_prev_yaw = front_curr_yaw

        steering_left_angle = FL_STEER_ANGLE.data
        steering_right_angle = FR_STEER_ANGLE.data
        articulation_angle = ARTICULATION_ANGLE.data

        rear_curr_x = REAR_P3D.pose.pose.position.x
        rear_curr_y = REAR_P3D.pose.pose.position.y

        trailer_curr_x = TRAILER_P3D.pose.pose.position.x
        trailer_curr_y = TRAILER_P3D.pose.pose.position.y

        front_curr_x = FRONT_P3D.pose.pose.position.x
        front_curr_y = FRONT_P3D.pose.pose.position.y

        """ [xf, yf, theta_r, gamma, theta_r - theta_t, phi] """
        _xk = np.array([front_curr_x,
                        front_curr_y,
                        rear_curr_yaw,
                        articulation_angle,
                        rear_curr_yaw - trailer_curr_yaw,
                        steering_left_angle])

        for i in range(Nt + 1):
            controller.par["x_sp", i, 0] = tray_xy_sp[t + i, 0]
            controller.par["x_sp", i, 1] = tray_xy_sp[t + i, 1]
            controller.par["x_sp", i, 2] = yaw[t + i]

        controller.fixvar("x", 0, _xk)
        solvetime = -time.time()
        controller.solve()
        solvetime += time.time()

        # Print status and make sure solver didn't fail.
        print("%d: %s" % (t,controller.stats["status"]))
        if controller.stats["status"] != "Solve_Succeeded":
            break
        else:
            rospy.loginfo("Current NMPC step took %.4f s" % (solvetime))
            controller.saveguess()
        _uk = np.squeeze(controller.var["u",0])
        _xk_sig = np.squeeze(controller.var["x",1])

        ang_vel = linearVeltoAngular(_uk[0], wheel_radius)

        print("Rear position: [%.5f, %.5f, %.5f]" % (rear_curr_x, rear_curr_y, rear_curr_yaw))
        print("Articulation: %.5f" % (articulation_angle))
        print("Steering LR [%.5f %.5f]" % (steering_left_angle, steering_right_angle))
        print("Sending [%.5f %.5f %.5f]" % (ang_vel, _xk_sig[3], _xk_sig[5]))

        vel_cmd.data = [ang_vel, ang_vel, ang_vel, ang_vel]
        steering_cmd.data = [_xk_sig[5], _xk_sig[5]]
        articulation_cmd.data = _xk_sig[3]
        vel_pub.publish(vel_cmd)
        steering_pub.publish(steering_cmd)
        articulation_pub.publish(articulation_cmd)

        current_trajectory.pose.pose.position.x = tray_xy_sp[t, 0]
        current_trajectory.pose.pose.position.y = tray_xy_sp[t, 1]

        curr_state_msg = Float64MultiArray()
        curr_state_msg.data = _xk
        curr_state_pub.publish(curr_state_msg)
        curr_setpoint_msg = Float64MultiArray()
        curr_setpoint_msg.data = np.concatenate((tray_xy_sp[t], np.array([yaw[t]]))).ravel()
        curr_setpoint_pub.publish(curr_setpoint_msg)
        curr_control_msg = Float64MultiArray()
        curr_control_msg.data = _uk
        curr_control_pub.publish(curr_control_msg)
        current_trailer_msg = Float64MultiArray()
        current_trailer_msg.data = [trailer_curr_x, trailer_curr_y]
        curr_trailer_pub.publish(current_trailer_msg)

        current_trajectory_pub.publish(current_trajectory)
        # update uprev
        controller.par["u_prev", 0] = _uk
        t+=1

        rate.sleep()           # Pause long enough to maintain correct rate.

    print("Finished after %d iterations." % (t))
    vel_cmd.data = [0, 0, 0, 0]  # Freno
    vel_pub.publish(vel_cmd)


if __name__ == "__main__":
    start()
