#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Drone landing tools
"""

import rospy
from geometry_msgs.msg import PoseStamped ,Quaternion, TransformStamped, TwistStamped, Vector3, Point
from mavros_msgs.msg import State

import tf
import numpy as np

import matplotlib.pyplot as plt  # creation plots


marker_pose = Point()


# init msgs
goal_msgs = PoseStamped()
position_msgs = PoseStamped()
marker_msgs = PoseStamped()
goal_vel = TwistStamped()
current_vel = TwistStamped()

arm_state = True

goal_init = False
file_name = "test.txt";

# init lists
# Инициализируем списки для построения графиков
drone_x = list()
drone_y = list()
drone_z = list()
drone_w = list()

drone_vx = list()
drone_vy = list()
drone_vz = list()

goal_x = list()
goal_y = list()
goal_z = list()
goal_w = list()

marker_x = list()
marker_y = list()
marker_z = list()
marker_w = list()

ctr_x = list()
ctr_y = list()
ctr_z = list()

contol_error_xy = list()
contol_error_z = list()

marker_error = list()
marker_error_xy = list()
marker_error_z = list()

time_plot = list()

old_time = 0.
timer = 0.


"""
Tools methods
"""

def getDistPoint(pose_1, pose_2, use2d=False, getHeight = False):
    """
    Get dist between 2 Point
    :param pose_1: Vector 1 of Point()
    :param pose_2: Vector 2 of Point()
    :return: distance
    """
    if getHeight:
        return np.abs(pose_2.z-pose_1.z)

    if use2d:
        v1 = np.array([pose_1.x, pose_1.y])
        v2 = np.array([pose_2.x, pose_2.y])
    else:
        v1 = np.array([pose_1.x, pose_1.y, pose_1.z])
        v2 = np.array([pose_2.x, pose_2.y, pose_2.z])
    dist = np.linalg.norm(v2-v1)
    return dist

def getQuatToEuler(x, y, z, w):
    """
    Transform quaternion to euler angels
    :param x:
    :param y:
    :param z:
    :param w:
    :return: euler angels
    """
    # type(pose) = geometry_msgs.msg.Pose
    euler = tf.transformations.euler_from_quaternion((x,y,z,w))
    return euler

def getQuatToEuler(quat):
    """
    Transform quaternion to euler angels
    :param x:
    :param y:
    :param z:
    :param w:
    :return: euler angels
    """
    # type(pose) = geometry_msgs.msg.Pose
    euler = tf.transformations.euler_from_quaternion((quat.x,quat.y,quat.z,quat.w))
    return euler

def getEulerToQuat(roll=0., pitch=0., yaw = 0.):
    """
    Transform euler angels to quaternion
    :param roll:
    :param pitch:
    :param yaw:
    :return: quaternion
    """
    # type(pose) = geometry_msgs.msg.Pose
    q = tf.transformations.quaternion_from_euler(roll,pitch,yaw)
    quat = Quaternion()
    quat.x = q[0]
    quat.y = q[1]
    quat.z = q[2]
    quat.w = q[3]
    return quat

def print_delay(msgs, delay = 1.0):
    """
    print with delay
    :param msgs:
    :param delay:
    :return:
    """
    global print_timer
    if print_timer > delay:
        print msgs
        print_timer = 0.

"""
 ROS methods
"""

def update_list():
    global timer, old_time

    if old_time == 0. or goal_init is False:
        old_time = position_msgs.header.stamp.to_sec()
        return

    # Рассчитываем время
    dt = position_msgs.header.stamp.to_sec() - old_time
    old_time = position_msgs.header.stamp.to_sec()
    if dt > 0:
        timer += dt
    else:
        return
    # Заполняем данные для графиков
    if arm_state:
        time_plot.append(timer)

        drone_x.append(position_msgs.pose.position.x)
        drone_y.append(position_msgs.pose.position.y)
        drone_z.append(position_msgs.pose.position.z)
        drone_w.append(getQuatToEuler(position_msgs.pose.orientation)[2])
        drone_vx.append(current_vel.twist.linear.x)
        drone_vy.append(current_vel.twist.linear.y)
        drone_vz.append(current_vel.twist.linear.z)

        goal_x.append(goal_msgs.pose.position.x)
        goal_y.append(goal_msgs.pose.position.y)
        goal_z.append(goal_msgs.pose.position.z)
        goal_w.append(getQuatToEuler(goal_msgs.pose.orientation)[2])
        marker_x.append(marker_msgs.pose.position.x)
        marker_y.append(marker_msgs.pose.position.y)
        marker_z.append(marker_msgs.pose.position.z)
        marker_w.append(getQuatToEuler(marker_msgs.pose.orientation)[2])

        ctr_x.append(goal_vel.twist.linear.x)
        ctr_y.append(goal_vel.twist.linear.y)
        ctr_z.append(goal_vel.twist.linear.z)

        ctr_error_xy_value = getDistPoint(current_vel.twist.linear,
                                    goal_vel.twist.linear, use2d=True)

        contol_error_xy.append(ctr_error_xy_value)

        ctr_error_z_value = getDistPoint(current_vel.twist.linear,
                                    goal_vel.twist.linear, getHeight=True)

        contol_error_z.append(ctr_error_z_value)

        marker_error_xy_value = getDistPoint(position_msgs.pose.position,
                                         marker_pose, use2d=True)

        marker_error_z_value = getDistPoint(position_msgs.pose.position,
                                         marker_pose, getHeight=True)

        marker_error_xy.append(marker_error_xy_value)
        marker_error_z.append(marker_error_z_value)

        marker_error_value = getDistPoint(position_msgs.pose.position,
                                         marker_pose)

        marker_error.append(marker_error_value)

def reg_vel_clab(data):
    """
    Функция считывания желаемой скорости дрона.

    :param data: скорость
    :type data: geometry_msgs.msg.TwistStamped
    """
    global goal_vel
    goal_vel = data

def current_vel_clb(data):
    """
    текущая скорость дрона
    :param data: скорость
    :type data: geometry_msgs.msg.TwistStamped
    """
    global current_vel
    current_vel = data

def goalPs_clb(data):
    """
    Get data from goal topic
    :param data:
    :return:
    """
    global goal_msgs, goal_init
    goal_msgs = data
    goal_init = True

def marker_clb(data):
    """
    Get data from found marker topic
    :param data:
    :return:
    """
    global marker_msgs
    marker_msgs=data

def pose_clb(data):
    """
    Get position of drone
    :param data:
    :return:
    """
    global position_msgs

    position_msgs = data

def state_clb(data):
    """
    Get drone's status
    :param data:
    :return:
    """
    global arm_state
    arm_state = data.armed

def ros_init():
    try:
        # inti node
        rospy.init_node('drone_reg_plotter', anonymous=True)

        rospy.Subscriber("/goal", PoseStamped, goalPs_clb)
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, pose_clb)

        rospy.Subscriber("/aruco/aruco", PoseStamped, marker_clb)
        rospy.Subscriber("/mavros/state", State, state_clb)  # mavros FCU state

        rospy.Subscriber("/mavros/setpoint_velocity/cmd_vel", TwistStamped, reg_vel_clab)
        rospy.Subscriber("/mavros/local_position/velocity_local", TwistStamped, current_vel_clb)

        return True
    except:
        return False


def save_on_file(filename):

    print "save to file:", file_name
    with open(file_name+".txt", 'w') as f:

        error_x = np.abs((position_msgs.pose.position.x - marker_pose.x))
        error_y = np.abs((position_msgs.pose.position.y - marker_pose.y))

        f.write("%s\t%s\n" % ("error_x:",error_x))
        f.write("%s\t%s\n" % ("error_y:",error_y))
        f.write("%s\t%s\n" % ("error_z:", marker_error_z[-1]))
        f.write("%s\t%s\n" % ("error_xy:", marker_error_xy[-1]))
        f.write("%s\t%s\n" % ("common__error:", marker_error[-1]))

        # time_plot
        f.write("%s\t" % "time_plot:")
        for item in time_plot:
            f.write("%s\t" % item)
        f.write("\n")

        # drone_x
        f.write("%s\t" % "drone_x:")
        for item in drone_x:
            f.write("%s\t" % item)
        f.write("\n")

        # drone_y
        f.write("%s\t" % "drone_y:")
        for item in drone_y:
            f.write("%s\t" % item)
        f.write("\n")

        # drone_z
        f.write("%s\t" % "drone_z:")
        for item in drone_z:
            f.write("%s\t" % item)
        f.write("\n")

        # drone_w
        f.write("%s\t" % "drone_w:")
        for item in drone_w:
            f.write("%s\t" % item)
        f.write("\n")

        # drone_vx
        f.write("%s\t" % "drone_vx:")
        for item in drone_vx:
            f.write("%s\t" % item)
        f.write("\n")

        # drone_vy
        f.write("%s\t" % "drone_vy:")
        for item in drone_vy:
            f.write("%s\t" % item)
        f.write("\n")

        # drone_vz
        f.write("%s\t" % "drone_vz:")
        for item in drone_vz:
            f.write("%s\t" % item)
        f.write("\n")

        # goal_x
        f.write("%s\t" % "goal_x:")
        for item in goal_x:
            f.write("%s\t" % item)
        f.write("\n")

        # goal_y
        f.write("%s\t" % "goal_y:")
        for item in goal_y:
            f.write("%s\t" % item)
        f.write("\n")

        # goal_z
        f.write("%s\t" % "goal_z:")
        for item in goal_z:
            f.write("%s\t" % item)
        f.write("\n")

        # goal_w
        f.write("%s\t" % "goal_w:")
        for item in goal_w:
            f.write("%s\t" % item)
        f.write("\n")

        # marker_x
        f.write("%s\t" % "marker_x:")
        for item in marker_x:
            f.write("%s\t" % item)
        f.write("\n")

        # marker_y
        f.write("%s\t" % "marker_y:")
        for item in marker_y:
            f.write("%s\t" % item)
        f.write("\n")

        # marker_z
        f.write("%s\t" % "marker_z:")
        for item in marker_z:
            f.write("%s\t" % item)
        f.write("\n")

        # marker_w
        f.write("%s\t" % "marker_w:")
        for item in marker_z:
            f.write("%s\t" % item)
        f.write("\n")

        # ctr_x
        f.write("%s\t" % "ctr_x:")
        for item in ctr_x:
            f.write("%s\t" % item)
        f.write("\n")

        # ctr_y
        f.write("%s\t" % "ctr_y:")
        for item in ctr_y:
            f.write("%s\t" % item)
        f.write("\n")

        # ctr_z
        f.write("%s\t" % "ctr_z:")
        for item in ctr_z:
            f.write("%s\t" % item)
        f.write("\n")

        # contol_error_xy
        f.write("%s\t" % "contol_error_xy:")
        for item in contol_error_xy:
            f.write("%s\t" % item)
        f.write("\n")

        # contol_error_z
        f.write("%s\t" % "contol_error_z:")
        for item in contol_error_z:
            f.write("%s\t" % item)
        f.write("\n")

        # marker_error
        f.write("%s\t" % "marker_error_xyz:")
        for item in marker_error:
            f.write("%s\t" % item)
        f.write("\n")

        # marker_error_xy
        f.write("%s\t" % "marker_error_xy:")
        for item in marker_error_xy:
            f.write("%s\t" % item)
        f.write("\n")

        # marker_error_z
        f.write("%s\t" % "marker_error_z:")
        for item in marker_error_z:
            f.write("%s\t" % item)
        f.write("\n")
        print "save done"

# plots
plt.ion()
def plot_error():
    # Рисуем графики
    ax1 = plt.subplot2grid((2, 2), (0, 0))
    ax1.plot(time_plot, marker_error_xy,'b-x', label='xy')
    ax1.plot(time_plot, marker_error_z,'g--', label='z')
    ax1.plot(time_plot, marker_error,'r', label='xyz')

    ax1.set_title('error to goal marker')
    ax1.set_ylabel('error, m')
    ax1.set_xlabel('t, s')
    ax1.legend()
    ax1.grid()

def plot_pose():
    # Рисуем графики
    ax1 = plt.subplot2grid((2, 2), (0, 1))
    ax1.plot(time_plot, drone_x,'r', label='drone x')
    ax1.plot(time_plot, goal_x,'r--', label='goal x')

    ax1.plot(time_plot, drone_y,'g', label='drone y')
    ax1.plot(time_plot, goal_y,'g--', label='goal y')

    ax1.plot(time_plot, drone_z,'b', label='drone z')
    ax1.plot(time_plot, goal_z,'b--', label='goal z')

    ax1.set_title('drone & goal postion')
    ax1.set_ylabel('pose, m')
    ax1.set_xlabel('t, s')
    ax1.legend()
    ax1.grid()

def plot_vel():
    # Рисуем графики
    ax1 = plt.subplot2grid((2, 2), (1, 0))
    ax1.plot(time_plot, drone_vx,'r', label='drone x')
    ax1.plot(time_plot, ctr_x,'r--', label='goal x')

    ax1.plot(time_plot, drone_vy,'g', label='drone y')
    ax1.plot(time_plot, ctr_y,'g--', label='goal y')

    ax1.plot(time_plot, drone_vz,'b', label='drone z')
    ax1.plot(time_plot, ctr_z,'b--', label='goal z')

    ax1.set_title('drone & goal velocity')
    ax1.set_ylabel('vel, m/s')
    ax1.set_xlabel('t, s')
    ax1.legend()
    ax1.grid()

def plot_ctr_error():
    # Рисуем графики
    ax1 = plt.subplot2grid((2, 2), (1, 1))
    ax1.plot(time_plot, contol_error_xy,'r', label='xy')
    ax1.plot(time_plot, contol_error_z,'b--', label='z')

    ax1.set_title('control errror')
    ax1.set_ylabel('error, m')
    ax1.set_xlabel('t, s')
    ax1.legend()
    ax1.grid()
