#!/usr/bin/env python
# coding: utf-8

import rospy
from geometry_msgs.msg import Pose, PoseStamped
from drone_msgs.msg import Goal
import tf
import numpy as np
from  std_srvs.srv import Empty

goal_pose = Goal()

init_goal_flag = False
resetPoseTopic = "/drone/goal_pose/reset"

localPoseTopic = "/mavros/local_position/pose"
localPose = None

def goal_clb(data):
    """
    Get goal pose
    """
    global goal_pose, init_goal_flag

    goal_pose= data
    init_goal_flag = True


def resetCb(req):
    print("reset goal pose")
    global localPose, goal_pose
    if localPose == None:
        print("local Pose not init")
        return

    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion((localPose.pose.orientation.x,
                                                                   localPose.pose.orientation.y,
                                                                   localPose.pose.orientation.z,
                                                                   localPose.pose.orientation.w))

    goal_pose.pose.point.x = localPose.pose.position.x
    goal_pose.pose.point.y = localPose.pose.position.y
    goal_pose.pose.point.z = localPose.pose.position.z

    goal_pose.pose.course = yaw
    pub_goal.publish(goal_pose)


def goal_offset_clb(data):
    """
    :type data: Pose
    :type oal_pose: Goal

    :return:
    """
    global goal_pose, pub_goal
    if not init_goal_flag:
        print "Not init goal pose"
        return

    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion((data.orientation.x,
                                                                   data.orientation.y,
                                                                   data.orientation.z,
                                                                   data.orientation.w))
    offset_yaw = (goal_pose.pose.course + yaw) % (np.pi*2.)

    goal_pose.pose.point.x += data.position.x
    goal_pose.pose.point.y += data.position.y
    goal_pose.pose.point.z += data.position.z

    goal_pose.pose.course = offset_yaw
    pub_goal.publish(goal_pose)

def localPoseClb(data):
    "Get local pose data"

    global localPose
    localPose = data

if __name__ == '__main__':
    rospy.init_node("remote_contol_node")

    rospy.Subscriber("/drone/goal_offset", Pose, goal_offset_clb)
    rospy.Subscriber("/goal_pose", Goal, goal_clb)
    rospy.Subscriber(localPoseTopic, PoseStamped, localPoseClb)

    pub_goal = rospy.Publisher("/goal_pose", Goal, queue_size=10)
    # сервис для отключения планировщика
    rospy.Service(resetPoseTopic, Empty, resetCb)
    rospy.spin()
