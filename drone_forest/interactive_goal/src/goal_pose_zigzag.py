#!/usr/bin/env python
# coding=utf8

import math
import rospy
from visualization_msgs.msg import InteractiveMarkerFeedback
from drone_msgs.msg import Goal
import tf.transformations as t
import numpy as np

# init params
angle_degree = 15.0
angle_degree_rad = 0.0
angle_speed = 2.0
goal_pose = None

target_course = 0.0


def get_yaw_from_quat(q):
    """
    Функция возвращает угол рысканья полученный из кватерниона.

    @param q: кватернион
    @type q: list
    @return: угол рысканья
    """
    return t.euler_from_quaternion(q, axes='sxyz')[2]


def MarkerFeedbackClb(data):
    """
    Geet marker feedback
    :type data: InteractiveMarkerFeedback
    :return:
    """
    global goal_pose, target_course

    if goal_pose == None:
        goal_pose = Goal()

    goal_pose.pose.point = data.pose.position
    target_course = get_yaw_from_quat((data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w))

if __name__ == '__main__':
    rospy.init_node("goal_pose_zigzag_node")
    print("swarm_goal_node: init")

    # get param
    angle_degree = rospy.get_param("~angle_rotate", angle_degree)
    angle_speed = rospy.get_param("~angle_speed", angle_speed)

    angle_degree_rad = math.radians(angle_degree)

    rospy.Subscriber("/basic_controls/feedback", InteractiveMarkerFeedback, MarkerFeedbackClb)

    goal_pub = rospy.Publisher("/goal_pose", Goal, queue_size=10)

    rate = rospy.Rate(20)

    try:
        while not rospy.is_shutdown():
            if goal_pose:

                course = math.sin((rospy.get_time()*angle_speed)*angle_degree_rad)

                goal_pose.pose.course = course + target_course

                goal_pub.publish(goal_pose)

            rate.sleep()
    except rospy.ROSInterruptException:
        pass
