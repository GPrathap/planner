#!/usr/bin/env python
# coding=utf8

"""
Safe node for emergency landing drone


1. If turnoff drone - disarm;
2. If not connect more 1 second - Land
3. if not gps and Arm - Land

"""

import math
import rospy
from quadrotor_msgs.msg import PositionCommand
from geometry_msgs.msg import PoseStamped, TwistStamped

use_cmd = False

pose_goal_msg = PoseStamped()
pose_goal_msg.header.frame_id = "map"
pose_goal_msg.pose.orientation.w = 1.

cmd_msg = TwistStamped()
cmd_msg.header.frame_id = "map"

def cmd_cb(data):
    """
    Note use.
    Callback diagnostics node
    :type data: PositionCommand
    """
    global pose_pub, pose_goal_msg, use_cmd, cmd_msg, cmd_pub
    if use_cmd:
        cmd_msg.header.stamp = rospy.Time.now()
        cmd_msg.twist.linear = data.velocity
        cmd_pub.publish(cmd_msg)
    else:
        pose_goal_msg.header.stamp = rospy.Time.now()
        pose_goal_msg.pose.position = data.position
        pose_pub.publish(pose_goal_msg)

if __name__ == '__main__':
    try:
        rospy.init_node("drone_path_controller", anonymous=True)

        # Subscriber
        rospy.Subscriber("/planning/pos_cmd", PositionCommand, cmd_cb)
        pose_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=10)

        cmd_pub = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=10)
	rospy.spin()
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            cmd_msg.twist.angular.z = 0.4
            cmd_pub.publish(cmd_msg)
            rate.sleep()

    except rospy.ROSInterruptException:
        exit(0)
