#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
The projection of the base_link to the plane (base_link --> base_stabilized --> base_footprint)
http://wiki.ros.org/navigation/Tutorials/RobotSetup/TF
"""
import rospy
import tf2_ros
from geometry_msgs.msg import  TransformStamped

# topics
topic_local_pos = "/mavros/local_position/pose"

# tf link
topic_tf_link = "base_link"
topic_stabilized_link = "base_stabilized"
topic_tf_footprint = "base_footprint"

# init params
tfBuffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tfBuffer)
tf_footprint_pose = tf2_ros.StaticTransformBroadcaster()
tf_stabilized_pose  = tf2_ros.StaticTransformBroadcaster()

# main function
if __name__ == '__main__':
    # init node
    rospy.init_node('msg_to_ft', anonymous=True)

    rate = rospy.Rate(100)      # set rate

    tf_fp = TransformStamped()
    tf_fp.transform.rotation.w = 1.0

    ft_stab = TransformStamped()

    while not rospy.is_shutdown():
        try:
            trans = tfBuffer.lookup_transform('base_link', 'map', rospy.Time())

            # Send foorprint -> stabolise
            tf_fp.header.stamp = rospy.Time.now()
            tf_fp.header.frame_id = topic_tf_footprint
            tf_fp.child_frame_id = topic_stabilized_link
            tf_fp.transform.translation.z = trans.transform.translation.z
            tf_footprint_pose.sendTransform(tf_fp)


            ft_stab.header.stamp = rospy.Time.now()
            ft_stab.header.frame_id = topic_stabilized_link
            ft_stab.child_frame_id = topic_tf_link
            ft_stab.transform.rotation = trans.transform.rotation
            tf_stabilized_pose.sendTransform(ft_stab)

            # print(tf_fp.transform.translation)

            rate.sleep()

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            continue


