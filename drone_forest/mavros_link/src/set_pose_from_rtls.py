#!/usr/bin/env python
# license removed for brevity

import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import TimeReference
from std_msgs.msg import Header
from sensor_msgs.msg import Imu
import tf

import time, math, random

imu_topic = '/mavros/imu/data'
topic = '/mavros/mocap/pose'
frame_id = 'map'
frame_child = 'drone'
frame_pose = 'map'

imu_data = Imu()

def imu_clb(data):
    global imu_data
    imu_data = data

def talker():
    global frame_id, frame_child, frame_pose, imu_data

    pub = rospy.Publisher(topic, PoseStamped, queue_size=10)

    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(20) # 10hz

    name_node = rospy.get_name()
    frame_id = rospy.get_param(name_node + '/frame_id', frame_id)
    frame_child = rospy.get_param(name_node + '/frame_child', frame_child)
    frame_pose = rospy.get_param(name_node + '/frame_pose', frame_pose)
    # rospy.Subscriber(imu_topic , Imu, imu_clb)

    listener = tf.TransformListener();


    while not rospy.is_shutdown():

        # Pos Drone via tf
        try:
            (pose, rot) = listener.lookupTransform(frame_id, frame_child, rospy.Time(0))
            msg = PoseStamped()
            msg.header = Header()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = frame_pose
            msg.pose.position.x = pose[0]
            msg.pose.position.y = pose[1]
            msg.pose.position.z = pose[2]
            msg.pose.orientation = imu_data.orientation
            print(pose)
            pub.publish(msg)
        except:
            continue

        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
