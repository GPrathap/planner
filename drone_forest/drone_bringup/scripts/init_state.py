#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Init IMU data from FCU.
This node necessary then ise SLAM position in th PX4

The node send fake position (map --> base_link) t(sec.) after exit.
"""

import rospy
import tf

parent_link = "map"
child_link = "base_link"

tf_br = tf.TransformBroadcaster()

time_delay = 5.0



if __name__ == '__main__':
    rospy.init_node('init_fcu_state_node', anonymous=True)
    rate = rospy.Rate(50)
    print("init state: start")

    time_delay = rospy.get_param("delay", time_delay)

    timer = 0.0
    old_time = rospy.get_time()

    while not rospy.is_shutdown():
            dt = rospy.get_time() - old_time
            timer += dt

            tf_br.sendTransform((0.0, 0.0, 0.3), (0.0, 0.0, 0.0, 1.0),
                                    rospy.Time.now(), child_link, parent_link)

            old_time = rospy.get_time()
            rate.sleep()
            print timer
            if timer >= time_delay:
                break
    print("init state: finish")