#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Projection map_slam to map.
It required for google cartographer.
"""

import rospy
from geometry_msgs.msg import Quaternion, TransformStamped
from std_msgs.msg import Float32
import tf2_ros
from mavros_msgs.msg import Altitude


import math

topic_imu = "/mavros/imu/data"
topic_altitude = "/drone/alt"
topic_altitude_fcu = "/mavros/altitude"

topic_tf_link = "/map_slam"
topic_tf_fp = "/map"

tf2_br_Alt = tf2_ros.TransformBroadcaster()

# Global variables

altitude = 0.1

_rotate = Quaternion()
_rotate.w = 1.0
t = TransformStamped()

use_fcu_altitude = False


def callbackAlt(data):
    """
    Callck baro altitude

    :param data: baro data
    :type data: Altitude
    :type _altitude_of_barometr: float
    """
    global altitude
    altitude = data.data
    altitude = data.data if data.data >= 0. else 0.0

def callbackAltFCU(data):
    """
    Callback alritude form FCU
    :type data: Altitude
    :return:
    """
    global altitude
    altitude = data.local if data.local >= 0.0 else 0.0



if __name__ == '__main__':
    # inti node
    rospy.init_node('msg_to_ft2', anonymous=True)
    rate = rospy.Rate(80)       # set rate 50 HZ

    # Get params
    use_fcu_altitude = rospy.get_param("fcu_altitude", use_fcu_altitude)

    # Subscriber
    if use_fcu_altitude:
        rospy.Subscriber(topic_altitude_fcu, Altitude, callbackAltFCU)
    else:
        rospy.Subscriber(topic_altitude, Float32, callbackAlt)

    t.header.frame_id = topic_tf_fp
    t.child_frame_id = topic_tf_link
    t.transform.translation.x = 0.0
    t.transform.translation.y = 0.0

    while not rospy.is_shutdown():
        try:
            t.header.stamp = rospy.Time.now()
            t.transform.translation.z = altitude
            t.transform.rotation = _rotate
            tf2_br_Alt.sendTransform(t)
        except:
            rospy.logerr("msgs to tf error")
        rate.sleep()






