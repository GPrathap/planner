#!/usr/bin/env python
# coding=utf8

"""
This if node of ROS rof conger gWGS 84 to local ENU
"""

import rospy
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped, Point
from mavros_msgs.msg import Altitude
from std_msgs.msg import Float32
from copy import deepcopy

import libgeographiclib_wrapper


# set topict
frame_id = 'map'

# Global value
offset = Point()
geo_local_pose_msg = Point()
local_pose_msg = PoseStamped()

use_rel_alt = False         # use relative altitude from fmu
use_sonar = False           # use sonar /drone/alt topic
sonar_max = 3.8             # max allow alt from sonar
set_rel_alt_origin = True   # set origin altitude relative current pose

geo_local_msgs = PoseStamped()

current_global_pose = None
origin_pose = None
height = 0.0
sonar = 0.0

def sonar_clb(data):
    '''
    Get sonar data
    :param data:
    :return:
    '''

    global sonar
    sonar = data.data

def global_pose_clb(data):
    """
    Robot Geodetic coordinates from GPS callback.

    :param data: GPS data
    :type data: sensor_msgs.msg.NavSatFix
    :type origin_pose: sensor_msgs.msg.NavSatFix
    :type current_global_pose: NavSatFix
    :type local_pose_msg: PoseStamped
    """
    global origin_pose, current_global_pose, geo_local_pose_msg, height,  use_rel_alt

    if origin_pose is None:
        rospy.logwarn("%s: Geodetic: Get GPS. Origin not set!." % rospy.get_name())
        return

    current_global_pose = data

    if not use_rel_alt:
        height = current_global_pose.altitude


    # global to local
    east, north = libgeographiclib_wrapper.Geodetic2ENU(current_global_pose.latitude,
                                                        current_global_pose.longitude,
                                                        origin_pose.latitude,
                                                        origin_pose.longitude)

    geo_local_pose_msg.x = east
    geo_local_pose_msg.y = north
    geo_local_pose_msg.z = height - origin_pose.altitude


def local_pose_clb(data):
    """
    Publish global ti local data when get local pose
    :type data: PoseStamped
    :return:
    """
    global local_pose_msg, offset, geo_local_pose_msg, sonar, use_sonar, sonar_max, geo_local_msgs
    local_pose_msg = data
    geo_local_msgs = deepcopy(data)

    geo_local_msgs.pose.position.x += offset.x
    geo_local_msgs.pose.position.y += offset.y
    geo_local_msgs.pose.position.z += offset.z

    if use_sonar and sonar < sonar_max:
        geo_local_msgs.pose.position.z = sonar

    pub_geo_local_pose.publish(geo_local_msgs)

def set_origin_clb(data):
    """
    Set origin pose
    :type data: NavSatFix
    :return:
    """
    global origin_pose, current_global_pose, height, use_rel_alt, set_rel_alt_origin, local_pose_msg, offset, geo_local_pose_msg, height
    
    origin_pose = data

    if set_rel_alt_origin:
        if use_rel_alt:
            origin_pose.altitude = height
        else:
            origin_pose.altitude = current_global_pose.altitude
    
    
     # global to local
    east, north = libgeographiclib_wrapper.Geodetic2ENU(current_global_pose.latitude,
                                                        current_global_pose.longitude,
                                                        origin_pose.latitude,
                                                        origin_pose.longitude)

    geo_local_pose_msg.x = east
    geo_local_pose_msg.y = north
    geo_local_pose_msg.z = height - origin_pose.altitude
    
    offset.x = geo_local_pose_msg.x - local_pose_msg.pose.position.x
    offset.y = geo_local_pose_msg.y - local_pose_msg.pose.position.y
    offset.z = geo_local_pose_msg.z - local_pose_msg.pose.position.z


def get_alt_clb(data):
    """
    Get altitude

    :type data: Altitude
    :return:
    """
    global height
    height = data.relative



if __name__ == '__main__':

    rospy.init_node('geo_reg_node', anonymous=True)

    rate = rospy.Rate(0.3)  # 0.3hz

    # subscribe
    use_rel_alt = rospy.get_param('~use_rel_alt', use_rel_alt)
    use_sonar = rospy.get_param('~use_sonar', use_sonar)
    sonar_max = rospy.get_param('~sonar_max', sonar_max)
    set_rel_alt_origin = rospy.get_param('~set_rel_alt_origin', set_rel_alt_origin)


    if use_rel_alt:
        rospy.Subscriber("/mavros/altitude", Altitude, get_alt_clb)
    if use_sonar:
        rospy.Subscriber('/drone/alt', Float32, sonar_clb)

    rospy.Subscriber('/mavros/global_position/global', NavSatFix, global_pose_clb)
    rospy.Subscriber('/geo/set_origin', NavSatFix, set_origin_clb)
    rospy.Subscriber('/mavros/local_position/pose', PoseStamped, local_pose_clb)

    # publisher to topics
    pub_geo_local_pose = rospy.Publisher('/geo/local_pose', PoseStamped, queue_size=10)

    try:
        while not rospy.is_shutdown():
            # Print error if not set params
            if origin_pose is None:
                rospy.logwarn("%s: Geodetic: Origin not set!." % rospy.get_name())
            if current_global_pose is None:
                rospy.logwarn("%s: Geodetic: Not GPS data." % rospy.get_name())
            rate.sleep()
    except rospy.ROSInterruptException:
        pass

