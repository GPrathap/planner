#!/usr/bin/env python
# coding=utf8

#######################################################################################################################
# This if node of ROS rof conger gWGS 84 to local ENU
#######################################################################################################################


import rospy
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from mavros_msgs.msg import Altitude, GlobalPositionTarget
import libgeographiclib_wrapper
from drone_msgs.msg import DronePose, Goal


topic_global = '/mavros/global_position/global'
topic_local = '/local_position'
topic_origin = '/set_origin'
topic_alt = '/mavros/altitude'
topic_global_set = '/mavros/setpoint_position/global'
topic_marker = "/goal_pose"

frame_id = 'map'

current_global_pose = None
origin_pose = None
local_pose_msg = PoseStamped()
heiht = 0.0
goal_global_msg = GlobalPositionTarget()

mask=int('111111111000', 2)


def global_pose_clb(data):
    """
    Robot Geodetic coordinates from GPS callback.

    :param data: GPS data
    :type data: sensor_msgs.msg.NavSatFix
    :type origin_pose: sensor_msgs.msg.NavSatFix
    :type current_global_pose: NavSatFix
    :type local_pose_msg: PoseStamped
    """
    global origin_pose, current_global_pose, local_pose_msg, heiht

    if origin_pose is None:
        rospy.logwarn("%s: Geodetic reference point is not set!"
                      "Set it as Geodetic world_frame pose." % rospy.get_name())
        return

    current_global_pose = data
    # current_global_pose.altitude = heiht
    heiht = current_global_pose.altitude
    east, north = libgeographiclib_wrapper.Geodetic2ENU(data.latitude,
                                                        data.longitude,
                                                        origin_pose.latitude,
                                                        origin_pose.longitude)

    enu_pose.pose.position.x = east
    enu_pose.pose.position.y = north
    enu_pose.pose.position.z = heiht - origin_pose.altitude

def set_origin_clb(data):
    """
    Set origin pose
    :type data: NavSatFix
    :return:
    """
    global origin_pose
    origin_pose = data

def get_alt_clb(data):
    """
    Get altitude

    :type data: Altitude
    :return:
    """
    global heiht
    heiht = data.amsl

def goal_cb(data):
    """
    Get goal pose
    :type goal_global_msg: GlobalPositionTarget
    :type data: Goal
    :return:
    """
    global  goal_global_msg, origin_pose, heiht
    if origin_pose is None:
        rospy.logwarn("%s: Geodetic reference point is not set!"
                      "Set it as Geodetic world_frame pose." % rospy.get_name())
        return
    lat, lon = libgeographiclib_wrapper.ENU2Geodetic(data.pose.point.x,
                                                     data.pose.point.y,
                                                     origin_pose.latitude,
                                                     origin_pose.longitude)
    goal_global_msg.latitude = lat
    goal_global_msg.longitude = lon
    goal_global_msg.altitude = data.pose.point.z + origin_pose.altitude
# main node

def main():
    global frame_id, current_global_pose, origin_pose, local_pose_msg, goal_global_msg

    rospy.Subscriber(topic_global, NavSatFix, global_pose_clb)
    rospy.Subscriber(topic_origin , NavSatFix, set_origin_clb)
    # rospy.Subscriber(topic_alt , Altitude, get_alt_clb)
    rospy.Subscriber(topic_marker, Goal, goal_cb)


    pub_pose = rospy.Publisher(topic_local, PoseStamped, queue_size=10)
    pub_global = rospy.Publisher(topic_global_set, GlobalPositionTarget, queue_size=10)

    rospy.init_node('common_position', anonymous=True)
    rate = rospy.Rate(20) # 10hz

    # get paramc
    name_node = rospy.get_name()


    while not rospy.is_shutdown():
        # Pos Drone via tf

        if origin_pose is None:
            rospy.logwarn("%s: Geodetic reference point is not set!"
                          "Set it as Geodetic world_frame pose." % rospy.get_name())
            continue
        if current_global_pose is None:
            rospy.logwarn("%s: Geodetic reference point is not set!"
                          "Set it as Geodetic world_frame pose." % rospy.get_name())
            continue

        try:
            enu_pose.header = Header()
            enu_pose.header.stamp = rospy.Time.now()
            enu_pose.header.frame_id = frame_id
            pub_pose.publish(enu_pose)

            goal_global_msg.coordinate_frame = GlobalPositionTarget.FRAME_GLOBAL_INT
            goal_global_msg.type_mask = mask
            goal_global_msg.header = Header()
            goal_global_msg.header.stamp = rospy.Time.now()
            goal_global_msg.header.frame_id = frame_id
            pub_global.publish(goal_global_msg)
            print("terrain", goal_global_msg.altitude)
        except:
            continue

        rate.sleep()




if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
