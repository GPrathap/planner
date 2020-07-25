#!/usr/bin/env python
# coding=utf8

import rospy
import numpy as np
import tf

import copy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import PoseStamped, TwistStamped, Point
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import Header

radius = 2.0
frame_id = "map"


position_topic = "/mavros/local_position/pose"
marker_topic = "/occupied_cells_vis_array"
point2_near_topic = "/point2/near"

position = np.array([0.0,0.0,0.0])
map_point_list = list()
marker_array = MarkerArray()


def get_pos_cb(data):
    """
    Get position
    :type data: PoseStamped
    :return:
    """
    global position
    position[0] = data.pose.position.x
    position[1] = data.pose.position.y
    position[2] = data.pose.position.z

def get_marker_array_cb(data):
    """

    :type data:
    :return:
    """
    global marker_array
    marker_array = data

def get_near_from_points(position, map_point,radius):
    """

    :param position: pose of target point
    :param map_point: point2 array
    :type map_point: PointCloud2

    :param radius: target near radius
    :return: Point2 of near points
    """
    if len(map_point_list) == 0:
        return None

    near_points= list()

    for i in   map_point_list:
        currentPoint = i

        dist = np.linalg.norm(position - currentPoint)
        if dist <= radius:
                near_points.append(i)

    return near_points

def point_from_markers(marker_array):
    """
    :param data:
    :return:
    """
    all_point_list = list()
    del all_point_list[:]
    for marker in marker_array.markers:
        if len(marker.points) > 0:
            for i in marker.points:
                all_point_list.append([i.x, i.y, i.z])
    return all_point_list

def convert_to_point2(data):
    """
    Converts list of Point to point2
    :param data:
    :return:
    """
    global marker_array
    head = Header()
    head.frame_id = frame_id
    head.stamp = rospy.Time.now()
    point2 = pc2.create_cloud_xyz32(head, data)
    return point2


if __name__ == '__main__':
    rospy.init_node('near_points', anonymous=True)
    rate = rospy.Rate(1)


    rospy.Subscriber(position_topic, PoseStamped, get_pos_cb)
    rospy.Subscriber(marker_topic, MarkerArray, get_marker_array_cb)

    near_point2_pub = rospy.Publisher(point2_near_topic, PointCloud2, queue_size=10)

    frame_id = rospy.get_param("~frame_id", frame_id)
    radius = float(rospy.get_param("~radius", radius))

    while not rospy.is_shutdown():
        # get_near_point2(position, map_point2, radius)
        _markers = copy.deepcopy(marker_array)
        map_point_list = point_from_markers(_markers)
        near_points = get_near_from_points(position, map_point_list, radius)
        if near_points != None:
            point_2 = convert_to_point2(near_points)
            near_point2_pub.publish(point_2)
        rate.sleep()
