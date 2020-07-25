#!/usr/bin/env python
# coding=utf8
# Сервер для бродкаста состояний дроной. Передаёт класс в бинарном формате и серриализует его

import rospy
from geometry_msgs.msg import Point
import tf.transformations as t

from drone_msgs.msg import DroneInfo, DroneInfoArray
from visualization_msgs.msg import MarkerArray, Marker

droneList_topic = "drone/list"

listOfDrones = DroneInfoArray()
markerArray = MarkerArray()

droneList_topic = "drone/list"
droneList_marker = "drone/list/markers"




def droneList_cb(data):
    """
    Get drone list data
    :type data: DroneInfoArray
    """
    global markerArray
    if len(markerArray.markers) != len(data.drones):
        while len(markerArray.markers) < len(data.drones):
                markerArray.markers.append(Marker())
        while len(markerArray.markers) > len(data.drones):
                del markerArray.markers[-1]

    drone = DroneInfo()
    team_cout = 1
    drone_cout = 1
    for i in range(len(data.drones)):
        drone = data.drones[i]
        # calculate team num
        if drone.team_num > team_cout:
            team_cout = drone.team_num

        if drone.id_drone > drone_cout:
            drone_cout = drone.id_drone

        color = get_color(drone.team_num, drone.id_drone,team_cout, drone_cout)
        name = "%d_%d" %(drone.team_num, drone.id_drone)
        markerArray.markers[i] = setup_market(name,
                                              drone.pose.point,
                                              drone.pose.course,
                                              drone.id_drone,
                                              color)
    marker_array_pub.publish(markerArray)

def get_color(team_num, drone_num, team_cout = 1, drone_cout = 1):
    color_range = 1000 / team_cout
    color_raw = lerp_val(drone_num, 0.,drone_cout,0.,color_range)
    item_raw = color_range * (team_num-1) + color_raw
    rgb = [0.,0.,0.]
    rgb[0] = item_raw%1000//100*0.1
    if item_raw >= 1000:
        rgb[0] = 10*0.1
    rgb[1] = item_raw%100//10*0.1
    rgb[2] = item_raw%1000*0.
    return rgb

def lerp_val(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def setup_market(name, point, course, id, colorRGBA, scale=0.4):
    """
    Настройка маркера для отображения в rviz
    :type point: Point
    :param point:
    :return:
    """
    quat = t.quaternion_from_euler(0,0,course)

    marker = Marker()
    marker.header.frame_id = "/map"
    marker.header.stamp = rospy.get_rostime()
    marker.ns = name
    marker.id = id
    marker.action = 0
    marker.pose.orientation.x = quat[0]
    marker.pose.orientation.y = quat[1]
    marker.pose.orientation.z = quat[2]
    marker.pose.orientation.w = quat[3]

    marker.scale.x = scale
    marker.scale.y = scale * 0.1
    marker.scale.z = scale * 0.1
    marker.type = Marker.ARROW
    marker.color.r = colorRGBA[0]
    marker.color.g = colorRGBA[1]
    marker.color.b = colorRGBA[2]
    marker.color.a = 1.0
    marker.pose.position.x = point.x
    marker.pose.position.y = point.y
    marker.pose.position.z = point.z
    return marker



if __name__ == '__main__':
    rospy.init_node("drone_marker_array_node")
    print("init drone_marker_array_node")
    rospy.Subscriber(droneList_topic, DroneInfoArray, droneList_cb)
    marker_array_pub = rospy.Publisher(droneList_marker, MarkerArray, queue_size=10)
    rospy.spin()