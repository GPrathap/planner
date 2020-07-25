#!/usr/bin/env python
# coding: utf-8

"""
The node for integration diagnostics from subsystems (mavros/pixhawk)
"""

import rospy
from rospy import Header
from mavros_msgs.msg import State, HomePosition
from sensor_msgs.msg import BatteryState
from sensor_msgs.msg import NavSatFix
from drone_msgs.msg import Diagnostics
from std_msgs.msg import Float32


# topic name
diag_topic = "drone/diagnostics"
gps_topic = "mavros/global_position/global"
battery_topic = "/mavros/battery"
state_topic = "/mavros/state"
home_gps_topic = "/mavros/home_position/home"
origin_topic = "/geo/set_origin"
health_topic = "drone/health"

# init params
diag_msg = Diagnostics()
diag_msg.health = 0.0

# init timer
state_timer = 0.0
battery_timer = 0.0
gps_timer = 0.0
home_gps_timer = 0.0

# init delay
state_delay = 2.0
battery_delay = 2.0
gps_delay = 0.5
home_gps_delay = 3.0


# callback data
def gps_home_clb(data):
    """
    GPS home callback from mavros. Set status if get msg
    :param data: HomePosition
    """
    global home_gps_timer, diag_msg
    home_gps_timer = 0.0
    diag_msg.init_home = True

def gps_clb(data):
    """
    gobal position status callback from mavros. Set status if get msg
    :type data: NavSatFix
    """
    global gps_timer, diag_msg
    gps_timer = 0.0
    diag_msg.gps_send = True
    diag_msg.status = data.status

def battery_clb(data):
    """
    Battary callback from mavros. Set battery value.
    :type diag_msg: Diagnostics
    :type data: BatteryState
    """
    global battery_timer, diag_msg
    battery_timer = 0.0
    diag_msg.battery = data.percentage

def state_clb(data):
    """
    Callback.
    :type diag_msg: Diagnostics
    :type data: State
    :return:
    """
    global state_timer, diag_msg
    state_timer = 0.0

    diag_msg.armed = data.armed
    diag_msg.mode = data.mode


def healht_clb(data):
    """
    Healht callback. Get healght from virtual gun system.
    :type diag_msg: Diagnostics
    :type data: Float32
    :return:
    """
    global diag_msg
    diag_msg.health= data

def origin_clb(data):
    """
    Oerigin callback. Set status origin init.
    :param data: NavSatFix
    :return:
    """
    global diag_msg
    diag_msg.init_origin = True


### main

if __name__ == '__main__':

    rospy.init_node('drone_diagnosrics_node', anonymous=True)
    rate = rospy.Rate(5)       # set rate

    # subscriber
    rospy.Subscriber(gps_topic, NavSatFix, gps_clb)
    rospy.Subscriber(battery_topic, BatteryState, battery_clb)
    rospy.Subscriber(state_topic, State, state_clb)
    rospy.Subscriber(home_gps_topic, HomePosition, gps_home_clb)
    rospy.Subscriber(origin_topic, NavSatFix, origin_clb)
    rospy.Subscriber(health_topic, Float32, healht_clb)

    # publisher
    diag_pub = rospy.Publisher(diag_topic,Diagnostics,queue_size=10)

    # set timer
    old_time = rospy.get_time()

    # loop
    try:
        while not rospy.is_shutdown():

            dt = rospy.get_time() - old_time
            old_time = rospy.get_time()

            home_gps_timer += dt
            state_timer += dt
            gps_timer += dt
            battery_timer += dt

            # change state if miss msgs
            if home_gps_timer > home_gps_delay:
                diag_msg.init_home = False
            if gps_timer > gps_delay:
                diag_msg.gps_send = False
                diag_msg.status.status = 0

            # pub data
            diag_msg.header.stamp = rospy.Time.now()
            diag_msg.header.frame_id = "base_link"
            diag_pub.publish(diag_msg)
            rate.sleep()
    except:
         print("exit")
         raise
