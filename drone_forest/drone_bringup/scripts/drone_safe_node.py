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
from mavros_msgs.msg import State, ExtendedState
from mavros_msgs.srv import SetMode, CommandBool
from drone_msgs.msg import Diagnostics
import tf.transformations as t
from sensor_msgs.msg import Imu
from std_msgs.msg import String

# init params
mavros_root = "/mavros"
OFFBOARD = "OFFBOARD"

max_angle = 110.0
disarm_delay = 1.0
roll = 0.0
pitch = 0.0
drone_status = {'armed': False, 'mode': None, 'landed': False}


# init delay
receive_delay = 1.0

diagnostics_data = Diagnostics()

def diagnostics_cb(data):
    """
    Note use.
    Callback diagnostics node
    :type data: Diagnostics
    """
    global diagnostics_data
    diagnostics_data = data

def state_cb(data):
    """
    Callback state FCU.

    @param data: state autopilot
    @type data: mavros_msgs.msg.State
    """
    global drone_status
    drone_status['armed'] = data.armed
    drone_status['mode'] = data.mode

def receive_clb(data):
    global receive_time
    receive_time = 0.0

def extended_state_cb(data):
    """
    Callback extended state.

    @param data: extended state FCU
    @type data: mavros_msgs.msg.ExtendedState
    """
    global drone_status
    landed_states = [None, True, False]
    drone_status["landed"] = landed_states[data.landed_state]

def imu_cb(data):
    """
    Callback IMU.

    @param data: IMU data
    @type data: sensor_msgs.msg.Imu
    """
    global roll, pitch

    roll_rad,pitch_rad,yaw = t.euler_from_quaternion([data.orientation.x,
                                              data.orientation.y,
                                              data.orientation.z,
                                              data.orientation.w], axes='sxyz')
    roll = abs(math.degrees(roll_rad))
    pitch = abs(math.degrees(pitch_rad))

def on_shutdown_cb():
    """
    Shutdown and auto landing drone.
    """
    rospy.loginfo("shutdown")
    set_mode("AUTO.LAND")


# Function of drone

def arm():
    """
    Arm of drone.
    """
    if not drone_status['armed']:
        rospy.loginfo("arming")
        arming = rospy.ServiceProxy(mavros_root + "/cmd/arming", CommandBool)
        arming(True)

def disarm():
    """
    Disarm of drone.
    """
    if drone_status['armed']:
        rospy.loginfo("disarming")
        arming = rospy.ServiceProxy(mavros_root + "/cmd/arming", CommandBool)
        arming(False)

def set_mode(new_mode):
    """
    Change fly mode of the drone

    @param new_mode: fly mode
    @type new_mode: String
    """
    if drone_status["mode"] != new_mode:
        set_mode_ = rospy.ServiceProxy(mavros_root + "/set_mode", SetMode)
        set_mode_(custom_mode=new_mode)

def handle_status():
    """
    Change to OFFBOARD mode.
    """

    if drone_status["mode"] != OFFBOARD:
        set_mode(OFFBOARD)

    if drone_status["mode"] == OFFBOARD and not drone_status["armed"]:
        arm()  # Check it

    if drone_status["landed"]:
        disarm()



if __name__ == '__main__':
    try:
        rospy.init_node("drone_safery_node", anonymous=True)

        # get param
        max_angle = rospy.get_param('~max_angle', max_angle)
        disarm_delay = rospy.get_param('~disarm_delay', disarm_delay)

        # Subscriber
        rospy.Subscriber(mavros_root + "/state", State, state_cb)
        rospy.Subscriber(mavros_root + "/extended_state", ExtendedState, extended_state_cb)
        rospy.Subscriber(mavros_root + "/imu/data", Imu, imu_cb)
        rospy.Subscriber("drone/diagnostics", Diagnostics, diagnostics_cb)
        rospy.Subscriber("drone/receive", String, receive_clb)

        rate = rospy.Rate(10)  # set rate 20 Hz
        old_time = rospy.get_time()
        flip_time = 0.0
        receive_time = 0.0

        rospy.on_shutdown(on_shutdown_cb)  # set mode if shutdown

        # set timer
        old_time = rospy.get_time()

        # loop
        while not rospy.is_shutdown():
            # print ('drone_status %s' % drone_status)
            dt = rospy.get_time() - old_time
            receive_time += dt

            # SAFE MODES

            # 1. Disarm the drone if it is turned over
            if roll > max_angle or pitch > max_angle:
                print("turned over!")
                flip_time += dt
                if flip_time > disarm_delay:
                    disarm()
            else:
                flip_time = 0.0

            ## 2. LAND if DATA LOSS
            #if receive_time > receive_delay:
            #    if drone_status['armed'] and drone_status['mode'] != "AUTO.LAND":
            #        set_mode("AUTO.LAND")

            ##3. if not gps and Arm - Land
            #if drone_status['armed'] and diagnostics_data.gps_send == False and drone_status['mode'] == "OFFBOARD":
            #    set_mode("AUTO.LAND")

            old_time = rospy.get_time()
            rate.sleep()

    except rospy.ROSInterruptException:
        exit(0)
