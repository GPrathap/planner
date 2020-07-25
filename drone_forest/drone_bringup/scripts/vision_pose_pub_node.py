#!/usr/bin/env python
# coding: utf-8

"""
The node for publish odometry to PX4(mavros/pixhawk)
"""

import rospy
from rospy import Header
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, Point, Quaternion, Twist
from std_srvs.srv import Trigger, TriggerResponse, TriggerRequest
import tf
from math import pi

pose_msgs = PoseWithCovarianceStamped()
local_offset_pose = Point()
reset_pose = Point()

global_offset = Twist()

pose_pub = None
half_pi = pi /2

roll_offset = 0.
pitch_offset = 0.
yaw_offset = 0.

def odometry_clb(data):
    """
    Odometry callback.
    :param data: NavSatFix
    :return:
    """
    global pose_msgs, pose_pub, local_offset_pose,reset_pose, roll_offset, pitch_offset, yaw_offset, global_offset
    pose_msgs.header = data.header
    pose_msgs.pose = data.pose
    # Rotate axis
    if roll_offset != 0. or pitch_offset != 0 or yaw_offset != 0. \
            or global_offset.angular.x or global_offset.angular.y or global_offset.angular.z:
        roll, pitch, yaw = getQuatToEuler(pose_msgs.pose.pose.orientation.x,
                                          pose_msgs.pose.pose.orientation.y,
                                          pose_msgs.pose.pose.orientation.z,
                                          pose_msgs.pose.pose.orientation.w)

        pose_msgs.pose.pose.orientation = getEulerToQuat(roll-roll_offset-global_offset.angular.x,
                                                         pitch-pitch_offset-global_offset.angular.y,
                                                         yaw-yaw_offset-global_offset.angular.z)



    if local_offset_pose.x != 0. or local_offset_pose.y != 0. or local_offset_pose.z != 0.:
        offet_with_rotate = qv_mult(pose_msgs.pose.pose.orientation, [local_offset_pose.x, local_offset_pose.y, local_offset_pose.z])
        # Calcuate offset
        pose_msgs.pose.pose.position.x -= offet_with_rotate[0]
        pose_msgs.pose.pose.position.y -= offet_with_rotate[1]
        pose_msgs.pose.pose.position.z -= offet_with_rotate[2]

    pose_msgs.pose.pose.position.x -= reset_pose.x + global_offset.linear.x
    pose_msgs.pose.pose.position.y -= reset_pose.y + global_offset.linear.y
    pose_msgs.pose.pose.position.z -= reset_pose.z + global_offset.linear.z

    # Publish
    pose_pub.publish(pose_msgs)

def reset_clb_srv(req):
    """
    Service when reset of state planner

    :param req:
    :return:
    """
    global reset_pose, pose_msgs , goal_sub, global_offset

    rospy.loginfo_once("reset pose")

    resp = TriggerResponse()
    resp.success = True
    resp.message = "Reset pose: True"
    reset_pose = pose_msgs.pose.pose.position
    print "reset position:", reset_pose

    global_offset.linear.x = 0.
    global_offset.linear.y = 0.
    global_offset.linear.z = 0.

    return resp

def getQuatToEuler(x, y, z, w):
    """
    Transform quaternion to euler angels
    :param x:
    :param y:
    :param z:
    :param w:
    :return: euler angels
    """
    # type(pose) = geometry_msgs.msg.Pose
    euler = tf.transformations.euler_from_quaternion((x,y,z,w))
    roll = euler[0]
    pitch = euler[1]
    yaw = euler[2]
    return roll,pitch,yaw

def getEulerToQuat(roll=0., pitch=0., yaw = 0.):
    """
    Transform euler angels to quaternion
    :param roll:
    :param pitch:
    :param yaw:
    :return: quaternion
    """
    # type(pose) = geometry_msgs.msg.Pose
    q = tf.transformations.quaternion_from_euler(roll,pitch,yaw)
    quat = Quaternion()
    quat.x = q[0]
    quat.y = q[1]
    quat.z = q[2]
    quat.w = q[3]
    return quat

def pose_offset_cl(data):
 global global_offset
 global_offset = data

 print "Get global offset:", data


# rotate vector v1 by quaternion q1
def qv_mult(quat, v1):
    q1 = [quat.x, quat.y, quat.z, quat.w]
    v1 = tf.transformations.unit_vector(v1)
    q2 = list(v1)
    q2.append(0.0)
    return tf.transformations.quaternion_multiply(
        tf.transformations.quaternion_multiply(q1, q2),
        tf.transformations.quaternion_conjugate(q1)
    )[:3]

if __name__ == '__main__':

    rospy.init_node('vision_pose_publish_node', anonymous=True)

    local_offset_pose.x = rospy.get_param("~offset_x", 0.)
    local_offset_pose.y = rospy.get_param("~offset_y", 0.)
    local_offset_pose.z = rospy.get_param("~offset_z", 0.)

    roll_offset = rospy.get_param("~roll_offset", roll_offset)
    pitch_offset = rospy.get_param("~pitch_offset", pitch_offset)
    yaw_offset = rospy.get_param("~yaw_offset", yaw_offset)

    print "offset_pose",local_offset_pose
    print "roll_offset:",roll_offset
    print "pitch_offset:",pitch_offset
    print "yaw_offset", yaw_offset
    # subscriber
    rospy.Subscriber("/t265/odom/sample", Odometry, odometry_clb)
    rospy.Subscriber("/vision_pose_publish/offset", Twist, pose_offset_cl)

    pose_pub = rospy.Publisher("/mavros/vision_pose/pose_cov", PoseWithCovarianceStamped, queue_size=10)
    reset_srv = rospy.Service("vision_pose_publish/reset", Trigger, reset_clb_srv)
    rospy.spin()

