#!/usr/bin/env python
# coding=utf8

"""
Позиционный решулятор в полётном контроллере PX4/Mavros
"""

import math
import numpy as np

import rospy
from mavros_msgs.msg import State, ExtendedState
from mavros_msgs.srv import SetMode, CommandBool
from geometry_msgs.msg import PoseStamped, Pose
from drone_msgs.msg import  Goal
from visualization_msgs.msg import Marker
import tf.transformations as t

# ---------------------------------------------------------------------------------------------------------------------
# ---------- Global params

use_geo_mode = True


# ---------------------------------------------------------------------------------------------------------------------
# ---------- Глобальные переменные

mavros_root = "/mavros"
OFFBOARD = "OFFBOARD"

# topics
goal_local_topic = "/goal_pose"             # напрямую
goal_planner_topic = "/goal_pose_to_reg"    # для планировщика
goal_global_topic = "/geo/goal_pose"         # через geolib

local_pose_topic = "/mavros/local_position/pose"
geo_pose_topic = "/geo/local_pose"


ctr_type = Goal.POSE  # По умолчанию используем управление по координатам

drone_geo_pose = Pose()  # Координаты дрона
drone_local_pose = Pose()  # Координаты дрона

geo_offset = [0.0, 0.0, 0.0]    # смещение глобальных координат относительно локальных

goal_msgs = Pose()
goal_msgs = PoseStamped()
goal_msgs.header.frame_id = 'map'

goal_point = Goal()

drone_status = {'armed': False, 'mode': None, 'landed': False}


goal_timer = 0.0
geo_pose_timer = 0.0
local_pose_timer = 0.0
goal_lost_time = 1.0
pose_lost_time = 0.5
print_delay = 3.


def nav_geo_pos_cb(data):
    global drone_geo_pose, geo_pose_timer

    drone_geo_pose = data.pose
    geo_pose_timer = 0.0

def nav_local_pos_cb(data):
    global drone_local_pose, local_pose_timer

    drone_local_pose = data.pose

    local_pose_timer = 0.0


def goal_cb(data):
    """
    Функция считывания целевой точки.

    @param data: Целевая точка
    @type data: drone_reg.msg.Goal
    """
    global ctr_type
    global goal_point
    global goal_timer

    ctr_type = data.ctr_type
    goal_point = data
    print ("goal_point:", goal_point)
    goal_timer = 0.0

def state_cb(data):
    """
    Функция считывания текущего статуса автопилота.

    @param data: статус автопилота
    @type data: mavros_msgs.msg.State
    """
    global drone_status
    drone_status['armed'] = data.armed
    drone_status['mode'] = data.mode

def extended_state_cb(data):
    """
    Функция считывания текущего расширенного статуса автопилота.

    @param data: расширенный статус автопилота
    @type data: mavros_msgs.msg.ExtendedState
    """
    global drone_status
    landed_states = [None, True, False]
    drone_status["landed"] = landed_states[data.landed_state]

def on_shutdown_cb():
    """
    Функция реагирования на экстренное завершение работы узла.
    """

    rospy.loginfo("shutdown")
    set_mode("AUTO_LOITER")



def arm():
    """
    Функция отправки команды arm.
    """
    if not drone_status['armed']:
        rospy.loginfo("arming")
        arming = rospy.ServiceProxy(mavros_root + "/cmd/arming", CommandBool)
        arming(True)


def disarm():
    """
    Функция отправки команды disarm.
    """
    if drone_status['armed']:
        rospy.loginfo("disarming")
        arming = rospy.ServiceProxy(mavros_root + "/cmd/arming", CommandBool)
        arming(False)


def set_mode(new_mode):
    """
    Функция отправки нового режима работы автопилота.

    @param new_mode: новый режим работы атопилота
    @type new_mode: String
    """
    if drone_status["mode"] != new_mode:
        set_mode_ = rospy.ServiceProxy(mavros_root + "/set_mode", SetMode)
        set_mode_(custom_mode=new_mode)



def setup_market(point):
    """
    Настройка маркера для отображения в rviz
    :type point: Point
    :param point:
    :return:
    """
    marker = Marker()
    marker.header.frame_id = "/map"
    marker.header.stamp = rospy.get_rostime()
    marker.ns = "goal_test_reg"
    marker.id = 0
    marker.action = 0
    marker.pose.orientation.x = 0
    marker.pose.orientation.y = 0
    marker.pose.orientation.z = 0
    marker.pose.orientation.w = 1.0

    marker.scale.x = 0.4
    marker.scale.y = 0.4
    marker.scale.z = 0.4
    marker.type = Marker.SPHERE
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.color.a = 1.0
    marker.pose.position.x = point.x
    marker.pose.position.y = point.y
    marker.pose.position.z = point.z

    return marker

def angle_between(vec1, vec2):
    """
      Функция возвращает угол между двумя двухмерными векторами.

    :param vec1:
    :param vec2:
    :return:
    """

    x = (vec2[1] - vec1[1])  # y
    y = -(vec2[0] - vec1[0])  # -x
    res = math.atan2(x, y) + math.pi

    if res > math.pi:
        res -= 2 * math.pi
    if res < -math.pi:
        res += 2 * math.pi

    return res

def lerp_point(current_pos, newGoal, step):
    """
    Интерполируем точку куда лететь
    :type target_pos: DronePose
    :type current_pos: []
    :type step: float
    :param current_pos: текущая позиция
    :param target_pos: желаемая позиция
    :param step: растояние на которое нужно интерполировать
    :return:
    """
    global prev_goal
    # вектор куда лететь
    current_vec = [newGoal.point.x - current_pos[0],
                   newGoal.point.y - current_pos[1],
                   newGoal.point.z - current_pos[2]]

    prev_vec = [prev_goal.point.x - current_pos[0],
                prev_goal.point.y - current_pos[1],
                prev_goal.point.z - current_pos[2]]

    dist = np.linalg.norm(current_vec)

    # calculate angle between prev and new point
    angle_between_goal = abs(angle_between(current_vec, prev_vec))
    dist_to_prev_point = np.linalg.norm(prev_vec)

    # если добрались до конечной, то отключаем интерполяцию
    if dist < step:
        prev_goal = newGoal
        return newGoal

    # если новая точка изменилась, то дагетаем до преведущей и потом поворачиваем
    if math.radians(30) < angle_between_goal < math.radians(180) and \
            dist_to_prev_point > (step / 2.0) and dist > dist_to_prev_point:
        return prev_goal


    val = current_vec / dist * step

    newGoal.point.x = val[0] + current_pos[0]
    newGoal.point.y = val[1] + current_pos[1]
    newGoal.point.z = val[2] + current_pos[2]

    prev_goal = newGoal

    return newGoal


def setup_market(pose):
    """
    Настройка маркера для отображения в rviz
    :type pose: PoseStamped
    :param pose:
    :return:
    """
    marker = Marker()
    marker.header.frame_id = "/map"
    marker.header.stamp = rospy.get_rostime()
    marker.ns = "target_point"
    marker.id = 0
    marker.action = 0
    marker.pose.orientation.x = 0
    marker.pose.orientation.y = 0
    marker.pose.orientation.z = 0
    marker.pose.orientation.w = 1.0

    marker.scale.x = 0.3
    marker.scale.y = 0.3
    marker.scale.z = 0.3
    marker.type = Marker.SPHERE
    marker.color.r = 0.0
    marker.color.g = 0.0
    marker.color.b = 0.1
    marker.color.a = 0.8
    marker.pose.position.x = pose.pose.position.x
    marker.pose.position.y = pose.pose.position.y
    marker.pose.position.z = pose.pose.position.z

    return marker


def solve_goal_point():
    global geo_offset, use_geo_mode, drone_geo_pose, drone_local_pose, goal_point, goal_msgs

    goal_msgs.header.stamp = rospy.get_rostime()

    goal_msgs.pose.position.x = goal_point.pose.point.x
    goal_msgs.pose.position.y = goal_point.pose.point.y
    goal_msgs.pose.position.z = goal_point.pose.point.z


    quaternion = t.quaternion_from_euler(0.0, 0.0, goal_point.pose.course)
    # type(pose) = geometry_msgs.msg.Pose
    goal_msgs.pose.orientation.x = quaternion[0]
    goal_msgs.pose.orientation.y = quaternion[1]
    goal_msgs.pose.orientation.z = quaternion[2]
    goal_msgs.pose.orientation.w = quaternion[3]

    if use_geo_mode:
        geo_offset[0] = drone_local_pose.position.x - drone_geo_pose.position.x
        geo_offset[1] = drone_local_pose.position.y - drone_geo_pose.position.y
        geo_offset[2] = drone_local_pose.position.z - drone_geo_pose.position.z

        goal_msgs.pose.position.x += geo_offset[0]
        goal_msgs.pose.position.y += geo_offset[1]
        goal_msgs.pose.position.z += geo_offset[2]

    return goal_msgs


# ---------------------------------------------------------------------------------------------------------------------
# ---------- Главная функция


if __name__ == '__main__':

    """
    Main loop ROS.
    """
    # Init ROS
    rospy.init_node("drone_pose_reg_node")
    # get params
    use_geo_mode = rospy.get_param('~use_geo_mode', use_geo_mode)

    # Subscriber
    rospy.Subscriber(mavros_root + "/state", State, state_cb)                           # mavros FCU state
    rospy.Subscriber(mavros_root + "/extended_state", ExtendedState, extended_state_cb) # mavros FCU extended state

    if use_geo_mode:
        rospy.Subscriber(goal_global_topic, Goal, goal_cb)
        rospy.Subscriber(geo_pose_topic, PoseStamped, nav_geo_pos_cb)
    else:
        rospy.Subscriber(goal_local_topic, Goal, goal_cb)

    rospy.Subscriber(local_pose_topic, PoseStamped, nav_local_pos_cb)


    # Publisher
    goal_pub = rospy.Publisher(mavros_root + "/setpoint_position/local", PoseStamped, queue_size=10)
    pub_marker = rospy.Publisher("/marker_reg_point", Marker, queue_size=10)

    # Основной цикл
    rate = rospy.Rate(30)
    old_time = rospy.get_time()

    # если что-то не так, выключаем
    rospy.on_shutdown(on_shutdown_cb)
    print_timer = 0.0
    try:
        while not rospy.is_shutdown():
            # Переключаем режимы работы в зависимости от текущей ситуации
            dt = rospy.get_time() - old_time
            goal_timer += dt
            local_pose_timer += dt
            geo_pose_timer += dt
            print_timer += dt

            old_time = rospy.get_time()


            # Публикуем управление
            if local_pose_timer < pose_lost_time or geo_pose_timer < pose_lost_time:
                goal_pub.publish(solve_goal_point())
                # рисуем маркер
                pub_marker.publish(setup_market(goal_msgs))
            else:
                if print_timer > print_delay:
                    print("lost navigation:", local_pose_timer, geo_pose_timer)
                    print_timer = 0.
                    # on_shutdown_cb()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
