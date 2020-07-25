#!/usr/bin/env python
# coding=utf8

"""
Позиционный решулятор по скорости для PX4/Mavros
"""

import math
import numpy as np

import rospy
from mavros_msgs.msg import State, ExtendedState
from mavros_msgs.srv import SetMode, CommandBool
from std_msgs.msg import Float32MultiArray
from drone_msgs.msg import DronePose, Goal
from geometry_msgs.msg import Vector3, TwistStamped, PoseStamped
import tf.transformations as t

from dynamic_reconfigure.server import Server
from drone_reg.cfg import DroneRegConfig
from visualization_msgs.msg import Marker
from pid_params_saver import YamlParams
import os.path

# ---------------------------------------------------------------------------------------------------------------------
# ---------- Global params

use_planner_flag = False
use_geo_mode = False


# ---------------------------------------------------------------------------------------------------------------------
# ---------- Глобальные переменные

mavros_root = "/mavros"
OFFBOARD = "OFFBOARD"
vel_pub = None
vel_field = TwistStamped()

# topics
goal_local_topic = "/goal_pose"             # напрямую
goal_planner_topic = "/goal_pose_to_reg"    # для планировщика
goal_global_topic = "geo/goal_pose"         # через geolib

local_pose_topic = "/mavros/local_position/pose"
geo_pose_topic = "/geo/local_pose"


ctr_type = Goal.POSE  # По умолчанию используем управление по координатам
drone_pose = DronePose()  # Координаты дрона

goal = DronePose()  # Целевая позиция дрона
prev_goal = DronePose()
goal.point.z = 1  # Начальная тока дрона - 2 м над точкой взлета

drone_status = {'armed': False, 'mode': None, 'landed': False}

yaml_path = _path = os.path.dirname(__file__)+'/../../cfg/pid_params.yaml'

# Коеффициенты регулятора
hor_kp = 1.7
hor_kd = 0.5
max_hor_vel = 2.0
ver_kp = 2.7
ver_kd = 1.5
max_ver_vel = 3.0
angular_p = 1.5
angular_d = 0.5

prev_error = 0.0
current_vel = TwistStamped()  # Текущая скорость дрона

goal_timer = 0.0
pose_timer = 0.0
goal_lost_time = 1.0
pose_lost_time = 0.5
print_delay = 3.
# ---------------------------------------------------------------------------------------------------------------------
# ---------- Функции подписки на топики
init_server = False
def cfg_callback(config, level):
    """
    Функция считывания параметров регулятора.

    @param config: конфицурация
    @type config: drone_reg.cfg.DroneReg
    @param level: уровень (не используется)
    @return config: конфицурация
    @rtype config: drone_reg.cfg.DroneReg
    """
    global hor_kp
    global hor_kd
    global angular_p, angular_d
    global max_hor_vel
    global ver_kp, ver_kd, max_ver_vel
    global init_server


    # Коеффициенты регулятора
    hor_kp = config["hor_kp"]
    hor_kd = config["hor_kd"]
    max_hor_vel = config["max_hor_vel"]

    ver_kp = config["ver_kp"]
    ver_kd = config["ver_kd"]
    max_ver_vel = config["max_ver_vel"]

    angular_p = config["angular_p"]
    angular_d = config["angular_d"]

    if init_server:
        # opening pid params file
        pps.params_open()
        print ("open from cfg callback")

        # setting values
        pps.params_set('hor_kp', config["hor_kp"])
        pps.params_set('hor_kd', config["hor_kd"])
        pps.params_set('max_hor_vel', config["max_hor_vel"])

        pps.params_set('ver_kp', config["ver_kp"])
        pps.params_set('ver_kd', config["ver_kd"])
        pps.params_set('max_ver_vel', config["max_ver_vel"])

        pps.params_set('angular_p', config["angular_p"])
        pps.params_set('angular_d', config["angular_d"])

        # saving to file
        pps.params_save()
        print ("save from cfg callback")

    init_server = True

    return config


def nav_pos_cb(data):
    global drone_pose, pose_timer

    quat = [data.pose.orientation.x,
            data.pose.orientation.y,
            data.pose.orientation.z,
            data.pose.orientation.w]
    drone_pose.course = get_yaw_from_quat(quat)

    drone_pose.point.x = data.pose.position.x
    drone_pose.point.y = data.pose.position.y
    drone_pose.point.z = data.pose.position.z

    pose_timer = 0.0

def goal_cb(data):
    """
    Функция считывания целевой точки.

    @param data: Целевая точка
    @type data: drone_reg.msg.Goal
    """
    global ctr_type
    global goal
    global goal_timer
    ctr_type = data.ctr_type
    goal = data.pose
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

def imu_cb(data):
    """
    Функция считывания данных инерциальной системы дрона.

    @param data: данные инерциальной ситсемы
    @type data: sensor_msgs.msg.Imu
    """
    drone_pose.course = get_yaw_from_quat([data.orientation.x, data.orientation.y,
                                           data.orientation.z, data.orientation.w])

def on_shutdown_cb():
    """
    Функция реагирования на экстренное завершение работы узла.
    """

    # Преобразование ENU to NED
    ctr_msg = Float32MultiArray()
    ctr_msg.data = ctr_msg.data = 0,0,0,0
    # Публикуем управление
    vel_pub.publish(ctr_msg)

    rospy.loginfo("shutdown")
    set_mode("AUTO_RTL")

def vel_field_cb(data):
    """
    Функция считывания вектора скорости от потенциального поля.
    :return:
    """
    global vel_field
    vel_field = data

# ---------------------------------------------------------------------------------------------------------------------
# ---------- Функции управления дроном

def get_control(goal):
    """
    Функция расчета управляющих команд дрона в виде линейных и угловой скоростей.

    @return: [x,y,z], ang - три линейные и курсовая скорости
    """
    global drone_pose

    global max_hor_vel, \
        hor_kp, \
        hor_kd, \
        max_ver_vel, \
        ver_kp, \
        ver_kd, \
        angular_p, \
        angular_d

    coords_vec = [goal.point.x - drone_pose.point.x,
                  goal.point.y - drone_pose.point.y,
                  goal.point.z - drone_pose.point.z]
    diff_ang = goal.course - drone_pose.course
    # print ('coords_vec = %s' % coords_vec)
    # Делаем пересчет координат в скорости
    # XY
    current_acc_vec = [current_vel.twist.linear.x, current_vel.twist.linear.y, current_vel.twist.linear.z]
    vel_ctr_vec = get_linear_vel_vec(coords_vec, current_acc_vec)
    # YAW
    ang = get_angular_vel(diff_ang, current_vel.twist.angular.z, angular_p, angular_d)
    # Ограничиваем управление
    res_ctr_vec = limit_vector(vel_ctr_vec, max_hor_vel)

    return res_ctr_vec, ang

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


def nav_vel_cb(data):
    """
    Функция считывания текущей скорости дрона.

    :param data: скорость
    :type data: geometry_msgs.msg.TwistStamped
    """
    global current_vel

    current_vel = data
# ---------------------------------------------------------------------------------------------------------------------
# ---------- Вспомогательные функции

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

def get_yaw_from_quat(q):
    """
    Функция возвращает угол рысканья полученный из кватерниона.

    @param q: кватернион
    @type q: list
    @return: угол рысканья
    """
    return t.euler_from_quaternion(q, axes='sxyz')[2]


def limit_vector(r, max_val):
    """
    Функция ограничивания максимального управления.

    @param r: вектор линейных скоростей
    @param max_val: максимальная скорость
    @return: преобразованных вектор линейных скоростей
    """
    l = np.linalg.norm(r)
    if l > max_val:
        r[0] * max_val / l,
        r[1] * max_val / l,
        r[2] * max_val / l

    return r


def get_linear_vel_vec(r, vel):
    global prev_error, \
        hor_kp, \
        hor_kd, max_hor_vel, ver_kp, ver_kd, max_ver_vel

    error = np.linalg.norm(r)

    v = [0, 0, 0]
    v[0] = r[0] * hor_kp - vel[0] * hor_kd
    v[1] = r[1] * hor_kp - vel[1] * hor_kd
    v[2] = r[2] * ver_kp - vel[2] * ver_kd
    for i in range(len(v)-1):
        if v[i] < -max_hor_vel:
            v[i] = -max_hor_vel
        elif v[i] > max_hor_vel:
            v[i] = max_hor_vel

    if v[2] < -max_ver_vel:
        v[2] = -max_ver_vel
    elif v[2] > max_ver_vel:
        v[2] = max_ver_vel

    prev_error = error
    return v


def get_angular_vel(ang,vel, k, d):
    """
    Фукнкция преестчета целевого курса в угловую скорость.
    @param ang: целевой курс (разность текущего и необходимого углов)
    @param k: коэффициент
    @return: угловая скорость
    """
    if ang is None:
        return 0

    # Нормализуем угол
    if ang > math.pi:
        ang -= 2 * math.pi
    if ang < -math.pi:
        ang += 2 * math.pi

    return k*ang-vel*d


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


def set_server_value(cfg_srv):
    global hor_kp, hor_kd, max_hor_vel, ver_kp, ver_kd, max_ver_vel, angular_p, angular_d, init_server, pps

    pps.params_open()
    print ("open from set server value")

    hor_kp = pps.params_get('hor_kp')
    hor_kd = pps.params_get('hor_kd')
    max_hor_vel = pps.params_get('max_hor_vel')
    ver_kp = pps.params_get('ver_kp')
    ver_kd = pps.params_get('ver_kd')
    max_ver_vel = pps.params_get('max_ver_vel')
    angular_p = pps.params_get('angular_p')
    angular_d = pps.params_get('angular_d')

    cfg_srv.update_configuration({'hor_kp': pps.params_get('hor_kp'),
                                  'hor_kd': pps.params_get('hor_kd'),
                                  'max_hor_vel': pps.params_get('max_hor_vel'),
                                  'ver_kp': pps.params_get('ver_kp'),
                                  'ver_kd': pps.params_get('ver_kd'),
                                  'max_ver_vel': pps.params_get('max_ver_vel'),
                                  'angular_p': pps.params_get('angular_p'),
                                  'angular_d': pps.params_get('angular_d')})
    pps.params_save()
    print ("save from set server value")


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
    :type pose: DronePose
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
    marker.pose.position.x = pose.point.x
    marker.pose.position.y = pose.point.y
    marker.pose.position.z = pose.point.z

    return marker

# ---------------------------------------------------------------------------------------------------------------------
# ---------- Главная функция

def main():
    """
    Main loop ROS.
    """

    global vel_pub, goal, drone_pose, drone_rot, goal_timer, pose_timer, use_planner_flag, use_geo_mode, yaml_path, pps
    # Init ROS
    rospy.init_node("drone_reg_vel_node")

    # get params
    yaml_path = rospy.get_param('~yaml_path', yaml_path)
    use_planner_flag = rospy.get_param('~use_planner',use_planner_flag)
    use_geo_mode = rospy.get_param('~use_geo_mode', use_geo_mode)

    pps = YamlParams(yaml_path)


    # init dynamic reconfigure server
    # update values from params file
    cfg_srv = Server(DroneRegConfig, cfg_callback)
    # update values from server
    set_server_value(cfg_srv)


    # Subscriber
    rospy.Subscriber(mavros_root + "/state", State, state_cb)                           # mavros FCU state
    rospy.Subscriber(mavros_root + "/extended_state", ExtendedState, extended_state_cb) # mavros FCU extended state
    rospy.Subscriber("/field_vel", TwistStamped, vel_field_cb)
    rospy.Subscriber(mavros_root + "/local_position/velocity", TwistStamped, nav_vel_cb)

    if use_geo_mode:
        rospy.Subscriber(goal_global_topic, Goal, goal_cb)
    else:
        if use_planner_flag:
            rospy.Subscriber(goal_planner_topic, Goal, goal_cb)
        else:
            rospy.Subscriber(goal_local_topic, Goal, goal_cb)

    if use_geo_mode:
        rospy.Subscriber(geo_pose_topic, PoseStamped, nav_pos_cb)
    else:
        rospy.Subscriber(local_pose_topic, PoseStamped, nav_pos_cb)


    # Publisher
    vel_pub = rospy.Publisher(mavros_root + "/setpoint_velocity/cmd_vel", TwistStamped, queue_size=10)
    pub_marker = rospy.Publisher("/marker_reg_point", Marker, queue_size=10)

    ctr_msg = TwistStamped()

    # Основной цикл
    rate = rospy.Rate(30)
    old_time = rospy.get_time()

    # если что-то не так, выключаем
    rospy.on_shutdown(on_shutdown_cb)
    print_timer = 0.0

    while not rospy.is_shutdown():
        # Переключаем режимы работы в зависимости от текущей ситуации
        dt = rospy.get_time() - old_time
        goal_timer += dt
        pose_timer += dt
        print_timer += dt

        old_time = rospy.get_time()

        # интерполируем точку
        # goal_lerp = lerp_point([drone_pose.point.x, drone_pose.point.y, drone_pose.point.z], goal, 0.5)

        # рисуем маркер
        pub_marker.publish(setup_market(goal))

        # Получаем управление в виде скоростей
        control = get_control(goal)
        v_vec = control[0]  # Линейные скорости
        v_course = control[1]  # Курсовая скорость

        ctr_msg.twist.linear.x = v_vec[0] + vel_field.twist.linear.x
        ctr_msg.twist.linear.y = v_vec[1] + vel_field.twist.linear.y
        ctr_msg.twist.linear.z = v_vec[2] + vel_field.twist.linear.z
        ctr_msg.twist.angular.z = v_course

        # Публикуем управление
        if pose_timer < pose_lost_time:
            vel_pub.publish(ctr_msg)
        else:
            if print_timer > print_delay:
                print("loss goal callback:", goal_timer, pose_timer)
                print_timer = 0.
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
