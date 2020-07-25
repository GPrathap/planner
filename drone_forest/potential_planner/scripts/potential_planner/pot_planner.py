#!/usr/bin/env python
# coding=utf8

import rospy
import numpy as np
import tf
import copy
import math
import sensor_msgs.point_cloud2 as pc2
import laser_geometry.laser_geometry as lg
from sensor_msgs.msg import LaserScan, PointCloud2
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3
from drone_msgs.msg import *
from std_msgs.msg import Bool
from std_srvs.srv import SetBool, SetBoolResponse
from visualization_msgs.msg import Marker

from dynamic_reconfigure.server import Server
from potential_planner.cfg import PotPlannerConfig
from params_saver import YamlParams
import os.path

# Параметры планировщика
r_field = 1.5                   # радиус действия поля отталкивания
c_rep = 0.4                     # коэффициент ф-ии отталкивания
k_vel = 0.5                     # коэффициент по скорости
angle_vec = 0.0                 # Угол на который поворачивается веткор отталкивания полей

# флаг входных данных
# True - point_cloud2
# False - lidar
input_point2 = False
use_drone_list = False
use_one_point = True
use_3d_avoid = True


# Инициализация глобальных переменных
sonar_data = list()
point_list = list()
norm_points = list()

lp = lg.LaserProjection()
target_pos = Goal()
current_pose = [0.0, 0.0, 0.0]   # Массив с текущей позицией дрона
current_course = 0.0            # текущий курс дрона
mod_vel = 0.0                   # Скорость дрона
dist_to_goal = 0.0              # текущее расстояние до цели

max_dist_lidar = 8.0            # макс дальность лидара

init_server = False
active_flag = True				# состояние, включен ли планировщик
use_geo_mode = False

drone_id = 1                    # id дрона в строю

# Список топиков
sonar_topic = "/scan"

local_goal_topic = "/goal_pose"
geo_goal_topic = "/geo/goal_pose"

local_pose_topic = "/mavros/local_position/pose"
geo_pose_topic = "/geo/local_pose"

topic_velocity = "/mavros/local_position/velocity_local"
octomap_topic = "/point2/near"


drone_list_topic = "drone/list"
yaml_path = os.path.dirname(__file__)+'/../../cfg/params.yaml'
cfg_srv = None

def cfg_callback(config, level):
    """
    Функция считывания параметров регулятора.

    @param config: конфицурация
    @type config: PotPlannerConfig
    @param level: уровень (не используется)
    @return config: конфицурация
    @rtype config: PotPlannerConfig
    """

    global r_field, c_rep ,k_vel, angle_vec, pps, init_server, active_flag

    # Коеффициенты регулятора
    r_field = config["r_field"]
    c_rep = config["c_rep"]
    k_vel = config["k_vel"]
    angle_vec = config["angle_vec"]

    if init_server:
        # opening pid params file
        pps.params_open()
        print ("open from cfg callback")

        # setting values
        pps.params_set('r_field', config["r_field"])
        pps.params_set('c_rep', config["c_rep"])
        pps.params_set('k_vel', config["k_vel"])
        pps.params_set('angle_vec', config["angle_vec"])

        # saving to file
        pps.params_save()
        print ("save from cfg callback")

        active_flag = config["run"]

    init_server = True
    return config

def set_server_value(cfg_srv):
    global r_field, c_rep, k_vel, angle_vec, active_flag
    global init_server, pps

    pps.params_open()
    print ("open from set server value")

    r_field = pps.params_get('r_field')
    c_rep = pps.params_get('c_rep')
    k_vel = pps.params_get('k_vel')
    angle_vec = pps.params_get('angle_vec')

    cfg_srv.update_configuration({'r_field': pps.params_get('r_field'),
                                  'c_rep': pps.params_get('c_rep'),
                                  'k_vel': pps.params_get('k_vel'),
                                  'angle_vec': pps.params_get('angle_vec'),
                                  'run': active_flag})
    pps.params_save()
    print ("save from set server value")


#                          Функции подписки на топики
##################################################################################
#                       Функция получения данных сонара
##################################################################################
def callbackSonar(data):
    """
    Читаем данные с сонара
    :type data: LaserScan
    :return:
    """
    global sonar_data
    sonar_data = data.ranges

##################################################################################
#                       Функция получения текущей скорости
##################################################################################
def callbackVelocity(data):
    """
    :param data:
    :type data: TwistStamped
    :return:
    """
    global mod_vel
    mod_vel = np.linalg.norm((data.twist.linear.x, data.twist.linear.y, data.twist.linear.z))


def rotate_vect(vec, rot):
    """
    Поворачиваем вектора vec относительно нуля на угол rot(рад)
    :type vec: list
    :type rot: float
    :return: возвращаем точку повёрнутую на нужный угол

    """
    rotate = np.array([[np.cos(-rot), -np.sin(-rot)],
                       [np.sin(-rot), np.cos(-rot)]])

    pos = np.array([[vec[0]],
                    [vec[1]]])
    val = np.dot(rotate, pos)
    return [val[0][0], val[1][0], vec[0]]

##################################################################################
#                       Функция получения текущих координат
##################################################################################
def callbackCurrentPose(data):
    """
    :type data: PoseStamped
    :param data:
    :return:
    """
    global current_pose, current_course

    current_pose = [data.pose.position.x, data.pose.position.y, data.pose.position.z]

    if not input_point2:
        # Получаем текущий курс дрона
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion((data.pose.orientation.x,
                                                                       data.pose.orientation.y,
                                                                       data.pose.orientation.z,
                                                                       data.pose.orientation.w))
        current_course = yaw


##################################################################################
#                       Функция получения координат цели
##################################################################################
def callbackTargetPose(data):
    """
    :type data: Goal
    :param data:
    :return:
    """
    global target_pos, current_pose, dist_to_goal
    target_pos = data
    # вычисляем вектор между текущим положением дрона и целью
    vec_to_goal = [target_pos.pose.point.x - current_pose[0],
                   target_pos.pose.point.y - current_pose[1],
                   target_pos.pose.point.z - current_pose[2]]
    # вычисляем длину вектора
    dist_to_goal = np.linalg.norm(vec_to_goal)


##################################################################################
#                       Функция получения облака точек из LaserScan
##################################################################################
def scan_cb(msg):
    global point_list
    # избавляемся от inf
    without_inf = list()
    for k in range(0, len(msg.ranges)):
        if np.isinf(msg.ranges[k]) or msg.ranges[k] > max_dist_lidar:
            without_inf.append(max_dist_lidar)
        else:
            without_inf.append(msg.ranges[k])
    # инвертируем сонар без inf
    msg.ranges = without_inf

    # получаем курс в градусах
    course = int(np.rad2deg(current_course))
    # поворачиваем массив расстояний с лидара(360 знач) на курс в градусах
    msg.ranges = msg.ranges[-course:] + msg.ranges[:-course]
    # convert the message of type LaserScan to a PointCloud2
    pc2_msg = lp.projectLaser(msg)
    # convert it to a pointlist
    point_list = pc2.read_points_list(pc2_msg)


##################################################################################
#                       Функция получения облака точек из Octomap
##################################################################################
def pointcloud_cb(msg):
    global point_list
    # конвертируем PointCloud2 в List
    # del point_list[:]
    point_list = pc2.read_points_list(msg)


##################################################################################
#                       Функция получения координат от других дронов
##################################################################################
def collbackDroneList(data):
    """
    Формируем список препятсвий из позиций дрона
    :type data: DroneInfoArray
    :type point_list: list
    :return:
    """
    global point_list, drone_id
    del point_list[:]

    for i in data.drones:
        # Не учитываем собственное положение
        if i.id_drone == drone_id:
            print "i.id_drone == drone_id"
            continue
        point = [i.pose.point.x,i.pose.point.y,i.pose.point.z]
        point_list.append(point)

    print("Get list of drones: %s" %(len(point_list)))


def set_active_srv(req):
    """
    Change state of planner
    """
    global active_flag, cfg_srv
    active_flag = req.data
    print("Change state:", active_flag)
    resp = SetBoolResponse()
    resp.success = True

    if cfg_srv is not None:
        # write in dyn. rec.
        cfg_srv.update_configuration({'run': active_flag})

    return resp


##################################################################################
#                       Функция получения вектора по двум точкам
##################################################################################
def get_vector_point2(point_drone, point_obs,z_free = 1.5):
    # Если точка не лежит в одной плоскости с дроном, то её не учитываем
    if abs(point_drone[2] - point_obs[2]) > z_free:
        return None
    return [point_drone[0] - point_obs[0], point_drone[1] - point_obs[1], 0.]

##################################################################################
#                       Функция получения вектора по трём точкам
##################################################################################
def get_vector_point3(point_drone, point_obs, z_free = 1.5):
    # Если точка не лежит в одной плоскости с дроном, то её не учитываем
    if abs(point_drone[2] - point_obs[2]) > z_free:
        return None
    return [point_drone[0] - point_obs[0], point_drone[1] - point_obs[1], point_drone[2] - point_obs[2]]



def get_vector_lidar(point_obs):
    return [0.0 - point_obs[0], 0.0 - point_obs[1], 0.0]     # , 0.0 - point_obs[2]]


##################################################################################
#                       Функция вычисления силы отталкивания
##################################################################################
def rep_force(dist_to_obs):
    # вида 1/х
    return c_rep / dist_to_obs - c_rep / r_field


# ---------------------------------------------------------------------------------------------------------------------
# ---------- Геометрические функции

def angle_between(x,y, isYaw = False):
    """
      Функция возвращает угол между двумя двухмерными векторами.

    :param vec1:
    :param vec2:
    :return:
    """

    res = math.atan2(x, y) + math.pi
    if isYaw:
        res -=  math.pi*0.5
    # if res > math.pi:
    #     res -= 2 * math.pi
    # if res < -math.pi:
    #     res += 2 * math.pi

    return res

def setup_vel_market(quat, vel):
    """
    Настройка маркера для отображения в rviz
    :type vec: Vector3
    :param point:
    :return:
    """
    marker = Marker()
    marker.header.frame_id = "/map"
    marker.header.stamp = rospy.get_rostime()
    marker.ns = "pot_planner"
    marker.id = 0
    marker.action = 0
    marker.pose.orientation.x = quat[0]
    marker.pose.orientation.y = quat[1]
    marker.pose.orientation.z = quat[2]
    marker.pose.orientation.w = quat[3]
    marker.scale.x = vel
    marker.scale.y = 0.1
    marker.scale.z = 0.1
    marker.type = Marker.ARROW
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 1.0
    if vel == 0.0:
        marker.color.a = 0.0
    else:
        marker.color.a = 1.0

    marker.pose.position.x = current_pose[0]
    marker.pose.position.y = current_pose[1]
    marker.pose.position.z = current_pose[2]

    return marker

def get_quat_from_vec(vec):
    yaw = angle_between(vec[0],vec[1], False)
    roll = angle_between(vec[1],vec[2], True)

    return tf.transformations.quaternion_from_euler(0., -roll, yaw)

##################################################################################
#                               Главная функция
##################################################################################

if __name__ == '__main__':
    rospy.init_node('potential_planner', anonymous=True)
    rate = rospy.Rate(10)

    yaml_path = rospy.get_param("~yaml_path", yaml_path)
    use_geo_mode = rospy.get_param('~use_geo_mode', use_geo_mode)
    drone_id = rospy.get_param('~drone_id', drone_id)
    input_point2 = rospy.get_param('~use_point2', input_point2)

    print("yaml_path:", yaml_path)
    pps = YamlParams(yaml_path)

    # init dynamic reconfigure server
    cfg_srv = Server(PotPlannerConfig, cfg_callback)
    # переопределение значений сервера из файла
    set_server_value(cfg_srv)
    # переопределение значений файла из сервера

    rospy.loginfo("\n  Potencial planner init\n"
                  "\tinput_drone_list: %s\n"
                  "\tinput_point2: %s\n"
                  "\tinput_laser_scan: %s\n"
                  "\tuse_one_point: %s" %(use_drone_list, input_point2,not input_point2, use_one_point))

    if use_drone_list:
        rospy.Subscriber(drone_list_topic, DroneInfoArray, collbackDroneList)
    else:
        if input_point2:
            # подписываемся на Octomap
            rospy.Subscriber(octomap_topic, PointCloud2, pointcloud_cb)
        else:
            # подписываемся на скан лидара
            rospy.Subscriber(sonar_topic, LaserScan, scan_cb)

    if use_geo_mode:
        rospy.Subscriber(geo_pose_topic, PoseStamped, callbackCurrentPose)
        rospy.Subscriber(geo_goal_topic, Goal, callbackTargetPose)
    else:
        rospy.Subscriber(local_pose_topic, PoseStamped, callbackCurrentPose)
        rospy.Subscriber(local_goal_topic, Goal, callbackTargetPose)


    # подписываемся на текущую позицию дрона
    # rospy.Subscriber(pose_topic, PoseStamped, callbackCurrentPose)
    # подписываемся на текущую целевую точку
    # rospy.Subscriber(goal_topic, Goal, callbackTargetPose)
    # подписываемся на текущую скорость
    rospy.Subscriber(topic_velocity, TwistStamped, callbackVelocity)

    # создаём паблишер для вектора скорости отталкивающих сил
    field_vel_pub = rospy.Publisher("/field_vel", TwistStamped, queue_size=10)
    # паблишер для состояния планировщика
    status_pub = rospy.Publisher("/drone/local_planner/potential/status", Bool, queue_size=10)
    active_pub = rospy.Publisher("/drone/local_planner/potential/active", Bool, queue_size=10)

    marker_pub = rospy.Publisher("/drone/local_planner/potential/marker", Marker, queue_size=10)
    # сервис для отключения планировщика
    rospy.Service("drone/local_planner/potential/set_active", SetBool, set_active_srv)

    rep_vec = [0.0, 0.0, 0.0]
    att_vec = [0.0, 0.0, 0.0]

    while not rospy.is_shutdown():

        active_pub.publish(active_flag)
        if not active_flag:
            status_pub.publish(False)
            rate.sleep()
            continue

        # обнуляем вектора полей
        rep_vec = [0.0, 0.0, 0.0]
        att_vec = [0.0, 0.0, 0.0]

        # point_list меняется бытрее прохода цикла
        # поэтому копируем его в переменную p_list
        # p_list = copy.deepcopy(point_list)

        # Создаём сообщение/обнуляем его в цикле
        vel_msg = TwistStamped()


        if use_one_point:
            # Ищем минимальную точку отталкивания
            # нормализуем облако относительно положения дрона
            del norm_points[:]

            for i in point_list:
                # Получаем вектор между препятствием и координатой дрона
                if input_point2 or use_drone_list:
                    if use_3d_avoid:
                        val = get_vector_point3(current_pose, i)
                    else:
                        val = get_vector_point2(current_pose, i)

                    if val is not None:
                        norm_points.append(val)
                else:
                    norm_points.append(get_vector_lidar(i))

            if len(norm_points) > 0:
                norm_array = np.linalg.norm(norm_points, axis=1)
                min_point_index = np.where(norm_array == np.min(norm_array))[0][0]
                mod_vec = norm_array[min_point_index]

                # Если расстояние до точки меньше порогового
                if mod_vec <= r_field:
                    # Нормализуем вектор и множим на силу поля отталкивания
                    norm_vec = (norm_points[min_point_index] / mod_vec * (rep_force(mod_vec)) + (k_vel * mod_vel))
                    # суммируем вектора отталкивания
                    rep_vec[0] = norm_vec[0]
                    rep_vec[1] = norm_vec[1]
                    rep_vec[2] = norm_vec[2]

        else:
            # Пробегаемся по всем точкам облака
            for i in point_list:
                # Получаем вектор между препятствием и координатой дрона
                if input_point2 or use_drone_list:
                    vec_i = get_vector_point2(current_pose, i)
                else:
                    vec_i = get_vector_lidar(i)
                print(vec_i)

                # Вычисляем длину вектора (расстояние до точки)
                mod_vec = np.linalg.norm(vec_i)

                # Если расстояние до точки меньше порогового
                if mod_vec <= r_field:
                    # Нормализуем вектор и множим на силу поля отталкивания
                    norm_vec = vec_i / mod_vec * (rep_force(mod_vec) + (k_vel * mod_vel))
                    # суммируем вектора отталкивания
                    rep_vec[0] += norm_vec[0]
                    rep_vec[1] += norm_vec[1]
                    rep_vec[2] += norm_vec[2]
                # если рассттояние до точки больше порогового + коридор
                # elif mod_vec > r_porog + r_kor:
                # Нормализуем вектор и множим на силу поля притяжения
                # norm_vec = vec_i / mod_vec * att_force(mod_vec)
                # суммируем вектора притяжения
                # att_vec[0] += norm_vec[0]
                # att_vec[1] += norm_vec[1]
                # att_vec[2] += norm_vec[2]

        # Поворачиваем угол оттакливания
        if angle_vec != 0:
            rep_vec = rotate_vect(rep_vec, np.deg2rad(angle_vec))

        # Прибавляем вектора к сообщению
        vel_msg.twist.linear.x += (rep_vec[0] + att_vec[0])
        vel_msg.twist.linear.y += (rep_vec[1] + att_vec[1])
        if use_3d_avoid:
            vel_msg.twist.linear.z += (rep_vec[2] + att_vec[2])
        else:
            vel_msg.twist.linear.z = 0.0

        # публикуем сообщение в топик
        field_vel_pub.publish(vel_msg)

        # # pub marker
        # vector = [vel_msg.twist.linear.x, vel_msg.twist.linear.y, vel_msg.twist.linear.z]
        # quat = get_quat_from_vec(vector)
        # marker_pub.publish(setup_vel_market(quat, np.linalg.norm(vector)))

        # если вектор отталкивания равен нулю публикуем 0 иначе 1
        if vel_msg.twist.linear.x == 0.0 and vel_msg.twist.linear.y == 0.0 and vel_msg.twist.linear.z == 0.0:
            status_pub.publish(False)
        else:
            status_pub.publish(True)

        rate.sleep()
