#!/usr/bin/env python
# coding=utf8

import math
import numpy as np
import rospy
import time

import copy
import tools

from geometry_msgs.msg import Point
file_name = "test_15"
tools.file_name = file_name

marker_pose = Point()
marker_pose.x = 5.
marker_pose.y = 5.
marker_pose.z = 1. - 0.09

tools.marker_pose = marker_pose

# ---------------------------------------------------------------------------------------------------------------------
# ---------- Главная функция
def main():
    """
    Основной цикл узла ROS.

    @return: код завершения программы
    """
    # Инициализируем узел ROS
    if tools.ros_init() is False:
        print "error inti"
        exit()

    # Основной цикл
    rate = rospy.Rate(20)

    run_flag = True
    while not rospy.is_shutdown():
        # Сичтываем навигационные данные

        if run_flag is True:
            if tools.arm_state is True:
                tools.update_list()
                print ('timer %s' % tools.timer)
            elif tools.arm_state is False:
                print ("================================")
                error_x = tools.np.abs((tools.position_msgs.pose.position.x - marker_pose.x))
                print "error x", error_x
                error_y = tools.np.abs((tools.position_msgs.pose.position.y - marker_pose.y))
                print "error y", error_y
                print "error xy", tools.marker_error_xy[-1]
                print "error z", tools.marker_error_z[-1]
                print "error", tools.marker_error[-1]
                print "time finish"
                tools.save_on_file(file_name)
                run_flag = False

        # draw plots
        tools.plot_error()
        tools.plot_pose()
        tools.plot_vel()
        tools.plot_ctr_error()

        tools.plt.pause(1.0 / 20.0)

        # rate.sleep()

    return 0


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
