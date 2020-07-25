#!/usr/bin/env python
# coding=utf8

import math
import numpy as np
import rospy
import time

import matplotlib.pyplot as plt  # creation plots
from mpl_toolkits.mplot3d import Axes3D

x_ = list()
y_ = list()
z_ = list()

def read_pose(file_name,x_, y_, z_):
    fo = open(file_name, "r+")
    del x_[:]
    del y_[:]
    del z_[:]

    for i in range(9):
        if i == 6:
            line = fo.readline()
            split_line = str.split(line, '\t')
            for k in range(1,len(split_line)-1):
                x_.append(float(split_line[k]))
        elif i == 7:
            line = fo.readline()
            split_line = str.split(line, '\t')
            for k in range(1,len(split_line)-1):
                y_.append(float(split_line[k]))
        elif i == 8:
            line = fo.readline()
            split_line = str.split(line, '\t')
            for k in range(1,len(split_line)-1):
                z_.append(float(split_line[k]))
        else:
            fo.readline()
    fo.close()
    return x_, y_, z_



if __name__ == '__main__':
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    file_name = "./tests/test_15.txt"

    read_pose(file_name, x_, y_,z_)

    # ax.plot(x_[10:], y_[10:], z_[10:], "b")
    # ax.plot(x_[10:], y_[10:], z_[10:],'rx')
    ax.plot(x_, y_, z_, "b")
    ax.plot(x_, y_, z_,'rx')

    ax.set_xlabel("x,m")
    ax.set_ylabel("y,m")
    ax.set_zlabel("z,m")

    ax.legend()

    plt.show()