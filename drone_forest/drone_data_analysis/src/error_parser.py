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

def read_pose(file_name):
    fo = open(file_name, "r+")
    value = 0.
    for i in range(5):
        if i == 4:
            line = fo.readline()
            split_line = str.split(line, '\t')
            value = float(split_line[1])
        else:
            fo.readline()
    fo.close()
    return value


if __name__ == '__main__':
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    file_name = "./tests/test_15.txt"
    for i in range(1,16):
        file_name = "./tests/test_%s.txt" % i
        print "%s:" % i, read_pose(file_name)

