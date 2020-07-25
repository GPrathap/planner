#!/usr/bin/env python
# coding=utf8

import math
import numpy as np
import rospy
import time
import random

import matplotlib.pyplot as plt  # creation plots


x_error = [0.0548553466796875, 0.07711076736450195, 0.07092094421386719, 0.023231983184814453, 0.04814624786376953,
           0.07869815826416016, 0.0801701545715332, 0.034079551696777344, 0.01596975326538086, 0.07865428924560547,
           0.05037403106689453, 0.09667015075683594, 0.019424915313720703, 0.027704238891601562, 0.020244598388671875]
y_error = [0.01011800765991211,0.02094554901123047,0.028472900390625, 0.06108856201171875, 0.008256912231445312,
           0.02553415298461914, 0.11565208435058594,0.012656211853027344, 0.01596975326538086, 0.015903472900390625,
           0.042120933532714844, 0.01617908477783203, 0.03708696365356445, 0.057756900787353516, 0.013055801391601562]


def positive_or_negative():
    if random.random() < 0.5:
        return 1
    else:
        return -1

if __name__ == '__main__':

    for i in range(len(x_error)):
      x_error[i] *= positive_or_negative()

    for i in range(len(y_error)):
      y_error[i] *= positive_or_negative()

    ax1 = plt.subplot2grid((1, 1), (0, 0))
    ax1.plot(x_error, y_error, 'o')
    ax1.plot([0,0], [1,-1], 'r-', label='drone z')
    ax1.plot([-1,1], [0,0], 'r-', label='drone z')

    plt.xlim(-.15, 0.15)
    plt.ylim(-.15, 0.15)

    ax1.set_ylabel('y, m')
    ax1.set_xlabel('x, m')
    ax1.grid()


    plt.show()

