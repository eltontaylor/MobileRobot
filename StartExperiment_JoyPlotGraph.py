#!/usr/bin/env python

import os
import cv2
import pandas as pd
import csv

import numpy as np
import array
import matplotlib.pyplot as plt
from matplotlib import animation
from matplotlib.animation import FuncAnimation
from collections import namedtuple

from psychopy import core
import sys
import xlwt
from copy import copy
from math import cos, sin, atan, asin, pi

import rospy
from sensor_msgs.msg import Joy

import time
import os 


global N, ind, width
global number1, number2, number3, graphData

fig = plt.figure()
ax = plt.axes(xlim=(0, 2), ylim=(0, 2))

N = 3
height = np.arange(N)
ind = np.arange(N)
width = 0.35
graphData = None

bar1, = ax.plot(ind[0], height[0], lw =2)
bar2, = ax.plot(ind[1], height[1], lw = 2)
bar3, = ax.plot(ind[2], height[2], lw = 2)

number1 = 0 #Front
number2 = 2 #Right
number3 = 4 #Left

#while graphData is None:

def init():
    bar1.set_data(ind[0],height[0])
    bar2.set_data(ind[1],height[1])
    bar3.set_data(ind[2],height[2])
    return bar1, bar2, bar3,


def animate(i, number1, number2, number3):
    global y1s, y2s, y3s
    graphData = open("/home/sinapse/Desktop/RewardData/LIDARData.txt", "r").read()

    plt.cla()
    ax.set_ylim(0,2)
    N = 3
    height = np.arange(N)

    ax.set_title('Movement indication of the robot')
    ax.set_ylabel('Movement speed')
    ax.set_xlabel('Directions')

    ax.set_xticks(ind)
    ax.set_xticklabels(('Front','Right','Left'))
    ax.set_yticks(ind)
    ax.set_yticklabels(('Stop','Slow Down','Normal')) #3 = stop, 2 = slow down, 1 = stop
    
    #i is perpetually increasing by 1
    #number1, number2, number3 prints 0,2,4 at the start
    number1 += i*6
    number2 += i*6
    number3 += i*6
    #print(number1, number2, number3) #multiples of 6 to fit array position locations
    print('iteration:' , i)
    print('number 1:', number1)
    """
    if ((y1s or y2s or y3s) == ('\n')):
        number1 -= i*6
        number2 -= i*6
        number3 -= i*6
        y1s = graphData[number1]
        y2s = graphData[number2]
        y3s = graphData[number3]
        print('im empty')"""
    
    y1s = graphData[number1]
    y2s = graphData[number2]
    y3s = graphData[number3]
    
    if(y1s == '1'):
        height[0] = 2
    elif(y1s == '2'):
        height[0] = 1
    elif(y1s == '3'):
        height[0] = 0
    
    if (y2s == '1'):
        height[1] = 2
    elif (y2s == '2'):
        height[1] = 1
    elif (y2s == '3'):
        height[1] = 0

    if (y3s == '1'):
        height[2] = 2
    elif (y3s == '2'):
        height[2] = 1
    elif (y3s == '3'):
        height[2] = 0

    print(height[0], height[1], height[2])

    p1 = ax.bar(ind[0], height[0], width, color = 'b', label = 'Front', linewidth = 2)
    p2 = ax.bar(ind[1], height[1], width, color = 'r', label = 'Right', linewidth = 2)
    p3 = ax.bar(ind[2], height[2], width, color = 'g', label = 'Left', linewidth = 2)
    

    bar1.set_data(ind[0], height[0])
    bar2.set_data(ind[1], height[1])
    bar3.set_data(ind[2], height[2])
    
    print(y1s, y2s, y3s)
    
    return bar1, bar2, bar3,

#NOTE: the frames affect the number times the function repeats. Eg: frames = 100 = repeat 100 times and therefore, print till the 100th array position 
anim = animation.FuncAnimation(fig, animate, init_func=init, frames = 1000000, fargs=(number1, number2, number3), interval=40, blit = False)
#anim.save("LIDARAnimation.mp4", fps = 30, extra-args = ['vcodec',     'libx264'])

ax.set_title('Movement indication of the robot')
ax.set_ylabel('Movement speed')
ax.set_xlabel('Directions')

ax.set_xticks(ind)
ax.set_xticklabels(('Front','Right','Left'))
ax.set_yticks(ind)
ax.set_yticklabels(('Stop','Slow Down','Normal')) #3 = stop, 2 = slow down, 1 = normal
fig.tight_layout()

plt.show()


