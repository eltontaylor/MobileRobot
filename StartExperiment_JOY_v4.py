import os
import cv2
import pandas as pd
import csv
import numpy as np
from psychopy import core
import sys
from copy import copy
from math import cos, sin, atan, asin, pi

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
import time
import os 

class LaserSubs(object):
    def __init__(self):
        scan = LaserScan()
        self.init_laser_range()
        rospy.Subscriber('/base_scan', LaserScan, self.LaserData)

    def LaserData(self,msg): 
        self.laser_ranges = msg.ranges

    def init_laser_range(self):
        self.laser_ranges = None
        #global starttime
        while self.laser_ranges is None:
            try:                
                self.laser_data = rospy.wait_for_message('/base_scan', LaserScan, timeout=5)
                self.laser_ranges = self.laser_data.ranges
                starttime = time.time()
                time.sleep(0.05)
            except:
                print ('Waiting for base_scan to be ready')
                time.sleep(0.05)

class processjoy(object):
    def __init__(self):
        self.laser_subs_object = LaserSubs()
    def detect_objects_nearby(self):
        if self.laser_subs_object.laser_ranges:
            global AverageRight
            global AverageFront
            global AverageLeft
            global AverageFR
            global AverageFL

            #Process Right LIDAR values data
            right_laser_values = self.laser_subs_object.laser_ranges[0:193]
            SumRight = sum(right_laser_values)
            length_right_laser_values = len(right_laser_values)
            AverageRight = SumRight/length_right_laser_values
            #print('Average Values of Hokuyo Sensor from the Right:' , AverageRight)

            #Process Right & Front LIDAR values data
            FR_laser_values = self.laser_subs_object.laser_ranges[193:386]
            SumFR = sum(FR_laser_values)
            length_FR_laser_values = len(FR_laser_values)
            AverageFR = SumFR/length_FR_laser_values
            #print('Average Values of Hokuyo Sensor from the Right:' , AverageFR)
            
            #Process Front LIDAR values data
            front_laser_values = self.laser_subs_object.laser_ranges[386:579]
            SumFront = sum(front_laser_values)
            length_front_laser_values = len(front_laser_values)
            AverageFront = SumFront/length_front_laser_values
            #print('Average Values of Hokuyo Sensor from the Front:' , AverageFront)
            #time.sleep(1)

            #Process Left & Front LIDAR values data
            FL_laser_values = self.laser_subs_object.laser_ranges[579:772]
            SumFL = sum(FL_laser_values)
            length_FL_laser_values = len(FL_laser_values)
            AverageFL = SumFL/length_FL_laser_values
            #print('Average Values of Hokuyo Sensor from the Front:' , AverageFL)

            #Process Left LIDAR values data
            left_laser_values = self.laser_subs_object.laser_ranges[772:963]
            SumLeft = sum(left_laser_values)
            length_left_laser_values = len(left_laser_values)
            AverageLeft = SumLeft/length_left_laser_values
            #print('Average Values of Hokuyo Sensor from the Left:' , AverageLeft)

def callback(data):
    laser_value = ProcessingJoyData.detect_objects_nearby()

    #e = int(time.time() - starttime)
    #print('{:02d}:{:02d}:{:02d}'.format(e//3600, (e % 3600//60), e%60))
    
    right = AverageRight
    left = AverageLeft
    front = AverageFront
    FL = AverageFL
    FR = AverageFR
    twist = Twist()
    global pub
    #global flags for future usage when integrating LIDAR with AMCL while Joy is a standalone
    global FrontFlag
    global LeftFlag
    global RightFlag
    global FrontLeftFlag
    global FrontRightFlag
    pub = rospy.Publisher('RosAria/cmd_vel', Twist, queue_size=1)
    
    #Front Direction
    #print ('front values:', front)
    if (front > 2.5):
        #print ('Clear line of sight on the front')
        FrontFlag = 1

    elif (front < 2.5 and front > 2.3):
        #print ('object in 1.5 metre range, slowing down')
        FrontFlag = 2
    
    elif (front < 2.3):
        #print ('Objects in 1 metre range, stopping')
        FrontFlag = 3

    #Right Direction
    #print ('right values:', right)
    if (right > 2.5):
        #print ('Clear line of sight on the right')
        RightFlag = 1

    elif (right < 2.5 and right > 2.3):
        #print ('object in 1.5 metre range, slowing down')
        RightFlag = 2
    
    elif (right < 2.3):
        #print ('Objects in 1 metre range, stopping')
        RightFlag = 3

    #Left Direction
    #print ('left values:', left)
    if (left > 2.5):
        #print ('Clear line of sight on the left')
        LeftFlag = 1

    elif (left < 2.5 and left > 2.3):
        #print ('object in 1.5 metre range, slowing down')
        LeftFlag = 2
    
    elif (left < 2.3):
        #print ('Objects in 1 metre range, stopping')
        LeftFlag = 3

    #Front Left Direction
    #print ('front left value:'. FL)
    if (FL > 2.5):
        #print ('Clear line of sight on the front left side')
        FrontLeftFlag = 1

    elif (FL < 2.5 and FL > 2.3):
        #print ('object in 1.5 metre range, slowing down')
        FrontLeftFlag = 2
    
    elif (FL < 2.3):
        #print ('Objects in 1 metre range, stopping')
        FrontLeftFlag = 3

    #Front right Direction
    #print ('front right value:'. FR)
    if (FR > 2.5):
        #print ('Clear line of sight on the front right side')
        FrontRightFlag = 1

    elif (FR < 2.5 and FR > 2.3):
        #print ('object in 1.5 metre range, slowing down')
        FrontRightFlag = 2
    
    elif (FR < 2.3):
        #print ('Objects in 1 metre range, stopping')
        FrontRightFlag = 3

    print ('FrontFlag:', FrontFlag , 'RightFlag:', RightFlag ,'LeftFlag:', LeftFlag , 'FrontLeftFlag:', FrontLeftFlag , 'FrontRightFlag:', FrontRightFlag)
    
    if (FrontFlag == 1 and FrontLeftFlag <= 3 and FrontRightFlag <= 3):
        twist.linear.x = 0.2*data.axes[1]
        pub.publish(twist)

    elif (FrontFlag == 2 and FrontLeftFlag <= 3 and FrontRightFlag <= 3):
        twist.linear.x = 0.1*data.axes[1]
        pub.publish(twist)

    elif (FrontFlag == 3 and FrontLeftFlag <= 3 and FrontRightFlag <= 3):
        twist.linear.x = 0
        pub.publish(twist)
    
    if (LeftFlag == 1 or RightFlag == 1 or FrontFlag == 3):
        twist.angular.z = 0.2*data.axes[0]
        pub.publish(twist)

    elif (LeftFlag == 2 or RightFlag == 2):
        twist.angular.z = 0.1*data.axes[0]
        pub.publish(twist)

    elif (LeftFlag == 3 or RightFlag == 3):
        twist.angular.z = 0 
        pub.publish(twist)

    if (FrontLeftFlag == 1 or FrontRightFlag == 1):
        FrontFlag = 1
        LeftFlag = 1
        RightFlag = 1        

    elif (FrontLeftFlag == 2 or FrontRightFlag == 2):
        FrontFlag = 2
        LeftFlag = 2
        RightFlag = 2

    elif (FrontLeftFlag == 3 and FrontRightFlag == 3):
        LeftFlag = 2
        RightFlag = 2


def startJoy():
    rospy.Subscriber("joy", Joy, callback)
    rospy.spin()


if __name__ == '__main__':
    rospy.init_node('base_scan')
    ProcessingJoyData = processjoy()
    rate = rospy.Rate(10)
    
    while not rospy.is_shutdown():
        processjoy()
        startJoy()
        rate.sleep()

