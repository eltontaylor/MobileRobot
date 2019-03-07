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
        #print ('\n Test checkpoint A0')
        self.init_laser_range()
        rospy.Subscriber('/base_scan', LaserScan, self.LaserData)
        #print ('\n Test checkpoint A1')

    def LaserData(self,msg): 
        self.laser_ranges = msg.ranges
        #print ('\n Test checkpoint B')

    def init_laser_range(self):
        self.laser_ranges = None
        global starttime
        #print ('\n Test checkpoint C')
        while self.laser_ranges is None:
            try:                
                self.laser_data = rospy.wait_for_message('/base_scan', LaserScan, timeout=5)
                #self.laser_data = scan
                #print ('\n Test checkpoint C1')
                self.laser_ranges = self.laser_data.ranges
                starttime = time.time()
                time.sleep(0.05)
                #print (self.laser_ranges)
            except:
                #rospy.loginfo("base_scan not ready yet, retrying ")
                print ('Waiting for base_scan to be ready')
                time.sleep(0.05)

class processjoy(object):
    def __init__(self):
        self.laser_subs_object = LaserSubs()
        #print ('\n Test checkpoint D1')
    def detect_objects_nearby(self):
        if self.laser_subs_object.laser_ranges:
            global AverageRight
            global AverageFront
            global AverageLeft
            global AverageFR
            global AverageFL
            #print (self.laser_subs_object.laser_ranges)

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

    e = int(time.time() - starttime)
    print('{:02d}:{:02d}:{:02d}'.format(e//3600, (e % 3600//60), e%60))
    
    right = AverageRight
    left = AverageLeft
    front = AverageFront
    FL = AverageFL
    FR = AverageFR
    twist = Twist()
    global pub
    pub = rospy.Publisher('RosAria/cmd_vel', Twist, queue_size=1)
    
    #Front Direction
    #print ('front values:', front)
    if (front > 2.5):
        #print ('Clear line of sight on the front')
        twist.linear.x = 0.2*data.axes[1]
        twist.angular.z = 0.2*data.axes[0]
        pub.publish(twist)
        print ('front clear')
        time.sleep(0.05)

    elif (front < 2.5 and front > 2.3):
        #print ('object in 1.5 metre range, slowing down')
        twist.linear.x = 0.05*data.axes[1]
        twist.angular.z = 0.2*data.axes[0]
        pub.publish(twist)
        print ('front slow')
        time.sleep(0.05)
    
    elif (front < 2.3):
        #print ('Objects in 1 metre range, stopping')
        twist.linear.x = 0
        twist.angular.z = 0.2*data.axes[0]
        pub.publish(twist)
        print ('front stop')
        time.sleep(0.05)

    #Right Direction
    #print ('right values:', right)
    if (right > 2.5):
        #print ('Clear line of sight on the right')
        twist.linear.x = 0.2*data.axes[1]
        twist.angular.z = 0.2*data.axes[0]
        pub.publish(twist)
        print ('right')
        time.sleep(0.05)

    elif (right < 2.5 and right > 2.3):
        #print ('object in 1.5 metre range, slowing down')
        twist.linear.x = 0.2*data.axes[1]
        twist.angular.z = 0.05*data.axes[0]
        pub.publish(twist)
        print ('right slow')
        time.sleep(0.05)
    
    elif (right < 2.3):
        #print ('Objects in 1 metre range, stopping')
        twist.linear.x = 0.2*data.axes[1]
        twist.angular.z = 0
        pub.publish(twist)
        print ('right stop')
        time.sleep(0.05)

    #Left Direction
    #print ('left values:', left)
    if (left > 2.5):
        #print ('Clear line of sight on the left')
        twist.linear.x = 0.2*data.axes[1]
        twist.angular.z = 0.2*data.axes[0]
        pub.publish(twist)
        print ('left')
        time.sleep(0.05)

    elif (left < 2.5 and left > 2.3):
        #print ('object in 1.5 metre range, slowing down')
        twist.linear.x = 0.2*data.axes[1]
        twist.angular.z = 0.05*data.axes[0]
        pub.publish(twist)
        print ('left slow')
        time.sleep(0.05)
    
    elif (left < 2.3):
        #print ('Objects in 1 metre range, stopping')
        twist.linear.x = 0.2*data.axes[1]
        twist.angular.z = 0
        pub.publish(twist)
        print ('left stop')
        time.sleep(0.05)

    #Front Left Direction
    #print ('front left value:'. FL)
    if (FL > 2.5):
        #print ('Clear line of sight on the front left side')
        twist.linear.x = 0.2*data.axes[1]
        twist.angular.z = 0.2*data.axes[0]
        pub.publish(twist)
        time.sleep(0.05)

    elif (FL < 2.5 and FL > 2.3):
        #print ('object in 1.5 metre range, slowing down')
        twist.linear.x = 0.2*data.axes[1]
        twist.angular.z = 0.2*data.axes[0]
        pub.publish(twist)
        time.sleep(0.05)
    
    elif (FL < 2.3):
        #print ('Objects in 1 metre range, stopping')
        twist.linear.x = 0.1*data.axes[1]
        twist.angular.z = 0.1*data.axes[0]
        pub.publish(twist)
        print ('front left stop')
        time.sleep(0.05)

    #Front right Direction
    #print ('front right value:'. FR)
    if (FR > 2.5):
        #print ('Clear line of sight on the front right side')
        twist.linear.x = 0.2*data.axes[1]
        twist.angular.z = 0.2*data.axes[0]
        pub.publish(twist)
        time.sleep(0.05)

    elif (FR < 2.5 and FR > 2.3):
        #print ('object in 1.5 metre range, slowing down')
        twist.linear.x = 0.2*data.axes[1]
        twist.angular.z = 0.2*data.axes[0]
        pub.publish(twist)
        time.sleep(0.05)
    
    elif (FR < 2.3):
        #print ('Objects in 1 metre range, stopping')
        twist.linear.x = 0.1*data.axes[1]
        twist.angular.z = 0.1*data.axes[0]
        pub.publish(twist)
        print ('front right stop')
        time.sleep(0.05)

def startJoy():
    #rospy.Publisher('RosAria/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber("joy", Joy, callback)
    rospy.spin()


if __name__ == '__main__':
    rospy.init_node('base_scan')
    ProcessingJoyData = processjoy()
    #print ('\n Test checkpoint E')
    rate = rospy.Rate(10)
    
    while not rospy.is_shutdown():
        processjoy()
        #ProcessingJoyData.detect_objects_nearby()
        startJoy()
        rate.sleep()

