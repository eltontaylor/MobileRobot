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
        #print ('\n Test checkpoint C')
        while self.laser_ranges is None:
            try:                
                self.laser_data = rospy.wait_for_message('/base_scan', LaserScan, timeout=5)
                #self.laser_data = scan
                #print ('\n Test checkpoint C1')
                self.laser_ranges = self.laser_data.ranges
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
            #print (self.laser_subs_object.laser_ranges)

            #Process Right LIDAR values data
            right_laser_values = self.laser_subs_object.laser_ranges[0:321]
            SumRight = sum(right_laser_values)
            length_right_laser_values = len(right_laser_values)
            AverageRight = SumRight/length_right_laser_values
            #print('Average Values of Hokuyo Sensor from the Right:' , AverageRight)
            
            #Process Front LIDAR values data
            front_laser_values = self.laser_subs_object.laser_ranges[321:642]
            SumFront = sum(front_laser_values)
            length_front_laser_values = len(front_laser_values)
            AverageFront = SumFront/length_front_laser_values
            #print('Average Values of Hokuyo Sensor from the Front:' , AverageFront)

            #Process Left LIDAR values data
            left_laser_values = self.laser_subs_object.laser_ranges[642:963]
            SumLeft = sum(left_laser_values)
            length_left_laser_values = len(left_laser_values)
            AverageLeft = SumLeft/length_left_laser_values
            #print('Average Values of Hokuyo Sensor from the Left:' , AverageLeft)

def callback(data):
    laser_value = ProcessingJoyData.detect_objects_nearby()
    right = AverageRight
    left = AverageLeft
    front = AverageFront
    twist = Twist()
    global pub
    pub = rospy.Publisher('RosAria/cmd_vel', Twist, queue_size=1)
    
    if (front < 0.5):
        #print ('Objects detected on the front')
        twist.linear.x = 0
        twist.angular.z = 0
        pub.publish(twist)

    else:
        #print ('Clear line of sight on the front')
        twist.linear.x = 0.1*data.axes[1]
        twist.angular.z = 0.1*data.axes[0]
        #print('\n linear: ', twist.linear.x )
        #print('\n angular :', twist.angular.x )
        pub.publish(twist)

    if (right < 0.5):
        #print ('Objects detected on the right')
        twist.linear.x = 0
        twist.angular.z = 0
        pub.publish(twist)

    else:
        #print ('Clear line of sight on the right')
        twist.linear.x = 0.1*data.axes[1]
        twist.angular.z = 0.1*data.axes[0]
        pub.publish(twist)

    if (left < 0.5):
        #print ('Objects detected on the left')
        twist.linear.x = 0
        twist.angular.z = 0
        pub.publish(twist)

    else:
        #print ('Clear line of sight on the left')
        twist.linear.x = 0.1*data.axes[1]
        twist.angular.z = 0.1*data.axes[0]
        pub.publish(twist)

def startJoy():
    #rospy.Publisher('RosAria/cmd_vel', Twist, queue_size=10)
    #pub = rospy.Publisher('RosAria/cmd_vel', Twist, queue_size=10)
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
        #print ('\n Test checkpoint E')
        rate.sleep()

