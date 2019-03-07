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
        print ('\n Test checkpoint A0')
        self.init_laser_range()
        rospy.Subscriber('/base_scan', LaserScan, self.LaserData)
        print ('\n Test checkpoint A1')

    def LaserData(self,msg): 
        self.laser_ranges = msg.ranges
        #print ('\n Test checkpoint B')

    def init_laser_range(self):
        self.laser_ranges = None
        #self.laser_ranges = range (0,60)
        print ('\n Test checkpoint C')
        while self.laser_ranges is None:
            try:                
                self.laser_data = rospy.wait_for_message('/base_scan', LaserScan, timeout=5)
                #self.laser_data = scan
                print ('\n Test checkpoint C1')
                self.laser_ranges = self.laser_data.ranges
                print (self.laser_ranges)
            except:
                rospy.loginfo("base_scan not ready yet, retrying ")
                print ('\n Test checkpoint C2')


class processjoy(object):
    def __init__(self):
        self.laser_subs_object = LaserSubs()
        print ('\n Test checkpoint D1')
    def detect_objects_nearby(self):
        if self.laser_subs_object.laser_ranges:

            last_laser_value = self.laser_subs_object.laser_ranges[-1]
            return last_laser_value


def callback(data):
    #PJD = processjoy()
    laser_value = ProcessingJoyData.detect_objects_nearby()
    twist = Twist()
    global pub
    pub = rospy.Publisher('RosAria/cmd_vel', Twist, queue_size=1)
    
    if (laser_value < 0.5):
        print ('\n Objects detected')
        twist.linear.x = 0
        twist.angular.z = 0
        pub.publish(twist)

    else:
        print ('\n Clear line of sight')
        twist.linear.x = 0.1*data.axes[1]
        twist.angular.z = 0.1*data.axes[0]
        print('\n linear: ', twist.linear.x )
        print('\n angular :', twist.angular.x )
        pub.publish(twist)


def startJoy():
    #rospy.Publisher('RosAria/cmd_vel', Twist, queue_size=10)
    #pub = rospy.Publisher('RosAria/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber("joy", Joy, callback)
    #rospy.spin()


if __name__ == '__main__':
    rospy.init_node('base_scan')
    ProcessingJoyData = processjoy()
    #print ('\n Test checkpoint E')
    rate = rospy.Rate(10)
    
    while not rospy.is_shutdown():
        processjoy()
        #ProcessingJoyData.detect_objects_nearby()
        startJoy()
        print ('\n Test checkpoint E')
        rate.sleep()

