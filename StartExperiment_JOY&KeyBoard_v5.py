#!/usr/bin/env python

import os
import cv2
import pandas as pd
import csv
import numpy as np
from psychopy import core
import sys, select, termios, tty
import xlwt
import curses
from copy import copy
from math import cos, sin, atan, asin, pi

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
import time
import os 

class SimpleKeyTeleop():
    def __init__(self, interface):
        self._interface = interface
        self._pub_cmd = rospy.Publisher('RosAria/cmd_vel', Twist)

        self._hz = rospy.get_param('~hz', 10)

        self._forward_rate = rospy.get_param('~forward_rate', 0.8)
        self._backward_rate = rospy.get_param('~backward_rate', 0.5)
        self._rotation_rate = rospy.get_param('~rotation_rate', 1.0)
        self._last_pressed = {}
        self._angular = 0
        self._linear = 0

    movement_bindings = {
        curses.KEY_UP:    ( 1,  0),
        curses.KEY_DOWN:  (-1,  0),
        curses.KEY_LEFT:  ( 0,  1),
        curses.KEY_RIGHT: ( 0, -1),
    }

    def run(self):
        rate = rospy.Rate(self._hz)
        self._running = True
        while self._running:
            while True:
                keycode = self._interface.read_key()
                if keycode is None:
                    break
                self._key_pressed(keycode)
            self._set_velocity()
            self._publish()
            rate.sleep()

    def _get_twist(self, linear, angular):
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        return twist

    def _set_velocity(self):
        now = rospy.get_time()
        keys = []
        for a in self._last_pressed:
            if now - self._last_pressed[a] < 0.4:
                keys.append(a)
        linear = 0.0
        angular = 0.0
        for k in keys:
            l, a = self.movement_bindings[k]
            linear += l
            angular += a
        if linear > 0:
            linear = linear * self._forward_rate
        else:
            linear = linear * self._backward_rate
        angular = angular * self._rotation_rate
        self._angular = angular
        self._linear = linear

    def _key_pressed(self, keycode):
        if keycode == ord('q'):
            self._running = False
            rospy.signal_shutdown('Bye')
        elif keycode in self.movement_bindings:
            self._last_pressed[keycode] = rospy.get_time()

    def _publish(self):
        twist = self._get_twist(self._linear, self._angular)
        self._pub_cmd.publish(twist)



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
    global FrontFlag
    global LeftFlag
    global RightFlag
    global FrontLeftFlag
    global FrontRightFlag
    global appendInfo
    clear = 2.2
    stop = 1.7
    pub = rospy.Publisher('RosAria/cmd_vel', Twist, queue_size=1)
    
    #Front Direction
    #print ('front values:', front)
    if (front > clear):
        #print ('Clear line of sight on the front')
        FrontFlag = 1

    elif (front < clear and front > stop):
        #print ('object in appx 95cm range, slowing down')
        FrontFlag = 2
    
    elif (front < stop):
        #print ('Objects in appx 65cm range, stopping')
        FrontFlag = 3

    #Right Direction
    #print ('right values:', right)
    if (right > clear):
        #print ('Clear line of sight on the right')
        RightFlag = 1

    elif (right < clear and right > stop):
        #print ('object in appx 95cm range, slowing down')
        RightFlag = 2
    
    elif (right < stop):
        #print ('Objects in appx 65cm range, stopping')
        RightFlag = 3

    #Left Direction
    #print ('left values:', left)
    if (left > clear):
        #print ('Clear line of sight on the left')
        LeftFlag = 1

    elif (left < clear and left > stop):
        #print ('object in appx 95cm range, slowing down')
        LeftFlag = 2
    
    elif (left < stop):
        #print ('Objects in appx 65cm range, stopping')
        LeftFlag = 3

    #Front Left Direction
    #print ('front left value:'. FL)
    if (FL > clear):
        #print ('Clear line of sight on the front left side')
        FrontLeftFlag = 1

    elif (FL < clear and FL > stop):
        #print ('object in appx 95cm range, slowing down')
        FrontLeftFlag = 2
    
    elif (FL < stop):
        #print ('Objects in appx 65cm range, stopping')
        FrontLeftFlag = 3

    #Front right Direction
    #print ('front right value:'. FR)
    if (FR > clear):
        #print ('Clear line of sight on the front right side')
        FrontRightFlag = 1

    elif (FR < clear and FR > stop):
        #print ('object in appx 95cm range, slowing down')
        FrontRightFlag = 2
    
    elif (FR < stop):
        #print ('Objects in appx 65cm range, stopping')
        FrontRightFlag = 3

    print ('FrontFlag:', FrontFlag , 'RightFlag:', RightFlag ,'LeftFlag:', LeftFlag , 'FrontLeftFlag:', FrontLeftFlag , 'FrontRightFlag:', FrontRightFlag)
    appendInfo = (FrontFlag, RightFlag, LeftFlag, FrontLeftFlag, FrontRightFlag)

    #Data.axis[1] = joystick moving front and back, data.axis[0] = joystick moving left and right
    if(data.axes[1] > 0): #Joystick is indicating to move forward
    	#If Front is clear, move as per normal
        if (FrontFlag == 1):
        	twist.linear.x = 0.2*data.axes[1]
        	pub.publish(twist)
    	#If Front is approaching an object, slow down
    	elif (FrontFlag == 2):
        	twist.linear.x = 0.1*data.axes[1]
        	pub.publish(twist)
    	#If Front senses an object, stop. 
    	elif (FrontFlag == 3):
        	twist.linear.x = 0
        	pub.publish(twist)
    
    #turn left twist.angular.z is positive, turn right twist.angular.z is negative
    #turn left data.axes[0] is positive, turn right, data.axes[0] is negative
    if(data.axes[0] > 0): #Joystick is indicating to move left
        if(FrontLeftFlag == 3 and LeftFlag >= 2):  #FrontLeft detects obstacle, stop
            twist.angular.z = 0 
            pub.publish(twist)
        elif(FrontLeftFlag <= 2 and LeftFlag >= 2): #FrontLeft detects no obstacle but LeftFlag detects obstacle, slow down
            twist.angular.z = 0.1*data.axes[0] #0.1 being set the max value for twist.angular.z
            pub.publish(twist)
        elif(FrontLeftFlag == 1 and LeftFlag == 1): #nothing is blocked, normal speed
            twist.angular.z = 0.2*data.axes[0]
            pub.publish(twist)

    if(data.axes[0] < 0): #Joystick is indicating to move right
        if(FrontRightFlag == 3 and RightFlag >= 2):  #FrontRight detects obstacle, stop
            twist.angular.z = 0 
            pub.publish(twist)
        elif(FrontRightFlag <= 2 and RightFlag >= 2): #FrontRight detects no obstacle but RightFlag detects obstacle, slow down
            twist.angular.z = 0.1*data.axes[0] #0.1 being set the max value for twist.angular.z
            pub.publish(twist)
        elif(FrontRightFlag == 1 and RightFlag == 1): #nothing is blocked, normal speed
            twist.angular.z = 0.2*data.axes[0]
            pub.publish(twist)

    #Reverse movement
    if(data.axes[1] < 0): #Joystick is indicating to move in a reverse direction
        twist.linear.x = 0.2*data.axes[1]
        pub.publish(twist)

    save = saving()

def saving():
    AI = appendInfo
    appendFile = open("/home/sinapse/Desktop/RewardData/LIDARData.txt", "a")
    #Save in a 2 by 2 array or it will print all integers vertically
    np.savetxt(appendFile, [AI], fmt="%d", delimiter=",")  
    appendFile.close()

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
        SimpleKeyTeleop()
        rate.sleep()
