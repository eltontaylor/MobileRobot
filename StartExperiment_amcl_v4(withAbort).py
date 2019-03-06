#!/usr/bin/env python
import os
import cv2
import pandas as pd
import csv
import numpy as np
from psychopy import core
import sys
from copy import copy
from math import cos, sin, atan, asin, pi
from PyQt5 import QtCore, QtGui, QtWidgets

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
import gui 
import time
import os 


class Experiment(object):
      
    
    def __init__(self, NumberOfTrials, RewardTimeOut, Orientation,Angle_tolerance, Position_tolerance, Stay_duration, Map, RewardLocationCSV, ExperimentParamFolder):
        """ Initialise user input
                Params :
                NumberOfTrials : number of trials in experiment
                RewardTimeout : number of seconds after which the trial ends and no reward is given
                Map : map of the area stored as .pgm
                RewardLocationCSV : csv file that contains reward locations
                ExperimentParamFolder : folder to save experiment related parameters

        """
        self.ExperimentParamFolder = ExperimentParamFolder
        cv2.namedWindow('CheckRewardLocation', cv2.WINDOW_NORMAL)
        self.map = cv2.imread(Map)
        print(np.shape(self.map))
        self.rewardlocation = pd.read_csv(RewardLocationCSV, index_col=0)
        print 'Number of saved reward locations loaded : ', len(self.rewardlocation)
        self.plot_saved_location()

        # Initialise Trial parameters
        self.numtrials = NumberOfTrials
        self.rewardtimeout = RewardTimeOut
        self.orientation = Orientation
        self.angle_tolerance=Angle_tolerance
        self.position_tolerance=Position_tolerance
        self.stay_duration=Stay_duration
        self._currenttrial = 0  # Trial number
        self.start_experiment()
    
  
        

    def plot_saved_location(self):
        """ Check if the saved reward locations are correct"""
        locationmap = copy(self.map)
        for index, row in self.rewardlocation.iterrows():
            cv2.circle(locationmap, (row['x1'], row['y1']), 20, (0, 0, 255), -1)
            cv2.putText(locationmap, '(%d, %d)' % (row['x1'], row['y1']), (row['x1'], row['y1']), cv2.FONT_HERSHEY_SIMPLEX,
                        2, (0, 0, 0), 4)
           
        print('Press Esc if reward locations are correct')
        while True:
            cv2.imshow('CheckRewardLocation', locationmap)
            if cv2.waitKey(20) & 0xFF == 27:
                break
        cv2.imwrite(os.path.join(self.ExperimentParamFolder, 'RewardLocationsused.tif'), locationmap)
        cv2.destroyAllWindows()
        

    def updatetrial(self):
        """ Update trial """
        #i = gui.Main()
        #x = i.get_settings()
        #min = x.get("maze_ITI_Min")
        #max = x.get("maze_ITI_Max")
        min = 5 #in seconds
        max = 10 # in seconds
        delay = np.random.uniform(min,max)
        print('min & max', min, max)
        print ('Inter Trial Interval (in seconds): ', delay)
        time.sleep(delay)
        self._currenttrial += 1


    def start_experiment(self):
        global currentrewardlocation
        global angle_desired

        """ Display a reward location and start timer"""
        while self._currenttrial < self.numtrials:
            
        
            print ('Trial %d beginning...' % self._currenttrial)
            
            
            currentreward = np.random.choice(len(self.rewardlocation), 1)
            currentrewardlocation = self.rewardlocation.iloc[currentreward]
            
            small_rotation=atan(abs(currentrewardlocation['y1']-currentrewardlocation['y2']) / abs(currentrewardlocation['x1']-currentrewardlocation['x2']))
            #print (currentrewardlocation['y1'],currentrewardlocation['y2'],currentrewardlocation['x1'],currentrewardlocation['x2'])
            #print (small_rotation)
            if (int(currentrewardlocation['y1']-currentrewardlocation['y2'])<=0) & (int(currentrewardlocation['x1']-currentrewardlocation['x2'])<=0):
                
                angle_desired=2*pi-small_rotation
                
            elif (int(currentrewardlocation['y1']-currentrewardlocation['y2'])>=0) & (int(currentrewardlocation['x1']-currentrewardlocation['x2'])>=0):
                angle_desired=pi-small_rotation
            
            elif (int(currentrewardlocation['y1']-currentrewardlocation['y2'])<=0) & (int(currentrewardlocation['x1']-currentrewardlocation['x2'])>=0):
                
                angle_desired=pi+small_rotation
                
            else:
                angle_desired=small_rotation
                
           
                    
            # Start trial - show reward
            print ('Go to reward location (%d, %d)' % (currentrewardlocation['x1'], currentrewardlocation['y1']))
            
           
            self.plotrewardlocation(currentrewardlocation)

            # Set up plot for realtime plotting
            cv2.namedWindow('Trial %d Reward' % self._currenttrial, cv2.WINDOW_NORMAL)
            cv2.resizeWindow('Trial %d Reward' % self._currenttrial, 800, 800)

            
            # Start timer for reward timeout
            
            timer = core.CountdownTimer(self.rewardtimeout)
            abstime = core.MonotonicClock()
            
            
            found=0
            check_duration=0
        
            elapsedtime = abstime.getTime()
            os.system('play -n synth %s sin %s' % (0.5, frequency))
            self.realtime_elapsedtime(rewardloc=currentrewardlocation,
                                          elapsedtime=elapsedtime, robotloc=(int(loc_x_rotate),int(loc_y_rotate)),foundOrNot=found)
            while (timer.getTime() > 0) & (found==0):
               
               #subcribe to robot location,joy and laser scan data
                rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, get_xy_loc)
                #rospy.Subscriber("joy", Joy, processjoy)
                #rospy.Subscriber("/base_scan", LaserScan, LaserData)

                # Print elapsed time on map and console
                elapsedtime = abstime.getTime()
                self.realtime_elapsedtime(rewardloc=currentrewardlocation,
                                          elapsedtime=elapsedtime, robotloc=(int(loc_x_rotate),int(loc_y_rotate)),foundOrNot=found)
                
                
                #print "\033[K Elapsed Time : ", elapsedtime, "\r",
                sys.stdout.flush()
                
                #if(abs(int(loc_x_rotate) - int(currentrewardlocation['x1']))<30) & (abs(int(loc_y_rotate) - int(currentrewardlocation['y1']))<30):
                #if ((abs(int(loc_x) - int(currentrewardlocation['x']))<40) & (abs(int(loc_y) - int(currentrewardlocation['y']))<40) & (abs(orien_z-self.orientation)<0.333)):
                    #print (abs(int(loc_x) - int(currentrewardlocation['x'])))
                if (abs(int(loc_x_rotate) - int(currentrewardlocation['x1']))<self.position_tolerance) & (abs(int(loc_y_rotate) - int(currentrewardlocation['y1']))<self.position_tolerance) & (abs(angle_desired-angle_current)<self.angle_tolerance) :   
                    check_duration = check_duration + 1
                    #print ('duration %d' %check_duration)
            
                    if (check_duration==self.stay_duration):
                        found=1
                        self.realtime_elapsedtime(rewardloc=currentrewardlocation,
                                          elapsedtime=elapsedtime, robotloc=(int(loc_x_rotate),int(loc_y_rotate)),foundOrNot=found)
                        
                        for x in range(0, 6):
                            
                            os.system('play -n synth %s sin %s' % (0.2, frequency))

                            time.sleep(.2)
                        print('\n Trial Ended; Target %s Found' % self._currenttrial)
                        break
                
                else:
                    check_duration=0
                  


            if(found==0):
                print('\n Trial Ended; Time Out')
                self.realtime_elapsedtime(rewardloc=currentrewardlocation,
                                          elapsedtime=elapsedtime, robotloc=(int(loc_x_rotate),int(loc_y_rotate)),foundOrNot=2)
                for x in range(0, 12): 
                    os.system('play -n synth %s sin %s' % (0.15, (frequency-x*100)))
                    
            cv2.destroyAllWindows()
            self.updatetrial()  # Trial ended - start of next trial

    def realtime_elapsedtime(self, rewardloc, elapsedtime, robotloc,foundOrNot):
        """ Plot elapsed time, current robot location and reward location. Plot every refresh rate """
        locationmap = copy(self.map)
       
        cv2.circle(locationmap, (rewardloc['x1'], rewardloc['y1']), 15, (0, 0, 255), 6)
        
        cv2.putText(locationmap, 'Time : %0.4f' % elapsedtime, (20, 200), cv2.FONT_HERSHEY_SIMPLEX,
                    4, (0, 0, 0), 10)
       
        #print robot current location on map
        cv2.circle(locationmap, robotloc, 5, (65, 140, 55), 10)
        if foundOrNot==1:
            cv2.putText(locationmap, '***Target Found***',(50,2640), cv2.FONT_HERSHEY_SIMPLEX,4, (255, 100, 0), 15)
            cv2.rectangle(locationmap, (10, 2500),(1300,2730) , (128, 62, 128),30)
            
        if foundOrNot==2:
            cv2.putText(locationmap, '***Fail! Timeout***',(50,2640), cv2.FONT_HERSHEY_SIMPLEX,4, (0, 180, 0), 15)
            cv2.rectangle(locationmap, (10, 2500),(1300,2730) , (128, 62, 128),30)
    
        cv2.imshow('Trial %d Reward' % self._currenttrial, locationmap)
       
        # waitKey(Delay) in milliseconds
        # HighGui functions like imshow() need a call of waitKey, in order to process its event loop.
        # Plotting will be at refresh rate. Will not affect timing
        cv2.waitKey(1)

        #Abort current experiment by pressing 'A'
        if (cv2.waitKey(1) & 0xFF == ord('q')):
            cv2.destroyAllWindows()
            print("aborted")  
            sys.exit()

    def plotrewardlocation(self, rewardloc):
        """ Plot the reward location on the Map"""
        locationmap = copy(self.map)
        cv2.namedWindow('Trial %d Reward' % self._currenttrial, cv2.WINDOW_NORMAL)
        cv2.circle(locationmap, (rewardloc['x1'], rewardloc['y1']), 10, (0, 0, 255), 20)
        cv2.imshow('Trial %d Reward' % self._currenttrial, locationmap)
        cv2.waitKey(2000)  # Show plot for 5 seconds
        cv2.destroyAllWindows()


def get_xy_loc(msg):
        
        #declare them as global variable
        global loc_x
        global loc_y
        global orien_z
        global orien_w
        global loc_x_rotate
        global loc_y_rotate
        global angle_current
         
        #assign the subsribed position value x&y to loc_x and loc_y
        loc_x=msg.pose.pose.position.x
        loc_y=msg.pose.pose.position.y
        orien_z=msg.pose.pose.orientation.z
        orien_w=msg.pose.pose.orientation.w
            
        print(loc_x,loc_y,orien_z)
           
        #convert the coordinate system from generated map into the reward location map coordinate system
        angle_rotate = -0 #in radiance, anticlockwise is positive
        size_x_map = 1312 #in pixel 1728 no difference when changed 3328
        size_y_map = 2784 #in pixel 1156 no difference when changed 1952
        locs_x_map_origin = -21 #in meter   //map7.yaml parameters (-31.4)
        locs_y_map_origin = -46.6 #in meter  //map7.yaml parameters (-23.4)
         
        loc_x=(loc_x - locs_x_map_origin)/0.025
        loc_y=(-locs_y_map_origin - loc_y)/0.025
            
        loc_x_new=loc_x-(size_x_map/2)
        loc_y_new=loc_y-(size_y_map/2)
            
        loc_x_rotate=loc_x_new*(cos(angle_rotate))-loc_y_new*(sin(angle_rotate))+(size_x_map/2)
        loc_y_rotate=loc_y_new*(cos(angle_rotate))+loc_x_new*(sin(angle_rotate))+(size_y_map/2)
            
        if(orien_z>=0) & (orien_w>=0):
            angle_current=asin(orien_z)*2
            
        elif (orien_z>0) & (orien_w<0):
            angle_current=2*pi-asin(orien_z)*2
            
        elif (orien_z<0) & (orien_w>0):
            angle_current=2*pi+asin(orien_z)*2

def SaveExpParameters(ExperimentParamFolder, **kwargs):
    with open(os.path.join(ExperimentParamFolder, 'ExperimentParameters.csv'), 'wb') as f:  # Just use 'w' mode in 3.x
        w = csv.writer(f)
        for key, value in kwargs.items():
            w.writerow([key, value])


if __name__ == '__main__':
    #initilize loc_x and loc_y to 0
    loc_x=0
    loc_y=0
    LocationMap = '/home/sinapse/catkin_ws/map7.pgm'  #change map  
    RewardLocationCSV = '/home/sinapse/Desktop/RewardData/rewardlocations.csv'
    orien_z=0
    orien_w=0
    loc_x_rotate=0
    loc_y_rotate=0
    current_orientation_w=0
    current_orientation_z=0
    angle_current=0
    angle_desired=0
  
    frequency = 1100 # Hertz
    # Define Experiment Parameters here
    ExperimentNumber =2
    ExperimentDate = '21022018'
    MonkeyName = 'Chimpian'
    NumberOfTrials = 1
    RewardTimeOut = 30 # In seconds
    Orientation=0
    Angle_tolerance=0.52
    Position_tolerance=100
    Stay_duration=2
    ExperimentFolder = '/home/sinapse/Desktop/RewardData/'  # Main folder for saving related data

    # Create a folder to save Experiment related parameters
    SaveDataFolder = os.path.join(ExperimentFolder, ExperimentDate, str(ExperimentNumber) + '_' + MonkeyName)
    if not os.path.exists(SaveDataFolder):
        os.makedirs(SaveDataFolder)

    # Save Experiment parameters
    SaveExpParameters(SaveDataFolder,
                      ExperimentNumber=ExperimentNumber,
                      ExperimentDate=ExperimentDate,
                      MonkeyName=MonkeyName,
                      NumberOfTrials=NumberOfTrials,
                      RewardTimeOut=RewardTimeOut)
    
    #The anonymous=True flag tells rospy to generate a unique name for the node so that you can have multiple listener.py nodes run easily. 
    rospy.init_node('robot_pose', anonymous=True)
    # Start Experiment

    Experiment(NumberOfTrials=2,
               RewardTimeOut= 5,
               Orientation=0,
               Angle_tolerance=1.57, #Tolerance for angle of deviation, in rads
               Position_tolerance=50, #The distance of the Powerbot to the location
               Stay_duration=2, #in Seconds
               Map=LocationMap,
               RewardLocationCSV=RewardLocationCSV,
               ExperimentParamFolder=SaveDataFolder)



