import cv2
import os
import pandas as pd
import numpy as np




class MarkRewardLocations(object):
    def __init__(self, Map, RewardLocationFolder):
        """ Displays map of the location and takes double click from the user to mark reward location
                Params:
                Map : map of the area stored as .pgm
                RewardLocationFolder : Folder to save csv file and reward location image
        """
        self.rewardlocfolder = RewardLocationFolder

        # Get reward locations
        cv2.namedWindow('LocationMap', cv2.WINDOW_NORMAL)
        self.map = cv2.imread(Map)
        print('Map coordinates ', np.shape(self.map)[0:2])
        #self.rewardlocation = {'x1': [], 'y1': []}
        self.rewardlocation = {'x1': [], 'y1': [],'x2': [], 'y2': []}
        cv2.setMouseCallback('LocationMap', self.mark_location_and_store)
        self.plot_location()
        self.save_reward_location()
        

    def mark_location_and_store(self, event, x, y, flags, param):
        """ Double click on the location map to get x, y coordinates for reward location"""
        global flag
        global start_x
        global start_y
        if event == cv2.EVENT_LBUTTONDBLCLK:
            flag = flag+1;
            
            if flag % 2 ==1:
                cv2.circle(self.map, (x, y), 20, (0, 0, 255), -1)
                cv2.putText(self.map, '(%d, %d)' % (x, y), (x, y), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 0), 4)
                self.rewardlocation['x1'].append(x)
                self.rewardlocation['y1'].append(y)
                start_x=x
                start_y=y
                
            else:
                cv2.arrowedLine(self.map,(start_x,start_y),(x,y), (0, 0, 0), 8)
                self.rewardlocation['x2'].append(x)
                self.rewardlocation['y2'].append(y)
 
    def plot_location(self):
        """ Plot image with reward location. Press esc to quit"""
        print('Press Esc when done marking locations')
        while True:
            cv2.imshow('LocationMap', self.map)
            cv2.resizeWindow('LocationMap', 800, 800)
            if cv2.waitKey(20) & 0xFF == 27:
                break
        cv2.destroyAllWindows()

    def save_reward_location(self):
        
        print ('Saving reward locations')
        df = pd.DataFrame.from_dict(self.rewardlocation)
        df.to_csv(os.path.join(self.rewardlocfolder, 'rewardlocations.csv')) #save the matrix of the reward location
        cv2.imwrite(os.path.join(self.rewardlocfolder, "rewardlocations.tif"), self.map) #save image of the reward location


if __name__ == '__main__':
    LocationMap = '/home/sinapse/catkin_ws/map7.pgm' #change map 
    RewardLocationFolder = '/home/sinapse/Desktop/RewardData'

    flag = 0
    start_x=0
    start_y=0
    MarkRewardLocations(Map=LocationMap,
                        RewardLocationFolder=RewardLocationFolder)
