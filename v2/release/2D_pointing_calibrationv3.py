from connection import *
from calibrationv3 import *

import numpy as np
import matplotlib.pyplot as plt
import time
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import threading
import sys


class Pointing:
    def __init__(self, table_length_actual = 1.6, table_width_actual = 1.6, table_height_actual = 0.7):    
        self.lpoint = (0,0)
        self.rpoint = (0,0)
        self.lcoord, self.rcoord = None, None
        
        # parameters specified by the server, used to extract important joints
        self.SHOULDERRIGHT = 2
        self.ELBOWRIGHT = 3
        self.WRISTRIGHT = 4
        self.SHOULDERLEFT = 5
        self.ELBOWLEFT = 6
        self.WRISTLEFT = 7
        self.joint_interest_coded_right = [self.SHOULDERRIGHT, self.ELBOWRIGHT, self.WRISTRIGHT]
        self.joint_interest_coded_left = [self.SHOULDERLEFT, self.ELBOWLEFT, self.WRISTLEFT]
        self.header_cnt = 4 # there are 4 values in the header
        
        # Several import parameters to set up
        self.table_depth_m = table_width_actual # this is the actual width of the table (z in meters)
        self.table_width_m = table_length_actual # this is the actual length of the table (x in meters)
        self.table_height_m = table_height_actual # this is the actual height of the table to the camera (y in meters)

        #table_width_pixel = table_width / 0.295 * 735 * 0.655 /table_depth # table width in pixel, calibrated using our own camera
        #table_height_pixel = table_height / 0.295 * 735 * 0.655 /table_depth # table length in pixel, calibrated using our own camera
        self.px_per_meter = 353/self.table_depth_m # pixels per meter at distance table_depth
        #Transform meters into pixels
        self.table_depth = 353 #(in pixels)
        self.table_width = self.table_width_m * self.px_per_meter
        self.table_height = self.table_height_m * self.px_per_meter
        self.ew_length_l = 59 # left arm length (from elbow to wrist), needs calibration first
        self.ew_length_r = 59 # right arm length (from elbow to wrist), needs calibration first
        self.ew_length_changing_l = 0 # left arm length calculated for each frame
        self.ew_length_changing_r = 0 # right arm length calculated for each frame
        self.center_coord = np.array([159, 87]) # Assume camera takes 320 x 176 images
        #1920x1080     
        
        # parameters after calibration
        self.cal = Calibration()
        self.point_limit_center_x_l = 0
        self.point_limit_center_z_l = self.table_depth/2
        self.point_limit_center_x_r = 0
        self.point_limit_center_z_r = self.table_depth/2
        self.point_limit_top_r = 0
        self.point_limit_top_l = 0
        self.point_limit_left_l = -self.table_width/2
        self.point_limit_right_r = self.table_width/2
        self.point_limit_left_r = -self.table_width/2
        self.point_limit_right_l = self.table_width/2
        self.calInd = -1
        self.table_x_original = 0
        self.table_z_original = 0
        # Start running
        #self.run()

    def run(self):
        # thread to update lpoint and rpoint, first thing to start
        threading.Thread(target=self.updatePoint).start()
        #threading.Thread(target=self.calibrateThread).start()
        # Plotting
        self.plot_all()
    
    def calibrateThread(self):
        if not self.cal.ew_calibrated and self.calInd == 1:
            self.cal.ew_calibrated = True
            self.ew_length_l = np.median(self.cal.ew_length_arr_l[int(len(self.cal.ew_length_arr_l)/2):int(len(self.cal.ew_length_arr_l)*4/5)])
            self.ew_length_r = np.median(self.cal.ew_length_arr_r[int(len(self.cal.ew_length_arr_r)/2):int(len(self.cal.ew_length_arr_r)*4/5)])
        self.cal.calibrate(self.calInd, self.ew_length_changing_l, self.ew_length_changing_r, self.lpoint, self.rpoint)
        print('calInd: ', self.calInd)
        print('lcoord: ', self.lcoord)
        print('rcoord: ', self.rcoord)
        print('lpoint: ', self.lpoint)
        print('rpoint: ', self.rpoint)
    
    def updatePoint(self):
        '''
        This method updates the lpoint and rpoint once receiving information from server
        '''
        for decoded in getDecoded():
            #print(decoded, '\n')
            self.lcoord, self.rcoord = self.getImportantJoints(decoded) # lcoord contains left joint coordinates for shoulder, elbow, wrist
            if self.lcoord is not None:
                #print(lcoord)
                self.lpoint  = self.calcPointing(self.lcoord[0], self.lcoord[2], self.lcoord[1], 'l')
                if not self.cal.calibrated:
                    self.ew_length_changing_l = np.linalg.norm(np.array(self.lcoord[0])-np.array(self.lcoord[2]))/2 
                    
            if self.rcoord is not None:
                #print(rcoord)
                self.rpoint  = self.calcPointing(self.rcoord[0], self.rcoord[2], self.rcoord[1], 'r')
                if not self.cal.calibrated:
                    self.ew_length_changing_r = np.linalg.norm(np.array(self.rcoord[0])-np.array(self.rcoord[2]))/2
            
            if not self.cal.calibrated:
                self.calibrateThread()
#            print('Interpolated pointing:', lpoint, " ", rpoint)
#            print('------------------------')

    def getImportantJoints(self, src):
        '''
        This function returns the coordinates for left/right wrists/elbows/shoulders (6 sets of 2 values: x, y for 2D image)
        '''
            
        if len(src) <= self.header_cnt:
            return None, None
        
        rresult = []    
        lresult = []
        for i in self.joint_interest_coded_right:
            try:
                rresult.append(src[3*i + self.header_cnt : 3*i + 2 + self.header_cnt]) # only take the x and y, not the Confidence
                if rresult[-1][0] == 0 and rresult[-1][1] == 0: # meaning this joint was not detected
                    rresult = None
                    break
            except:
                rresult = None
                break
            
        for i in self.joint_interest_coded_left:
            try:
                lresult.append(src[3*i + self.header_cnt : 3*i + 2 + self.header_cnt]) # only take the x and y, not the Confidence
                if lresult[-1][0] == 0 and lresult[-1][1] == 0: # meaning this joint was not detected
                    lresult = None
                    break
            except:
                lresult = None
                break
            
        return lresult, rresult

    def calcPointing(self, shoulder, wrist, elbow, direction):
        '''
        Shoulder, wrist and elbow should contain (x,y) coordinates
        These are coordinates projection on 2D, denoted as "_p" in the following code
        Table plane: y = table_height (in pixel)
        Assumption:
            1. A person's shoulder does not move
            2. A person's distance to the camera is the table's depth, and camera's view angle is parallel with table
            3. A person's shoulder to elbow is the same length with elbow to wrist
        '''
        # Shift origin of frame from topleft to center, which should be the focus of the camera
        shoulder = np.array(shoulder) - self.center_coord
        wrist = np.array(wrist) - self.center_coord
        elbow = np.array(elbow) - self.center_coord
        
        s_x_p, s_y_p = shoulder
        s_x, s_y = shoulder # assumption 1
        w_x_p, w_y_p = wrist
        e_x_p, e_y_p = elbow
        s_z = s_z_p = e_z_p = w_z_p = self.table_depth # assumption 2
        if direction == 'l':
            ew_length = se_length = self.ew_length_l # assumption 3
        else:
            ew_length = se_length = self.ew_length_r # assumption 3
        
        # method 1
        ## First calculate the elbow coordinate, start with elbow_y
        #coefficients_for_elbow = [(e_x_p/e_y_p)**2 + 1 + (e_z_p/e_y_p)**2,
                                #-2*(e_x_p*s_x/e_y_p + s_y + e_z_p*s_z/e_y_p),
                                #s_x**2 + s_y**2 + s_z**2 - se_length**2]
        #e_y = np.min(np.roots(coefficients_for_elbow)) # the point closer to origin/camera
        #e_x = e_x_p * e_y / e_y_p
        #e_z = e_z_p * e_y / e_y_p
        
        ## Then calculate the wrist coordinate, start with wrist_y
        #coefficients_for_wrist = [(w_x_p/w_y_p)**2 + 1 + (w_z_p/w_y_p)**2,
                                #-2*(w_x_p*e_x/w_y_p + e_y + w_z_p*e_z/w_y_p),
                                #e_x**2 + e_y**2 + e_z**2 - ew_length**2]
        #w_y = np.min(np.roots(coefficients_for_wrist)) # the point closer to origin/camera
        #w_x = w_x_p * w_y / w_y_p
        #w_z = w_z_p * w_y / w_y_p
        
        ## Finally calculate the pointing coordinate on the table
        #table_y = self.table_height
        #table_x = w_x - (w_y - table_y) / (e_y - w_y) * (e_x - w_x) # in pixel
        #table_z = w_z - (w_y - table_y) / (e_y - w_y) * (e_z - w_z) # in pixel
        
        ## map table_x back to meters
        ##table_x = table_x * table_depth / 0.655 * 0.295 /table_width / 735  
        
        ## method 2
        #coefficients_for_t1 = [e_x_p**2 + e_y_p**2 + s_z_p**2,
                               #-2*(e_x_p*s_x_p + e_y_p*s_y_p + s_z_p**2),
                               #s_x_p**2 + s_y_p**2 + s_z_p**2 - se_length**2]
        #t1 = np.min(np.roots(coefficients_for_t1))
        #e_x = e_x_p*t1
        #e_y = e_y_p*t1
        #e_z = s_z_p*t1
        
        #coefficients_for_t2 = [w_x_p**2 + w_y_p**2 + s_z**2,
                               #-2*(w_x_p*e_x + w_y_p*e_y + e_z*s_z),
                               #e_x**2 + e_y**2 + e_z**2 - ew_length**2]
        #t2 = np.min(np.roots(coefficients_for_t2))
        #w_x = w_x_p*t2
        #w_y = w_y_p*t2
        #w_z = s_z_p*t2
        
        ### Finally calculate the pointing coordinate on the table
        #table_y = self.table_height
        #table_x = w_x - (w_y - table_y) / (e_y - w_y) * (e_x - w_x) # in pixel
        #table_z = w_z - (w_y - table_y) / (e_y - w_y) * (e_z - w_z) # in pixel
        
        # method 3
        coefficients_for_t = [w_x_p**2 + w_y_p**2 + s_z_p**2,
                               -2*(w_x_p*s_x_p + w_y_p*s_y_p + s_z_p**2),
                               s_x_p**2 + s_y_p**2 + s_z_p**2 - (2*se_length)**2]
        t = np.min(np.roots(coefficients_for_t))
        w_x = w_x_p*t
        w_y = w_y_p*t
        w_z = s_z_p*t
        
        #coefficients_for_t2 = [w_x_p**2 + w_y_p**2 + s_z**2,
                               #-2*(w_x_p*e_x + w_y_p*e_y + e_z*s_z),
                               #e_x**2 + e_y**2 + e_z**2 - ew_length**2]
        #t2 = np.min(np.roots(coefficients_for_t2))
        #w_x = w_x_p*t2lpoint
        #w_y = w_y_p*t2
        #w_z = s_z_p*t2
        
        ## Finally calculate the pointing coordinate on the table
        table_y = self.table_height
        table_x = w_x - (w_y - table_y) / (s_y - w_y) * (s_x - w_x) # in pixel
        table_z = w_z - (w_y - table_y) / (s_y - w_y) * (s_z - w_z) # in pixel
        
        ## normalize before return if calibrated
        #if self.cal.calibrated:
            #self.table_z_original = table_z
            #self.table_x_original = table_x
            #if direction == 'l':
                #if table_x < self.point_limit_center_x_l:
                    #table_x = (table_x - self.point_limit_center_x_l)/(self.point_limit_center_x_l- self.point_limit_right_l)
                #else:
                    #table_x = (table_x - self.point_limit_center_x_l)/(self.point_limit_left_l - self.point_limit_center_x_l)
                #table_z = (table_z - self.point_limit_top_l)/(self.point_limit_center_z_l - self.point_limit_top_l)
            #else:
                #if table_x < self.point_limit_center_x_r:
                    #table_x = (table_x - self.point_limit_center_x_r )/(self.point_limit_center_x_r- self.point_limit_right_r)
                #else:
                    #table_x = (table_x - self.point_limit_center_x_r)/(self.point_limit_left_r - self.point_limit_center_x_r)
                #table_z = (table_z - self.point_limit_top_r)/(self.point_limit_center_z_r - self.point_limit_top_r)
            
        return [table_x, table_z] # returned points are in pixels

    def plot_all(self):
        #Plot where the pointing position is on the table
        #The table has a width of (-1,1) and depth of (0.0, 1.6)
        fig = plt.figure(figsize=(15, 15))
        ax = fig.add_subplot(111)        
        self.startTime = time.time()
        
        def animate(i):
            ax.clear()
            
            if self.cal.calibrated:
                self.calInd = -2
                ax.set_xlim(-1, 1)
                ax.set_ylim(0, 1.2)
                plt.gca().invert_yaxis()
                plt.gca().invert_xaxis()
                
                ax.text(1, 1,"Left: %.1f %.1f\nRight: %.1f %.1f" %(self.lpoint[0], self.lpoint[1], self.rpoint[0], self.rpoint[1]), fontsize=15, horizontalalignment='right', verticalalignment='top')
                
                #normalize                
                if self.lpoint[0] < self.point_limit_center_x_l:
                    self.lpoint[0] = (self.lpoint[0] - self.point_limit_center_x_l)/(self.point_limit_center_x_l- self.point_limit_right_l)
                else:
                    self.lpoint[0] = (self.lpoint[0] - self.point_limit_center_x_l)/(self.point_limit_left_l - self.point_limit_center_x_l)
                self.lpoint[1] = (self.lpoint[1] - self.point_limit_top_l)/(self.point_limit_center_z_l - self.point_limit_top_l)
                
                if self.rpoint[0] < self.point_limit_center_x_r:
                    self.rpoint[0] = (self.rpoint[0] - self.point_limit_center_x_r )/(self.point_limit_center_x_r- self.point_limit_right_r)
                else:
                    self.rpoint[0] = (self.rpoint[0] - self.point_limit_center_x_r)/(self.point_limit_left_r - self.point_limit_center_x_r)
                self.rpoint[1] = (self.rpoint[1] - self.point_limit_top_r)/(self.point_limit_center_z_r - self.point_limit_top_r)
                    
                ax.plot(self.lpoint[0], self.lpoint[1], 'bo', label='left', markersize= 20)
                ax.plot(self.rpoint[0], self.rpoint[1], 'ro', label='right', markersize= 20)
                ax.legend()
                print(self.lpoint, self.rpoint)
            else:
                ## Calibration            
                ax.set_xlim(-1, 1)
                ax.set_ylim(0, 2)
                interval = time.time() - self.startTime
                if interval < 10:
                    self.calInd = 0
                    ax.text(0, 1,"Calibrating #1 %.1fs left\nUse left hand to point to the bottom left corner.\nUse right hand to point to the bottom right corner." %(10-interval), fontsize=20, horizontalalignment='center', verticalalignment='center')
                elif interval < 20:
                    self.calInd = 1
                    ax.text(0, 1, "Calibrating #2 %.1fs left\nUse both hands to point to the center." %(20-interval), fontsize=20, horizontalalignment='center', verticalalignment='center')
                elif interval < 30:
                    self.calInd = 2
                    ax.text(0, 1, "Calibrating #3 %.1fs left\nUse left hand to point to the top left corner.\nUse right hand to point to the top right corner." %(30-interval), fontsize=20, horizontalalignment='center', verticalalignment='center')
                elif interval < 40:
                    self.calInd = 3
                    ax.text(0, 1, "Calibrating #4 %.1fs left\nUse LEFT hand to point to the TOP RIGHT corner." %(40-interval), fontsize=20, horizontalalignment='center', verticalalignment='center')
                elif interval < 50:
                    self.calInd = 4
                    ax.text(0, 1, "Calibrating #5 %.1fs left\nUse RIGHT hand to point to the TOP LEFT corner." %(50-interval), fontsize=20, horizontalalignment='center', verticalalignment='center')
                      
                else:
                    self.cal.calibrated = True
                    #calInd 1
                    
                    self.point_limit_center_x_l = np.median(self.cal.dummy_tablex_center_arr_l[int(len(self.cal.dummy_tablex_center_arr_l)/2):int(len(self.cal.dummy_tablex_center_arr_l)*4/5)])
                    self.point_limit_center_z_l = np.median(self.cal.dummy_tablez_center_arr_l[int(len(self.cal.dummy_tablez_center_arr_l)/2):int(len(self.cal.dummy_tablez_center_arr_l)*4/5)])
                    self.point_limit_center_x_r = np.median(self.cal.dummy_tablex_center_arr_r[int(len(self.cal.dummy_tablex_center_arr_r)/2):int(len(self.cal.dummy_tablex_center_arr_r)*4/5)])
                    self.point_limit_center_z_r = np.median(self.cal.dummy_tablez_center_arr_r[int(len(self.cal.dummy_tablez_center_arr_r)/2):int(len(self.cal.dummy_tablez_center_arr_r)*4/5)])
                    
                    # calInd 2
                    self.point_limit_top_r = np.median(self.cal.dummy_tablez_top_arr_r[int(len(self.cal.dummy_tablez_top_arr_r)/2):int(len(self.cal.dummy_tablez_top_arr_r)*4/5)])
                    self.point_limit_top_l = np.median(self.cal.dummy_tablez_top_arr_l[int(len(self.cal.dummy_tablez_top_arr_l)/2):int(len(self.cal.dummy_tablez_top_arr_l)*4/5)])
                    self.point_limit_left_l = np.median(self.cal.dummy_tablex_left_arr_l[int(len(self.cal.dummy_tablex_left_arr_l)/2):int(len(self.cal.dummy_tablex_left_arr_l)*4/5)])
                    self.point_limit_right_r = np.median(self.cal.dummy_tablex_right_arr_r[int(len(self.cal.dummy_tablex_right_arr_r)/2):int(len(self.cal.dummy_tablex_right_arr_r)*4/5)])
                    
                    # calInd 3and4
                    self.point_limit_left_r = np.median(self.cal.dummy_tablex_left_arr_r[int(len(self.cal.dummy_tablex_left_arr_r)/2):int(len(self.cal.dummy_tablex_left_arr_r)*4/5)])
                    self.point_limit_right_l = np.median(self.cal.dummy_tablex_right_arr_l[int(len(self.cal.dummy_tablex_right_arr_l)/2):int(len(self.cal.dummy_tablex_right_arr_l)*4/5)])
                    
                    print(self.ew_length_l, self.ew_length_r)
                    print('point_limit_center_x_l', self.point_limit_center_x_l, 'point_limit_center_z_l', self.point_limit_center_z_l ,'point_limit_center_x_r', self.point_limit_center_x_r, 'point_limit_center_z_r', self.point_limit_center_z_r)
                    print('point_limit_top_r', self.point_limit_top_r, 'point_limit_top_l', self.point_limit_top_l, 'point_limit_left_l', self.point_limit_left_l, 'point_limit_right_r', self.point_limit_right_r)
                    print('point_limit_left_r', self.point_limit_left_r, 'point_limit_right_l', self.point_limit_right_l)
                    # For testing
                    #self.point_limit_bottom_l = 3
                    #self.point_limit_bottom_r = 4
                
        ani = FuncAnimation(fig, animate)
        plt.show()
        

                
if __name__ == '__main__':
    p = Pointing()
    p.run()




#//POSE_COCO_BODY_PARTS{
#// { 0,  "Nose" },
#// { 1,  "Neck" },
#// { 2,  "RShoulder" },
#// { 3,  "RElbow" },
#// { 4,  "RWrist" },
#// { 5,  "LShoulder" },
#// { 6,  "LElbow" },
#// { 7,  "LWrist" },
#// { 8,  "RHip" },
#// { 9,  "RKnee" },
#// { 10, "RAnkle" },
#// { 11, "LHip" },
#// { 12, "LKnee" },
#// { 13, "LAnkle" },
#// { 14, "REye" },
#// { 15, "LEye" },
#// { 16, "REar" },
#// { 17, "LEar" },
#// { 18, "Bkg" },
#//}

