from connection import *
from calibration_moreComplex import *
from visual import *

import numpy as np
import matplotlib.pyplot as plt
import time
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import threading
import sys


class Pointing:
    def __init__(self, table_length_actual = 1.6, table_width_actual = 1.6, table_height_actual = 0.65):    
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
        self.Tz = self.table_depth = 353 #(in pixels)
        self.Tx = self.table_width = self.table_width_m * self.px_per_meter
        self.Ty = self.table_height = self.table_height_m * self.px_per_meter
        self.ew_length_l = 59 # left arm length (from elbow to wrist), needs calibration first
        self.ew_length_r = 59 # right arm length (from elbow to wrist), needs calibration first
        self.ew_length_changing_l = 0 # left arm length calculated for each frame
        self.ew_length_changing_r = 0 # right arm length calculated for each frame
        self.center_coord = np.array([159, 87]) # Assume camera takes 320 x 176 images
        #1920x1080     
        
        # parameters after calibration
        self.cal = Calibration()
        self.s_x_c = 0
        self.s_y_c = 0
        self.e_x_c = 0
        self.e_y_c = 0
        self.theta = 0 # angle of x axis roated
        self.A = [0,0,0]
        self.B = [0,0,0]
        self.C = [0,0,0]
        
        self.calInd = -1
        
        # Start running
        #self.run()

    def run(self):
        # thread to update lpoint and rpoint, first thing to start
        threading.Thread(target=self.updatePoint).start()
        threading.Thread(target=self.calibrateThread).start()
        # Plotting
        self.plot_all()
    
    def calibrateThread(self):
        while not self.cal.calibrated:
            if self.lcoord is not None and self.rcoord is not None:
                self.cal.calibrate(self.calInd, self.lcoord[0], self.rcoord[1])
            #print('calInd: ', self.calInd)
            #print('lcoord: ', self.lcoord)
            #print('rcoord: ', self.rcoord)
            #print('lpoint: ', self.lpoint)
            #print('rpoint: ', self.rpoint)
        self.calcRotateAngle()
        self.calcTableLimit()
        self.calcArmLength()
        
    def calcRotateAngle(self):
        t1 = self.s_x_c*self.Ty*(self.s_y_c-self.e_y_c)
        t2 = self.Tx*self.s_y_c*(self.s_y_c-self.e_y_c)/2 + (self.Tz**2) * (self.e_x_c - self.s_x_c)
        
        t3 = self.s_x_c*self.Tz*(self.s_y_c-self.e_y_c)
        t4 = self.Ty*self.Tz*(self.s_x_c - self.e_x_c)
        t5 = self.Tx*self.Tz*(self.s_y_c-self.e_y_c)/2 + self.s_y_c*self.Tz*(self.e_x_c-self.s_x_c)
        
        self.coefficients_for_angle = [-t1**2-t3**2,
                                  -2*t1*t2-2*t3*t4,
                                  -t2**2+t1**2-t4**2-2*t3*t5,
                                  2*t2*t1-2*t4*t5,
                                  t2**2-t5**2]
        self.theta = np.min(np.arccos(np.roots(self.coefficients_for_angle)))
    
    def calcTableLimit(self):
        self.A = [self.Tx/2, self.Ty*np.cos(self.theta)-self.Tz*np.sin(self.theta), self.Ty*np.sin(self.theta)+self.Tz*np.cos(self.theta)]
        self.B = [-self.Tx/2, self.Ty*np.cos(self.theta)-self.Tz*np.sin(self.theta), self.Ty*np.sin(self.theta)+self.Tz*np.cos(self.theta)]
        self.C = [self.Tx/2, self.Ty*np.cos(self.theta), self.Ty*np.sin(self.theta)]
    
    def calcArmLength(self):
        t = self.Tz/((self.s_y_c- self.e_y_c)*np.sin(self.theta) + self.Tz)
        self.ew_length_r = np.linalg.norm([(self.s_x_c - self.e_x_c*t), (self.s_y_c - self.e_y_c*t), (1-t)*(self.s_y_c*np.tan(self.theta)+ self.Tz/np.cos(self.theta))])
    
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
                    self.ew_length_changing_l = np.linalg.norm(np.array(self.lcoord[0])-np.array(self.lcoord[1])) 
                    
            if self.rcoord is not None:
                #print(rcoord)
                self.rpoint  = self.calcPointing(self.rcoord[0], self.rcoord[2], self.rcoord[1], 'r')
                if not self.cal.calibrated:
                    self.ew_length_changing_r = np.linalg.norm(np.array(self.rcoord[0])-np.array(self.rcoord[1]))
                
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
        if not self.cal.calibrated:
            return None,None
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
        s_z = s_z_p = e_z_p = w_z_p = (self.s_y_c*np.tan(self.theta)+ self.Tz/np.cos(self.theta))
        #s_z = s_z_p = e_z_p = w_z_p = self.table_depth # assumption 2
        if direction == 'l':
            ew_length = se_length = self.ew_length_l # assumption 3
        else:
            ew_length = se_length = self.ew_length_r # assumption 3
        
        # First calculate the elbow coordinate, start with elbow_y
        coefficients_for_elbow = [(e_x_p/e_y_p)**2 + 1 + (e_z_p/e_y_p)**2,
                                -2*(e_x_p*s_x/e_y_p + s_y + e_z_p*s_z/e_y_p),
                                s_x**2 + s_y**2 + s_z**2 - se_length**2]
        e_y = np.min(np.roots(coefficients_for_elbow)) # the point closer to origin/camera
        e_x = e_x_p * e_y / e_y_p
        e_z = e_z_p * e_y / e_y_p
        
        # Then calculate the wrist coordinate, start with wrist_y
        coefficients_for_wrist = [(w_x_p/w_y_p)**2 + 1 + (w_z_p/w_y_p)**2,
                                -2*(w_x_p*e_x/w_y_p + e_y + w_z_p*e_z/w_y_p),
                                e_x**2 + e_y**2 + e_z**2 - ew_length**2]
        w_y = np.min(np.roots(coefficients_for_wrist)) # the point closer to origin/camera
        w_x = w_x_p * w_y / w_y_p
        w_z = w_z_p * w_y / w_y_p
        # Finally calculate the pointing coordinate on the table
        #table_y = self.table_height
        #table_x = w_x - (w_y - table_y) / (e_y - w_y) * (e_x - w_x) # in pixel
        #table_z = w_z - (w_y - table_y) / (e_y - w_y) * (e_z - w_z) # in pixel
        pointed = np.dot(np.linalg.inv([[e_x-w_x, self.A[0]-self.B[0], self.C[0]-self.B[0]],
                       [e_y-w_y,  self.A[1]-self.B[1], self.C[1]-self.B[1]],
                       [e_z-w_z,  self.A[2]-self.B[2], self.C[2]-self.B[2]]]),
               np.array([e_x-self.B[0], e_y-self.B[1], e_z-self.B[2]])
              )
        
        # map table_x back to meters
        #table_x = table_x * table_depth / 0.655 * 0.295 /table_width / 735  
        
        # map point to 2D
        table_x = np.inner(pointed-np.array(self.A), np.array(self.B)-np.array(self.A))/np.linalg.norm(np.array(self.B)-np.array(self.A))
        table_z = np.inner(pointed-np.array(self.A), np.array(self.C)-np.array(self.A))/np.linalg.norm(np.array(self.C)-np.array(self.A))
        
        return (table_x, table_z) # returned points are in pixels

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
                ax.set_xlim(0, self.Tx)
                ax.set_ylim(0, self.Tz)
                #plt.gca().invert_yaxis()
                #plt.gca().invert_xaxis()
                #ax.plot((self.lpoint[0]/self.point_limit_left_l),
                        #(self.lpoint[1] - self.point_limit_top_l)/(self.point_limit_bottom_l - self.point_limit_top_l),
                        #'bo', label='left hand', markersize= 20)
                #ax.plot((-self.rpoint[0]/self.point_limit_right_r),
                        #(self.rpoint[1] - self.point_limit_top_r)/(self.point_limit_bottom_r - self.point_limit_top_r),
                        #'ro', label='right hand', markersize= 20)
                ax.plot(self.lpoint[0], self.lpoint[1], 'bo', label='left hand', markersize= 20)
                ax.plot(self.rpoint[0], self.rpoint[1], 'ro', label='right hand', markersize= 20)
                ax.legend()
                #print(self.lpoint, self.rpoint)
            else:
                ## Calibration            
                ax.set_xlim(-1, 1)
                ax.set_ylim(-1, 1)
                interval = time.time() - self.startTime
                if interval < 10:
                    self.calInd = 0
                    if (interval > 6):
                        ax.text(0, 0,"%.1fs left\nPlease stand still." %(10-interval), fontsize=20, horizontalalignment='center', verticalalignment='center')
                        #self.cal.calibrate(0, self.ew_length_changing, self.lpoint, self.rpoint)
                    else:                        
                        ax.text(0, 0,"Calibrating No. 1\nPlease stand still.", fontsize=20, horizontalalignment='center', verticalalignment='center')
                elif interval < 20:
                    self.calInd = 1
                    if (interval > 16):
                        ax.text(0, 0, "%.1fs left\nUse left hand to point to the bottom left corner.\nUse right hand to point to the bottom right corner." %(20-interval), fontsize=20, horizontalalignment='center', verticalalignment='center')
                        #self.cal.calibrate(1, self.ew_length_changing, self.lpoint, self.rpoint)
                        #print(self.lpoint, self.rpoint)
                    else:
                        ax.text(0, 0, "Calibrating No. 2\nUse left hand to point to the bottom left corner.\nUse right hand to point to the bottom right corner.", fontsize=20, horizontalalignment='center', verticalalignment='center')
                #elif interval < 30:
                    #self.calInd = 2
                    #if (interval > 26):
                        #ax.text(0, 0, "%.1fs left\nUse left hand to point to the top left corner.\nUse right hand to point to the top right corner." %(30-interval), fontsize=20, horizontalalignment='center', verticalalignment='center')
                        ##self.cal.calibrate(2, self.ew_length_changing, self.lpoint, self.rpoint)
                    #else:
                        #ax.text(0, 0, "Calibrating No. 3\nUse left hand to point to the top left corner.\nUse right hand to point to the top right corner.", fontsize=20, horizontalalignment='center', verticalalignment='center')
                else:
                    self.cal.calibrated = True
                    self.s_x_c = np.median(self.cal.s_x_arr_r[int(len(self.cal.s_x_arr_r)/2):int(len(self.cal.s_x_arr_r)*4/5)]) - self.center_coord[0]
                    self.s_y_c = np.median(self.cal.s_y_arr_r[int(len(self.cal.s_y_arr_r)/2):int(len(self.cal.s_y_arr_r)*4/5)]) - self.center_coord[1]
                    self.e_x_c = np.median(self.cal.e_x_arr_r[int(len(self.cal.e_x_arr_r)/2):int(len(self.cal.e_x_arr_r)*4/5)]) - self.center_coord[0]
                    self.e_y_c = np.median(self.cal.e_y_arr_r[int(len(self.cal.e_y_arr_r)/2):int(len(self.cal.e_y_arr_r)*4/5)]) - self.center_coord[1]
                    #self.point_limit_top_r = np.median(self.cal.dummy_tablez_top_arr_r[int(len(self.cal.dummy_tablez_top_arr_r)/2):int(len(self.cal.dummy_tablez_top_arr_r)*4/5)])
                    #self.point_limit_top_l = np.median(self.cal.dummy_tablez_top_arr_l[int(len(self.cal.dummy_tablez_top_arr_l)/2):int(len(self.cal.dummy_tablez_top_arr_l)*4/5)])
                    #self.point_limit_bottom_r = np.median(self.cal.dummy_tablez_bottom_arr_r[int(len(self.cal.dummy_tablez_bottom_arr_r)/2):int(len(self.cal.dummy_tablez_bottom_arr_r)*4/5)])
                    #self.point_limit_bottom_l = np.median(self.cal.dummy_tablez_bottom_arr_l[int(len(self.cal.dummy_tablez_bottom_arr_l)/2):int(len(self.cal.dummy_tablez_bottom_arr_l)*4/5)])
                    #self.point_limit_left_l = np.median(self.cal.dummy_tablex_left_arr_l[int(len(self.cal.dummy_tablex_left_arr_l)/2):int(len(self.cal.dummy_tablex_left_arr_l)*4/5)])
                    #self.point_limit_right_r = np.median(self.cal.dummy_tablex_right_arr_r[int(len(self.cal.dummy_tablex_right_arr_r)/2):int(len(self.cal.dummy_tablex_right_arr_r)*4/5)])
                    #print(self.ew_length_l, self.ew_length_r)
                    #print(self.point_limit_tolp_r, self.point_limit_top_l, self.point_limit_bottom_r, self.point_limit_bottom_l, self.point_limit_left_l, self.point_limit_right_r)
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

