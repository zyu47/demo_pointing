import tkinter
from tkinter import messagebox
 
# hide main window
root = tkinter.Tk()
root.withdraw()
 
# message box display
messagebox.showinfo("CWC","System is Calibrated!")

#import socket
#import struct
#import numpy as np
#import matplotlib.pyplot as plt
#import time
#from matplotlib.animation import FuncAnimation
#import threading
#import sys

#shoulder, elbow, wrist = (208.33749389648438, 55.83479690551758), (215.16392517089844, 93.94319915771484), (209.661376953125, 133.4442138671875)
#ew_length = 58.728342895

## Several import parameters to set up
#table_depth = 1.6 # this is the actual length of the table
#table_width = 1.6 # this is the actual width of the table
#table_height = 0.65 # this is the actual height of the table to the camera

##table_width_pixel = table_width / 0.295 * 735 * 0.655 /table_depth # table width in pixel, calibrated using our own camera
##table_height_pixel = table_height / 0.295 * 735 * 0.655 /table_depth # table length in pixel, calibrated using our own camera
#px_per_meter = 224.44/table_depth # pixels per meter at distance table_depth
##Transform meters into pixels
#table_depth = 224.44
#table_width = table_width * px_per_meter
#table_height = table_height * px_per_meter
##ew_length = ew_length * px_per_meter
##se_length = se_length * px_per_meter

#center_coord = np.array([159, 87]) # Assume camera takes 320 x 176 images
##1920x1080     

##def calcPointing(shoulder, wrist, elbow):
#'''
#Shoulder, wrist and elbow should contain (x,y) coordinates
#These are coordinates projection on 2D, denoted as "_p" in the following code
#Table plane: y = table_height (in pixel)
#Assumption:
    #1. A person's shoulder does not move
    #2. A person's distance to the camera is the table's depth, and camera's view angle is parallel with table
    #3. A person's shoulder to elbow is the same length with elbow to wrist
#'''
## Shift origin of frame from topleft to center, which should be the focus of the camera
#shoulder = np.array(shoulder) - center_coord
#wrist = np.array(wrist) - center_coord
#elbow = np.array(elbow) - center_coord

#s_x_p, s_y_p = shoulder
#s_x, s_y = shoulder # assumption 1
#w_x_p, w_y_p = wrist
#e_x_p, e_y_p = elbow
#s_z = s_z_p = e_z_p = w_z_p = table_depth # assumption 2
#se_length = ew_length # assumption 3
    
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
#table_y = table_height
#table_x = w_x - (w_y - table_y) / (e_y - w_y) * (e_x - w_x) # in pixel
#table_z = w_z - (w_y - table_y) / (e_y - w_y) * (e_z - w_z) # in pixel

## map table_x back to meters
##table_x = table_x * table_depth / 0.655 * 0.295 /table_width / 735  

##return (table_x, table_z) # returned points are in pixels

    
