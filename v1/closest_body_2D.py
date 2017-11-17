import socket
import struct
import numpy as np
import matplotlib.pyplot as plt
import time
from matplotlib.animation import FuncAnimation
import threading
import sys

src_addr = '129.82.45.252'
src_port = 9009

stream_id = 512


def connect():
    """
    Connect to a specific port
    """

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    try:
        sock.connect((src_addr, src_port))
    except:
        print("Error connecting to {}:{}".format(src_addr, src_port))
        return None

    try:
        print("Sending stream info")
        sock.sendall(struct.pack('<i', stream_id))
    except:
        print("Error: Stream rejected")
        return None

    print("Successfully connected to host")
    return sock


def decode_frame_openpose(raw_frame):
    # The format is given according to the following assumption of network data

    # Expect little endian byte order
    endianness = "<"

    # [ commonTimestamp | frame type | Tracked body count | Engaged
    header_format = "qiHH"

    timestamp, frame_type, tracked_body_count, engaged = struct.unpack(endianness + header_format,
                                                                       raw_frame[:struct.calcsize(header_format)])

    # For each of the 18 joints, the following info is transmitted
    # [ Position.X | Position.Y | Confidence ]
    joint_format = "3f"

    frame_format = (joint_format * 18)

    # Unpack the raw frame into individual pieces of data as a tuple
    frame_pieces = struct.unpack(endianness + (frame_format * (1 if engaged else 0)),
                                 raw_frame[struct.calcsize(header_format):])

    decoded = (timestamp, frame_type, tracked_body_count, engaged) + frame_pieces
    #print "decoded:", decoded
    return decoded


def recv_all(sock, size):
    result = b''
    while len(result) < size:
        data = sock.recv(size - len(result))
        if not data:
            raise EOFError("Error: Received only {} bytes into {} byte message".format(len(data), size))
        result += data
    return result


def recv_skeleton_frame(sock):
    """
    To read each stream frame from the server
    """
    (load_size,) = struct.unpack("<i", recv_all(sock, struct.calcsize("<i")))
    #print("load_size = ", load_size)
    return recv_all(sock, load_size)

######## following codes get the elbow and wrist information from RGB skeleton

SHOULDERRIGHT = 2
ELBOWRIGHT = 3
WRISTRIGHT = 4
SHOULDERLEFT = 5
ELBOWLEFT = 6
WRISTLEFT = 7
joint_interest_coded_right = [SHOULDERRIGHT, ELBOWRIGHT, WRISTRIGHT]
joint_interest_coded_left = [SHOULDERLEFT, ELBOWLEFT, WRISTLEFT]

header_cnt = 4 # there are 4 values in the header

def getImportantJoints(src):
    '''
    This function returns the coordinates for left/right wrists/elbows/shoulders (6 sets of 2 values: x, y for 2D image)
    '''
        
    if len(src) <= header_cnt:
        return None, None
    
    rresult = []    
    #for i in range(len(joint_interest_coded_right)):
        #rresult[joint_interest_coded_right[i]] = None
        
    lresult = []
    #for i in range(len(joint_interest_coded_left)):
        #lresult[joint_interest_coded_left[i]] = None
    
    for i in joint_interest_coded_right:
        try:
            rresult.append(src[3*i + header_cnt : 3*i + 2 + header_cnt]) # only take the x and y, not the Confidence
            if rresult[-1][0] == 0 and rresult[-1][1] == 0: # meaning this joint was not detected
                rresult = None
                break
        except:
            rresult = None
            break
        
    for i in joint_interest_coded_left:
        try:
            lresult.append(src[3*i + header_cnt : 3*i + 2 + header_cnt]) # only take the x and y, not the Confidence
            if lresult[-1][0] == 0 and lresult[-1][1] == 0: # meaning this joint was not detected
                lresult = None
                break
        except:
            lresult = None
            break
        
    return lresult, rresult

# find out the actual length of arm while still
ew_length = 0# this is the length from elbow to wrist in pixel, needs to be updated until stabled ---- elbow-wrist
stable_frame_cnt_ew = 0 # this is the number of frames since last update  ---- elbow-wrist

#se_length = 0# this is the length from elbow to wrist in pixel, needs to be updated until stabled ---- shoulder-elbow
#stable_frame_cnt_se = 0 # this is the number of frames since last update ---- shoulder-elbow
def update_arm_length(coord):
    '''
    This function updates the ew_length in the beginning of engaging.
    If the ew_length does not increase for 10 consecutive frames, consider the person is engaged
    The threshold for checking ew_length changes or not is chosen to be 5
    Parameter coord contains 3 lists, each containg x,y
    '''
    #print('update arm length line 217')
    global stable_frame_cnt_ew
    global ew_length
    #global stale_frame_cnt_se
    #global se_length
    
    coord = np.array(coord)
    ew_length_current = np.linalg.norm (coord[2] - coord[1])
    #se_length_current = np.linalg.norm (coord[1] - coord[0])
    print('-----------', ew_length_current)
    #print('-----------', se_length_current)
    
    if ew_length_current < ew_length or (ew_length - ew_length_current) <= 3:
        stable_frame_cnt_ew = stable_frame_cnt_ew + 1
        ew_length = np.max(ew_length, ew_length_current)
    else:
        ew_length = ew_length_current
        stable_frame_cnt_ew = 0
    
    #if se_length_current < se_length or (se_length - se_length_current) <= 5:
        #stable_frame_cnt = stable_frame_cnt + 1
    #else:
        #se_length = se_length_current
        #stable_frame_cnt = 0
    print(ew_length)
    #if stable_frame_cnt_ew >= 10 and stable_frame_cnt_se >= 10:
    if stable_frame_cnt_ew >= 10:
        return True
        
# Several import parameters to set up
table_depth = 1.6 # this is the actual length of the table
table_width = 1.6 # this is the actual width of the table
table_height = 0.65 # this is the actual height of the table to the camera

#table_width_pixel = table_width / 0.295 * 735 * 0.655 /table_depth # table width in pixel, calibrated using our own camera
#table_height_pixel = table_height / 0.295 * 735 * 0.655 /table_depth # table length in pixel, calibrated using our own camera
px_per_meter = 224.44/table_depth # pixels per meter at distance table_depth
#Transform meters into pixels
table_depth = 224.44
table_width = table_width * px_per_meter
table_height = table_height * px_per_meter
#ew_length = ew_length * px_per_meter
ew_length = 59
#se_length = se_length * px_per_meter

center_coord = np.array([159, 87]) # Assume camera takes 320 x 176 images
#1920x1080     

def calcPointing(shoulder, wrist, elbow):
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
    shoulder = np.array(shoulder) - center_coord
    wrist = np.array(wrist) - center_coord
    elbow = np.array(elbow) - center_coord
    
    s_x_p, s_y_p = shoulder
    s_x, s_y = shoulder # assumption 1
    w_x_p, w_y_p = wrist
    e_x_p, e_y_p = elbow
    s_z = s_z_p = e_z_p = w_z_p = table_depth # assumption 2
    se_length = ew_length # assumption 3
     
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
    table_y = table_height
    table_x = w_x - (w_y - table_y) / (e_y - w_y) * (e_x - w_x) # in pixel
    table_z = w_z - (w_y - table_y) / (e_y - w_y) * (e_z - w_z) # in pixel
    
    # map table_x back to meters
    #table_x = table_x * table_depth / 0.655 * 0.295 /table_width / 735  
    
    return (table_x, table_z) # returned points are in pixels


lpoint = (0, 0) # inferred pointing coordinate on the table from left arm
rpoint = (0, 1) # inferred pointing coordinate on the table from right arm

def updatePoint():
    global lpoint
    global rpoint
    '''
    Connect to the kinect server and get the coordinate
    '''
    stabled = False # whether the person is stabled/enaged or not
    s = None
    while True:
        s = connect()
        if s is None:
            print("Server is not available, try again...")
            time.sleep(5)
            continue
        else:
            break
    print("Server is connected!")
    while True:
        try:
            stabled = True
            f = recv_skeleton_frame(s)
            decoded = decode_frame_openpose(f)
            #print(decoded, '\n')
            lcoord, rcoord = getImportantJoints(decoded) # lcoord contains left joint coordinates for shoulder, elbow, wrist
            print('coordinates_left: ', lcoord)
            print('coordinates_right: ', rcoord)
            if not stabled:
                #print("not stabled")
                #print(lcoord, ' ', rcoord)
                if lcoord is not None:
                    #print('updating arm length from line 312')
                    stabled = update_arm_length(lcoord)
                if rcoord is not None:
                    stabled = update_arm_length(rcoord)
            else:
                #print("stabled")
                if lcoord is not None:
                    lpoint  = calcPointing(lcoord[0], lcoord[2], lcoord[1])
                if rcoord is not None:
                    rpoint  = calcPointing(rcoord[0], rcoord[2], rcoord[1])
                print('Interpolated pointing:', lpoint, " ", rpoint)
                print('------------------------')
        except Exception as e:
            #print('Exception catched line 323 ', e)
            continue


if __name__ == '__main__':
    threading.Thread(target=updatePoint).start()
    
    #Plot where the pointing position is on the table
    #The table has a width of (-1,1) and depth of (0.0, 1.6)
    fig = plt.figure(figsize=(15, 15))
    ax = fig.add_subplot(111)
    
    def animate(i):
        #print(lpoint)
        ax.clear()
        ax.set_xlim(-table_width/2, table_width/2) # width of the table, ie table_x
        ax.set_ylim(0.0, table_depth) # length of the table, ie table_z
        plt.gca().invert_yaxis()
        ax.plot(lpoint[0], lpoint[1], 'bo', label='left hand')
        ax.plot(rpoint[0], rpoint[1], 'ro', label='right hand')
    
    ani = FuncAnimation(fig, animate)
    plt.show()




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

