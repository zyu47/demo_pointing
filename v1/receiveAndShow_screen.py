import socket
import struct
import numpy as np
import matplotlib.pyplot as plt
import time
from matplotlib.animation import FuncAnimation
import threading
import sys
from scipy.signal import savgol_filter


src_addr = '129.82.45.102'
# src_addr = '127.0.0.1'
src_port = 8000

stream_id = 32;

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
        sock.sendall(struct.pack('<i', stream_id));
    except:
        print("Error: Stream rejected")
        return None
    print("Successfully connected to host")
    return sock


def decode_frame(raw_frame):
    # The format is given according to the following assumption of network data

    # Expect little endian byte order
    endianness = "<"

    # [ commonTimestamp | frame type | Tracked body count | Engaged
    header_format = "qiBB"

    timestamp, frame_type, tracked_body_count, engaged = struct.unpack(endianness + header_format, raw_frame[:struct.calcsize(header_format)])

    # For each body, a header is transmitted
    # TrackingId | HandLeftConfidence | HandLeftState | HandRightConfidence | HandRightState ]
    body_format = "Q4B"

    # For each of the 25 joints, the following info is transmitted
    # [ JointType | TrackingState | Position.X | Position.Y | Position.Z | Orientation.W | Orientation.X | Orientation.Y | Orientation.Z ]
    joint_format = "BB7f"

    frame_format = body_format + (joint_format * 25)

    
    # Unpack the raw frame into individual pieces of data as a tuple
    frame_pieces = struct.unpack(endianness + (frame_format * (1 if engaged else 0)), raw_frame[struct.calcsize(header_format):])
    
    decoded = (timestamp, frame_type, tracked_body_count, engaged) + frame_pieces

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
    #print load_size
    return recv_all(sock, load_size)
    

# following codes get the elbow and wrist information from the kinect sensor
class Pointing:
    def __init__(self):
        self.WRISTLEFT = 6  # JointType specified by kinect
        self.WRISTRIGHT = 10
        self.ELBOWLEFT = 5
        self.ELBOWRIGHT = 9
        self.joint_interest_coded = [self.WRISTLEFT, self.WRISTRIGHT, self.ELBOWLEFT, self.ELBOWRIGHT]

        self.joint_info = {i: None for i in self.joint_interest_coded}  # contains left/right wrists/elbows coordinates
        self.joint_info_buffer = {i: [] for i in self.joint_interest_coded}
        self.prev_joint_info = {i: None for i in self.joint_interest_coded}

        self.lpoint_buffer = []
        self.rpoint_buffer = []
        self.lpoint_tmp = (0.0, -0.6)
        self.rpoint_tmp = (0.0, -0.6)
        self.prev_lpoint = (0.0, -0.6)
        self.prev_rpoint = (0.0, -0.6)
        self.lpoint = (0.0, -0.6)  # inferred pointing coordinate on the table from left arm
        self.rpoint = (0.0, -0.6)  # inferred pointing coordinate on the table from right arm

    def get_pointing_main(self, src, is_smoothing_joint=True, is_smoothing_point=True):
        try:
            self._get_wrist_elbow(src)
            if is_smoothing_joint:
                pass
                self._smoothing_joint(11, 2)
                # self._smoothing_point_weight(10)
                self._smoothing_joint_mean(11)
            self._get_pointing(True)  # True is coordinates on screen
            if is_smoothing_point:
                pass
                # self._smoothing_point_weight(1)
                self._smoothing_point_mean(11)
                self._smoothing_point(11, 2)
            self.lpoint = (self.lpoint_tmp[0]-0.25, self.lpoint_tmp[1])
            self.rpoint = (self.rpoint_tmp[0]+0.25, self.rpoint_tmp[1])
        except Exception as e:
            print(e)

    def _get_wrist_elbow(self, src):
        '''
        This function retrieves the coordinates for left/right wrists/elbows (4 sets of 3 values: x, y, z)
        @:param src: decoded frame retrieved from the decode_frame() function
        '''
        try:
            for i in range(25):
                if src[(i+1)*9] in self.joint_interest_coded:
                    self.joint_info[src[(i+1)*9]] = src[(i+1)*9 + 2: (i+2)*9 + 5]
        except IndexError:
            print('Not enough coordinates to unpack')

    def _smoothing_joint(self, window_length=5, polyorder=2):
        for k, v in self.joint_info_buffer.items():
            if len(v) >= window_length:
                self.joint_info_buffer[k].pop(0)
                self.joint_info_buffer[k].append(self.joint_info[k])
                joint_smoothed = savgol_filter(self.joint_info_buffer[k], window_length, polyorder, axis=0).tolist()
                self.joint_info[k] = joint_smoothed[window_length//2]
            else:
                self.joint_info_buffer[k].append(self.joint_info[k])
            self.prev_joint_info[k] = self.joint_info[k]

    def _smoothing_joint_weight(self, c):
        for k, v in self.prev_joint_info.items():
            if v is not None:
                self.joint_info[k] = (self._weighting(self.joint_info[k][0], self.prev_joint_info[k][0], c),
                                      self._weighting(self.joint_info[k][1], self.prev_joint_info[k][1], c),
                                      self._weighting(self.joint_info[k][2], self.prev_joint_info[k][2], c),)

    def _smoothing_joint_mean(self, window_length=5):
        for k, v in self.joint_info_buffer.items():
            if len(v) >= window_length:
                self.joint_info_buffer[k].pop(0)
                self.joint_info_buffer[k].append(self.joint_info[k])
                self.joint_info[k] = np.mean(self.joint_info_buffer[k], axis=0)
            else:
                self.joint_info_buffer[k].append(self.joint_info[k])
            self.prev_joint_info[k] = self.joint_info[k]

    def _smoothing_point(self, window_length=5, polyorder=2):
        '''
        Smoothing function for left and right pointing coordinates
        :param window_length:
        :param polyorder:
        :return:
        '''
        if len(self.lpoint_buffer) >= window_length:
            self.lpoint_buffer.pop(0)
            self.lpoint_buffer.append(self.lpoint_tmp)
            left_smoothed = savgol_filter(self.lpoint_buffer, window_length, polyorder, axis=0).tolist()
            self.lpoint_tmp = left_smoothed[int(window_length/2)]
        else:
            self.lpoint_buffer.append(self.lpoint_tmp)

        if len(self.rpoint_buffer) >= window_length:
            self.rpoint_buffer.pop(0)
            self.rpoint_buffer.append(self.rpoint_tmp)
            self.rpoint_buffer = savgol_filter(self.rpoint_buffer, window_length, polyorder, axis=0).tolist()
            self.rpoint_tmp = self.rpoint_buffer[int(window_length/2)]
        else:
            self.rpoint_buffer.append(self.rpoint_tmp)

        self.prev_lpoint = self.lpoint_tmp
        self.prev_rpoint = self.rpoint_tmp

    def _smoothing_point_weight(self, c):
        self.lpoint_tmp = (self._weighting(self.lpoint_tmp[0], self.prev_lpoint[0], c),
                       self._weighting(self.lpoint_tmp[1], self.prev_lpoint[1], c))
        self.rpoint_tmp = (self._weighting(self.rpoint_tmp[0], self.prev_rpoint[0], c),
                       self._weighting(self.rpoint_tmp[1], self.prev_rpoint[1], c))

    def _smoothing_point_mean(self, window_length=5):
        if len(self.lpoint_buffer) >= window_length:
            self.lpoint_buffer.pop(0)
            self.lpoint_buffer.append(self.lpoint_tmp)
            self.lpoint_tmp = np.mean(self.lpoint_buffer, axis=0)
        else:
            self.lpoint_buffer.append(self.lpoint_tmp)

        if len(self.rpoint_buffer) >= window_length:
            self.rpoint_buffer.pop(0)
            self.rpoint_buffer.append(self.rpoint_tmp)
            self.rpoint_tmp = np.mean(self.rpoint_buffer, axis=0)
        else:
            self.rpoint_buffer.append(self.rpoint_tmp)

        self.prev_lpoint = self.lpoint_tmp
        self.prev_rpoint = self.rpoint_tmp

    def _weighting(self, curr, prev, c):
        diff = np.abs(curr-prev)
        w = np.exp(-c*diff)
        return w*curr + (1-w)*prev

    def _get_pointing(self, screen=True):
        lwrist = self.joint_info[self.WRISTLEFT]
        rwrist = self.joint_info[self.WRISTRIGHT]
        lelbow = self.joint_info[self.ELBOWLEFT]
        relbow = self.joint_info[self.ELBOWRIGHT]

        self.lpoint_tmp = self._calc_coordinates(lwrist, lelbow, screen)
        self.rpoint_tmp = self._calc_coordinates(rwrist, relbow, screen)

    def _calc_coordinates(self, wrist, elbow, screen=True):
        if screen:
            '''
            Both wrist and elbow should contain (x,y,z) coordinates
            screen plane: z=0, ie pz = 0
            Line equation:
                (ex - px)/(ex - wx) = (ez - pz)/(ez - wz) = ez/(ez - wz)
                (ey - py)/(ey - wy) = (ez - pz)/(ez - wz) = ez/(ez - wz)
            so:
                px = ex - ez(ex-wx) / (ez-wz)
                py = ey - ez(ey-wy) / (ez-wz)
            '''
            if (elbow[2] - wrist[2]) == 0:
                return -np.inf, -np.inf
            screen_x = elbow[0] - elbow[2] * (elbow[0] - wrist[0]) / (elbow[2] - wrist[2])
            screen_y = elbow[1] - elbow[2] * (elbow[1] - wrist[1]) / (elbow[2] - wrist[2])

            return screen_x, screen_y
        else:

            '''
            Both wrist and elbow should contain (x,y,z) coordinates
            Table plane: y = -0.582
            Line equation: 
            y = (y2-y1)/(x2-x1) * (x-x1) + y1
            z = (z2-z1)/(y2-y1) * (y-y1) + z1
            so:
            x = x1 - (y1-y) / (y2-y1) * (x2-x1)
            z = z1 - (y1-y) / (y2-y1) * (z2-z1)
            '''
            if (elbow[1] - wrist[1]) == 0:
                return -np.inf, -np.inf
            table_y = -0.582
            table_x = wrist[0] - (wrist[1] - table_y) / (elbow[1] - wrist[1]) * (elbow[0] - wrist[0])
            table_z = wrist[2] - (wrist[1] - table_y) / (elbow[1] - wrist[1]) * (elbow[2] - wrist[2])

            return table_x, table_z

    def test_run(self):
        '''
        Start a thread for testing purpose only
        '''
        threading.Thread(target=self.update_point).start()

    def update_point(self):
        '''
        Connect to the kinect server and get the coordinate
        Used for testing only
        Use
        '''
        s = connect()
        if s is None:
            sys.exit(0)
        # cnt = 0
        # start = time.time()
        while True:
            try:
                f = recv_skeleton_frame(s)
                self.get_pointing_main(decode_frame(f))
                print(p.lpoint, p.rpoint)
            except Exception as e:
                print(e)
                s.close()
                break
            # cnt += 1
            # if cnt == 30:
            #     print('fps: ', 30/(time.time()-start))
            #     start = time.time()
            #     cnt = 0


if __name__ == '__main__':
    for i in range(10):
        print('***********************************************')
    p = Pointing()
    p.test_run()
    # Plot where the pointing position is on the table
    # The table has a width of (-1,1) and depth of (1.0, 1.6)
    fig = plt.figure(figsize=(7, 7))
    ax = fig.add_subplot(111)

    def animate(i):
        ax.clear()
        ax.set_xlim(-1, 1)  # width of the table, ie table_x
        ax.set_ylim(0.3, -1)  # length of the table, ie table_z
        plt.gca().invert_yaxis()
        llabel = '(%.2f,     %.2f)' % (p.lpoint[0], p.lpoint[1])
        rlabel = '(%.2f,     %.2f)' % (p.rpoint[0], p.rpoint[1])
        ax.plot(p.lpoint[0], p.lpoint[1], 'bo', label=llabel)
        ax.plot(p.rpoint[0], p.rpoint[1], 'ro', label=rlabel)
        plt.legend(prop={'size': 35})
    
    ani = FuncAnimation(fig, animate)
    plt.show()
