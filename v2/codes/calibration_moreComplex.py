

class Calibration:
    def __init__(self):
        # parameters used to calibrate
        self.s_x_arr_r = []
        self.e_x_arr_r = []
        self.s_y_arr_r = []
        self.e_y_arr_r = []
        self.dummy_tablez_top_arr_r = []
        self.dummy_tablex_right_arr_r = [] 
        self.ew_length_arr_l = []  
        self.ew_length_arr_r = []  
        self.calibrated = False
        self.ew_calibrated = False
        
        # parameters got from calibration
        #self.dummy_table_bottom = 0
        #self.dummy_table_top = 0
        #self.dummy_table_left = 0
        #self.dummy_table_right = 0
        
    def calibrate(self, seq, shoulder, elbow):
        if seq == 0:
            self.s_x_arr_r.append(shoulder[0])
            self.s_y_arr_r.append(shoulder[1])
        elif seq == 1:
            self.e_x_arr_r.append(elbow[0])
            self.e_y_arr_r.append(elbow[1])
            #self.dummy_tablez_bottom_arr_r.append(rpoint[1])
            #self.dummy_tablex_left_arr_l.append(lpoint[0])
            #self.dummy_tablez_bottom_arr_l.append(lpoint[1])
            #print(lpoint, rpoint)
        #elif seq == 2:
            #self.dummy_tablex_right_arr_r.append(rpoint[0])
            #self.dummy_tablez_top_arr_r.append(rpoint[1])
            #self.dummy_tablex_left_arr_l.append(lpoint[0])
            #self.dummy_tablez_top_arr_l.append(lpoint[1])
