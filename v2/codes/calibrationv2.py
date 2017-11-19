

class Calibration:
    def __init__(self):
        # parameters used to calibrate
        self.dummy_tablez_center_arr_l = []
        self.dummy_tablez_center_arr_r = []
        self.dummy_tablex_center_arr_l = []
        self.dummy_tablex_center_arr_r = []
        self.dummy_tablez_top_arr_l = []
        self.dummy_tablex_left_arr_l = [] # left boundary when pointed by left hand
        self.dummy_tablez_top_arr_r = []
        self.dummy_tablex_right_arr_r = [] 
        self.dummy_tablex_right_arr_l = []
        self.dummy_tablex_left_arr_r = []
        self.ew_length_arr_l = []  
        self.ew_length_arr_r = []  
        self.calibrated = False
        self.ew_calibrated = False
        
        # parameters got from calibration
        #self.dummy_table_bottom = 0
        #self.dummy_table_top = 0
        #self.dummy_table_left = 0
        #self.dummy_table_right = 0
        
    def calibrate(self, seq, ew_length_changing_l, ew_length_changing_r, lpoint, rpoint):
        if seq == 0:
            self.ew_length_arr_l.append(ew_length_changing_l)
            self.ew_length_arr_r.append(ew_length_changing_r)
        elif seq == 1:
            self.dummy_tablex_center_arr_r.append(rpoint[0])
            self.dummy_tablez_center_arr_r.append(rpoint[1])
            self.dummy_tablex_center_arr_l.append(lpoint[0])
            self.dummy_tablez_center_arr_l.append(lpoint[1])
            #print(lpoint, rpoint)
        elif seq == 2:
            self.dummy_tablex_right_arr_r.append(rpoint[0])
            self.dummy_tablez_top_arr_r.append(rpoint[1])
            self.dummy_tablex_left_arr_l.append(lpoint[0])
            self.dummy_tablez_top_arr_l.append(lpoint[1])
        elif seq == 3:
            self.dummy_tablex_right_arr_l.append(lpoint[0])
        elif seq == 4:
            self.dummy_tablex_left_arr_r.append(rpoint[0])
