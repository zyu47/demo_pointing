'''
table:
C-------D
---------
----E----
---------
A-------B

'''

class Calibration:
    def __init__(self):
        # parameters used to calibrate
        self.calibrated = False
        self.ew_calibrated = False
        
        self.ew_length_arr_l = []  
        self.ew_length_arr_r = []  
        
        self.point_limit_left = {'Ax_arr':[], 'Az_arr':[], 'Ex_arr':[], 'Ez_arr':[],
                                 'Cx_arr':[], 'Cz_arr':[], 'Dx_arr':[], 'Dz_arr':[]}
        
        self.point_limit_right = {'Bx_arr':[], 'Bz_arr':[], 'Ex_arr':[], 'Ez_arr':[],
                                 'Dx_arr':[], 'Dz_arr':[], 'Cx_arr':[], 'Cz_arr':[]}

        
        # parameters got from calibration
        #self.dummy_table_bottom = 0
        #self.dummy_table_top = 0
        #self.dummy_table_left = 0
        #self.dummy_table_right = 0
        
    def calibrate(self, seq, ew_length_changing_l, ew_length_changing_r, lpoint, rpoint):
        if seq == 0:
            self.ew_length_arr_l.append(ew_length_changing_l)
            self.ew_length_arr_r.append(ew_length_changing_r)
        elif seq == -2:
            self.point_limit_left['Ax_arr'].append(lpoint[0])
            self.point_limit_left['Az_arr'].append(lpoint[1])
            self.point_limit_right['Bx_arr'].append(rpoint[0])
            self.point_limit_right['Bz_arr'].append(rpoint[1])
        elif seq == 1:
            self.point_limit_left['Ex_arr'].append(lpoint[0])
            self.point_limit_left['Ez_arr'].append(lpoint[1])
            self.point_limit_right['Ex_arr'].append(rpoint[0])
            self.point_limit_right['Ez_arr'].append(rpoint[1])
            #print(lpoint, rpoint)
        elif seq == 2:
            self.point_limit_left['Cx_arr'].append(lpoint[0])
            self.point_limit_left['Cz_arr'].append(lpoint[1])
            self.point_limit_right['Dx_arr'].append(rpoint[0])
            self.point_limit_right['Dz_arr'].append(rpoint[1])
        elif seq == 3:
            self.point_limit_left['Dx_arr'].append(lpoint[0])
            self.point_limit_left['Dz_arr'].append(lpoint[1])
        elif seq == 4:
            self.point_limit_right['Cx_arr'].append(rpoint[0])
            self.point_limit_right['Cz_arr'].append(rpoint[1])
