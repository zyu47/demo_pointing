import numpy as np
p2d = __import__('2D_pointing_moreComplex')

p = p2d.Pointing()
p.s_x_c = 107.9823226928711 - 159
p.s_y_c = 116.03087615966797 - 87
p.e_x_c = 100.01809692382812 - 159
p.e_y_c = 172.99375915527344 - 87

p.calcRotateAngle()
p.calcTableLimit()
p.calcArmLength()
print(p.s_y_c*np.tan(p.theta)+ p.Tz/np.cos(p.theta))
print(p.A)
print(p.B)
print(p.C)
print(p.ew_length_r)
D = [(110.01750183105469, 114.95789337158203), (89.9739990234375, 141.0186767578125), (65.98894500732422, 165.9996795654297)]
B = [(107.97676849365234, 116.03658294677734), (67.96641540527344, 156.0148468017578), (21.997976303100586, 194.0050048828125)]
p.rcoord = D
p.cal.calibrated = True
p.rpoint  = p.calcPointing(p.rcoord[0], p.rcoord[2], p.rcoord[1], 'r')
print(p.rpoint)

print([-(p.s_x_c*p.Tz*(p.e_y_c - p.s_y_c))**2,
                                  -(p.s_x_c*p.Ty*(p.s_y_c - p.e_y_c))**2-2*p.s_x_c*p.Tz*(p.e_x_c-p.s_x_c)*p.Ty*p.Tz*(p.e_y_c - p.s_y_c),
                                  (p.s_x_c*p.Ty*(p.s_y_c-p.e_y_c))**2 - 2*((p.e_x_c - p.s_x_c)*(p.Tz**2) + p.Tx*p.s_y_c*(p.s_y_c-p.e_y_c)/2)*p.s_x_c*p.Ty*(p.s_y_c - p.e_y_c) - 2*p.Tz*(p.s_x_c*p.s_y_c - p.e_x_c*p.s_y_c + p.Tx*p.e_y_c/2 + p.Tx*p.s_y_c/2)*p.s_x_c*p.Tz*(p.e_y_c-p.s_y_c) - ((p.e_x_c-p.s_x_c)*p.Ty*p.Tz)**2,
                                  2*((p.e_x_c - p.s_x_c)*(p.Tz**2) + p.Tx*p.s_y_c*(p.s_y_c-p.e_y_c)/2)*p.s_x_c*p.Ty*(p.s_y_c - p.e_y_c) - ((p.e_x_c - p.s_x_c)*(p.Tz**2) + p.Tx*p.s_y_c*(p.s_y_c-p.e_y_c)/2)**2 - 2*p.Tz*(p.s_x_c*p.s_y_c - p.e_x_c*p.s_y_c + p.Tx*p.e_y_c/2 + p.Tx*p.s_y_c/2)*(p.e_x_c-p.s_x_c)*p.Ty*p.Tz,
                                  ((p.e_x_c - p.s_x_c)*(p.Tz**2) + p.Tx*p.s_y_c*(p.s_y_c-p.e_y_c)/2)**2 - (p.Tz*(p.s_x_c*p.s_y_c - p.e_x_c*p.s_y_c + p.Tx*p.e_y_c/2+ p.Tx*p.s_y_c/2))**2
                                  ])
print(p.s_y_c-p.e_y_c)


#[1011024**2+408408**2, (2022048*2986698+2*5503908*408408), (2986698**2+5503908**2+2022048*170274-408408**2), 2*2986698*170274-2*5503908*408408, 170274**2-5503908**2]
