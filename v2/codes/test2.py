import numpy as np
p2d = __import__('2D_pointing_calibrationv2')

p = p2d.Pointing()

p.ew_length_r = 63.5172321529
p.ew_length_l = 56.7397148331

p.rcoord = [(85.9852523803711, 124.05491638183594), (62.000736236572266, 143.98744201660156), (28.016387939453125, 173.0023651123047)]
p.lcoord = [(153.9945831298828, 119.99686431884766), (174.99449157714844, 138.00599670410156), (203.02813720703125, 164.00279235839844)]
p.cal.calibrated = True
p.rpoint  = p.calcPointing(p.rcoord[0], p.rcoord[2], p.rcoord[1], 'r')
p.lpoint  = p.calcPointing(p.lcoord[0], p.lcoord[2], p.lcoord[1], 'l')


print(p.rpoint)
print(p.lpoint)

p.rcoord = [(86.00584411621094, 124.99128723144531), (62.992183685302734, 143.0208740234375), (28.030654907226562, 173.01339721679688)]
p.rpoint  = p.calcPointing(p.rcoord[0], p.rcoord[2], p.rcoord[1], 'r')
print(p.rpoint)
