import numpy as np
f = open('res/test0201')
rpoint_arr = []
for line in f:
    if 'calInd:  2' in line:
        f.readline()
        f.readline()
        f.readline()
        rpoint = f.readline()
        rpoint = rpoint.split('rpoint:  (')[1].split(')')[0].split(', ')
        #print(rpoint)
        rpoint_arr.append(np.array(rpoint, dtype=np.float32))
print(len(rpoint_arr))
print(np.median(rpoint_arr, axis = 0))
f.close()
