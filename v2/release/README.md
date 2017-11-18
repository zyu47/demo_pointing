There are three files I used for 2D pointing.

1. Connection.py        It contains the code from Guru to receive decoded frame.
                        getDecoded() is an iterator which gives the decoded frame
                        Usage (see 2D_pointing_calibrationv3.py line 79): for decoded in getDecoded()
                        

2. calibrationv3.py     This defines a class for calibration. The class is used in Pointing class

3. 2D_pointing_calibrationv3.py
                        This is the actual class to calculate pointing coordinate based on 2D images
                        To use this class, create an object (p = Pointing()) and call p.run()
                        (Notice there are several parameters that have default parameters but need to be changed if setting is changed)
                        
Work flow of Pointing class
1. An object p is created, and all parameters in __init__ is initialized
2. After calling p.run(), first a thread to update points is started. (details later)
3. Then the plot_all() method is called to show the result.
    3.1 First thing that is going to show is the calibration messages
        There are 5 steps in calibration:
            "Use left hand to point to the bottom left corner.\nUse right hand to point to the bottom right corner."
            "Use both hands to point to the center."
            "Use left hand to point to the top left corner.\nUse right hand to point to the top right corner."
            "Use LEFT hand to point to the TOP RIGHT corner."
            "Use RIGHT hand to point to the TOP LEFT corner."
        Each step takes 10s to run.
    3.2 After calibration (the flag calibrated in calibrationv3 class is set to True), the plot is going to show left and right hand coordinates 

Details of updating points
After calibration, arm length and the boundary of table which people are pointing at will be set. 
Every time it receives a decoded frame from getDecoded()(2D_pointing_calibrationv3.py:79), it is going to get the lpoint and rpoint. These are two instance members that can be accessed by p.lpoint and p.rpoint. 
The raw lpoint and rpoint are in pixel, and they will be normalized based on the calibrated parameters (2D_pointing_calibrationv3.py:185).
The final lpoint and rpoint are in the following ranges:
    the first parameter: from -1 to 1 (left and right edge, respectively)
    the second parameter: from 0 to 1.22 (top and bottom edge, respectively)
