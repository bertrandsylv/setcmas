#!/usr/bin/env python3
'''

   Sylvain BERTRAND, 2023
   (all variables in SI unit)
   
   
   variables used by the functions of this script
       nbRobots: nb of robots in the fleet (>1)    
       robotNo: no of the current robot for which control is coputed (1 .. nbRobots)
       poses:  size (3 x nbRobots) 
               eg. of use: for robotNo, its pose can be obtained by: poses[:,robotNo-1]   (indexes in Python start from 0)
                           poses[0,robotNo-1]: x-coordinate of robot position (in m)
                           poses[1,robotNo-1]: y-coordinate of robot position (in m)
                           poses[2,robotNo-1]: orientation angle of robot (in rad)
   
'''

import numpy as np
import math
#import rospy
#import rviz_marker
import rviz_marker_array




# ==============   "GLOBAL" VARIABLES KNOWN BY ALL THE FUNCTIONS ===================
# use keyword "global" inside a function if the variable needs to be modified by the function


# ===================================================================================


kp = 0.8


# -- do not modify --
global t0, firstCall
t0 = 0.0
firstCall = True
# -------------------

# =======================================
def control(robotNo, nbRobots, poses, tt):
# =======================================

    # -- do not modify --
    global t0, firstCall   

    # get time and remove offset
    if (firstCall):
        t0 = tt
        firstCall = False
        print("***** Trajectory tracking script from the Exercises folder *****")
    
    t = tt - t0
    # -------------------


    # Reference trajectory
    xRef = 2.*np.cos(0.2*t)
    yRef = 2.*np.sin(0.2*t)
    vxRef = -2.*0.2*np.sin(0.2*t)
    vyRef = 2.*0.2*np.cos(0.2*t)

    # publish marker for visu in RViz
    rviz_marker_array.publish_marker(xRef, yRef, 0.0, robotNo, color=[0.98,0.91,0.31], scale=0.1, marker_name='traj_ref', alpha=0.5)


    # control law
    vx = kp*(xRef-poses[0, 0]) + vxRef
    vy = kp*(yRef-poses[1, 0]) + vyRef


    # deadzone
    if (np.sqrt(vx*vx + vy*vy)<0.01):
        vx = 0.0
        vy = 0.0
    
    return vx,vy
    
# ====================================   

