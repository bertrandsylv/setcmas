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




# ==============   "GLOBAL" VARIABLES KNOWN BY ALL THE FUNCTIONS ===================
# use keyword "global" inside a function if the variable needs to be modified by the function


# ===================================================================================




# ========================================
def control(robotNo, nbRobots, poses, t):
# ========================================
    
    # control gain 
    kv = 0.5/nbRobots

    
    # adjacency matrix (full connected graph)
    Adj = np.ones((nbRobots,nbRobots))
    for i in range(nbRobots):
        Adj[i,i] = 0.0


    vx = 0.0
    vy = 0.0
    
    for i in range(nbRobots):
        vx = vx -kv*Adj[robotNo-1,i]*(poses[0, robotNo-1] - poses[0, i])
        vy = vy -kv*Adj[robotNo-1,i]*(poses[1, robotNo-1] - poses[1, i])

    # deadzone
    if (np.sqrt(vx*vx + vy*vy)<0.01):
        vx = 0.0
        vy = 0.0
    
    return vx,vy
    
# ====================================   

