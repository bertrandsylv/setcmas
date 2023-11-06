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
import rviz_marker




# ==============   "GLOBAL" VARIABLES KNOWN BY ALL THE FUNCTIONS ===================
# use keyword "global" inside a function if the variable needs to be modified by the function


# ===================================================================================
iWPL = 0
WPL = [[2,2], [-2,2], [-2,-2], [2,-2], [2,2]]
epsilonWP = 0.10

kF = 1.5
kvL = 0.8


# =======================================
def control(robotNo, nbRobots, poses, t):
# =======================================
    global iWPL

    rRef = []
    rRef.append( np.array([ [0.] , [0.] ]) )
    rRef.append( np.array([ [-1.5] , [0.866] ]) )
    rRef.append( np.array([ [-1.5] , [-0.866] ]) )
    if (nbRobots>3):
    	rRef.append( np.array([ [1.5] , [-0.866] ]) )
    	if (nbRobots>4):	
    		rRef.append( np.array([ [1.5] , [0.866] ]) )


    xRef = WPL[iWPL][0]
    yRef = WPL[iWPL][1]
    # publish marker for visu in RViz
    rviz_marker.publish_marker(xRef, yRef, 0.0, robotNo, color=[0.98,0.91,0.31], scale=0.3, marker_name='wp_ref', alpha=1.0)


    distLtoWP = np.sqrt( (poses[0,0]-xRef)**2 + (poses[1,0]-yRef)**2 )

    if (distLtoWP<epsilonWP):
        if (iWPL<len(WPL)-1):
            iWPL=iWPL+1
        else:
            iWPL = 0
        print("WP validated by leader")
	

    vxL = kvL*(xRef-poses[0, 0])
    vyL = kvL*(yRef-poses[1, 0])

    if (robotNo==1):  #leader
        vx = vxL
        vy = vyL
    else: #followers
        vx = - kF * ( (poses[0, robotNo-1] - poses[0,0]) - rRef[robotNo-1][0] ) +  vxL
        vy = - kF * ( (poses[1, robotNo-1] - poses[1,0]) - rRef[robotNo-1][1] ) +  vyL


    # deadzone
    if (np.sqrt(vx*vx + vy*vy)<0.01):
        vx = 0.0
        vy = 0.0
    
    return vx,vy
    
# ====================================   

