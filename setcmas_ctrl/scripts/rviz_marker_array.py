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

import rospy
from visualization_msgs.msg import Marker, MarkerArray



# ==============   "GLOBAL" VARIABLES KNOWN BY ALL THE FUNCTIONS ===================
# use keyword "global" inside a function if the variable needs to be modified by the function


# ===================================================================================

global firstCall, marker_array, pubMarkerArray, marker_id
firstCall = True

marker_array = MarkerArray()
marker_id = 0


# =======================================
def publish_marker(x, y, z, robotNo, color=[0.25,0.25,0.25], scale=0.2, marker_name='ref', alpha=0.5):
# =======================================
    global firstCall, marker_array, pubMarkerArray, marker_id

    # get time and remove offset
    if (firstCall):
        pubMarkerArray = rospy.Publisher('/tb3_'+str(robotNo)+'/'+marker_name, MarkerArray, queue_size=10)
        firstCall = False	


    marker = Marker()
    marker.header.frame_id = "world" #frame_id
    marker.ns = "tb3_"+str(robotNo)  #TO BE MODIFIED FOR 
    marker.header.stamp = rospy.Time.now()
    marker.id = marker_id
    marker_id = marker_id + 1
    marker.type = Marker.SPHERE
    marker.action = 0  #ADD
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0
    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = z
    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]
    marker.color.a = alpha
    marker.scale.x = scale
    marker.scale.y = scale
    marker.scale.z = scale

    marker_array.markers.append(marker)

    pubMarkerArray.publish(marker_array)
    
# ====================================   

