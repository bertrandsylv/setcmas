#!/usr/bin/env python3
'''
   Sylvain BERTRAND, 2021 

   Convert pose topic to odom topic
   (all variables in SI unit)

'''

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped



# node init
# --------------
rospy.init_node('pose_to_odom', anonymous=True)


# publishers
# ----------------
pubOdom = rospy.Publisher('odom', Odometry, queue_size=10)
msgOdom = Odometry()

# -----------------------------------------------------------------------------
def callBackPoseStamped(data):
# -----------------------------------------------------------------------------
	msgOdom.header = data.header
	msgOdom.pose.pose = data.pose
	pubOdom.publish(msgOdom)
# -----------------------------------------------------------------------------


# subscribers
# ------------
rospy.Subscriber("pose", PoseStamped, callBackPoseStamped)



# main node loop
# ---------------
# -----------------------------------------------------------------------------
if __name__ == '__main__':
# -----------------------------------------------------------------------------
	rospy.spin()   
# -----------------------------------------------------------------------------

