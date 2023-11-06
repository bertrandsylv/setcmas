#!/usr/bin/env python3
'''
   Sylvain BERTRAND, 2023 

   Trigger communication of localization information depending on custom condition (CTC)
   (all variables in SI unit)

'''

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import UInt16
import numpy as np



# global variables
# -------------------
global last_pos, last_vit
global last_Odom
last_Odom = Odometry()

global firstCall
firstCall = True


# node init
# --------------
rospy.init_node('comm', anonymous=True)


# parameters
# --------------
exercise = rospy.get_param('~exercise', True)

if (exercise):
    import exercises.ctc as ctc
else:
    import ctc as ctc


# publishers
# ----------------
pubOdom = rospy.Publisher('trig_odom', Odometry, queue_size=10)
#msgOdom = Odometry()

pubCommEvent = rospy.Publisher('trig_event', UInt16, queue_size=10)
msgTrigEvent = UInt16()


# -----------------------------------------------------------------------------
def callBackOdom(data):
# -----------------------------------------------------------------------------
	global last_pos, last_vit, last_Odom, firstCall
	
	pos = [data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z]
	vit = [data.twist.twist.linear.x, data.twist.twist.linear.y, data.twist.twist.linear.z]

	if (firstCall):
		last_pos = list(pos)
		last_vit = list(vit)
		last_Odom = data
		firstCall = False

	t = rospy.get_time()	

	if ctc.ctc(np.array(pos), np.array(vit), np.array(last_pos), np.array(last_vit), t):
		last_pos = list(pos)
		last_vit = list(vit)
		last_Odom = data
		pubOdom.publish(data)
		pubCommEvent.publish(UInt16(1))
	else:
		pubOdom.publish(last_Odom)
		pubCommEvent.publish(UInt16(0))

		
# -----------------------------------------------------------------------------


# subscribers
# ------------
rospy.Subscriber("odom", Odometry, callBackOdom)



# main node loop
# ---------------
# -----------------------------------------------------------------------------
if __name__ == '__main__':
# -----------------------------------------------------------------------------
	rospy.spin()
# -----------------------------------------------------------------------------

