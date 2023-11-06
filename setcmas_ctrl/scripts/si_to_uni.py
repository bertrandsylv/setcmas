#!/usr/bin/env python3
'''
   Sylvain BERTRAND, 2021 

   Convert cartesian velocity vector to unicycle control V, omega
   (all variables in SI unit)

'''

import rospy
import numpy as np
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Quaternion, Point32
from std_msgs.msg import Bool
import tf


# node init
# --------------
rospy.init_node('si_to_uni', anonymous=True)


# control frequency (assumed to be identical for position and orientation ctrl loops)
# -------------------------------------------------------------------------------------
frequency = 5.0
Ts = 1.0/frequency
cmdPubRate = rospy.Rate(frequency)


# parameters
# ------------
komega = rospy.get_param('komega', 1.5)


# init global variables
# ----------------------
x = 0.0
y = 0.0
theta = 0.0
vx = 0.0
vy = 0.0
firstOdomReceived = False
firstCmdCartVelReceived = False
pause = False


# publishers
# ----------------
pubCmdVel = rospy.Publisher('cmd_vel', Twist, queue_size=10)




# -----------------------------------------------------------------------------
def callBackOdometry(data):
# -----------------------------------------------------------------------------
    global x,y,theta, firstOdomReceived
    # assign robot coordinates
    x =  data.pose.pose.position.x
    y =  data.pose.pose.position.y
    [rool, pitch, yaw] = tf.transformations.euler_from_quaternion([data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w])
    theta = yaw
    firstOdomReceived = True
# -----------------------------------------------------------------------------



# -----------------------------------------------------------------------------
def callBackCmdCartVel(data):
# -----------------------------------------------------------------------------
    global vx, vy, firstCmdCartVelReceived
    vx = data.x
    vy = data.y
    firstCmdCartVelReceived = True
# -----------------------------------------------------------------------------



# -----------------------------------------------------------------------------
def callBackPause(data):
# -----------------------------------------------------------------------------
    global pause
    pause = data.data
# -----------------------------------------------------------------------------




# subscribers
# ------------
rospy.Subscriber("odom", Odometry, callBackOdometry)
rospy.Subscriber("cmd_cart_vel", Point32, callBackCmdCartVel)
rospy.Subscriber("pause", Bool, callBackPause)




# main node loop
# ---------------

# -----------------------------------------------------------------------------
if __name__ == '__main__':
# -----------------------------------------------------------------------------

    cmdVelMsg = Twist()
    
    while not (firstOdomReceived & firstCmdCartVelReceived):
        #wait for message
        nothing_to_do = 0.0
        
    
    # control msg
    cmdVelMsg.linear.y = 0.0
    cmdVelMsg.linear.z = 0.0
    cmdVelMsg.angular.x = 0.0
    cmdVelMsg.angular.y = 0.0

    while not rospy.is_shutdown():


        V = np.sqrt( vx**2 + vy**2)

        #if (V==0.0):  # TO BE MODIFIED : V<epsilon
        if (V<0.01):
            thetaRef = theta
            omega = 0.0
        else:    
            thetaRef = np.arctan2(vy, vx)
    
            # avoid U-turns    
            if math.fabs(theta-thetaRef)>math.pi:
                thetaRef += math.copysign(2*math.pi, theta)        

            # angular speed    
            omega = komega * (thetaRef - theta)

            # deadzone
            if (np.fabs(omega)<0.01):
                omega = 0.0


        if pause:
            cmdVelMsg.linear.x = 0.0
            cmdVelMsg.angular.z = 0.0
        else:
            cmdVelMsg.linear.x = V
            cmdVelMsg.angular.z = omega


        # msg publication
        pubCmdVel.publish(cmdVelMsg)
        
        cmdPubRate.sleep()

# -----------------------------------------------------------------------------
