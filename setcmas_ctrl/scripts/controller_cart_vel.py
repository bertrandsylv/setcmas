#!/usr/bin/env python3
'''
   Sylvain BERTRAND, 2023

   Distributed controller for multi-robots
   (all variables in SI unit)
'''

import rospy
import numpy as np
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Quaternion, Point32
import tf


#import leader_follower_formation as ctrlr



# node init
# --------------
rospy.init_node('ctrl_cart_vel', anonymous=True)


# control frequency (assumed to be identical for position and orientation ctrl loops)
# -------------------------------------------------------------------------------------
frequency = 5.0
Ts = 1.0/frequency
cmdPubRate = rospy.Rate(frequency)


# parameters
# ------------
Vnom = rospy.get_param('~Vnom',0.5) # nominal speed (m/s)    #0.10
safeDist = rospy.get_param('~safeDist', 0.2)
robotNo = rospy.get_param('~robotNo', 1)
nbRobots = rospy.get_param('~nbRobots', 3)
algo = rospy.get_param('~algo', "consensus")
exercise = rospy.get_param('~exercise', True)



# choose from which Python script to run control algo (consensus by default)
# ---------------------------------------------------------------------------

if (exercise):
    if (algo=="consensus"):
        import exercises.consensus as ctrlr
	#import consensus as ctrlr
    elif (algo=="leader_follower_formation"):
        import exercises.leader_follower_formation as ctrlr
	#import leader_follower_formation as ctrlr
    elif (algo=="wp_nav"):
        import exercises.wp_nav as ctrlr
	#import wp_nav as ctrlr
    elif (algo=="traj_tracking"):
        import exercises.traj_tracking as ctrlr
	#import traj_tracking as ctrlr
    else:
        rospy.logerr("Error: no valid control algorithm selected")
else:
## Use this line to run .py from the scripts directory
    ctrlr = __import__(algo)



if (nbRobots>8):
    rospy.logerr("Error: max nb of robots is 8")



# init global variables
# ----------------------
global poses
poses = np.zeros((3,nbRobots)) # row index: x,y,theta   col index: robot no

global firstOdomReceived
firstOdomReceived = []
for i in range(nbRobots):
    firstOdomReceived.append(False)


# publishers
# ----------------
pubCmdCartVel = rospy.Publisher('cmd_cart_vel', Point32, queue_size=10)



# -----------------------------------------------------------------------------
def callBackOdometry1(data):
# -----------------------------------------------------------------------------
    global poses,firstOdomReceived
    # assign robot coordinates
    no_robot = 0
    [roll, pitch, yaw] = tf.transformations.euler_from_quaternion([data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w])
    poses[0,no_robot] =  data.pose.pose.position.x
    poses[1,no_robot] =  data.pose.pose.position.y
    poses[2,no_robot] = yaw

    #str = "Callback robot %s: x=%s  y=%s"%(no_robot+1, pose1[0], pose1[1])
    #rospy.loginfo(str)

    firstOdomReceived[no_robot] = True
# -----------------------------------------------------------------------------

# -----------------------------------------------------------------------------
def callBackOdometry2(data):
# -----------------------------------------------------------------------------
    global poses,firstOdomReceived
    # assign robot coordinates
    no_robot = 1
    [roll, pitch, yaw] = tf.transformations.euler_from_quaternion([data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w])
    poses[0,no_robot] =  data.pose.pose.position.x
    poses[1,no_robot] =  data.pose.pose.position.y
    poses[2,no_robot] = yaw

    firstOdomReceived[no_robot] = True
# -----------------------------------------------------------------------------

# -----------------------------------------------------------------------------
def callBackOdometry3(data):
# -----------------------------------------------------------------------------
    global poses,firstOdomReceived
    # assign robot coordinates
    no_robot = 2
    [roll, pitch, yaw] = tf.transformations.euler_from_quaternion([data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w])
    poses[0,no_robot] =  data.pose.pose.position.x
    poses[1,no_robot] =  data.pose.pose.position.y
    poses[2,no_robot] = yaw

    firstOdomReceived[no_robot] = True
# -----------------------------------------------------------------------------

# -----------------------------------------------------------------------------
def callBackOdometry4(data):
# -----------------------------------------------------------------------------
    global poses,firstOdomReceived
    # assign robot coordinates
    no_robot = 3
    [roll, pitch, yaw] = tf.transformations.euler_from_quaternion([data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w])
    poses[0,no_robot] =  data.pose.pose.position.x
    poses[1,no_robot] =  data.pose.pose.position.y
    poses[2,no_robot] = yaw

    firstOdomReceived[no_robot] = True
# -----------------------------------------------------------------------------

# -----------------------------------------------------------------------------
def callBackOdometry5(data):
# -----------------------------------------------------------------------------
    global poses,firstOdomReceived
    # assign robot coordinates
    no_robot = 4
    [roll, pitch, yaw] = tf.transformations.euler_from_quaternion([data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w])
    poses[0,no_robot] =  data.pose.pose.position.x
    poses[1,no_robot] =  data.pose.pose.position.y
    poses[2,no_robot] = yaw

    firstOdomReceived[no_robot] = True
# -----------------------------------------------------------------------------

# -----------------------------------------------------------------------------
def callBackOdometry6(data):
# -----------------------------------------------------------------------------
    global poses,firstOdomReceived
    # assign robot coordinates
    no_robot = 5
    [roll, pitch, yaw] = tf.transformations.euler_from_quaternion([data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w])
    poses[0,no_robot] =  data.pose.pose.position.x
    poses[1,no_robot] =  data.pose.pose.position.y
    poses[2,no_robot] = yaw

    firstOdomReceived[no_robot] = True
# -----------------------------------------------------------------------------

# -----------------------------------------------------------------------------
def callBackOdometry7(data):
# -----------------------------------------------------------------------------
    global poses,firstOdomReceived
    # assign robot coordinates
    no_robot = 6
    [roll, pitch, yaw] = tf.transformations.euler_from_quaternion([data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w])
    poses[0,no_robot] =  data.pose.pose.position.x
    poses[1,no_robot] =  data.pose.pose.position.y
    poses[2,no_robot] = yaw

    firstOdomReceived[no_robot] = True
# -----------------------------------------------------------------------------

# -----------------------------------------------------------------------------
def callBackOdometry8(data):
# -----------------------------------------------------------------------------
    global poses,firstOdomReceived
    # assign robot coordinates
    no_robot = 7
    [roll, pitch, yaw] = tf.transformations.euler_from_quaternion([data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w])
    poses[0,no_robot] =  data.pose.pose.position.x
    poses[1,no_robot] =  data.pose.pose.position.y
    poses[2,no_robot] = yaw

    firstOdomReceived[no_robot] = True
# -----------------------------------------------------------------------------



# subscribers
# ------------
rospy.Subscriber("odom1", Odometry, callBackOdometry1)
rospy.Subscriber("odom2", Odometry, callBackOdometry2)
rospy.Subscriber("odom3", Odometry, callBackOdometry3)
if (nbRobots>3):
    rospy.Subscriber("odom4", Odometry, callBackOdometry4)
    if (nbRobots>4):
        rospy.Subscriber("odom5", Odometry, callBackOdometry5)
        if (nbRobots>5):
            rospy.Subscriber("odom6", Odometry, callBackOdometry6)
            if (nbRobots>6):
                rospy.Subscriber("odom7", Odometry, callBackOdometry7)
                if (nbRobots>7):
                    rospy.Subscriber("odom8", Odometry, callBackOdometry8)



# main node loop
# ---------------

# -----------------------------------------------------------------------------
if __name__ == '__main__':
# -----------------------------------------------------------------------------

   
    # wait for odometries received from all the robots before starting the controller
    str = "Robot %s: waiting"%(robotNo)
    rospy.loginfo(str)
    
    allFirstOdomReceived = False

    while not (allFirstOdomReceived):
        for i in range(nbRobots):
            if i==0:
                allFirstOdomReceived = firstOdomReceived[0]
            else:
                allFirstOdomReceived = allFirstOdomReceived & firstOdomReceived[i]

    str = "Robot %s: starting"%(robotNo)
    rospy.loginfo(str)



    # control loop
   
    cmdCartVelMsg = Point32()
    
    
    
    while not rospy.is_shutdown():
        vx = 0.0
        vy = 0.0

        t = rospy.get_time()  # time in secs

        vx,vy = ctrlr.control(robotNo, nbRobots, poses, t)

        
        V = np.sqrt(vx**2+vy**2)
        
        # scale factor to saturate to Vnom
        if (V>Vnom):
            k_corr = Vnom/V
            vx = k_corr*vx
            vy = k_corr*vy
        #str = "Robot %s: vx=%s  vy=%s"%(robotNo, vx, vy)
        #rospy.loginfo(str)
        
        #str = "Robots positions:  1: x=%s  y=%s  | 2:x=%s  y=%s  |  3: x=%s  y=%s"%(poses[0,0], poses[1,0], poses[0,1], poses[1,1], poses[0,2], poses[1,2])
        #rospy.loginfo(str)
        
        # control msg
        cmdCartVelMsg.x = vx
        cmdCartVelMsg.y = vy
        cmdCartVelMsg.z = 0.0
        
        # msg publication
        pubCmdCartVel.publish(cmdCartVelMsg)
        
        cmdPubRate.sleep()

# -----------------------------------------------------------------------------

