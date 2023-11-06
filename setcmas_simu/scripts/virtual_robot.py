#!/usr/bin/python3
'''
   Sylvain BERTRAND, 2023 

   Virtual robot (pose from unicycle dynamics) from Twist msg 
   (all variables in SI unit)

'''

import rospy
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseStamped, Point, Vector3, Quaternion
from std_msgs.msg import Empty
import tf
#from tf.transformations import quaternion_from_euler

from visualization_msgs.msg import Marker


# node init
# --------------
rospy.init_node('virtual_robot', anonymous=False)
#ns=str(rospy.get_namespace().strip('/'))

# node frequency
# -----------------
nodeFreq = 10.0
Ts = 1.0/nodeFreq
nodeRate = rospy.Rate(nodeFreq)


# parameters
# ------------


#no of robot (in case of multi-robots system)
robot_no = rospy.get_param("~robot_no", 1)

# ids of frames
frame_id = rospy.get_param('~frame_id', 'world')
child_frame_id = rospy.get_param('~child_frame_id', 'odom')

# initialize pose of robot
x0 = rospy.get_param('~x0',0.0) # initial position (x0,y0) (m)
y0 = rospy.get_param('~y0',0.0)
theta0 = rospy.get_param('~theta0',0.0) # initial heading angle (theta0) (rad)
x = x0
y = y0
theta = theta0


# global variables
# -----------------
V = 0.0
Omega = 0.0


# color list for robot markers in RViz (8 robots)
color_list_rgb = [[1,0,0], [0,1,0], [0,0,1], [1,0.91,0.31], [1,1,1], [0.96,0.18,0.92], [1,0.49,0.37], [0,0,0]]
# if not enough predefined colors, use gray color by default
while len(color_list_rgb)<robot_no:
    color_list_rgb.append([0.5,0.5,0.5])


# publishers
# ----------------
pubOdom = rospy.Publisher('odom', Odometry, queue_size=10)
pubPose = rospy.Publisher('pose', PoseStamped, queue_size=10)
pubMarker = rospy.Publisher('marker', Marker, queue_size=10)


# messages
# ----------------
msgOdom = Odometry()
msgPose = PoseStamped()

marker = Marker()
marker.header.frame_id = frame_id
marker.header.stamp = rospy.Time.now()
marker.ns = "robot"
marker.id = 0
marker.type = Marker.MESH_RESOURCE
marker.mesh_resource = "package://setcmas_simu/meshes/tb3.stl"
marker.action = 0  #ADD
#marker.pose.orientation.x = 0.0
#marker.pose.orientation.y = 0.0
#marker.pose.orientation.z = 0.0
#marker.pose.orientation.w = 1.0
marker.scale.x = 1
marker.scale.y = 1
marker.scale.z = 1
marker.color.a = 1.0#0.7
marker.color.r = color_list_rgb[robot_no-1][0] #0.0
marker.color.g = color_list_rgb[robot_no-1][1] #1.0
marker.color.b = color_list_rgb[robot_no-1][2] #0.0



# -----------------------------------------------------------------------------
def callBackTwist(data):
# -----------------------------------------------------------------------------
    global V, Omega
    V = data.linear.x
    Omega = data.angular.z
# -----------------------------------------------------------------------------


# -----------------------------------------------------------------------------
def callBackReset(data):
# -----------------------------------------------------------------------------
    global x, y, theta
    x = x0
    y = y0
    theta = theta0
# -----------------------------------------------------------------------------


# subscribers
# ------------
#rospy.Subscriber('/'+ns+'/cmd_vel', Twist, callBackTwist)
rospy.Subscriber('cmd_vel', Twist, callBackTwist)
rospy.Subscriber('reset', Empty, callBackReset)


# main node loop
# ---------------
# -----------------------------------------------------------------------------
if __name__ == '__main__':
# -----------------------------------------------------------------------------
    
    #global x, y, theta

    tfbr = tf.TransformBroadcaster()
    
    while not rospy.is_shutdown():
        

        # update virtual robot pose
        theta += Ts*Omega
        x += Ts*V*math.cos(theta)
        y += Ts*V*math.sin(theta)
        
        #str = "(x=%s  y=%s theta=%s)"%(x, y, theta)
        #rospy.loginfo(str)
        
        t = rospy.Time.now()
        
        position = Point(x, y, 0.0)        
        q = tf.transformations.quaternion_from_euler(0.0, 0.0, theta)
        quat = Quaternion(q[0], q[1], q[2], q[3])


        # fill messages
        # ----------------        

	# pose
        msgPose.header.seq += 1
        msgPose.header.stamp = t
        msgPose.header.frame_id = frame_id
        msgPose.pose.position = position
        msgPose.pose.orientation = quat
        
        # odometry
        msgOdom.header.seq +=1 
        msgOdom.header.stamp = t
        msgOdom.header.frame_id = frame_id
        msgOdom.child_frame_id = child_frame_id
        msgOdom.pose.pose.position = position
        msgOdom.pose.pose.orientation = quat
        msgOdom.twist.twist.linear = Vector3(V, 0.0, 0.0)
        msgOdom.twist.twist.angular = Vector3(0.0, 0.0, Omega)

        # marker
        marker.header.stamp = t
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.0
        marker.pose.orientation = quat
        
        
        # msgs publication
        pubPose.publish(msgPose)
        pubOdom.publish(msgOdom)
        pubMarker.publish(marker)
    
        # tf broadcast
        tfbr.sendTransform( (x, y, 0), q, t, child_frame_id, frame_id)
    
        
        # wait one step
        nodeRate.sleep()
# -----------------------------------------------------------------------------

