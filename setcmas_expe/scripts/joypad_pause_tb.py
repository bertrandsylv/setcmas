#!/usr/bin/env python3
'''
   Sylvain BERTRAND, 2022

   Publish Pause mesages to turtlebots
   (all variables in SI unit)

'''

import rospy
import numpy as np
from std_msgs.msg import Bool
from sensor_msgs.msg import Joy


# node init
# --------------
rospy.init_node('joypad_cmd_vel', anonymous=False)



# parameters
# ------------
AllInPause = True


# global variables
# -----------------
buttons = None
boolMsg = Bool()


# control frequency
# -----------------
frequency = 10.0
Ts = 1.0/frequency
pausePubRate = rospy.Rate(frequency)


# publishers
# ----------------
pubCmdVelTB1 = rospy.Publisher('/tb3_1/pause', Bool, queue_size=10)
pubCmdVelTB2 = rospy.Publisher('/tb3_2/pause', Bool, queue_size=10)
pubCmdVelTB3 = rospy.Publisher('/tb3_3/pause', Bool, queue_size=10)
pubCmdVelTB4 = rospy.Publisher('/tb3_4/pause', Bool, queue_size=10)
pubCmdVelTB5 = rospy.Publisher('/tb3_5/pause', Bool, queue_size=10)
pubCmdVelTB6 = rospy.Publisher('/tb3_6/pause', Bool, queue_size=10)
pubCmdVelTB7 = rospy.Publisher('/tb3_7/pause', Bool, queue_size=10)
pubCmdVelTB8 = rospy.Publisher('/tb3_8/pause', Bool, queue_size=10)


# -----------------------------------------------------------------------------
def callBackJoy(data):
# -----------------------------------------------------------------------------
    global buttons, AllInPause

    for i in range(0, len(data.buttons)):
        if buttons == None or data.buttons[i] != buttons[i]:
            if i == 7 and data.buttons[i] == 1: 
                if (AllInPause):
                    AllInPause = False
                    rospy.loginfo("Start !")
                else:
                    AllInPause = True
                    rospy.loginfo("Pause !")                
    buttons = data.buttons


#-----------------------------------------------------------------------------




# subscribers
# ------------
rospy.Subscriber("joy", Joy, callBackJoy)



# main node loop
# ---------------

# -----------------------------------------------------------------------------
if __name__ == '__main__':
# -----------------------------------------------------------------------------
    AllInPause = True
    
    rospy.loginfo("Pause !")
    
    while not rospy.is_shutdown():
        if (AllInPause):
            boolMsg.data = True
        else:
            boolMsg.data = False
        # msg publication
        pubCmdVelTB1.publish(boolMsg)
        pubCmdVelTB2.publish(boolMsg)
        pubCmdVelTB3.publish(boolMsg)
        pubCmdVelTB4.publish(boolMsg)
        pubCmdVelTB5.publish(boolMsg)
        pubCmdVelTB6.publish(boolMsg)
        pubCmdVelTB7.publish(boolMsg)
        pubCmdVelTB8.publish(boolMsg)
        
        pausePubRate.sleep()
# -----------------------------------------------------------------------------
