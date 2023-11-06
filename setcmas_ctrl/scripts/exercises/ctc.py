#!/usr/bin/env python3
'''

   Sylvain BERTRAND, 2023
   (all variables in SI unit)
   
   Communication Triggering Condition (CTC)

   All the following arrays are numpy arrays (1 row x 3 columns)
   pos : 3D position of  robot (m)
   vit : 3D velocity of robot (m/s)
   last_pos : last transmitted 3D position of  robot (m)
   last_ : last transmitted 3D velocity of robot (m/s)

   
'''

import numpy as np
import math



# ==============   "GLOBAL" VARIABLES KNOWN BY ALL THE FUNCTIONS ===================
# use keyword "global" inside a function if the variable needs to be modified by the function


# ===================================================================================


# -- do not modify --
global t0, firstCall
t0 = 0.0
firstCall = True
# -------------------





# =======================================
def ctc(pos, vit, last_pos, last_vit, tt):
# =======================================

    
    
    # -- do not modify --
    global t0, firstCall   

    # get time and remove offset
    if (firstCall):
        t0 = tt
        firstCall = False
        print("***** CTC script from the Exercises folder *****")
    
    t = tt - t0
    # --------------------
    
    

    
    # TO BE COMPLETED
    
    '''
    if ( *** MY_CONDITION *** ):
        return True
    else:
        return False
    '''
    
    return True

# ====================================   

