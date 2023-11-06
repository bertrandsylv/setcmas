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


# =======================================
def ctc(pos, vit, last_pos, last_vit, t):
# =======================================
    
    if (np.linalg.norm(pos-last_pos)>0.25):
        return True
    else:
        return False

# ====================================   

