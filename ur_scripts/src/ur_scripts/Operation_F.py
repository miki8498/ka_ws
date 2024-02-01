#! /usr/bin/env python3
import numpy as np
import math
from geometry_msgs.msg import Quaternion


class Operation_:

    def __init__(self) ->None :
        pass
        
    def get_quaternion_from_euler(self, roll, pitch, yaw):
 
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
       
        return Quaternion(qx, qy, qz, qw)

    def euler_from_quaternion(self, list):
        t0     = +2.0 * (list[3] * list[0] + list[1] * list[2])
        t1     = +1.0 - 2.0 * (list[0] * list[0] + list[1] * list[1])
        roll_x = math.atan2(t0, t1)
        
        t2 = +2.0 * (list[3] * list[1] - list[2] * list[0])
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
        
        t3    = +2.0 * (list[3] * list[2] + list[0] * list[1])
        t4    = +1.0 - 2.0 * (list[1] * list[1] + list[2] * list[2])
        yaw_z = math.atan2(t3, t4)
        
        return [roll_x, pitch_y, yaw_z] # in radians
    
   