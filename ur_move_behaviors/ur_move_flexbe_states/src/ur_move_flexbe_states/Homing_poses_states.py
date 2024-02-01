#! /usr/bin/env python3

from flexbe_core              import EventState
from ur_scripts.Constants_F import *

class HomingPose(EventState):
    '''
        Goal:
            define default poses for the primary robot to left the cable and make space to the follower robot
        '''

    def __init__(self):
        super(HomingPose, self).__init__(outcomes = ['done', 'failed'],input_keys = ['robot'], output_keys = ['homing'])
        
 
    def execute(self, userdata):

        if(self.robot == UR_right):
            print("ssssssssss")
            self.homing = [0.186, 0.115, 0.685, 3.080, 0.812, 0.002]     
        elif(self.robot == UR_left):
            self.homing = [-0.217, -0.134, 0.531,-3.133, 0.618, -3.141]
        else:
            return 'failed'

        return 'done'
    
    def on_enter(self,userdata):
        '''
            Input:
            '''

        self.robot = userdata.robot
     
    def on_exit(self, userdata):
        '''
            Output:
            '''
        
        userdata.homing = self.homing

