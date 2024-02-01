#! /usr/bin/env python3

from flexbe_core              import EventState
from ur_scripts.Constants_F import *

class ReadyToPick(EventState):
    '''
        Goal:
            define default poses for the primary robot to left the cable and make space to the follower robot
        '''

    def __init__(self, delta_z = 0.1):
        super(ReadyToPick, self).__init__(outcomes = ['done'],input_keys = ['pick_pose'], output_keys = ['over_pick_pose'])
        self.delta_z = delta_z
 
    def execute(self, userdata):

        self.over_pick_pose = [self.pick_pose[0], self.pick_pose[1], self.pick_pose[2] + self.delta_z, self.pick_pose[3], self.pick_pose[4], self.pick_pose[5]]
        return 'done'
    
    def on_enter(self,userdata):
        '''
            Input:
            '''

        self.pick_pose = userdata.pick_pose
        print("PICK POSE ", self.pick_pose)
     
    def on_exit(self, userdata):
        '''
            Output:
            '''
        
        userdata.over_pick_pose = self.over_pick_pose
        print("over PICK POSE ", self.over_pick_pose)

