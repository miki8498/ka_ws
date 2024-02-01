#! /usr/bin/env python3

from flexbe_core              import EventState
from flexbe_core.proxy        import ProxySubscriberCached
from ur_scripts.Constants_F import *
from sensor_msgs.msg          import JointState
import numpy as np  
import copy

class CurrentJointPose(EventState):
    
    '''
        Goal:
            have the current joint pose of the robot
        '''

    def __init__(self,topic = '/joint_states') :
        super(CurrentJointPose, self).__init__(outcomes = ['done'], input_keys = ['robot_type'], output_keys = ['joint_ur', 'joint_listlist'])
        self._topic    = topic
        self._sub      =  ProxySubscriberCached({self._topic: JointState})
 
    def execute(self, userdata):
        msg = self._sub.get_last_msg(self._topic)
        self.sub_cb(msg, self.robot_type)

        return 'done'

    def on_enter(self,userdata):
        
        '''
            Input:
                robot_type: robot type (string)
            '''
            
        self.robot_type = userdata.robot_type

        
    def on_exit(self, userdata):
        
        '''
            Output:
                pose_ur: current joint pose of the robot (list(list))
                joint_ur: current joint pose of the robot (list)
            '''
              
        userdata.joint_listlist = [self.joint_position]
        userdata.joint_ur = self.joint_position
        print("joint_ur", userdata.joint_ur)
       

    def sub_cb(self, msg, ur_type):

        indice = np.zeros(6)
        print(ur_type)
    
        # salvo gli indici in una lista
        indice[0] = msg.name.index('{}_shoulder_pan_joint'.format(ur_type))
        indice[1] = msg.name.index('{}_shoulder_lift_joint'.format(ur_type))
        indice[2] = msg.name.index('{}_elbow_joint'.format(ur_type))
        indice[3] = msg.name.index('{}_wrist_1_joint'.format(ur_type))
        indice[4] = msg.name.index('{}_wrist_2_joint'.format(ur_type))
        indice[5] = msg.name.index('{}_wrist_3_joint'.format(ur_type))
        
        # salvo le posizioni in una lista
        self.joint_position = [copy.deepcopy(msg.position[int(indice[0])]), copy.deepcopy(msg.position[int(indice[1])]), copy.deepcopy(msg.position[int(indice[2])]), 
                copy.deepcopy(msg.position[int(indice[3])]), copy.deepcopy(msg.position[int(indice[4])]), copy.deepcopy(msg.position[int(indice[5])])]
