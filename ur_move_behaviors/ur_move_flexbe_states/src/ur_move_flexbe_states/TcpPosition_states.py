#! /usr/bin/env python3

from flexbe_core               import EventState
from ur_scripts.Operation_F import Operation_
from ur_scripts.Constants_F  import *
import rospy
import tf


class TcpPosition(EventState):
    
    '''
        Goal:
            obtain through a frame listener the coordinates of the point of the gripper respect to the world frame
        Parameters:
            tm_tool: for techman robot to choose between the two tools (bool)
        '''

    def __init__(self):
        super(TcpPosition, self).__init__(outcomes = ['done'],input_keys= ['robot_type'], output_keys = ['tcp_position'])
        self.operation = Operation_()
        self.listener  = tf.TransformListener()

    def execute(self, userdata):
        rospy.sleep(0.5)

        (trans,rot) = self.listener.lookupTransform('world', '{}_finger_tip'.format(self.robot_type), rospy.Time(0))
           
        euler = self.operation.euler_from_quaternion(rot)
        self.tcp_pose = [trans[0],trans[1],trans[2],euler[0],euler[1],euler[2]]

        return 'done'   
    
    def on_enter(self,userdata):
        '''
            Input:
                robot_type: name of robot (string)
            '''
        self.robot_type = userdata.robot_type

    def on_exit(self, userdata):
        '''
            Output:
                tcp_position: tcp pose respect to world frame (list)
            '''
        userdata.tcp_position = self.tcp_pose
        print("tcp_position", userdata.tcp_position)
