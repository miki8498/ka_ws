#! /usr/bin/env python3

from flexbe_core                import EventState
from geometry_msgs.msg          import Pose
import PyKDL as kdl
import copy
import tf
import rospy


class TcpW_to_eeBase(EventState):
    
    '''
        Goal:
            obtain the pose ee_to_base from the pose tcp_to_world
        '''

    def __init__(self):
        super(TcpW_to_eeBase, self).__init__(outcomes = ['done'],input_keys = ['robot_type', 'tcpW_pose'], output_keys = ['ee_base'])
        self.listener = tf.TransformListener()
        
    def execute(self, userdata):
        
        tbase_world = self.tf_to_list(origin_frame = 'world', dest_frame = '{}_base'.format(self.robot_type))
        tcp_end = self.tf_to_list(origin_frame = '{}_flange'.format(self.robot_type), dest_frame = '{}_finger_tip'.format(self.robot_type))
        
        if(type(self.pose) == list and type(self.pose[0]) == float):

            tcp_world = self.tcp_wrt_world(self.pose)
            T = tbase_world.Inverse()*tcp_world*tcp_end.Inverse()
            
            self.ee_base = Pose()
            self.ee_base.position.x = T.p.x()
            self.ee_base.position.y = T.p.y()
            self.ee_base.position.z = T.p.z()
            self.ee_base.orientation.x = T.M.GetQuaternion()[0]
            self.ee_base.orientation.y = T.M.GetQuaternion()[1]
            self.ee_base.orientation.z = T.M.GetQuaternion()[2]
            self.ee_base.orientation.w = T.M.GetQuaternion()[3]
        else:
            self.ee_base = []
            for i in range(len(self.pose)):
                tcp_world = self.tcp_wrt_world(self.pose[i])
                T = tbase_world.Inverse()*tcp_world*tcp_end.Inverse()
                self.ee_base.append([T.p.x(), T.p.y(), T.p.z(), T.M.GetRPY()[0], T.M.GetRPY()[1], T.M.GetRPY()[2]])
            print(self.ee_base[-1])
        return 'done'
        
    def on_enter(self,userdata):        
        
        '''
            Input:
                robot_type: name of the robot type (string)
                tcpW_pose: pose tcp_to_world (list)
            '''
            
        self.robot_type = userdata.robot_type
        self.pose       = copy.deepcopy(userdata.tcpW_pose)
        
    def on_exit(self, userdata):
        
        '''
            Output:
                ee_base: pose ee_to_base (list)
            '''
        userdata.ee_base = self.ee_base

    def tcp_wrt_world(self, position):
        #Homogeneous matrix of the TCP wrt the world
        T_tcworld   = kdl.Frame()
        T_tcworld.p = kdl.Vector(position[0],position[1],position[2])
        T_tcworld.M = kdl.Rotation.RPY(position[3],position[4],position[5])
        return T_tcworld
    
    def tf_to_list(self, origin_frame, dest_frame):

        (trans,rot) = self.listener.lookupTransform(origin_frame, dest_frame, rospy.Time(0))

        tf   = kdl.Frame()
        tf.p = kdl.Vector(trans[0],trans[1],trans[2])
        tf.M = kdl.Rotation.Quaternion(rot[0],rot[1],rot[2],rot[3])

        return tf


        
