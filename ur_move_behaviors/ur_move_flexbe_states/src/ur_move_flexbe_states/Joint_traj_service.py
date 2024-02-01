#! /usr/bin/env python3

from flexbe_core                import EventState
from ur_scripts.Constants_F      import *
from ur_kinematics_robot.srv    import *
from geometry_msgs.msg          import Pose
from tqdm                       import tqdm
import numpy as np
import copy
import rospy 
import PyKDL as kdl
import tf

UR5e = "UR5e"


class JointTrajService(EventState):
    
    '''
        Goal:
            obtain through a service call the joint trajectory of the robot
        Params:
            check_q6: if True, the robot will check if the joint q6 is in the range [-pi/2,pi/2] (bool), if not it corrects it
        '''

    def __init__(self,  check_q6=True):
        super(JointTrajService, self).__init__(outcomes = ['done', 'failed'], input_keys = ['robot_type','trajectory', 'last_joints_traj'], output_keys = ['success', 'max_error', 'ik_solution'])

        self.check_q6       = check_q6
        self.desired_config = [0,2,5,7]
        self._srv           = rospy.ServiceProxy('ur_inverse_k', UrInverseKinematics)


    def execute(self, userdata):
        if self.res is not None:
            
            self.success   =  [self.res.success[self.desired_config[0]], self.res.success[self.desired_config[1]], self.res.success[self.desired_config[2]], self.res.success[self.desired_config[3]]]
            self.max_error =  [self.res.max_error[self.desired_config[0]], self.res.max_error[self.desired_config[1]],self.res.max_error[self.desired_config[2]], self.res.max_error[self.desired_config[3]]]
            
            for i in range(len(self.trajectory)):
                for j in range(6):
                    for z in range(4):
                        self.ik_solution[i,z,j] = self.res.solution[i].joint_matrix[z].data[j]

            return 'done'
        return 'failed'

    def on_enter(self, userdata):
        
        '''
            Input:
                trajectory: cartesian trajectory to convert (list(list))
                type_: name of the robot (string)
                last_joints_traj: last joints from which the robot will start his movement (list)
            '''
        self.listener         = tf.TransformListener()
        self.trajectory       = userdata.trajectory
        self.robot_type       = userdata.robot_type
        self.last_joints_traj = userdata.last_joints_traj  
        print('robot_type: ', self.robot_type)
        if(self.robot_type == type_right):  
            self.robot   = UR5e
        elif(self.robot_type == type_left):
            self.robot   = UR5e

        if(type(self.trajectory[0]) == float):
            self.trajectory = [self.trajectory]    
        
        req_ik    = UrInverseKinematicsRequest()
      
        poses             = [Pose() for i in range(len(self.trajectory))]
        self.ik_solution  = np.zeros((len(poses),4,6))
        tbase_world = self.tf_to_list(origin_frame = 'world', dest_frame = '{}_base'.format(self.robot_type))
        tcp_end = self.tf_to_list(origin_frame = '{}_flange'.format(self.robot_type), dest_frame = '{}_finger_tip'.format(self.robot_type))
   
        
        #Conversion of each cartesian point from tcp_world to ee_base and from list to Pose
        for i in tqdm(range(len(self.trajectory))):
            tcp_world = self.tcp_wrt_world(self.trajectory[i])
            ee_point  = self.whole_kinematics(tbase_world, tcp_world, tcp_end)    
            poses[i]  = ee_point

        req_ik.reference_pose = copy.deepcopy(poses)
        req_ik.desired_config = self.desired_config
        req_ik.ur_type        = self.robot
        req_ik.check_q6       = self.check_q6
        req_ik.verbose        = True
        req_ik.last_joints    = self.last_joints_traj
        
        self.res = self._srv(req_ik)

    def on_exit(self, userdata):
        '''
            Output:
                success: list of booleans that indicates if there's a joints limit exceed (list(bool))
                max_error: indicates the maximum error between the desired and the obtained pose (list(float))
                ik_solution: list of matrices that contains the joints solution for each point of the trajectory (list(list(list)))
            '''
            
        userdata.success     = self.success
        userdata.max_error   = self.max_error
        userdata.ik_solution = self.ik_solution
        print(self.ik_solution[-1])

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
    
    def whole_kinematics(self,tbase_world, tcp_world, tcp_end):

        T = tbase_world.Inverse()*tcp_world*tcp_end.Inverse()

        ee_base = Pose()
        ee_base.position.x = T.p.x()
        ee_base.position.y = T.p.y()
        ee_base.position.z = T.p.z()
        ee_base.orientation.x = T.M.GetQuaternion()[0]
        ee_base.orientation.y = T.M.GetQuaternion()[1]
        ee_base.orientation.z = T.M.GetQuaternion()[2]
        ee_base.orientation.w = T.M.GetQuaternion()[3]

        return ee_base