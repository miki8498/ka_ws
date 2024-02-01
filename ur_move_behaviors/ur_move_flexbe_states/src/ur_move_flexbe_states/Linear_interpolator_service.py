#! /usr/bin/env python3

from flexbe_core               import EventState
from ur_scripts.Operation_F    import Operation_
from ur_scripts.Interpolator_F import Interpolator_
from ur_scripts.Constants_F    import *
from ur_kinematics_robot.srv   import *
from geometry_msgs.msg         import Pose
import numpy as np
import copy
import rospy 

UR5e = "UR5e"

class InterpolateActionState(EventState):
    '''
        Goal:
            compute the interpolated trajectory between two poses and convert it in joint 
        Parameters:
            srv_name: name of the service (string)
            check_q6: check the q6 joint and correct it (bool)
            tm_tool: for techman robot (bool)
        '''

    def __init__(self, srv_name = '/ur_inverse_k',  check_q6=True, period=10):
        super(InterpolateActionState ,self).__init__(outcomes = ['done', 'failed'], input_keys = ['UR', 'start_pose', 'ee_pose', 'last_joints'], output_keys = ['success','max_error', 'ik_solution', 'trajectory_interpolated'])
        self.interpolator   = Interpolator_()
        self.operation      = Operation_()
        self._srv_name      = srv_name
        self.check_q6       = check_q6
        self.desired_config = [0,2,5,7]
        self._srv = rospy.ServiceProxy('ur_inverse_k', UrInverseKinematics)
        self.period = period
   
    def execute(self, userdata):

        #Compute the cartesian inteprolated trajectory
        trajectory_interpolation = self.interpolator.interpolate(self.rate, self.start_pose, self.ee_point, self.period)


        #Convert it in list of lists
        self.trajectory_interpolated = [[pose.position.x, pose.position.y, pose.position.z,
                  pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w] for pose in trajectory_interpolation]
        
        #Make it in joint space
        poses             = [Pose() for i in range(len(trajectory_interpolation))]
        self.ik_solution  = np.zeros((len(poses),4,6))
        
        self.req_ik.reference_pose = copy.deepcopy(trajectory_interpolation)
        self.req_ik.desired_config = self.desired_config
        self.req_ik.ur_type        = self.id
        self.req_ik.check_q6       = self.check_q6
        self.req_ik.verbose        = False
        self.req_ik.last_joints    = self.last_joints
                
        self.res = self._srv(self.req_ik)

        if self.res is not None:
            self.success   =  [self.res.success[self.desired_config[0]], self.res.success[self.desired_config[1]], self.res.success[self.desired_config[2]], self.res.success[self.desired_config[3]]]
            self.max_error =  [self.res.max_error[self.desired_config[0]], self.res.max_error[self.desired_config[1]],self.res.max_error[self.desired_config[2]], self.res.max_error[self.desired_config[3]]]
            
            for i in range(len(trajectory_interpolation)):
                for j in range(6):
                    for z in range(4):
                        self.ik_solution[i,z,j] = self.res.solution[i].joint_matrix[z].data[j]

        if(self.success):return 'done'
        else:            return 'failed'

    def on_enter(self, userdata):
        '''
            Input:
                UR: name of the robot type (string)
                ee_pose: goal pose (ee to base) (list)
                last_joints: joints from which the robot starts the movement (list)
            '''
        self.UR          = userdata.UR
        self.ee_point    = userdata.ee_pose
        self.last_joints = userdata.last_joints
        
        if(self.UR == UR_right):  
            self.id   = UR5e
            self.rate = rate_right
        elif(self.UR == UR_left):
            self.id   = UR5e
            self.rate = rate_left
      
        self.start_pose = userdata.start_pose
        self.req_ik     = UrInverseKinematicsRequest()
        
    def on_exit(self, userdata):
        '''
            Output:
                success: list of booleans that indicates if there's a joints limit exceed (list(bool))
                max_error: indicates the maximum error between the desired and the obtained pose (list(float))
                ik_solution: list of matrices that contains the joints solution for each point of the trajectory (list(list(list)))
                trajectory_interpolated: cartesian trajectory (list(lists))
            '''
        
        userdata.success                 = self.success
        userdata.max_error               = self.max_error
        userdata.ik_solution             = self.ik_solution
        userdata.trajectory_interpolated = self.trajectory_interpolated
        

