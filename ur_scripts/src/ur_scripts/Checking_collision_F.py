#! /usr/bin/env python3.8
import rospy
from ur_scripts.Constants_F       import *
from collision.srv import *
from collision.msg import *
import copy

class Checking_Collision_:
    def __init__(self) -> None:
        
        self.checking_collision_servive = rospy.ServiceProxy('collision_check' , CollisionServer)
        self.rate = rospy.Rate(500)

    def callback(self, data):
        self.success = data.data

    def mono_collision(self, id, trajectory,gripper_motion):
        if(id == 'UR_left') : robot_type = 'ur_left'
        elif(id == 'UR_right') : robot_type = 'ur_right'

        #Creation message for collision to publish 
        configuration = InfoData()
        req_ck = CollisionServerRequest()
        req_ck.n_robots = 1
        req_ck.ur_type  = robot_type
        req_ck.gripper_config = gripper_motion
        config = []
       
        for i in trajectory:
            config = []
            for j in i:                
                config.append(j)
            configuration.data = config
            req_ck.joint_config.append(copy.deepcopy(configuration))
        
        ck_res = self.checking_collision_servive(req_ck)
        success = ck_res.success  

        return success

    def double_collision(self, robot, prime_traj, second_traj,gripper_motion):
        
        config_right  = [] 
        config_left = []
        
        if(len(prime_traj) < len(second_traj)):
            for i in range(len(second_traj) - len(prime_traj)):
                prime_traj.append(prime_traj[-1])
        elif(len(prime_traj) > len(second_traj)):
            for i in range(len(prime_traj) - len(second_traj)):
                second_traj.append(second_traj[-1])
                
        #Creation message for collision to publish 
        configuration_right  = InfoData()
        configuration_left = InfoData()
        req_ck = CollisionServerRequest()
        req_ck.n_robots = 2     
        req_ck.gripper_config = gripper_motion
       
        for i,z in zip(prime_traj,second_traj):
            config_right = []
            config_left = []
            for j,c in zip(i,z): 
                if(robot == 'UR_right'):        
                    config_right.append(j)
                    config_left.append(c)
                else:
                    config_right.append(c)
                    config_left.append(j)
            configuration_right.data  = config_right
            configuration_left.data = config_left
            req_ck.joint_config_right.append(copy.deepcopy(configuration_right))
            req_ck.joint_config_left.append(copy.deepcopy(configuration_left))
        
        import time
        start = time.time() 
        ck_res = self.checking_collision_servive(req_ck)
        stop = time.time()
        print("TEMPO IMPIEGATO PER IL SERVIZIO: ", stop - start)
        
        success = ck_res.success  

        return success