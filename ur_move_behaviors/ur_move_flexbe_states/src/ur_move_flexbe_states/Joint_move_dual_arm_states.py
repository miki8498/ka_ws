from flexbe_core                        import EventState
from ur_scripts.Constants_F           import *
from std_msgs.msg      import Float64MultiArray
from tqdm import tqdm
import threading
import copy
import rospy


class JointDualArmMove(EventState):

    def __init__(self):
        super(JointDualArmMove, self).__init__(outcomes = ['done'],input_keys = ['robot_type', 'robot_trajectory', 'other_trajectory'])
    
    def execute(self, userdata):
    
        robot_trajectory  = copy.deepcopy(self.robot_trajectory)
        other_trajectory = copy.deepcopy(self.other_trajectory)


        if(self.robot_type == type_left): 
            t1 = threading.Thread(target = self.join_move,   args = [500, type_left, robot_trajectory])
            t2 = threading.Thread(target = self.join_move,  args = [500, type_right, other_trajectory])
            t1.start()
            t2.start() 
            t1.join()
            t2.join() 
        elif(self.robot_type == type_right):
            
            t1 = threading.Thread(target = self.joint_move,   args = [500,type_right, robot_trajectory])
            t2 = threading.Thread(target = self.joint_move,  args = [500,type_left, other_trajectory])
            t1.start()
            t2.start() 
            t1.join()
            t2.join() 
     
        return 'done'   

    def on_enter(self, userdata):
        self.robot_type       = userdata.robot_type
        self.robot_trajectory = userdata.robot_trajectory
        self.other_trajectory = userdata.other_trajectory

    def joint_move(self, rate, robot_type, trajectory):
        joint_pub  = rospy.Publisher("{}/joint_group_pos_controller/command".format(robot_type),  Float64MultiArray,queue_size=1)

        self.rate = rospy.Rate(rate)
        for i in tqdm(trajectory):
            joint_pose      = Float64MultiArray()
            joint_pose.data = copy.deepcopy(i)

            joint_pub.publish(joint_pose)
            self.rate.sleep()
        

        
