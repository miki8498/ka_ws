from flexbe_core            import EventState
from ur_scripts.Constants_F import *
from std_msgs.msg      import Float64MultiArray
from tqdm import tqdm
import copy
import rospy

class JointSingleArmMove(EventState):

    def __init__(self):
        super(JointSingleArmMove, self).__init__(outcomes = ['done'],input_keys = ['robot_type','trajectory'])
    
    def execute(self, userdata):

        if self.robot_type == type_left:      
            rate = rate_left
        elif self.robot_type == type_right: 
            rate = rate_right

        self.rate = rospy.Rate(rate)
        for i in tqdm(self.trajectory):
            joint_pose      = Float64MultiArray()
            joint_pose.data = copy.deepcopy(i)

            self.joint_pub.publish(joint_pose)
            self.rate.sleep()
        
        return 'done'   

    def on_enter(self, userdata):
     
        self.robot_type = userdata.robot_type
        self.trajectory = userdata.trajectory

        self.joint_pub  = rospy.Publisher("{}/joint_group_pos_controller/command".format(self.robot_type),  Float64MultiArray,queue_size=1)

  
