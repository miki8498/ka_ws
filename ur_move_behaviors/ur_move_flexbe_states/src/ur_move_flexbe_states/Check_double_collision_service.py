from flexbe_core import EventState
from ur_scripts.Checking_collision_F    import Checking_Collision_
from ur_scripts.Constants_F             import *
import copy 

class CheckDoubleCollision(EventState):
    
    '''
        Goal:
            check if there is a collision between the two robots or with the environment
        '''

    def __init__(self, open=False):
        super(CheckDoubleCollision, self).__init__(outcomes = ['done','failed'],input_keys = ['robot', 'trajectory_robot', 'trajectory_other'],output_keys = ['success'])
        self.open = open
        self.collision  = Checking_Collision_()
    
    def execute(self, userdata):

        self.success = self.collision.double_collision(self.robot, self.trajectory_robot, self.trajectory_other, self.gripper_config)

        if(self.success): 
            return 'done' 
        else:
            return 'failed'
    

    def on_enter(self, userdata):
        '''
            Input:
                primary: name of the robot (string)
                trajectory_primary: joint trajectory 1° robot (list(lists)). It is not necessarly the primary trajectory, the important thing is that it concide with the
                trajectory of the robot put as primary in the previous param
                trajectory_follower: joint trajectory 2° robot (list(lists))
                gripper_config: pose of the finger config (float)
            '''
        self.robot             = userdata.robot
        self.trajectory_robot  = copy.deepcopy(userdata.trajectory_robot)
        self.trajectory_other = copy.deepcopy(userdata.trajectory_other)
        
        # Moveit collision needs the gripper config in the form [float, float]
        if(self.open):
            self.gripper_config = open
        else:
            self.gripper_config = close

    def on_exit(self, userdata):
        '''
            Output:
                success: True if there is no collision, False otherwise (bool)
            '''
        userdata.success = self.success