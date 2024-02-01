from flexbe_core               import EventState
from ur_scripts.Constants_F    import *
from ur_scripts.Interpolator_F import Interpolator_
from ur_scripts.Operation_F    import *




class CartLinearInterpolator(EventState):
    '''
        Goal:
            computation of the follower trajectory when the pirmary robot pose is fixed and we need to consider the presence of the cable between the two robots during every movement.

        '''

    def __init__(self, period = 0.01):
        super(CartLinearInterpolator, self).__init__(outcomes = ['done'],input_keys = ['robot', 'start_pose', 'goal_pose'], output_keys = ['trajectory'])
        
        self.period     = period
        self.interpolator   = Interpolator_()
        self.operation      = Operation_()
        
    def execute(self, userdata):
        self.trajectory_interpolated = []
        
        trajectory_interpolation = self.interpolator.interpolate(self.rate, self.start_pose, self.goal_pose, self.period)

        for pose in trajectory_interpolation:
            rpy = self.operation.euler_from_quaternion([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
            self.trajectory_interpolated.append([pose.position.x, pose.position.y, pose.position.z,
                    rpy[0], rpy[1], rpy[2]])

        print("trajectory_interpolation[0]: ", self.trajectory_interpolated[0])
        print("trajectory_interpolation[-1]: ", self.trajectory_interpolated[-1])
        return 'done'

    def on_enter(self, userdata):
        '''
            Input:
                fix_primary_point: fixed pose of the primary robot (list)
                start_point_follower: starting pose of the follower robot (list)
                final_point_follower: final pose of the follower robot (list)
                radius: length of the cable (float)
            '''
        self.UR          = userdata.robot
        self.start_pose = userdata.start_pose
        self.goal_pose   = userdata.goal_pose

        if(self.UR == UR_right):  
            self.rate = rate_right
        elif(self.UR == UR_left):
            self.rate = rate_left

    def on_exit(self, userdata):
        '''
            Output:
                follower_trajectory: cartesian follower trajectory (list(list))
                last_follower_point: last point of the follower trajectory (list)
            
            '''
        userdata.trajectory = self.trajectory_interpolated