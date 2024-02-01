from flexbe_core import EventState

class FinalJointTrajectory(EventState):
    
    '''
        Goal:
            creation of the final joint trajectory considering the configuration chosen for the robot
        '''

    def __init__(self,):
        super(FinalJointTrajectory, self).__init__(outcomes = ['done','failed'],input_keys = ['ik_solution', 'kin_choice'],output_keys = ['Final_trajectory', 'last_joints'])
        

    def execute(self, userdata):

        self.Final_traj = []
        for i in  self.ik_solution:
            column =  i[self.kin_choice ,:]
            self.Final_traj.append(column)

        if(len(self.Final_traj) == 0):
            return 'failed'
        return 'done'   

    def on_enter(self, userdata):
        '''
            Input:
                ik_solution: list of matrices that contains the joints solution for each point of the trajectory (list(list(list)))
                ik_choice: configuration chosen for the trajectory (int)
            '''
        self.ik_solution = userdata.ik_solution
        self.kin_choice  = userdata.kin_choice

    def on_exit(self, userdata):
        '''
            Output:
                Final_trajectory: joint trajectory (list(list))
                last_joints: last joints pose of the trajectory (list)
            '''
        userdata.Final_trajectory = self.Final_traj
        userdata.last_joints      = self.Final_traj[-1]

       
