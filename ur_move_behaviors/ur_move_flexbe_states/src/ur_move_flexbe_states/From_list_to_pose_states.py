from flexbe_core import EventState
from geometry_msgs.msg import Pose
from ur_scripts.Operation_F import Operation_

class FromListToPose(EventState):
    
    '''
        Goal:
            state to convert a list of poses in a geometry_msgs.msg/Pose type.
        '''

    def __init__(self):
        super(FromListToPose, self).__init__(outcomes = ['done'],input_keys = ['list_pose'], output_keys =['Pose_pose'])

        self.operations = Operation_()
        
    def execute(self, userdata):
     
        self.Pose_pose = Pose()
        self.Pose_pose.position.x = self.list_pose[0]
        self.Pose_pose.position.y = self.list_pose[1]
        self.Pose_pose.position.z = self.list_pose[2]
        orient = self.operations.get_quaternion_from_euler(self.list_pose[3],self.list_pose[4],self.list_pose[5]) 
        self.Pose_pose.orientation.x = orient.x
        self.Pose_pose.orientation.y = orient.y
        self.Pose_pose.orientation.z = orient.z
        self.Pose_pose.orientation.w = orient.w

        return 'done'   

    def on_enter(self, userdata):
        '''
            Input:
                list_pose: list pose (list[x,y,z,roll,pitch,yaw])
            '''
      
        self.list_pose = userdata.list_pose
        print("list_pose: ", self.list_pose)

    def on_exit(self, userdata):
        '''
            Output:
                Pose_pose: geometry_msgs.msg/Pose pose 
            '''
        userdata.Pose_pose = self.Pose_pose
        print("Pose_pose: ", self.Pose_pose)
  
