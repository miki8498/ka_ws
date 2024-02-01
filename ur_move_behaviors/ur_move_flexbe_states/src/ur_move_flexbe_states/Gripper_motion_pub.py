from flexbe_core import EventState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from ur_scripts.Constants_F import *
from robotiq_control.msg import *
import rospy 
import actionlib


class GripperMotion(EventState):

    def __init__(self, open=False, simulation=False):
        super(GripperMotion, self).__init__(outcomes = ['done'],input_keys = ['id_type'])
        self.hand_ur_right = ['ur_right_bl_to_leftFinger', 'ur_right_leftFinger_to_rightFinger']
        self.hand_ur_left  = ['ur_left_bl_to_leftFinger', 'ur_left_leftFinger_to_rightFinger']
        self.open          = open
        self.simulation    = simulation
    
    def execute(self, userdata):
        rospy.sleep(0.5)

        if(self.open):
            if(self.simulation):
                self.simu_move(open)
            else:
                self.real_move(open_real)
        else:
            if(self.simulation):
                self.simu_move(close)
            else:
                self.real_move(close_real)
       
        return 'done'   

    def on_enter(self, userdata):
      
        self.id_type = userdata.id_type
        if(self.id_type == type_left):
            self.gripper_joints = self.hand_ur_left
        elif(self.id_type == type_right):
            self.gripper_joints = self.hand_ur_right
        
        if(self.simulation):
            self.hand_pub = rospy.Publisher("{}/hand_controller/command".format(self.id_type),  JointTrajectory, queue_size=0)
        else:
            self.gripper_control = actionlib.SimpleActionClient('{}/robotiq_hand_e'.format(self.id_type), CommandRobotiqGripperAction)
            self.gripper_control.wait_for_server()


    def simu_move(self, pose):

        gripper_command  = JointTrajectory()
        joint_traj_point = JointTrajectoryPoint()

        joint_traj_point.positions            = pose
        joint_traj_point.velocities           = [0,0]
        joint_traj_point.accelerations        = [0,0]
        joint_traj_point.time_from_start.secs = 1
        gripper_command.points.append(joint_traj_point) 
        gripper_command.joint_names = self.gripper_joints

        self.hand_pub.publish(gripper_command)

    def real_move(self, pose):
        gripper_command          = CommandRobotiqGripperGoal()
        gripper_command.position = pose
        gripper_command.speed    = 10
        gripper_command.force    = 0

        self.gripper_control.send_goal(gripper_command)