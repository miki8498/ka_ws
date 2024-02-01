#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from ur_move_flexbe_behaviors.linear_interpolation_sm import linear_interpolationSM
from ur_move_flexbe_states.Check_double_collision_service import CheckDoubleCollision
from ur_move_flexbe_states.Current_joint_pose_states import CurrentJointPose
from ur_move_flexbe_states.Gripper_motion_pub import GripperMotion
from ur_move_flexbe_states.Joint_move_single_arm_states import JointSingleArmMove
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Thu Jan 25 2024
@author: mk
'''
class Pick_and_place_armSM(Behavior):
	'''
	pick and place movements coputation
	'''


	def __init__(self):
		super(Pick_and_place_armSM, self).__init__()
		self.name = 'Pick_and_place_arm'

		# parameters of this behavior
		self.add_parameter('priority', 0)
		self.add_parameter('simulation', False)

		# references to used behaviors
		self.add_behavior(linear_interpolationSM, 'Pick')
		self.add_behavior(linear_interpolationSM, 'Place')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:761 y:626, x:297 y:491
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['UR', 'ur_type', 'start_pose', 'pick_pose', 'place_pose', 'last_joints_in', 'other_type'], output_keys=['last_joints_place'])
		_state_machine.userdata.UR = ''
		_state_machine.userdata.ur_type = ''
		_state_machine.userdata.start_pose = []
		_state_machine.userdata.pick_pose = []
		_state_machine.userdata.place_pose = []
		_state_machine.userdata.last_joints_in = []
		_state_machine.userdata.last_joints_place = []
		_state_machine.userdata.other_type = ''

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]

		# x:230 y:234, x:520 y:139, x:230 y:365, x:80 y:183, x:430 y:365
		_sm_check_collisions_0 = ConcurrencyContainer(outcomes=['finished', 'failed'], input_keys=['robot', 'joint_trajectory_pick', 'joint_trajectory_place', 'other_joint_listlist'], conditions=[
										('finished', [('Check collision pick', 'done'), ('Check collision place', 'done')]),
										('failed', [('Check collision pick', 'failed')]),
										('failed', [('Check collision place', 'failed')])
										])

		with _sm_check_collisions_0:
			# x:105 y:59
			OperatableStateMachine.add('Check collision pick',
										CheckDoubleCollision(open=True),
										transitions={'done': 'finished', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'robot': 'robot', 'trajectory_robot': 'joint_trajectory_pick', 'trajectory_other': 'other_joint_listlist', 'success': 'success'})

			# x:276 y:61
			OperatableStateMachine.add('Check collision place',
										CheckDoubleCollision(open=False),
										transitions={'done': 'finished', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'robot': 'robot', 'trajectory_robot': 'joint_trajectory_place', 'trajectory_other': 'other_joint_listlist', 'success': 'success'})



		with _state_machine:
			# x:58 y:58
			OperatableStateMachine.add('Current joint other',
										CurrentJointPose(),
										transitions={'done': 'Pick'},
										autonomy={'done': Autonomy.Off},
										remapping={'robot_type': 'other_type', 'joint_ur': 'other_joint', 'joint_listlist': 'other_joint_listlist'})

			# x:604 y:310
			OperatableStateMachine.add('Gripper close',
										GripperMotion(open=False, simulation=self.simulation),
										transitions={'done': 'Move place'},
										autonomy={'done': Autonomy.Off},
										remapping={'id_type': 'ur_type'})

			# x:710 y:504
			OperatableStateMachine.add('Gripper open',
										GripperMotion(open=True, simulation=self.simulation),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off},
										remapping={'id_type': 'ur_type'})

			# x:603 y:219
			OperatableStateMachine.add('Move pick',
										JointSingleArmMove(),
										transitions={'done': 'Gripper close'},
										autonomy={'done': Autonomy.Off},
										remapping={'robot_type': 'ur_type', 'trajectory': 'joint_trajectory_pick'})

			# x:670 y:415
			OperatableStateMachine.add('Move place',
										JointSingleArmMove(),
										transitions={'done': 'Gripper open'},
										autonomy={'done': Autonomy.Off},
										remapping={'robot_type': 'ur_type', 'trajectory': 'joint_trajectory_place'})

			# x:222 y:51
			OperatableStateMachine.add('Pick',
										self.use_behavior(linear_interpolationSM, 'Pick',
											parameters={'priority': self.priority}),
										transitions={'finished': 'Place', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'UR': 'UR', 'ur_type': 'ur_type', 'start_pose': 'start_pose', 'goal_pose': 'pick_pose', 'last_joints_in': 'last_joints_in', 'joint_trajectory': 'joint_trajectory_pick', 'last_joints_out': 'last_joints_pick'})

			# x:434 y:46
			OperatableStateMachine.add('Place',
										self.use_behavior(linear_interpolationSM, 'Place',
											parameters={'priority': self.priority}),
										transitions={'finished': 'Check collisions', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'UR': 'UR', 'ur_type': 'ur_type', 'start_pose': 'pick_pose', 'goal_pose': 'place_pose', 'last_joints_in': 'last_joints_pick', 'joint_trajectory': 'joint_trajectory_place', 'last_joints_out': 'last_joints_place'})

			# x:582 y:125
			OperatableStateMachine.add('Check collisions',
										_sm_check_collisions_0,
										transitions={'finished': 'Move pick', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'robot': 'UR', 'joint_trajectory_pick': 'joint_trajectory_pick', 'joint_trajectory_place': 'joint_trajectory_place', 'other_joint_listlist': 'other_joint_listlist'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
