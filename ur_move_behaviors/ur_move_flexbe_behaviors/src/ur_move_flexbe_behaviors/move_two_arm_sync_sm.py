#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from dlo_flexbe_behaviors.from_pick_to_place_sm import From_pick_to_placeSM
from dlo_flexbe_behaviors.move_sm import MoveSM
from dlo_flexbe_behaviors.pick__sm import PickSM
from dlo_flexbe_behaviors.release_sm import ReleaseSM
from ur_move_flexbe_states.Joint_move_single_arm_states import JointSingleArmMove
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Tue Jan 30 2024
@author: mk
'''
class Move_two_arm_syncSM(Behavior):
	'''
	rrg
	'''


	def __init__(self):
		super(Move_two_arm_syncSM, self).__init__()
		self.name = 'Move_two_arm_sync'

		# parameters of this behavior
		self.add_parameter('z_over_pick', 0.01)
		self.add_parameter('priority', 0)
		self.add_parameter('period_pose_high', 10)
		self.add_parameter('period_to_pick', 10)
		self.add_parameter('z_to_move', 0.01)
		self.add_parameter('period_move_after_pick', 10)
		self.add_parameter('period_to_place', 10)
		self.add_parameter('gripper_open', False)
		self.add_parameter('gripper_close', False)
		self.add_parameter('simulation', False)
		self.add_parameter('z_over_place', 0.01)
		self.add_parameter('period_over_place', 10)

		# references to used behaviors
		self.add_behavior(From_pick_to_placeSM, 'From_pick_to_place')
		self.add_behavior(MoveSM, 'Move from pick to place')
		self.add_behavior(MoveSM, 'Move to pick')
		self.add_behavior(PickSM, 'Pick ')
		self.add_behavior(PickSM, 'Pick _2')
		self.add_behavior(ReleaseSM, 'Release')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:1054 y:426, x:466 y:322
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['robot_1', 'robot_type_1', 'last_joints_in_1', 'start_pose_1', 'robot_type_2', 'start_pose_2', 'place_pose_2', 'last_joints_in_2', 'pick_pose_2', 'pick_pose_1', 'robot_2', 'place_pose_1'])
		_state_machine.userdata.robot_1 = ''
		_state_machine.userdata.robot_type_1 = ''
		_state_machine.userdata.last_joints_in_1 = []
		_state_machine.userdata.start_pose_1 = []
		_state_machine.userdata.pick_pose_2 = []
		_state_machine.userdata.place_pose_2 = []
		_state_machine.userdata.robot_2 = ''
		_state_machine.userdata.robot_type_2 = ''
		_state_machine.userdata.start_pose_2 = []
		_state_machine.userdata.place_pose_1 = []
		_state_machine.userdata.last_joints_in_2 = []
		_state_machine.userdata.pick_pose_1 = []

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:95 y:107
			OperatableStateMachine.add('Pick ',
										self.use_behavior(PickSM, 'Pick ',
											parameters={'delta_z': self.z_over_pick, 'priority': self.priority, 'period_over_pick': self.period_pose_high, 'period_to_pick': self.period_to_pick}),
										transitions={'finished': 'Pick _2', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'robot': 'robot_1', 'robot_type': 'robot_type_1', 'pick_pose': 'pick_pose_1', 'last_joints_in': 'last_joints_in_1', 'start_pose': 'start_pose_1', 'traj_to_pick': 'traj_to_pick_1', 'last_joints_to_pick': 'last_joints_to_pick_1', 'traj_over_pick': 'traj_over_pick_1'})

			# x:1002 y:170
			OperatableStateMachine.add('Move from pick to place',
										self.use_behavior(MoveSM, 'Move from pick to place',
											default_keys=['robot_type','traj_adj','traj_to_pp'],
											parameters={'gripper_config': self.gripper_open, 'simulation': self.simulation}),
										transitions={'finished': 'Move over place', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:1025 y:294
			OperatableStateMachine.add('Move over place',
										JointSingleArmMove(),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off},
										remapping={'robot_type': 'robot_type_1', 'trajectory': 'traj_over_place'})

			# x:770 y:85
			OperatableStateMachine.add('Move to pick',
										self.use_behavior(MoveSM, 'Move to pick',
											default_keys=['robot_type','traj_adj','traj_to_pp'],
											parameters={'gripper_config': self.gripper_close, 'simulation': self.simulation}),
										transitions={'finished': 'Move from pick to place', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:92 y:181
			OperatableStateMachine.add('Pick _2',
										self.use_behavior(PickSM, 'Pick _2',
											parameters={'delta_z': self.z_over_pick, 'priority': self.priority, 'period_over_pick': self.period_pose_high, 'period_to_pick': self.period_to_pick}),
										transitions={'finished': 'From_pick_to_place', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'robot': 'robot_2', 'robot_type': 'robot_type_2', 'pick_pose': 'pick_pose_2', 'last_joints_in': 'last_joints_in_2', 'start_pose': 'start_pose_2', 'traj_to_pick': 'traj_to_pick_2', 'last_joints_to_pick': 'last_joints_to_pick_2', 'traj_over_pick': 'traj_over_pick_2'})

			# x:535 y:41
			OperatableStateMachine.add('Release',
										self.use_behavior(ReleaseSM, 'Release',
											default_keys=['robot','robot_type','place_pose','last_joints_in'],
											parameters={'delta_z': self.z_over_place, 'priority': self.priority, 'period_over_place': self.period_over_place}),
										transitions={'finished': 'Move to pick', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'traj_over_place': 'traj_over_place', 'last_joints_over_place': 'last_joints_over_place'})

			# x:350 y:42
			OperatableStateMachine.add('From_pick_to_place',
										self.use_behavior(From_pick_to_placeSM, 'From_pick_to_place',
											default_keys=['robot','robot_type','pick_pose','last_joints_in','place_pose'],
											parameters={'delta_z': self.z_to_move, 'priority': self.priority, 'period_over_pick': self.period_move_after_pick, 'period_to_place': self.period_to_place}),
										transitions={'finished': 'Release', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'last_joints_to_place': 'last_joints_to_place', 'traj_to_place': 'traj_to_place', 'traj_over_table': 'traj_over_table'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
