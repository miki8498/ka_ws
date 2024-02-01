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
from ur_move_flexbe_states.Ready_to_pick_states import ReadyToPick
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Tue Jan 30 2024
@author: mk
'''
class From_pick_to_placeSM(Behavior):
	'''
	fe
	'''


	def __init__(self):
		super(From_pick_to_placeSM, self).__init__()
		self.name = 'From_pick_to_place'

		# parameters of this behavior
		self.add_parameter('delta_z', 0.03)
		self.add_parameter('priority', 0)
		self.add_parameter('period_over_pick', 10)
		self.add_parameter('period_to_place', 10)

		# references to used behaviors
		self.add_behavior(linear_interpolationSM, 'Over pick pose')
		self.add_behavior(linear_interpolationSM, 'Place pose')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:706 y:194, x:395 y:212
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['robot', 'robot_type', 'pick_pose', 'last_joints_in', 'place_pose'], output_keys=['last_joints_to_place', 'traj_to_place', 'traj_over_table'])
		_state_machine.userdata.robot = ''
		_state_machine.userdata.robot_type = ''
		_state_machine.userdata.pick_pose = []
		_state_machine.userdata.last_joints_in = []
		_state_machine.userdata.place_pose = []
		_state_machine.userdata.last_joints_to_place = []
		_state_machine.userdata.traj_to_place = []
		_state_machine.userdata.traj_over_table = []

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:60 y:77
			OperatableStateMachine.add('Delta z pose',
										ReadyToPick(delta_z=self.delta_z),
										transitions={'done': 'Over pick pose'},
										autonomy={'done': Autonomy.Off},
										remapping={'pick_pose': 'pick_pose', 'over_pick_pose': 'over_table_pose'})

			# x:221 y:66
			OperatableStateMachine.add('Over pick pose',
										self.use_behavior(linear_interpolationSM, 'Over pick pose',
											parameters={'priority': self.priority, 'period': self.period_over_pick}),
										transitions={'finished': 'Place pose', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'UR': 'robot', 'ur_type': 'robot_type', 'start_pose': 'pick_pose', 'goal_pose': 'over_table_pose', 'last_joints_in': 'last_joints_in', 'joint_trajectory': 'traj_over_table', 'last_joints_out': 'last_joints_over_table'})

			# x:416 y:66
			OperatableStateMachine.add('Place pose',
										self.use_behavior(linear_interpolationSM, 'Place pose',
											parameters={'priority': self.priority, 'period': self.period_to_place}),
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'UR': 'robot', 'ur_type': 'robot_type', 'start_pose': 'over_table_pose', 'goal_pose': 'place_pose', 'last_joints_in': 'last_joints_over_table', 'joint_trajectory': 'traj_to_place', 'last_joints_out': 'last_joints_to_place'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
