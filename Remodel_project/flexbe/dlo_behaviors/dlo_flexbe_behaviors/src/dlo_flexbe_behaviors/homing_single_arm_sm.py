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
from ur_move_flexbe_states.Current_joint_pose_states import CurrentJointPose
from ur_move_flexbe_states.Homing_poses_states import HomingPose
from ur_move_flexbe_states.TcpPosition_states import TcpPosition
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Thu Jan 25 2024
@author: mk
'''
class Homing_single_armSM(Behavior):
	'''
	go to homing pose
	'''


	def __init__(self):
		super(Homing_single_armSM, self).__init__()
		self.name = 'Homing_single_arm'

		# parameters of this behavior
		self.add_parameter('priority', 0)
		self.add_parameter('period', 10)

		# references to used behaviors
		self.add_behavior(linear_interpolationSM, 'linear_interpolation')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:589 y:418, x:130 y:365
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['robot', 'robot_type'], output_keys=['joint_trajectory', 'last_joints_out', 'homing'])
		_state_machine.userdata.robot = ''
		_state_machine.userdata.robot_type = ''
		_state_machine.userdata.joint_trajectory = []
		_state_machine.userdata.last_joints_out = []
		_state_machine.userdata.homing = []

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:92 y:40
			OperatableStateMachine.add('Homing pose',
										HomingPose(),
										transitions={'done': 'Current_joint_state', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'robot': 'robot', 'homing': 'homing'})

			# x:258 y:59
			OperatableStateMachine.add('Current_joint_state',
										CurrentJointPose(),
										transitions={'done': 'Current tcp pose'},
										autonomy={'done': Autonomy.Off},
										remapping={'robot_type': 'robot_type', 'joint_ur': 'joint_ur', 'joint_listlist': 'joint_listlist'})

			# x:533 y:271
			OperatableStateMachine.add('linear_interpolation',
										self.use_behavior(linear_interpolationSM, 'linear_interpolation',
											parameters={'priority': self.priority, 'period': self.period}),
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'UR': 'robot', 'ur_type': 'robot_type', 'start_pose': 'tcp_position', 'goal_pose': 'homing', 'last_joints_in': 'joint_ur', 'joint_trajectory': 'joint_trajectory', 'last_joints_out': 'last_joints_out'})

			# x:433 y:61
			OperatableStateMachine.add('Current tcp pose',
										TcpPosition(),
										transitions={'done': 'linear_interpolation'},
										autonomy={'done': Autonomy.Off},
										remapping={'robot_type': 'robot_type', 'tcp_position': 'tcp_position'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
