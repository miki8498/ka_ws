#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from dlo_flexbe_behaviors.homing_single_arm_sm import Homing_single_armSM
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
class initialize_urSM(Behavior):
	'''
	go to homing pose double robot
	'''


	def __init__(self):
		super(initialize_urSM, self).__init__()
		self.name = 'initialize_ur'

		# parameters of this behavior
		self.add_parameter('priority', 0)
		self.add_parameter('gripper_config', False)
		self.add_parameter('simulation', False)
		self.add_parameter('period', 10)

		# references to used behaviors
		self.add_behavior(Homing_single_armSM, 'Homing_single_left')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:602 y:363, x:130 y:365
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['robot', 'type', 'other'], output_keys=['last_joints', 'homing'])
		_state_machine.userdata.robot = ''
		_state_machine.userdata.type = ''
		_state_machine.userdata.last_joints = []
		_state_machine.userdata.homing = []
		_state_machine.userdata.other = ''

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:59 y:41
			OperatableStateMachine.add('Gripper open',
										GripperMotion(open=self.gripper_config, simulation=self.simulation),
										transitions={'done': 'Current joint other'},
										autonomy={'done': Autonomy.Off},
										remapping={'id_type': 'type'})

			# x:219 y:33
			OperatableStateMachine.add('Current joint other',
										CurrentJointPose(),
										transitions={'done': 'Homing_single_left'},
										autonomy={'done': Autonomy.Off},
										remapping={'robot_type': 'other', 'joint_ur': 'other_joint', 'joint_listlist': 'other_joint_listlist'})

			# x:439 y:49
			OperatableStateMachine.add('Homing_single_left',
										self.use_behavior(Homing_single_armSM, 'Homing_single_left',
											parameters={'priority': self.priority, 'period': self.period}),
										transitions={'finished': 'Check double collision', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'robot': 'robot', 'robot_type': 'type', 'joint_trajectory': 'joint_trajectory', 'last_joints_out': 'last_joints', 'homing': 'homing'})

			# x:508 y:262
			OperatableStateMachine.add('Move',
										JointSingleArmMove(),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off},
										remapping={'robot_type': 'type', 'trajectory': 'joint_trajectory'})

			# x:474 y:137
			OperatableStateMachine.add('Check double collision',
										CheckDoubleCollision(open=False),
										transitions={'done': 'Move', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'robot': 'robot', 'trajectory_robot': 'joint_trajectory', 'trajectory_other': 'other_joint_listlist', 'success': 'success'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
