#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from ur_move_flexbe_states.Gripper_motion_pub import GripperMotion
from ur_move_flexbe_states.Joint_move_single_arm_states import JointSingleArmMove
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Tue Jan 30 2024
@author: mk
'''
class MoveSM(Behavior):
	'''
	fe
	'''


	def __init__(self):
		super(MoveSM, self).__init__()
		self.name = 'Move'

		# parameters of this behavior
		self.add_parameter('gripper_config', False)
		self.add_parameter('simulation', False)

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:531 y:258, x:89 y:430
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['robot_type', 'traj_adj', 'traj_to_pp'])
		_state_machine.userdata.robot_type = ''
		_state_machine.userdata.traj_adj = []
		_state_machine.userdata.traj_to_pp = []

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:70 y:60
			OperatableStateMachine.add('Move over pick',
										JointSingleArmMove(),
										transitions={'done': 'Move to pick'},
										autonomy={'done': Autonomy.Off},
										remapping={'robot_type': 'robot_type', 'trajectory': 'traj_adj'})

			# x:198 y:150
			OperatableStateMachine.add('Move to pick',
										JointSingleArmMove(),
										transitions={'done': 'Close gripper'},
										autonomy={'done': Autonomy.Off},
										remapping={'robot_type': 'robot_type', 'trajectory': 'traj_to_pp'})

			# x:319 y:242
			OperatableStateMachine.add('Close gripper',
										GripperMotion(open=self.gripper_config, simulation=self.simulation),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off},
										remapping={'id_type': 'robot_type'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
