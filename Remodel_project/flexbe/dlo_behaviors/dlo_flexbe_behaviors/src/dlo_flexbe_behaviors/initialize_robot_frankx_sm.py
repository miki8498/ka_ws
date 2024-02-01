#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from dlo_flexbe_states.frankx_gripper import FrankxGripper
from dlo_flexbe_states.frankx_homing import FrankxMoveHome
from dlo_flexbe_states.frankx_set_frames import FrankxSetFrames
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Thu Jul 27 2023
@author: xxx
'''
class initialize_robot_frankxSM(Behavior):
	'''
	xxx
	'''


	def __init__(self):
		super(initialize_robot_frankxSM, self).__init__()
		self.name = 'initialize_robot_frankx'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:745 y:73, x:104 y:251
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['ee_frame', 'world_frame'])
		_state_machine.userdata.gripper_open_width = 0.02
		_state_machine.userdata.gripper_force = 10
		_state_machine.userdata.ee_frame = ''
		_state_machine.userdata.world_frame = ''

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:30 y:40
			OperatableStateMachine.add('open_gripper',
										FrankxGripper(topic="/panda_1/franka_gripper/gripper_action"),
										transitions={'done': 'robot_homing', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'width': 'gripper_open_width', 'force': 'gripper_force'})

			# x:246 y:51
			OperatableStateMachine.add('robot_homing',
										FrankxMoveHome(srv_name="/panda_1/frankx_ros/homing"),
										transitions={'done': 'set_frames', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:493 y:64
			OperatableStateMachine.add('set_frames',
										FrankxSetFrames(srv_ee="/panda_1/frankx_ros/set_end_effector_frame", srv_world="/panda_1/frankx_ros/set_world_frame"),
										transitions={'done': 'finished', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'ee_frame': 'ee_frame', 'world_frame': 'world_frame'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
