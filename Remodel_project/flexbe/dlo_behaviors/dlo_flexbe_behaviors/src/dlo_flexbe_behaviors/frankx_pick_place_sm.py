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
from dlo_flexbe_states.frankx_move_state import FrankxMove
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Thu Jul 27 2023
@author: xxx
'''
class frankx_pick_placeSM(Behavior):
	'''
	xxx
	'''


	def __init__(self):
		super(frankx_pick_placeSM, self).__init__()
		self.name = 'frankx_pick_place'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:30 y:365, x:454 y:199
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['pick_pose', 'place_pose'])
		_state_machine.userdata.ee_frame = "panda_1_hand_tcp"
		_state_machine.userdata.world_frame = "world"
		_state_machine.userdata.gripper_open_width = 0.02
		_state_machine.userdata.gripper_close_width = 0.01
		_state_machine.userdata.gripper_force = 10
		_state_machine.userdata.pick_pose = []
		_state_machine.userdata.place_pose = []

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:30 y:40
			OperatableStateMachine.add('gripper_ready',
										FrankxGripper(topic="/panda_1/franka_gripper/gripper_action"),
										transitions={'done': 'approach_move', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'width': 'gripper_open_width', 'force': 'gripper_force'})

			# x:621 y:344
			OperatableStateMachine.add('departure_move',
										FrankxMove(topic="/panda_1/frankx_ros/frankx_trajectory_action", z_offset=0.1, clamp_z=False),
										transitions={'done': 'home_move', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pose': 'place_pose'})

			# x:416 y:44
			OperatableStateMachine.add('grasp_move',
										FrankxMove(topic="/panda_1/frankx_ros/frankx_trajectory_action", z_offset=0.0, clamp_z=True),
										transitions={'done': 'gripper_grasp', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pose': 'pick_pose'})

			# x:663 y:48
			OperatableStateMachine.add('gripper_grasp',
										FrankxGripper(topic="/panda_1/franka_gripper/gripper_action"),
										transitions={'done': 'place_move', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'width': 'gripper_close_width', 'force': 'gripper_force'})

			# x:913 y:259
			OperatableStateMachine.add('gripper_open',
										FrankxGripper(topic="/panda_1/franka_gripper/gripper_action"),
										transitions={'done': 'departure_move', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'width': 'gripper_open_width', 'force': 'gripper_force'})

			# x:341 y:342
			OperatableStateMachine.add('home_move',
										FrankxMoveHome(srv_name="/panda_1/frankx_ros/homing"),
										transitions={'done': 'finished', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:919 y:63
			OperatableStateMachine.add('place_move',
										FrankxMove(topic="/panda_1/frankx_ros/frankx_trajectory_action", z_offset=0.00, clamp_z=True),
										transitions={'done': 'gripper_open', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pose': 'place_pose'})

			# x:214 y:41
			OperatableStateMachine.add('approach_move',
										FrankxMove(topic="/panda_1/frankx_ros/frankx_trajectory_action", z_offset=0.1, clamp_z=False),
										transitions={'done': 'grasp_move', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pose': 'pick_pose'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
