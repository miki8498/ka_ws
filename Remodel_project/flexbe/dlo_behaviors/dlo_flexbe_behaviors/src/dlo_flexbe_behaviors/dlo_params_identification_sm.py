#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from dlo_flexbe_behaviors.frankx_pick_place_sm import frankx_pick_placeSM
from dlo_flexbe_behaviors.initialize_robot_frankx_sm import initialize_robot_frankxSM
from dlo_flexbe_states.frankx_homing import FrankxMoveHome
from dlo_flexbe_states.get_bending_move_state import ComputeBendingMove
from dlo_flexbe_states.params_identification_save_pick_place_move import ParamsIdentSave
from dlo_flexbe_states.set_dlo_state_from_vision_state import SetDloStateFromVisionS
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Mon Jul 24 2023
@author: xxx
'''
class dlo_params_identificationSM(Behavior):
	'''
	xxx
	'''


	def __init__(self):
		super(dlo_params_identificationSM, self).__init__()
		self.name = 'dlo_params_identification'

		# parameters of this behavior

		# references to used behaviors
		self.add_behavior(frankx_pick_placeSM, 'frankx_pick_place')
		self.add_behavior(initialize_robot_frankxSM, 'initialize_robot_frankx')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:303 y:593, x:600 y:410
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.ee_frame = "panda_1_hand_tcp"
		_state_machine.userdata.world_frame = "world"
		_state_machine.userdata.gripper_width_open = 0.03
		_state_machine.userdata.gripper_width_close = 0.01
		_state_machine.userdata.gripper_force = 1.0
		_state_machine.userdata.max_range = 0.1
		_state_machine.userdata.min_range = 0.05
		_state_machine.userdata.dtheta_range = 0.0

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:52 y:141
			OperatableStateMachine.add('initialize_robot_frankx',
										self.use_behavior(initialize_robot_frankxSM, 'initialize_robot_frankx'),
										transitions={'finished': 'retrieve_dlo_state_from_camera', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'ee_frame': 'ee_frame', 'world_frame': 'world_frame'})

			# x:1020 y:216
			OperatableStateMachine.add('frankx_pick_place',
										self.use_behavior(frankx_pick_placeSM, 'frankx_pick_place'),
										transitions={'finished': 'homing2', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'pick_pose': 'pick_pose', 'place_pose': 'place_pose'})

			# x:863 y:574
			OperatableStateMachine.add('get_new_dlo',
										SetDloStateFromVisionS(srv_name="/dlo_state_planner/set_dlo_state_from_vision"),
										transitions={'done': 'saving', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:1063 y:457
			OperatableStateMachine.add('homing2',
										FrankxMoveHome(srv_name="/panda_1/frankx_ros/homing"),
										transitions={'done': 'get_new_dlo', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:399 y:82
			OperatableStateMachine.add('retrieve_dlo_state_from_camera',
										SetDloStateFromVisionS(srv_name="/dlo_state_planner/set_dlo_state_from_vision"),
										transitions={'done': 'bending_move', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:545 y:558
			OperatableStateMachine.add('saving',
										ParamsIdentSave(srv_name="/dlo_state_planner/get_all_dlo_states"),
										transitions={'done': 'finished', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pick_pose': 'pick_pose', 'place_pose': 'place_pose', 'idx': 'idx', 'dx': 'dx', 'dy': 'dy', 'dtheta': 'dtheta', 'identifier': 'identifier'})

			# x:792 y:46
			OperatableStateMachine.add('bending_move',
										ComputeBendingMove(srv_name="dlo_state_planner/params_random_bending_move"),
										transitions={'done': 'frankx_pick_place', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'max_range': 'max_range', 'min_range': 'min_range', 'dtheta_range': 'dtheta_range', 'pick_pose': 'pick_pose', 'place_pose': 'place_pose', 'idx': 'idx', 'dx': 'dx', 'dy': 'dy', 'dtheta': 'dtheta', 'identifier': 'identifier'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
