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
from dlo_flexbe_states.get_best_move_params_state import ComputeBestMoveParams
from dlo_flexbe_states.params_identification_save_pick_place_move import ParamsIdentSave
from dlo_flexbe_states.set_dlo_params_state import SetDloParams
from dlo_flexbe_states.set_dlo_state_from_vision_state import SetDloStateFromVisionS
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Mon Jul 24 2023
@author: xxx
'''
class dlo_params_best_moveSM(Behavior):
	'''
	xxx
	'''


	def __init__(self):
		super(dlo_params_best_moveSM, self).__init__()
		self.name = 'dlo_params_best_move'

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
		_state_machine.userdata.param_bending = 0.5
		_state_machine.userdata.param_damping = 5
		_state_machine.userdata.param_mass = 0.02
		_state_machine.userdata.save_log = True

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:52 y:141
			OperatableStateMachine.add('initialize_robot_frankx',
										self.use_behavior(initialize_robot_frankxSM, 'initialize_robot_frankx'),
										transitions={'finished': 'set_params', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'ee_frame': 'ee_frame', 'world_frame': 'world_frame'})

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

			# x:772 y:65
			OperatableStateMachine.add('move_for_best_params',
										ComputeBestMoveParams(srv_name="dlo_state_planner/params_best_move"),
										transitions={'done': 'frankx_pick_place', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'save_log': 'save_log', 'pick_pose': 'pick_pose', 'place_pose': 'place_pose', 'idx': 'idx', 'dx': 'dx', 'dy': 'dy', 'dtheta': 'dtheta', 'identifier': 'identifier'})

			# x:454 y:82
			OperatableStateMachine.add('retrieve_dlo_state_from_camera',
										SetDloStateFromVisionS(srv_name="/dlo_state_planner/set_dlo_state_from_vision"),
										transitions={'done': 'move_for_best_params', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:545 y:558
			OperatableStateMachine.add('saving',
										ParamsIdentSave(srv_name="/dlo_state_planner/get_all_dlo_states"),
										transitions={'done': 'finished', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pick_pose': 'pick_pose', 'place_pose': 'place_pose', 'idx': 'idx', 'dx': 'dx', 'dy': 'dy', 'dtheta': 'dtheta', 'identifier': 'identifier'})

			# x:272 y:89
			OperatableStateMachine.add('set_params',
										SetDloParams(srv_name="/dlo_state_planner/set_dlo_params"),
										transitions={'done': 'retrieve_dlo_state_from_camera', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'kb': 'param_bending', 'kd': 'param_damping', 'm': 'param_mass'})

			# x:1020 y:216
			OperatableStateMachine.add('frankx_pick_place',
										self.use_behavior(frankx_pick_placeSM, 'frankx_pick_place'),
										transitions={'finished': 'homing2', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'pick_pose': 'pick_pose', 'place_pose': 'place_pose'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
