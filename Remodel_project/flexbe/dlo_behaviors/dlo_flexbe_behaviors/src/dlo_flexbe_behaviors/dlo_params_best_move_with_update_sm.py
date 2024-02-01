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
from dlo_flexbe_states.compare_current_dlo_to_prediction import CompareDLOtoPred
from dlo_flexbe_states.find_params_state import FindDloParams
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
class dlo_params_best_move_with_updateSM(Behavior):
	'''
	xxx
	'''


	def __init__(self):
		super(dlo_params_best_move_with_updateSM, self).__init__()
		self.name = 'dlo_params_best_move_with_update'

		# parameters of this behavior

		# references to used behaviors
		self.add_behavior(frankx_pick_placeSM, 'frankx_pick_place')
		self.add_behavior(initialize_robot_frankxSM, 'initialize_robot_frankx')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:25 y:259, x:600 y:410
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.ee_frame = "panda_1_hand_tcp"
		_state_machine.userdata.world_frame = "world"
		_state_machine.userdata.gripper_width_open = 0.03
		_state_machine.userdata.gripper_width_close = 0.01
		_state_machine.userdata.gripper_force = 1.0
		_state_machine.userdata.param_mass = 0.05
		_state_machine.userdata.save_log = True
		_state_machine.userdata.indices = [-1]

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

			# x:263 y:600
			OperatableStateMachine.add('find_params',
										FindDloParams(srv_name="dlo_state_planner/find_params"),
										transitions={'done': 'set_params', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'indices': 'indices', 'mass': 'param_mass', 'kd': 'param_damping', 'kb': 'param_bending'})

			# x:1020 y:216
			OperatableStateMachine.add('frankx_pick_place',
										self.use_behavior(frankx_pick_placeSM, 'frankx_pick_place'),
										transitions={'finished': 'homing2', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'pick_pose': 'pick_pose', 'place_pose': 'place_pose'})

			# x:954 y:577
			OperatableStateMachine.add('get_new_dlo',
										SetDloStateFromVisionS(srv_name="/dlo_state_planner/set_dlo_state_from_vision"),
										transitions={'done': 'compare', 'failed': 'failed'},
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
										remapping={'save_log': 'save_log', 'pick_pose': 'pick_pose', 'place_pose': 'place_pose', 'idx': 'idx', 'dx': 'dx', 'dy': 'dy', 'dtheta': 'dtheta', 'prediction': 'prediction', 'identifier': 'identifier'})

			# x:454 y:82
			OperatableStateMachine.add('retrieve_dlo_state_from_camera',
										SetDloStateFromVisionS(srv_name="/dlo_state_planner/set_dlo_state_from_vision"),
										transitions={'done': 'move_for_best_params', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:537 y:604
			OperatableStateMachine.add('saving',
										ParamsIdentSave(srv_name="/dlo_state_planner/get_all_dlo_states", srv_param_name="/dlo_state_planner/get_dlo_params"),
										transitions={'done': 'find_params', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pick_pose': 'pick_pose', 'place_pose': 'place_pose', 'idx': 'idx', 'dx': 'dx', 'dy': 'dy', 'dtheta': 'dtheta', 'identifier': 'identifier', 'prediction': 'prediction'})

			# x:203 y:455
			OperatableStateMachine.add('set_params',
										SetDloParams(srv_name="/dlo_state_planner/set_dlo_params"),
										transitions={'done': 'finished', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'kb': 'param_bending', 'kd': 'param_damping', 'm': 'param_mass'})

			# x:732 y:629
			OperatableStateMachine.add('compare',
										CompareDLOtoPred(srv_name="dlo_state_planner/get_dlo_state"),
										transitions={'done': 'saving', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'prediction': 'prediction'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
