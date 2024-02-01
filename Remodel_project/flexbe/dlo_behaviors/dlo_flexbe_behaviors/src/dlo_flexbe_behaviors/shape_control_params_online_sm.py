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
from dlo_flexbe_states.add_param_ident_file import AddParamIdent
from dlo_flexbe_states.error_from_target_state import ErrorFromTargetShape
from dlo_flexbe_states.find_action_move_state import FindActionMove
from dlo_flexbe_states.find_params_state import FindDloParams
from dlo_flexbe_states.params_identification_save_pick_place_move import ParamsIdentSave
from dlo_flexbe_states.reset_dlo_state import ResetDloStateAndTarget
from dlo_flexbe_states.save_pick_place_move_all_dlo_state import SaveAllPickPlaceMoveDlo
from dlo_flexbe_states.set_dlo_params_state import SetDloParams
from dlo_flexbe_states.set_dlo_state_from_vision_state import SetDloStateFromVisionS
from dlo_flexbe_states.set_target_dlo_from_file import SetTargetDloFromFile
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Wed Jul 26 2023
@author: xxx
'''
class shape_control_params_onlineSM(Behavior):
	'''
	xxx
	'''


	def __init__(self):
		super(shape_control_params_onlineSM, self).__init__()
		self.name = 'shape_control_params_online'

		# parameters of this behavior

		# references to used behaviors
		self.add_behavior(frankx_pick_placeSM, 'frankx_pick_place')
		self.add_behavior(initialize_robot_frankxSM, 'initialize_robot_frankx')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:117 y:586, x:792 y:318
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.ee_frame = 'panda_1_hand_tcp'
		_state_machine.userdata.world_frame = 'world'
		_state_machine.userdata.indices_find_params = [-1]
		_state_machine.userdata.mass = 0.05

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:75 y:417
			OperatableStateMachine.add('initialize_robot_frankx',
										self.use_behavior(initialize_robot_frankxSM, 'initialize_robot_frankx'),
										transitions={'finished': 'set_target_from_file', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'ee_frame': 'ee_frame', 'world_frame': 'world_frame'})

			# x:471 y:33
			OperatableStateMachine.add('current_shape',
										SetDloStateFromVisionS(srv_name="/dlo_state_planner/set_dlo_state_from_vision"),
										transitions={'done': 'find_action', 'failed': 'failed'},
										autonomy={'done': Autonomy.Full, 'failed': Autonomy.Off})

			# x:1116 y:437
			OperatableStateMachine.add('error',
										ErrorFromTargetShape(srv_name="dlo_state_planner/get_all_dlo_states", _error_threshold=0.01),
										transitions={'done': 'save_for_params', 'finished': 'save_for_params', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'finished': Autonomy.Off, 'failed': Autonomy.Off})

			# x:710 y:50
			OperatableStateMachine.add('find_action',
										FindActionMove(srv_name="dlo_state_planner/find_action", error_th=0.01, save_log=True),
										transitions={'done': 'finished', 'perform_action': 'frankx_pick_place', 'failed': 'failed'},
										autonomy={'done': Autonomy.Full, 'perform_action': Autonomy.Full, 'failed': Autonomy.Off},
										remapping={'pick_pose': 'pick_pose', 'place_pose': 'place_pose', 'idx': 'idx', 'dx': 'dx', 'dy': 'dy', 'dtheta': 'dtheta', 'identifier': 'identifier'})

			# x:557 y:586
			OperatableStateMachine.add('find_params',
										FindDloParams(srv_name="dlo_state_planner/find_params"),
										transitions={'done': 'set_params', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'indices': 'indices_find_params', 'mass': 'mass', 'kd': 'kd', 'kb': 'kb'})

			# x:908 y:62
			OperatableStateMachine.add('frankx_pick_place',
										self.use_behavior(frankx_pick_placeSM, 'frankx_pick_place'),
										transitions={'finished': 'new_current_shape', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'pick_pose': 'pick_pose', 'place_pose': 'place_pose'})

			# x:1101 y:153
			OperatableStateMachine.add('new_current_shape',
										SetDloStateFromVisionS(srv_name="/dlo_state_planner/set_dlo_state_from_vision"),
										transitions={'done': 'save_all', 'failed': 'new_current_shape'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:220 y:49
			OperatableStateMachine.add('reset_state_and_pred',
										ResetDloStateAndTarget(srv_name="/dlo_state_planner/reset_dlo_state", reset_state=True, reset_target=False, reset_pred=True),
										transitions={'done': 'current_shape', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:1107 y:318
			OperatableStateMachine.add('save_all',
										SaveAllPickPlaceMoveDlo(output_folder_name="output"),
										transitions={'done': 'error', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pick_pose': 'pick_pose', 'place_pose': 'place_pose', 'idx': 'idx', 'dx': 'dx', 'dy': 'dy', 'dtheta': 'dtheta', 'identifier': 'identifier'})

			# x:1092 y:578
			OperatableStateMachine.add('save_for_params',
										ParamsIdentSave(),
										transitions={'done': 'add_movement_for_param_ident', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pick_pose': 'pick_pose', 'place_pose': 'place_pose', 'idx': 'idx', 'dx': 'dx', 'dy': 'dy', 'dtheta': 'dtheta', 'identifier': 'identifier'})

			# x:258 y:603
			OperatableStateMachine.add('set_params',
										SetDloParams(srv_name="/dlo_state_planner/set_dlo_params"),
										transitions={'done': 'finished', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'kb': 'kb', 'kd': 'kd', 'm': 'mass'})

			# x:92 y:245
			OperatableStateMachine.add('set_target_from_file',
										SetTargetDloFromFile(srv_name="/dlo_state_planner/set_target_dlo_state", file_name="I_135deg_black.txt"),
										transitions={'done': 'reset_state_and_pred', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:759 y:579
			OperatableStateMachine.add('add_movement_for_param_ident',
										AddParamIdent(srv_name="/dlo_state_planner/add_param_ident_file"),
										transitions={'done': 'find_params', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'identifier': 'identifier'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
