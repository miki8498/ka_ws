#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from dlo_flexbe_behaviors.initialize_ur_sm import initialize_urSM
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
from ur_move_flexbe_behaviors.move_one_arm_sm import Move_one_armSM
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Wed Jul 26 2023
@author: xxx
'''
class Pickandplace1armSM(Behavior):
	'''
	xxx
	'''


	def __init__(self):
		super(Pickandplace1armSM, self).__init__()
		self.name = 'Pick and place 1 arm'

		# parameters of this behavior
		self.add_parameter('priority', 0)
		self.add_parameter('open', True)
		self.add_parameter('close', False)
		self.add_parameter('simulation', True)

		# references to used behaviors
		self.add_behavior(Move_one_armSM, 'Move_one_arm')
		self.add_behavior(initialize_urSM, 'initialize_ur')

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
		_state_machine.userdata.robot = 'UR_right'
		_state_machine.userdata.type = 'ur_right'
		_state_machine.userdata.pick_pose = [0.186, 0.115, 0.285, 3.080, 0.812, 0.002]
		_state_machine.userdata.place_pose = [0.186, 0.135, 0.285, 3.080, 0.812, 0.002]
		_state_machine.userdata.other = 'ur_left'

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:61 y:374
			OperatableStateMachine.add('initialize_ur',
										self.use_behavior(initialize_urSM, 'initialize_ur',
											parameters={'priority': self.priority, 'gripper_config': self.open, 'simulation': self.simulation}),
										transitions={'finished': 'Move_one_arm', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'robot': 'robot', 'type': 'type', 'other': 'other', 'last_joints': 'last_joints', 'homing': 'homing'})

			# x:759 y:579
			OperatableStateMachine.add('add_movement_for_param_ident',
										AddParamIdent(srv_name="/dlo_state_planner/add_param_ident_file"),
										transitions={'done': 'find_params', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'identifier': 'identifier'})

			# x:227 y:47
			OperatableStateMachine.add('current_shape',
										SetDloStateFromVisionS(srv_name="/dlo_state_planner/set_dlo_state_from_vision"),
										transitions={'done': 'find_action', 'failed': 'failed'},
										autonomy={'done': Autonomy.Full, 'failed': Autonomy.Off})

			# x:1116 y:437
			OperatableStateMachine.add('error',
										ErrorFromTargetShape(srv_name="dlo_state_planner/get_all_dlo_states", _error_threshold=0.01),
										transitions={'done': 'save_for_params', 'finished': 'save_for_params', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'finished': Autonomy.Off, 'failed': Autonomy.Off})

			# x:468 y:10
			OperatableStateMachine.add('find_action',
										FindActionMove(srv_name="dlo_state_planner/find_action", error_th=0.01, save_log=True),
										transitions={'done': 'find_action', 'perform_action': 'find_action', 'failed': 'failed'},
										autonomy={'done': Autonomy.Full, 'perform_action': Autonomy.Full, 'failed': Autonomy.Off},
										remapping={'pick_pose': 'pick_pose', 'place_pose': 'place_pose', 'idx': 'idx', 'dx': 'dx', 'dy': 'dy', 'dtheta': 'dtheta', 'identifier': 'identifier'})

			# x:557 y:586
			OperatableStateMachine.add('find_params',
										FindDloParams(srv_name="dlo_state_planner/find_params"),
										transitions={'done': 'set_params', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'indices': 'indices_find_params', 'mass': 'mass', 'kd': 'kd', 'kb': 'kb'})

			# x:1101 y:153
			OperatableStateMachine.add('new_current_shape',
										SetDloStateFromVisionS(srv_name="/dlo_state_planner/set_dlo_state_from_vision"),
										transitions={'done': 'save_all', 'failed': 'new_current_shape'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:144 y:156
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
										transitions={'done': 'set_params', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'kb': 'kb', 'kd': 'kd', 'm': 'mass'})

			# x:92 y:245
			OperatableStateMachine.add('set_target_from_file',
										SetTargetDloFromFile(srv_name="/dlo_state_planner/set_target_dlo_state", file_name="I_135deg_black.txt"),
										transitions={'done': 'reset_state_and_pred', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:755 y:51
			OperatableStateMachine.add('Move_one_arm',
										self.use_behavior(Move_one_armSM, 'Move_one_arm',
											parameters={'z_over_pick': 0.2, 'priority': self.priority, 'period_to_pick': 3, 'z_to_move': 0.03, 'gripper_open': self.open, 'gripper_close': self.close, 'simulation': self.simulation, 'z_over_place': 0.2}),
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'robot': 'robot', 'robot_type': 'type', 'last_joints_in': 'last_joints', 'start_pose': 'homing', 'pick_pose': 'pick_pose', 'place_pose': 'place_pose'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
