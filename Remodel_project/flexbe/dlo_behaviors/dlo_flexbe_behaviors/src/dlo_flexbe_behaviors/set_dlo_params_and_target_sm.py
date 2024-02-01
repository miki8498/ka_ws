#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from dlo_flexbe_states.reset_dlo_state import ResetDloStateAndTarget
from dlo_flexbe_states.set_dlo_params_state import SetDloParams
from dlo_flexbe_states.set_target_dlo_from_vision_state import SetTargetDloFromVision
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Thu Jul 27 2023
@author: xxx
'''
class set_dlo_params_and_targetSM(Behavior):
	'''
	xxx
	'''


	def __init__(self):
		super(set_dlo_params_and_targetSM, self).__init__()
		self.name = 'set_dlo_params_and_target'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:740 y:273, x:130 y:365
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['kd', 'kb'])
		_state_machine.userdata.kd = 25
		_state_machine.userdata.kb = 0.1
		_state_machine.userdata.mass = 0.02

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:36 y:120
			OperatableStateMachine.add('reset',
										ResetDloStateAndTarget(srv_name="/dlo_state_planner/reset_dlo_state", reset_state=True, reset_target=True, reset_pred=True),
										transitions={'done': 'set_params', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:247 y:116
			OperatableStateMachine.add('set_params',
										SetDloParams(srv_name="/dlo_state_planner/set_dlo_params"),
										transitions={'done': 'set_target', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'kb': 'kb', 'kd': 'kd', 'm': 'mass'})

			# x:451 y:158
			OperatableStateMachine.add('set_target',
										SetTargetDloFromVision(srv_name="/dlo_state_planner/set_target_dlo_state_from_vision"),
										transitions={'done': 'finished', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
