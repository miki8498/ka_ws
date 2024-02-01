#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from dlo_flexbe_states.find_params_state import FindDloParams
from dlo_flexbe_states.set_dlo_params_state import SetDloParams
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Fri Jul 28 2023
@author: xxx
'''
class find_params_identificationSM(Behavior):
	'''
	xxx
	'''


	def __init__(self):
		super(find_params_identificationSM, self).__init__()
		self.name = 'find_params_identification'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:679 y:245, x:401 y:376
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.indices = [-1]
		_state_machine.userdata.mass = 0.02

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:150 y:87
			OperatableStateMachine.add('find_params',
										FindDloParams(srv_name="dlo_state_planner/find_params"),
										transitions={'done': 'set_estimated_params', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'indices': 'indices', 'mass': 'mass', 'kd': 'kd', 'kb': 'kb'})

			# x:368 y:85
			OperatableStateMachine.add('set_estimated_params',
										SetDloParams(srv_name="/dlo_state_planner/set_dlo_params"),
										transitions={'done': 'finished', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'kb': 'kb', 'kd': 'kd', 'm': 'mass'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
