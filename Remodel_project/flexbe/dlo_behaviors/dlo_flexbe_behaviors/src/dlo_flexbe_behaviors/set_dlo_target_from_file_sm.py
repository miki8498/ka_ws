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
from dlo_flexbe_states.set_target_dlo_from_file import SetTargetDloFromFile
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Tue Aug 22 2023
@author: xxx
'''
class set_dlo_target_from_fileSM(Behavior):
	'''
	xxx
	'''


	def __init__(self):
		super(set_dlo_target_from_fileSM, self).__init__()
		self.name = 'set_dlo_target_from_file'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		file_name = "prova.txt"
		# x:814 y:268, x:69 y:218
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:242 y:71
			OperatableStateMachine.add('reset',
										ResetDloStateAndTarget(srv_name="/dlo_state_planner/reset_dlo_state", reset_state=True, reset_target=True, reset_pred=True),
										transitions={'done': 'set', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:559 y:122
			OperatableStateMachine.add('set',
										SetTargetDloFromFile(srv_name="/dlo_state_planner/set_target_dlo_state", file_name=file_name),
										transitions={'done': 'finished', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
