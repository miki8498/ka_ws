#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from dlo_flexbe_states.get_dlo_state import GetDloStateCall
from dlo_flexbe_states.reset_dlo_state import ResetDloStateAndTarget
from dlo_flexbe_states.save_dlo_state_to_file import SaveDloStateToFile
from dlo_flexbe_states.set_dlo_state_from_vision_state import SetDloStateFromVisionS
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Thu Jul 27 2023
@author: xxx
'''
class get_and_save_dlo_state_to_fileSM(Behavior):
	'''
	xxx
	'''


	def __init__(self):
		super(get_and_save_dlo_state_to_fileSM, self).__init__()
		self.name = 'get_and_save_dlo_state_to_file'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:865 y:274, x:130 y:365
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:36 y:120
			OperatableStateMachine.add('reset',
										ResetDloStateAndTarget(srv_name="/dlo_state_planner/reset_dlo_state", reset_state=True, reset_target=True, reset_pred=True),
										transitions={'done': 'set', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:624 y:141
			OperatableStateMachine.add('save',
										SaveDloStateToFile(file_name="prova.txt"),
										transitions={'done': 'finished', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'dlo': 'dlo'})

			# x:204 y:57
			OperatableStateMachine.add('set',
										SetDloStateFromVisionS(srv_name="/dlo_state_planner/set_dlo_state_from_vision"),
										transitions={'done': 'get', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:428 y:101
			OperatableStateMachine.add('get',
										GetDloStateCall(srv_name="/dlo_state_planner/get_dlo_state"),
										transitions={'done': 'save', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'dlo': 'dlo'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
