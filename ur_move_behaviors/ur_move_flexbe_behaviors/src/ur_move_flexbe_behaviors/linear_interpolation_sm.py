#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from ur_move_flexbe_states.Check_configuration_states import CheckConfiguration
from ur_move_flexbe_states.Final_joint_traj_states import FinalJointTrajectory
from ur_move_flexbe_states.Linear_interpolator_service import InterpolateActionState
from ur_move_flexbe_states.TcpW_to_eeBase_states import TcpW_to_eeBase
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Thu Jan 25 2024
@author: mk
'''
class linear_interpolationSM(Behavior):
	'''
	linear interpolation for movement
	'''


	def __init__(self):
		super(linear_interpolationSM, self).__init__()
		self.name = 'linear_interpolation'

		# parameters of this behavior
		self.add_parameter('priority', 0)
		self.add_parameter('period', 10)

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:693 y:337, x:370 y:338
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['UR', 'ur_type', 'start_pose', 'goal_pose', 'last_joints_in'], output_keys=['joint_trajectory', 'last_joints_out'])
		_state_machine.userdata.UR = ''
		_state_machine.userdata.ur_type = ''
		_state_machine.userdata.start_pose = []
		_state_machine.userdata.goal_pose = []
		_state_machine.userdata.last_joints_in = []
		_state_machine.userdata.joint_trajectory = []
		_state_machine.userdata.last_joints_out = []

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]

		# x:223 y:219, x:130 y:365, x:230 y:365
		_sm_tcpw_eebase_0 = ConcurrencyContainer(outcomes=['finished', 'failed'], input_keys=['UR', 'start_pose', 'goal_pose'], output_keys=['start_ee', 'goal_ee'], conditions=[
										('finished', [('TcpW_to_eeBase_start', 'done'), ('TcpW_to_eeBase_goal', 'done')])
										])

		with _sm_tcpw_eebase_0:
			# x:67 y:46
			OperatableStateMachine.add('TcpW_to_eeBase_start',
										TcpW_to_eeBase(),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off},
										remapping={'robot_type': 'UR', 'tcpW_pose': 'start_pose', 'ee_base': 'start_ee'})

			# x:276 y:46
			OperatableStateMachine.add('TcpW_to_eeBase_goal',
										TcpW_to_eeBase(),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off},
										remapping={'robot_type': 'UR', 'tcpW_pose': 'goal_pose', 'ee_base': 'goal_ee'})



		with _state_machine:
			# x:28 y:122
			OperatableStateMachine.add('TcpW_eeBase',
										_sm_tcpw_eebase_0,
										transitions={'finished': 'Linear interpolator', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'UR': 'ur_type', 'start_pose': 'start_pose', 'goal_pose': 'goal_pose', 'start_ee': 'start_ee', 'goal_ee': 'goal_ee'})

			# x:560 y:42
			OperatableStateMachine.add('Final joint trajectory',
										FinalJointTrajectory(),
										transitions={'done': 'finished', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'ik_solution': 'ik_solution', 'kin_choice': 'ik_choice', 'Final_trajectory': 'joint_trajectory', 'last_joints': 'last_joints_out'})

			# x:194 y:47
			OperatableStateMachine.add('Linear interpolator',
										InterpolateActionState(srv_name='/ur_inverse_k', check_q6=True, period=self.period),
										transitions={'done': 'Check configuration', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'UR': 'UR', 'start_pose': 'start_ee', 'ee_pose': 'goal_ee', 'last_joints': 'last_joints_in', 'success': 'success', 'max_error': 'max_error', 'ik_solution': 'ik_solution', 'trajectory_interpolated': 'trajectory_interpolated'})

			# x:369 y:48
			OperatableStateMachine.add('Check configuration',
										CheckConfiguration(prior_config=self.priority),
										transitions={'done': 'Final joint trajectory', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'success': 'success', 'max_error': 'max_error', 'ik_solution': 'ik_solution', 'ik_choice': 'ik_choice'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
