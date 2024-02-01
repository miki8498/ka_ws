#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from ur_move_flexbe_states.Cart_linear_interpolator import CartLinearInterpolator
from ur_move_flexbe_states.Check_Radius_staes import CheckRadius
from ur_move_flexbe_states.Check_configuration_states import CheckConfiguration
from ur_move_flexbe_states.Check_double_collision_service import CheckDoubleCollision
from ur_move_flexbe_states.Final_joint_traj_states import FinalJointTrajectory
from ur_move_flexbe_states.From_list_to_pose_states import FromListToPose
from ur_move_flexbe_states.Joint_traj_service import JointTrajService
from ur_move_flexbe_states.TcpW_to_eeBase_states import TcpW_to_eeBase
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Wed Jan 31 2024
@author: mk
'''
class two_move_robotsSM(Behavior):
	'''
	dfe
	'''


	def __init__(self):
		super(two_move_robotsSM, self).__init__()
		self.name = 'two_move_robots'

		# parameters of this behavior
		self.add_parameter('period', 1)
		self.add_parameter('priority', 0)

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:691 y:492, x:388 y:325
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['robot1', 'robot2', 'start_pose1', 'start_pose2', 'goal_pose1', 'goal_pose2', 'robot_type1', 'robot_type2', 'last_joints1', 'last_joints2'])
		_state_machine.userdata.robot1 = 'UR_right'
		_state_machine.userdata.robot2 = 'UR_left'
		_state_machine.userdata.start_pose1 = [0.061, 0.121, 0.486, 3.080, 0.812, 0.002]
		_state_machine.userdata.start_pose2 = [0.12, 0.121, 0.486, 3.080, 0.812, 0.002]
		_state_machine.userdata.goal_pose1 = [-0.217, -0.134, 0.531, -3.133, 0.618, -3.141]
		_state_machine.userdata.goal_pose2 = [-0.217, -0.114, 0.531, -3.133, 0.618, -3.141]
		_state_machine.userdata.robot_type1 = 'ur_right'
		_state_machine.userdata.robot_type2 = 'ur_left'
		_state_machine.userdata.last_joints1 = [-0.028815620430155953, -0.9809188862605618, -1.9710239942713814, -0.9500126966489781, 1.6555399313062988, -0.019683670821893706]
		_state_machine.userdata.last_joints2 = [3.1400460570663977, -0.7638479956128976, -1.7638578324783003, -1.566860150042011, 1.563874491853153, 2.9259121668623322e-05]

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]

		# x:247 y:192, x:130 y:365, x:230 y:365
		_sm_tcpw_eebase_2_0 = ConcurrencyContainer(outcomes=['finished', 'failed'], input_keys=['robot_type2', 'start_pose2', 'goal_pose2'], output_keys=['ee_start_pose2', 'ee_goal_pose2'], conditions=[
										('finished', [('TcpW_eeBase_start_2', 'done'), ('TcpW_eeBase_goal_2', 'done')])
										])

		with _sm_tcpw_eebase_2_0:
			# x:125 y:61
			OperatableStateMachine.add('TcpW_eeBase_start_2',
										TcpW_to_eeBase(),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off},
										remapping={'robot_type': 'robot_type2', 'tcpW_pose': 'start_pose2', 'ee_base': 'ee_start_pose2'})

			# x:301 y:62
			OperatableStateMachine.add('TcpW_eeBase_goal_2',
										TcpW_to_eeBase(),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off},
										remapping={'robot_type': 'robot_type2', 'tcpW_pose': 'goal_pose2', 'ee_base': 'ee_goal_pose2'})


		# x:284 y:197, x:130 y:365, x:230 y:365
		_sm_tcpw_eebase_1_1 = ConcurrencyContainer(outcomes=['finished', 'failed'], input_keys=['robot_type1', 'start_pose1', 'goal_pose1'], output_keys=['ee_start_pose1', 'ee_goal_pose1'], conditions=[
										('finished', [('TcpW_eeBase 1 start', 'done'), ('TcpW_eeBase 1 goal', 'done')])
										])

		with _sm_tcpw_eebase_1_1:
			# x:106 y:63
			OperatableStateMachine.add('TcpW_eeBase 1 start',
										TcpW_to_eeBase(),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off},
										remapping={'robot_type': 'robot_type1', 'tcpW_pose': 'start_pose1', 'ee_base': 'ee_start_pose1'})

			# x:312 y:62
			OperatableStateMachine.add('TcpW_eeBase 1 goal',
										TcpW_to_eeBase(),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off},
										remapping={'robot_type': 'robot_type1', 'tcpW_pose': 'goal_pose1', 'ee_base': 'ee_goal_pose1'})


		# x:271 y:207, x:130 y:458, x:230 y:458
		_sm_linear_interpolator_2 = ConcurrencyContainer(outcomes=['finished', 'failed'], input_keys=['robot1', 'robot2', 'start_pose1', 'start_pose2', 'goal_pose1', 'goal_pose2'], output_keys=['cart_traj1', 'cart_traj2'], conditions=[
										('finished', [('Linear interpolator 1', 'done'), ('Linear interpolator 2', 'done')])
										])

		with _sm_linear_interpolator_2:
			# x:137 y:69
			OperatableStateMachine.add('Linear interpolator 1',
										CartLinearInterpolator(period=self.period),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off},
										remapping={'robot': 'robot1', 'start_pose': 'start_pose1', 'goal_pose': 'goal_pose1', 'trajectory': 'cart_traj1'})

			# x:319 y:66
			OperatableStateMachine.add('Linear interpolator 2',
										CartLinearInterpolator(period=self.period),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off},
										remapping={'robot': 'robot2', 'start_pose': 'start_pose2', 'goal_pose': 'goal_pose2', 'trajectory': 'cart_traj2'})


		# x:326 y:249, x:172 y:169, x:230 y:458, x:505 y:175, x:430 y:458
		_sm_container_3_3 = ConcurrencyContainer(outcomes=['finished', 'failed'], input_keys=['success_1', 'max_error_1', 'ik_solution_1', 'success_2', 'max_error_2', 'ik_solution_2'], output_keys=['ik_choice_1', 'ik_choice_2'], conditions=[
										('finished', [('Check config1', 'done'), ('Check config2', 'done')]),
										('failed', [('Check config1', 'failed')]),
										('failed', [('Check config2', 'failed')])
										])

		with _sm_container_3_3:
			# x:188 y:54
			OperatableStateMachine.add('Check config1',
										CheckConfiguration(prior_config=self.priority),
										transitions={'done': 'finished', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'success': 'success_1', 'max_error': 'max_error_1', 'ik_solution': 'ik_solution_1', 'ik_choice': 'ik_choice_1'})

			# x:364 y:48
			OperatableStateMachine.add('Check config2',
										CheckConfiguration(prior_config=self.priority),
										transitions={'done': 'finished', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'success': 'success_2', 'max_error': 'max_error_2', 'ik_solution': 'ik_solution_2', 'ik_choice': 'ik_choice_2'})


		# x:325 y:240, x:222 y:199, x:521 y:180, x:330 y:365, x:430 y:365
		_sm_container_2_4 = ConcurrencyContainer(outcomes=['finished', 'failed'], input_keys=['ik_solution_1', 'ik_solution_2', 'ik_choice_1', 'ik_choice_2'], output_keys=['Final_traj_1', 'Final_traj_2', 'sphere_last_joints1', 'sphere_last_joints2'], conditions=[
										('finished', [('Final traj 1', 'done'), ('Final traj 2', 'done')]),
										('failed', [('Final traj 1', 'failed')]),
										('finished', [('Final traj 2', 'failed')])
										])

		with _sm_container_2_4:
			# x:204 y:73
			OperatableStateMachine.add('Final traj 1',
										FinalJointTrajectory(),
										transitions={'done': 'finished', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'ik_solution': 'ik_solution_1', 'kin_choice': 'ik_choice_1', 'Final_trajectory': 'Final_traj_1', 'last_joints': 'sphere_last_joints1'})

			# x:373 y:72
			OperatableStateMachine.add('Final traj 2',
										FinalJointTrajectory(),
										transitions={'done': 'finished', 'failed': 'finished'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'ik_solution': 'ik_solution_2', 'kin_choice': 'ik_choice_2', 'Final_trajectory': 'Final_traj_2', 'last_joints': 'sphere_last_joints2'})


		# x:288 y:204, x:161 y:209, x:230 y:365, x:401 y:195, x:430 y:365
		_sm_container_5 = ConcurrencyContainer(outcomes=['finished', 'failed'], input_keys=['robot_type1', 'robot_type2', 'cart_traj1', 'robot2_corrected_traj', 'last_joints1', 'last_joints2'], output_keys=['success_1', 'max_error_1', 'ik_solution_1', 'success_2', 'max_error_2', 'ik_solution_2'], conditions=[
										('finished', [('Joint traj fixed', 'done'), ('Joint traj moving', 'done')]),
										('failed', [('Joint traj fixed', 'failed')]),
										('failed', [('Joint traj moving', 'failed')])
										])

		with _sm_container_5:
			# x:138 y:45
			OperatableStateMachine.add('Joint traj fixed',
										JointTrajService(check_q6=True),
										transitions={'done': 'finished', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'robot_type': 'robot_type1', 'trajectory': 'cart_traj1', 'last_joints_traj': 'last_joints1', 'success': 'success_1', 'max_error': 'max_error_1', 'ik_solution': 'ik_solution_1'})

			# x:339 y:45
			OperatableStateMachine.add('Joint traj moving',
										JointTrajService(check_q6=True),
										transitions={'done': 'finished', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'robot_type': 'robot_type2', 'trajectory': 'robot2_corrected_traj', 'last_joints_traj': 'last_joints2', 'success': 'success_2', 'max_error': 'max_error_2', 'ik_solution': 'ik_solution_2'})



		with _state_machine:
			# x:117 y:167
			OperatableStateMachine.add('fde',
										FromListToPose(),
										transitions={'done': 'freg'},
										autonomy={'done': Autonomy.Off},
										remapping={'list_pose': 'start_pose1', 'Pose_pose': 'ee_start_pose1'})

			# x:747 y:61
			OperatableStateMachine.add('Check radius',
										CheckRadius(),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off},
										remapping={'robot1_traj': 'cart_traj1', 'robot2_traj': 'cart_traj2', 'robot2_corrected_traj': 'robot2_corrected_traj'})

			# x:901 y:148
			OperatableStateMachine.add('Container',
										_sm_container_5,
										transitions={'finished': 'Container_3', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'robot_type1': 'robot_type1', 'robot_type2': 'robot_type2', 'cart_traj1': 'cart_traj1', 'robot2_corrected_traj': 'robot2_corrected_traj', 'last_joints1': 'last_joints1', 'last_joints2': 'last_joints2', 'success_1': 'success_1', 'max_error_1': 'max_error_1', 'ik_solution_1': 'ik_solution_1', 'success_2': 'success_2', 'max_error_2': 'max_error_2', 'ik_solution_2': 'ik_solution_2'})

			# x:919 y:403
			OperatableStateMachine.add('Container_2',
										_sm_container_2_4,
										transitions={'finished': 'Check collision', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'ik_solution_1': 'ik_solution_1', 'ik_solution_2': 'ik_solution_2', 'ik_choice_1': 'ik_choice_1', 'ik_choice_2': 'ik_choice_2', 'Final_traj_1': 'Final_traj_1', 'Final_traj_2': 'Final_traj_2', 'sphere_last_joints1': 'sphere_last_joints1', 'sphere_last_joints2': 'sphere_last_joints2'})

			# x:916 y:274
			OperatableStateMachine.add('Container_3',
										_sm_container_3_3,
										transitions={'finished': 'Container_2', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'success_1': 'success_1', 'max_error_1': 'max_error_1', 'ik_solution_1': 'ik_solution_1', 'success_2': 'success_2', 'max_error_2': 'max_error_2', 'ik_solution_2': 'ik_solution_2', 'ik_choice_1': 'ik_choice_1', 'ik_choice_2': 'ik_choice_2'})

			# x:534 y:43
			OperatableStateMachine.add('Linear interpolator',
										_sm_linear_interpolator_2,
										transitions={'finished': 'Check radius', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'robot1': 'robot1', 'robot2': 'robot2', 'start_pose1': 'ee_start_pose1', 'start_pose2': 'ee_start_pose2', 'goal_pose1': 'ee_goal_pose1', 'goal_pose2': 'ee_goal_pose2', 'cart_traj1': 'cart_traj1', 'cart_traj2': 'cart_traj2'})

			# x:30 y:40
			OperatableStateMachine.add('TcpW_eeBase_1',
										_sm_tcpw_eebase_1_1,
										transitions={'finished': 'TcpW_eeBase_2', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'robot_type1': 'robot_type1', 'start_pose1': 'start_pose1', 'goal_pose1': 'goal_pose1', 'ee_start_pose1': 'ee_start_pose1', 'ee_goal_pose1': 'ee_goal_pose1'})

			# x:243 y:42
			OperatableStateMachine.add('TcpW_eeBase_2',
										_sm_tcpw_eebase_2_0,
										transitions={'finished': 'TcpW_eeBase_2', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'robot_type2': 'robot_type2', 'start_pose2': 'start_pose2', 'goal_pose2': 'goal_pose2', 'ee_start_pose2': 'ee_start_pose2', 'ee_goal_pose2': 'ee_goal_pose2'})

			# x:115 y:253
			OperatableStateMachine.add('freg',
										FromListToPose(),
										transitions={'done': 'sdfadg'},
										autonomy={'done': Autonomy.Off},
										remapping={'list_pose': 'goal_pose1', 'Pose_pose': 'ee_goal_pose1'})

			# x:124 y:444
			OperatableStateMachine.add('rehre',
										FromListToPose(),
										transitions={'done': 'Linear interpolator'},
										autonomy={'done': Autonomy.Off},
										remapping={'list_pose': 'goal_pose2', 'Pose_pose': 'ee_goal_pose2'})

			# x:119 y:347
			OperatableStateMachine.add('sdfadg',
										FromListToPose(),
										transitions={'done': 'rehre'},
										autonomy={'done': Autonomy.Off},
										remapping={'list_pose': 'start_pose2', 'Pose_pose': 'ee_start_pose2'})

			# x:905 y:531
			OperatableStateMachine.add('Check collision',
										CheckDoubleCollision(open=False),
										transitions={'done': 'finished', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'robot': 'robot1', 'trajectory_robot': 'Final_traj_1', 'trajectory_other': 'Final_traj_2', 'success': 'success'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
