#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from dlo_flexbe_behaviors.pick__sm import PickSM
from dlo_flexbe_behaviors.release_sm import ReleaseSM
from ur_move_flexbe_states.Cart_linear_interpolator import CartLinearInterpolator
from ur_move_flexbe_states.Check_Radius_staes import CheckRadius
from ur_move_flexbe_states.Check_configuration_states import CheckConfiguration
from ur_move_flexbe_states.Check_double_collision_service import CheckDoubleCollision
from ur_move_flexbe_states.Final_joint_traj_states import FinalJointTrajectory
from ur_move_flexbe_states.From_list_to_pose_states import FromListToPose
from ur_move_flexbe_states.Gripper_motion_pub import GripperMotion
from ur_move_flexbe_states.Joint_move_dual_arm_states import JointDualArmMove
from ur_move_flexbe_states.Joint_traj_service import JointTrajService
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
		self.add_parameter('delta_z_over_table', 0.01)
		self.add_parameter('period_over_pick', 10)
		self.add_parameter('period_to_pick', 10)
		self.add_parameter('delta_z_after_pick', 0.01)
		self.add_parameter('period_after_pick', 10)
		self.add_parameter('close', False)
		self.add_parameter('simulation', False)

		# references to used behaviors
		self.add_behavior(ReleaseSM, 'Container/Release')
		self.add_behavior(ReleaseSM, 'Container/Release_2')
		self.add_behavior(PickSM, 'Pick ')
		self.add_behavior(PickSM, 'Pick _2')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:93 y:567, x:388 y:325
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['robot1', 'robot2', 'start_pose1', 'start_pose2', 'place_pose1', 'place_pose2', 'robot_type1', 'robot_type2', 'last_joints1', 'last_joints2', 'pick_pose1', 'pick_pose2'])
		_state_machine.userdata.robot1 = ''
		_state_machine.userdata.robot2 = ''
		_state_machine.userdata.start_pose1 = []
		_state_machine.userdata.start_pose2 = []
		_state_machine.userdata.place_pose1 = []
		_state_machine.userdata.place_pose2 = []
		_state_machine.userdata.robot_type1 = ''
		_state_machine.userdata.robot_type2 = ''
		_state_machine.userdata.last_joints1 = []
		_state_machine.userdata.last_joints2 = []
		_state_machine.userdata.pick_pose1 = []
		_state_machine.userdata.pick_pose2 = []

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]

		# x:271 y:207, x:130 y:458, x:230 y:458
		_sm_linear_interpolator_0 = ConcurrencyContainer(outcomes=['finished', 'failed'], input_keys=['robot1', 'robot2', 'start_pose1', 'start_pose2', 'goal_pose1', 'goal_pose2'], output_keys=['cart_traj1', 'cart_traj2'], conditions=[
										('finished', [('Linear interpolator 1', 'done'), ('Linear interpolator 2', 'done')])
										])

		with _sm_linear_interpolator_0:
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


		# x:288 y:204, x:161 y:209, x:230 y:365, x:401 y:195, x:430 y:365
		_sm_joint_traj_service_1 = ConcurrencyContainer(outcomes=['finished', 'failed'], input_keys=['robot_type1', 'robot_type2', 'cart_traj1', 'robot2_corrected_traj', 'last_joints1', 'last_joints2'], output_keys=['success_1', 'max_error_1', 'ik_solution_1', 'success_2', 'max_error_2', 'ik_solution_2'], conditions=[
										('finished', [('Joint traj fixed', 'done'), ('Joint traj moving', 'done')]),
										('failed', [('Joint traj fixed', 'failed')]),
										('failed', [('Joint traj moving', 'failed')])
										])

		with _sm_joint_traj_service_1:
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


		# x:214 y:175, x:130 y:365, x:230 y:365
		_sm_gripper_close_2 = ConcurrencyContainer(outcomes=['finished', 'failed'], input_keys=['robot_type1', 'robot_type2'], conditions=[
										('finished', [('Gripper close 1', 'done'), ('Gripper close 2', 'done')])
										])

		with _sm_gripper_close_2:
			# x:85 y:50
			OperatableStateMachine.add('Gripper close 1',
										GripperMotion(open=self.close, simulation=self.simulation),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off},
										remapping={'id_type': 'robot_type1'})

			# x:245 y:49
			OperatableStateMachine.add('Gripper close 2',
										GripperMotion(open=self.close, simulation=self.simulation),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off},
										remapping={'id_type': 'robot_type2'})


		# x:279 y:232, x:130 y:365, x:230 y:365
		_sm_from_list_to_pose_3 = ConcurrencyContainer(outcomes=['finished', 'failed'], input_keys=['start_pose1', 'goal_pose1', 'start_pose2', 'goal_pose2'], output_keys=['ee_start_pose1', 'ee_goal_pose1', 'ee_start_pose2', 'ee_goal_pose2'], conditions=[
										('finished', [('list to pose start 1', 'done'), ('list to pose goal 1', 'done'), ('list to pose start 2', 'done'), ('list to pose goal 2', 'done')])
										])

		with _sm_from_list_to_pose_3:
			# x:84 y:50
			OperatableStateMachine.add('list to pose start 1',
										FromListToPose(),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off},
										remapping={'list_pose': 'start_pose1', 'Pose_pose': 'ee_start_pose1'})

			# x:461 y:184
			OperatableStateMachine.add('list to pose goal 2',
										FromListToPose(),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off},
										remapping={'list_pose': 'goal_pose2', 'Pose_pose': 'ee_goal_pose2'})

			# x:470 y:84
			OperatableStateMachine.add('list to pose start 2',
										FromListToPose(),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off},
										remapping={'list_pose': 'start_pose2', 'Pose_pose': 'ee_start_pose2'})

			# x:254 y:52
			OperatableStateMachine.add('list to pose goal 1',
										FromListToPose(),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off},
										remapping={'list_pose': 'goal_pose1', 'Pose_pose': 'ee_goal_pose1'})


		# x:325 y:240, x:222 y:199, x:521 y:180, x:330 y:365, x:430 y:365
		_sm_final_traj_4 = ConcurrencyContainer(outcomes=['finished', 'failed'], input_keys=['ik_solution_1', 'ik_solution_2', 'ik_choice_1', 'ik_choice_2'], output_keys=['Final_traj_1', 'Final_traj_2', 'sphere_last_joints1', 'sphere_last_joints2'], conditions=[
										('finished', [('Final traj 1', 'done'), ('Final traj 2', 'done')]),
										('failed', [('Final traj 1', 'failed')]),
										('finished', [('Final traj 2', 'failed')])
										])

		with _sm_final_traj_4:
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


		# x:219 y:189, x:65 y:162, x:145 y:434, x:392 y:181, x:430 y:365
		_sm_container_5 = ConcurrencyContainer(outcomes=['finished', 'failed'], input_keys=['robot1', 'robot2', 'robot_type1', 'robot_type2', 'pick_pose1', 'pick_pose2', 'last_joints_to_pick1', 'last_joints_to_pick2'], output_keys=['traj_after_pick1', 'traj_after_pick2', 'last_joints_after_pick1', 'last_joints_after_pick2', 'over_place_pose1', 'over_place_pose2'], conditions=[
										('finished', [('Release', 'finished'), ('Release_2', 'finished')]),
										('failed', [('Release', 'failed')]),
										('failed', [('Release_2', 'failed')])
										])

		with _sm_container_5:
			# x:79 y:44
			OperatableStateMachine.add('Release',
										self.use_behavior(ReleaseSM, 'Container/Release',
											parameters={'delta_z': self.delta_z_after_pick, 'priority': self.priority, 'period_over_place': self.period_after_pick}),
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'robot': 'robot1', 'robot_type': 'robot_type1', 'place_pose': 'pick_pose1', 'last_joints_in': 'last_joints_to_pick1', 'traj_over_place': 'traj_after_pick1', 'last_joints_over_place': 'last_joints_after_pick1', 'over_place_pose': 'over_place_pose1'})

			# x:260 y:40
			OperatableStateMachine.add('Release_2',
										self.use_behavior(ReleaseSM, 'Container/Release_2',
											parameters={'delta_z': self.delta_z_after_pick, 'priority': self.priority, 'period_over_place': self.period_after_pick}),
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'robot': 'robot2', 'robot_type': 'robot_type2', 'place_pose': 'pick_pose2', 'last_joints_in': 'last_joints_to_pick2', 'traj_over_place': 'traj_after_pick2', 'last_joints_over_place': 'last_joints_after_pick2', 'over_place_pose': 'over_place_pose2'})


		# x:326 y:249, x:172 y:169, x:230 y:458, x:505 y:175, x:430 y:458
		_sm_check_configuration_6 = ConcurrencyContainer(outcomes=['finished', 'failed'], input_keys=['success_1', 'max_error_1', 'ik_solution_1', 'success_2', 'max_error_2', 'ik_solution_2'], output_keys=['ik_choice_1', 'ik_choice_2'], conditions=[
										('finished', [('Check config1', 'done'), ('Check config2', 'done')]),
										('failed', [('Check config1', 'failed')]),
										('failed', [('Check config2', 'failed')])
										])

		with _sm_check_configuration_6:
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



		with _state_machine:
			# x:43 y:105
			OperatableStateMachine.add('Pick ',
										self.use_behavior(PickSM, 'Pick ',
											parameters={'delta_z': self.delta_z_over_table, 'priority': self.priority, 'period_over_pick': self.period_over_pick, 'period_to_pick': self.period_to_pick}),
										transitions={'finished': 'Pick _2', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'robot': 'robot1', 'robot_type': 'robot_type1', 'pick_pose': 'pick_pose1', 'last_joints_in': 'last_joints1', 'start_pose': 'start_pose1', 'traj_to_pick': 'traj_to_pick1', 'last_joints_to_pick': 'last_joints_to_pick1', 'traj_over_pick': 'traj_over_pick1'})

			# x:810 y:420
			OperatableStateMachine.add('Check collision',
										CheckDoubleCollision(open=False),
										transitions={'done': 'Move over pick', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'robot': 'robot1', 'trajectory_robot': 'Final_traj_1', 'trajectory_other': 'Final_traj_2', 'success': 'success'})

			# x:813 y:224
			OperatableStateMachine.add('Check configuration',
										_sm_check_configuration_6,
										transitions={'finished': 'Final traj', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'success_1': 'success_1', 'max_error_1': 'max_error_1', 'ik_solution_1': 'ik_solution_1', 'success_2': 'success_2', 'max_error_2': 'max_error_2', 'ik_solution_2': 'ik_solution_2', 'ik_choice_1': 'ik_choice_1', 'ik_choice_2': 'ik_choice_2'})

			# x:1054 y:116
			OperatableStateMachine.add('Check radius',
										CheckRadius(),
										transitions={'done': 'Joint traj service'},
										autonomy={'done': Autonomy.Off},
										remapping={'robot1_traj': 'cart_traj1', 'robot2_traj': 'cart_traj2', 'robot2_corrected_traj': 'robot2_corrected_traj'})

			# x:418 y:20
			OperatableStateMachine.add('Container',
										_sm_container_5,
										transitions={'finished': 'From_list_to_pose', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'robot1': 'robot1', 'robot2': 'robot2', 'robot_type1': 'robot_type1', 'robot_type2': 'robot_type2', 'pick_pose1': 'pick_pose1', 'pick_pose2': 'pick_pose2', 'last_joints_to_pick1': 'last_joints_to_pick1', 'last_joints_to_pick2': 'last_joints_to_pick2', 'traj_after_pick1': 'traj_after_pick1', 'traj_after_pick2': 'traj_after_pick2', 'last_joints_after_pick1': 'last_joints_after_pick1', 'last_joints_after_pick2': 'last_joints_after_pick2', 'over_place_pose1': 'over_place_pose1', 'over_place_pose2': 'over_place_pose2'})

			# x:812 y:324
			OperatableStateMachine.add('Final traj',
										_sm_final_traj_4,
										transitions={'finished': 'Check collision', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'ik_solution_1': 'ik_solution_1', 'ik_solution_2': 'ik_solution_2', 'ik_choice_1': 'ik_choice_1', 'ik_choice_2': 'ik_choice_2', 'Final_traj_1': 'Final_traj_1', 'Final_traj_2': 'Final_traj_2', 'sphere_last_joints1': 'sphere_last_joints1', 'sphere_last_joints2': 'sphere_last_joints2'})

			# x:635 y:14
			OperatableStateMachine.add('From_list_to_pose',
										_sm_from_list_to_pose_3,
										transitions={'finished': 'Linear interpolator', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'start_pose1': 'over_place_pose1', 'goal_pose1': 'place_pose2', 'start_pose2': 'over_place_pose2', 'goal_pose2': 'place_pose2', 'ee_start_pose1': 'ee_start_pose1', 'ee_goal_pose1': 'ee_goal_pose1', 'ee_start_pose2': 'ee_start_pose2', 'ee_goal_pose2': 'ee_goal_pose2'})

			# x:569 y:568
			OperatableStateMachine.add('Gripper close',
										_sm_gripper_close_2,
										transitions={'finished': 'Move after pick z', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'robot_type1': 'robot_type1', 'robot_type2': 'robot_type2'})

			# x:838 y:137
			OperatableStateMachine.add('Joint traj service',
										_sm_joint_traj_service_1,
										transitions={'finished': 'Check configuration', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'robot_type1': 'robot_type1', 'robot_type2': 'robot_type2', 'cart_traj1': 'cart_traj1', 'robot2_corrected_traj': 'robot2_corrected_traj', 'last_joints1': 'last_joints_to_pick1', 'last_joints2': 'last_joints_to_pick2', 'success_1': 'success_1', 'max_error_1': 'max_error_1', 'ik_solution_1': 'ik_solution_1', 'success_2': 'success_2', 'max_error_2': 'max_error_2', 'ik_solution_2': 'ik_solution_2'})

			# x:911 y:21
			OperatableStateMachine.add('Linear interpolator',
										_sm_linear_interpolator_0,
										transitions={'finished': 'Check radius', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'robot1': 'robot1', 'robot2': 'robot2', 'start_pose1': 'ee_start_pose1', 'start_pose2': 'ee_start_pose2', 'goal_pose1': 'ee_goal_pose1', 'goal_pose2': 'ee_goal_pose2', 'cart_traj1': 'cart_traj1', 'cart_traj2': 'cart_traj2'})

			# x:351 y:575
			OperatableStateMachine.add('Move after pick z',
										JointDualArmMove(),
										transitions={'done': '43'},
										autonomy={'done': Autonomy.Off},
										remapping={'robot_type': 'robot_type1', 'robot_trajectory': 'traj_after_pick1', 'other_trajectory': 'traj_after_pick2'})

			# x:1013 y:467
			OperatableStateMachine.add('Move over pick',
										JointDualArmMove(),
										transitions={'done': 'Move to pick'},
										autonomy={'done': Autonomy.Off},
										remapping={'robot_type': 'robot_type1', 'robot_trajectory': 'traj_over_pick1', 'other_trajectory': 'traj_over_pick2'})

			# x:853 y:539
			OperatableStateMachine.add('Move to pick',
										JointDualArmMove(),
										transitions={'done': 'Gripper close'},
										autonomy={'done': Autonomy.Off},
										remapping={'robot_type': 'robot_type1', 'robot_trajectory': 'traj_to_pick1', 'other_trajectory': 'traj_to_pick2'})

			# x:215 y:48
			OperatableStateMachine.add('Pick _2',
										self.use_behavior(PickSM, 'Pick _2',
											parameters={'delta_z': self.delta_z_over_table, 'priority': self.priority, 'period_over_pick': self.period_over_pick, 'period_to_pick': self.period_to_pick}),
										transitions={'finished': 'Container', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'robot': 'robot2', 'robot_type': 'robot_type2', 'pick_pose': 'pick_pose2', 'last_joints_in': 'last_joints2', 'start_pose': 'start_pose2', 'traj_to_pick': 'traj_to_pick2', 'last_joints_to_pick': 'last_joints_to_pick2', 'traj_over_pick': 'traj_over_pick2'})

			# x:140 y:571
			OperatableStateMachine.add('43',
										JointDualArmMove(),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off},
										remapping={'robot_type': 'robot_type1', 'robot_trajectory': 'Final_traj_1', 'other_trajectory': 'Final_traj_2'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
