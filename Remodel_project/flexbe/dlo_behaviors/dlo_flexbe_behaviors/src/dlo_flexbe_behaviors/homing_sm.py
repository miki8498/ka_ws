#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from dlo_flexbe_behaviors.homing_single_arm_sm import Homing_single_armSM
from ur_move_flexbe_states.Check_double_collision_service import CheckDoubleCollision
from ur_move_flexbe_states.Gripper_motion_pub import GripperMotion
from ur_move_flexbe_states.Joint_move_dual_arm_states import JointDualArmMove
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Thu Jan 25 2024
@author: mk
'''
class HomingSM(Behavior):
	'''
	go to homing pose double robot
	'''


	def __init__(self):
		super(HomingSM, self).__init__()
		self.name = 'Homing'

		# parameters of this behavior
		self.add_parameter('gripper_config', False)
		self.add_parameter('simulation', False)

		# references to used behaviors
		self.add_behavior(Homing_single_armSM, 'Homing_single_left')
		self.add_behavior(Homing_single_armSM, 'Homing_single_right')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:553 y:455, x:130 y:365
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], output_keys=['homing_left', 'homing_right', 'last_joints_left', 'last_joints_right'])
		_state_machine.userdata.robot_left = 'UR_left'
		_state_machine.userdata.robot_right = 'UR_right'
		_state_machine.userdata.type_right = 'ur_right'
		_state_machine.userdata.type_left = 'ur_left'
		_state_machine.userdata.homing_left = []
		_state_machine.userdata.last_joints_left = []
		_state_machine.userdata.homing_right = []
		_state_machine.userdata.last_joints_right = []

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]

		# x:270 y:265, x:130 y:365, x:230 y:365
		_sm_gripper_open_0 = ConcurrencyContainer(outcomes=['finished', 'failed'], input_keys=['type_right', 'type_left'], conditions=[
										('finished', [('Gripper open right', 'done'), ('Gripper open left', 'done')])
										])

		with _sm_gripper_open_0:
			# x:135 y:93
			OperatableStateMachine.add('Gripper open right',
										GripperMotion(open=self.gripper_config, simulation=self.simulation),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off},
										remapping={'id_type': 'type_right'})

			# x:306 y:91
			OperatableStateMachine.add('Gripper open left',
										GripperMotion(open=self.gripper_config, simulation=self.simulation),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off},
										remapping={'id_type': 'type_left'})



		with _state_machine:
			# x:68 y:42
			OperatableStateMachine.add('Gripper open',
										_sm_gripper_open_0,
										transitions={'finished': 'Homing_single_right', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'type_right': 'type_right', 'type_left': 'type_left'})

			# x:487 y:46
			OperatableStateMachine.add('Homing_single_left',
										self.use_behavior(Homing_single_armSM, 'Homing_single_left',
											parameters={'priority': 3}),
										transitions={'finished': 'Check collision', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'robot': 'robot_left', 'robot_type': 'type_left', 'joint_trajectory': 'joint_trajectory_left', 'last_joints_out': 'last_joints_left', 'homing': 'homing_left'})

			# x:244 y:42
			OperatableStateMachine.add('Homing_single_right',
										self.use_behavior(Homing_single_armSM, 'Homing_single_right',
											parameters={'priority': 3}),
										transitions={'finished': 'Homing_single_left', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'robot': 'robot_right', 'robot_type': 'type_right', 'joint_trajectory': 'joint_trajectory_right', 'last_joints_out': 'last_joints_right', 'homing': 'homing_right'})

			# x:652 y:328
			OperatableStateMachine.add('Move ',
										JointDualArmMove(),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off},
										remapping={'robot_type': 'type_right', 'robot_trajectory': 'joint_trajectory_right', 'other_trajectory': 'joint_trajectory_left'})

			# x:677 y:187
			OperatableStateMachine.add('Check collision',
										CheckDoubleCollision(open=self.gripper_config),
										transitions={'done': 'Move ', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'robot': 'robot_right', 'trajectory_robot': 'joint_trajectory_right', 'trajectory_other': 'joint_trajectory_left', 'success': 'success'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
