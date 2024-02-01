# FLEXBE 
FlexBE consists of the following ROS packages:

- flexbe_behavior_engine: ROS stack consisting of all packages to define and run behaviors.

- flexbe_app: ROS package to provide the user interface for editing and monitoring behaviors.

Refer to the respective sub-pages for more detailed information.

To create a repository for your own behaviors, you should use the terminal command:
    
    ```bash
    $ rosrun flexbe_widget create_repo <name_of_folder>
    ```

Creating a new behavior in flexbe, you can decide in which repository store that behavior. All behavior informations can be found in the  flexbe_behaviors folder cointained in the repository created, in particular in the manifest and src folders. 

The whole cabling task, is contained in the 'Clip task final version' Behavior.The task starts by a call to the database wires through the state 'DatabaseService'. 

## LAUNCH FILES AND ROSRUN FILES 
Before starting the whole task, there are some files to launch:
 ```bash
    $ roslaunch ur10_sim ur10_spawn.launch
    $ roslaunch flexbe_app flexbe_full.launch
    $ rosrun collision collision
    $ rosrun uc1_c_spline uc1_c_spline_main
    $ rosrun ur_kinematics_robot ur_kinematics_main  
    $ rosrun opt_follower Optimization_follower.py
```

## DatabaseService
This state uses a method in which a service client calls the database and returns a message with a list of waypoints referred to a certain connection (the one we want). In the database_wires folder, there is a certain number of connections specified. 
In this state, the user can decide which connection consider, by setting the parameter 'connection' (int) to the desired value.
To obtain the waypoints to follow for the cabling route, we use the function 'get_path' in which there's a call to the topic 'gearbox_handler/reset_to_connection_list' which resets the list of connection to the one we are interested in, and then to '/gearbox_handler/group_connection_provider' to obtain the waypoints whose rotation should be brought respect to the world. 
From the waypoints 'origin' and 'destination' we can know what type of component we will have to connect (normal or terminal block) from the direction of the quaternions. 

```python
type_cable_origin      = self.define_component(start)
type_cable_destination = self.define_component(end)
```

Then we compute the cable length and an apprximate starting position of the follower 
```python
self.Follower.first_point_follower(self.original_path)
```
If we're working with a terminal block, we should also compute a regrasp point using spherical coordinates. 
Through the function 

```python
self.Db.check_traj(self.original_path, self.type_component, inclination, self.first_point)
```
we extend the trajectory adding points to avoid collisions with channels. 
As output it returns:

- db_path: geometry_msgs/Pose[] , extended waypoints
- type_component: (int) type of component of the starting point
- original_path: geometry_msgs/Pose[] original waypoints
- cable_length: (float) length of the cable
- regrasp_point: (list) point where the robot has to regrasp the cable
- type_comp_destination: (int) type of component of the finish point

## ActionOnTypeComponent 
Setting some values like start follower and primary pose, goal follower and primary pose, last joints clip follower and primary, theta, and computation of
alpha and z point in case of regrasp. In particular, if the cable is too long, we need to consider the necessity of an higher alpha angle and theta angle so that the follower robot can reach a confortbale position. 

## ChoiceScrewTerminalBlock
Determine which type of action to perform (screwdriver, terminal_block, screw_terminal) based on the type of components to connect.

## Clip_task_with_Sending
Behavior creating an approximate (terminal block) or final (normal compoenent) cabling trajectory for the primary robot. It returns which one of the two robots is the primary and which the follower. In this behavior is also performed the grasp of the cable from the warehouse taking into account which robot will go to which component (depending on the type of component the grasping orientation will be different). Here there's the explanation for the states of this behavior, while the other behaviors will be explained separately.

### Current pose right 
We obtain the current joint pose of the right robot so that the interpolation can consider from which configuration the robot will start.

### Open gripper container
TO be sure the grippers are open, we do a call to the respective services (ex.'{}/right_robotiq_hand_e'.format(type_right)' of type CommandRobotiqGripperAction) to open them. The minimun value for the pose is 0 (close) and the maximun is 0.45 (open).

### Clip position container 
State to obtain the position of the clip frame which will be the pose the robot should reach to take the cable through a tf listener. It will return a list with 3 different lists: 1 with the approach to the clip, 1 with the grasp pose, 1 with the retreat pose to remove the cable from the clip.

## Primary_for_cableClip_grasp
This behavior is used to compute a first shortened trajectory for the primary robot, chose which robot will be the primary and which the follower and grasp the cable from the warehouse taking into account which robot will go to which component (depending on the type of component the grasping orientation will be different). Here there's the explanation for the states of this behavior, while the other behaviors will be explained separately.

### Clip approach primary/follower (ChoiceForApproachClip)
The state assigns to the specified robot the correct clip positions and depending on the type of the starting component the correct approach. 

### Change orientation (HowToReachClip)
This state is used to change the orientation of the robot to reach the clip if the component to consider is a normal one. Through PyKDL library, it returns the correct orientation to reach the clip.

### Assign clip (AssignClip)
Since now we know which clip poses the primary and the follower robots should reach, we can assign them to the correct robot (left or right one).

## Primary Trajectory behavior
This behavior is used to compute a first shortened trajectory for the primary robot and chose which robot will perform it. Here there's the explanation for the states of this behavior:

- Cartesian trajectory (CartPrimaryTrajectory): compute the primary cartesian trajectory for the robot considering the type of the origin component and his height. It returns the trajectory (list(list)).
-  Joint service container (JointTrajService): it calls the service to compute the joint trajectory from the cartesian one. It returns the joint trajectory (list(list)). The service is 'UrInverseKinematics' and the name of topic 'ur_inverse_k'. 
Example of message:
```python

    req_ik    = UrInverseKinematicsRequest()
      
    poses             = [Pose() for i in range(len(self.trajectory))]
    self.ik_solution  = np.zeros((len(poses),4,6))
    #Conversion of each cartesian point from tcp_world to ee_base and from list to Pose
    for i in tqdm(range(len(self.trajectory))):
        ee_point  = self.kin.whole_kinematics(self.trajectory[i])    
        poses[i]  = ee_point

    req_ik.reference_pose = copy.deepcopy(poses)
    req_ik.desired_config = self.desired_config
    req_ik.ur_type        = type_
    req_ik.check_q6       = self.check_q6
    req_ik.verbose        = True
    req_ik.last_joints    = self.last_joints_traj
    
    self.res = self._srv(req_ik)
```
where desired_config is a list containing the configuration of the robot we want to consider (in our case on the 8 possible, we consider only the one with the elbow up).
- Check configuration container (CheckConfiguration): the state check the whole joint trajectory configuration for configuration, to be sure that thre are no singularity points. The boolean parameter 'choice_primary' is used to distinguish between the case in which we should chose which robot will perform the cabling trajectory and being the primary (since in this case we make a comparison between the right and left possible configurations), and the normal movements. 
The reason behind this is that the robot cannot change his configuration during the whole process otherwise it risks to break the cable or something around him. So the configuration it should keep is the one with the highest priority (elbow up).
- Choice primary (ChoicePrimary): this state is used to chose which robot will be the primary and which the follower. It compares the two chosen configurations and choses the one with the highest priority (elbow up). It returns the names of the primary and follower robots and the configuration to use. 

## Clip_task_ur_with_pc behavior
This is a generic behavior used to compute the trajectories each robot should perform to reach the clip and take the cable and make the robots move.
First of all we take the current joint pose of the robot to have our last_joints, the tcp pose respect to the world to know our starting point, and pose the robot should reach to take distance from the warehouse. Then there's a series of 'Intepolator start and goal' behaviors to compute the trajectories to perform, and after th whole computation, the real movement. Here there's the explanation for the states of this behavior:

- Tcp pose ur (TcpPosition): it returns the current tcp pose of the robot respect to the world. The tm_tool parameter is used for the techman robot to choose between two different tools (screwdriver and terminal block opener).
- SendingTrajectory : it sends the trajectory to the robot to perform the movement. The service is 'TrajectoryPublisherAction' and the name of topic '/receiving_trajectory'.

## Interpolator start and goal
This is a generic behavior that can be used to compute the interpolated trajectory between two poses. After creating the cartesian trajectory, it converts it in joint, check the configuration and if there would be possible collisions performing it. Here there's the explanation for the states of this behavior:

- TcpW_to_eeBase: it converts the tcp pose respect to the world in the ee_base pose respect to the base. It returns the ee_base pose.
- InterpolateActionState : it computes the cartesian linear interpolated trajectory between the start and goal pose. The service is 'interpolate' and the name of topic 'interpolate_linear'. Then it converts it in joint trajectory using the service 'UrInverseKinematics' and the name of topic 'ur_inverse_k'.
- FinalJointTrajectory: creation of the final joint trajectory considering the configuration chosen for the robot. It returns the final joint trajectory.
- CheckDoubleCollision: check if performing a certain movement, the robots will collide with each other or with the environment. It returns a boolean value. The service is 'CollisionServer_' and the name of topic 'collision_check'.

## Terminal_block no move behavior
Generic behavior used to compute techman trajectories to perform the opening and closing of the terminal blocks. 
Default parameter of the behavior: tm_tool , we can define out of the behavior which tool we will consider for those computations. False for terminal blocks, True for screwdriver. The input is simply the pose of the terminal block we want to open/close. 

## Interpolator start and goal techman
The only difference with the other interpolator behavior is the lack of collision check since the techman robot will perform simple movements that a priori won't collide with anything.

## Terminal_block behavior
Compute the trajectory to open/close the terminal block. Here there's the explanation for the states of this behavior:

- PosScrew : 

## Terminal blocksapproach behavior

- EndEffectorRobot:

## Terminal blocks inverse kinematics behavior

- InverseKinService:

## Screwdriver behavior
Generic behavior used to compute techman trajectories to perform the opening and closing of normal compoenents using a screwdriver. To start the closing, we use a state called 'StartScrewdriver' which sends a message to the topic '/kolver/start_screw' to start the screw. To stop it, we keep reading the screwdriver_state msg.motor_active: when it becomes False, it means the motos has stopped working and we can go on. Every time the screwdriver is activated, we need to reset it after his work throught a Trigger msg to the topic '/kolver/reset'.

## Type_9 no move behavior
Case in which the starting component is a terminal block. So this behavior is used to compute the whole cabling task trajectories that will simply be sent later to the Move behavior. Here there's the explanation for the states of this behavior:

- PassingParam: define primary and follower robot and their type, it's only a passage of parameters. 
- NearDatabase: Define poses for primary and follower to move from the default pose to the point in which the cable should be connected as start. For terminal block components, since the hole is on the component, we first move over the component with a certain height and then insert the cable. While poses for the primary robot are computed based on the inserting point, for the follower, we only define [alpha, theta, roll, pitch, yaw] and let the sphere interpolator compute its trajectory, since we need to consider the presence of the cable between the two robots during every movement. 
- CablingComputation: compute the cable length from the regrasping point to the follower point. 
- SS:
- SplittingTraj: split the cabling primary trajectory exploiting the indexes previously found as before regrasp and after regrasp.
- PrimaryToDefault: define default poses for the primary robot to left the cable and make space to the follower robot
- FollowerToDb: Define poses for the follower robot to move from the last pose of the optimizer to the point in which the primary robot left the cable after the cabling path. First we lift the follower robot to a certain height without changing its orientation to avoid collisions with the channels, then we move over the channel were the primary left the cable, and then keeping being on the same spot, we change the orientation to the one useful for the inserting.

## Sphere Interpolator behavior

- FromListToPose: state to convert a list of poses in a geometry_msgs.msg/Pose type. It returns the pose as geometry_msgs.msg/Pose type
- SphereInterpolator: taking the length of the cable as the radius of a sphere, it computes the position of the follower robot, based on the position of the primary one. 

## Final_cabling_trajectories_primary behavior

- RegraspPoint: creation of the new waypoints to follow for the primary trajectory considering the new point of regrasp. In particular there is also the compuation of the centimeters of cable to take for the follower robot after the regrasp of the cable, since the regrasping point doesn't corresponf to the point in the channel. 
- IndexFinding: find the indexes in which there happen the rotation of the gripper to perform the regrasping, in order to split the primary cabling trajectory in 2 parts (before and after the regrasp). It returns the indexes. 

## Follower_trajectory behavior

- DefaultPoseFollower: in order for the optimizer to properly work, we should be sure that, performing the cabling trajectory, the primary robot never collides. To do so, we define some poses for the follower robot out of the primary workspace. In this way we are certain to be checking only collsiion with the database from the primary robot. 
- OptimizerService:

## Fixed primary

- FixPrimary: computation of the follower trajectory when the pirmary robot pose is fixed and we need to consider the presence of the cable between the two robots during every movement.

## Final_cabling_trajectories behavior

## Destination trajectory

- FollowerToDestination: compute the cartesian trajectory to reach the finish component 

## Type_1 no move behavior

- NearDatabaseType1: Define poses for primary and follower to move from the default pose to the point in which the cable should be connected as start. For normal components, since the hole is on the side of the component, we first move over the component with a certain shift, then at the height of the hole with previous shift, and then insert the cable. While poses for the primary robot are computed based on the inserting point, for the follower, we only define [alpha, theta, roll, pitch, yaw] and let the sphere interpolator compute its trajectory, since we need to consider the presence of the cable between the two robots during every movement. 
- OpeningPoint: splitting the primary trajectory so that the primary robot can perform the cabling and, reached the last point in the channels, can open the fingers, let the cable in the channel, and then perform the last part of the cabling trajectory (the one in which it exits from the channel)

## Follower_trajectory_type1 behavior

- OptimizerServiceType1:
