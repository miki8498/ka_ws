# Collision Checker for 2 Universal Robots
This package contains a collision checker for 2 Universal Robots. The collision checker is based on the [FCL library] implemented in Moveit. It initializes a server that can check collisions for one or 2 robots. Depending on the number of robots, it will receive as input a vector of 6 doubles (robot joints values) or 2 vectors of the same type, one for each robot. If we want to check only 1 robot, we should specify his name. The server will return a boolean value that indicates if the robot is in collision or not. The collision checker is implemented in the CollisionChecker.cpp file.

## Installation
### FMT library
The collision checker uses the [FMT library] for complete a string with the name of the robot specified in the CollisionChecker.cpp file.To install the library, run the following commands:
```sh
    sudo add-apt-repository universe
    sudo apt update
    sudo apt install libfmt-dev
```
and then include it in the target_link_libraries() method in the CMakeLists.txt file.

## Collision Checker cpp
The CollisionChecker.cpp file contains the implementation of the collision checker. The class CollisionChecker is initialized with the the rate, the name of the 1° robot, the name of the 2°, the name of the planning group (*), the coordinates of the database (x,y,z,rx,ry,rz), the coordinates of the warehouse(x,y,z,rx,ry,rz).

*The planning group is the name of the group of joints that we want to control. This group is specified when the moveit folder is created with the Moveit Setup Assistant. The name of the group is specified in the srdf file. 

## Compute Collision cpp
The ComputeCollision.cpp file contains the implementation of the server that receives the joints values and returns a boolean value that indicates if the robot is in collision or not. 
The collision check is done considering possible collision with the environment and with the other robot (or the robot itself). 

The methods of the class ComputeCollision are:
 - addCollisionObjects(): adds the collision objects to the planning scene. Input : name you want to give to the object, the name of the frame you ant to spawn it respect to, the path of the stl file, the cooridinates of the object (x,y,z,rx,ry,rz).
 - initializeRos(): initializes the ros node and serveres. Servers initialized: collision_check, collision_check_opt, distance_collision. The difference between the first two resides in the acm matrix they're using. In the acm matrix all the possible collisions are specified, and you could also decide to remove or add one depending on the situation. The distance_collision server is used to compute the distance between each pair of collision objects in the acm matrix. 
 - parameter(): sets the collision we want to remove form the acm matrices. 
 - fix_joint(): if we want to check collision for just one robot, we can use this method which will fix the joints of the unconsidered robot. Through the move group, we get the current joint state of the robots, so we need to simply get the joints of the robot we are not interested in using the JointModelGroup defined in the Moveit Setup Assistant.
 ```c++
    this->move_group.getCurrentState()->copyJointGroupPositions(state.getJointModelGroup(fmt::format("{}",ur_arm)),this->point_arm.positions);
```
- collision_with_environment(): checks if the robot is in collision with the environment. Input: the state (joints) of the robot. Output: boolean value that indicates if the robot is in collision or not.
- selfCollision(): checks if the robot is in self collision or in collision with the other robot. Input: the state (joints) of the robot. Output: boolean value that indicates if the robot is in collision or not.

After both check, in case of collision we can use 

```c++
    res1.print();
    res2.print();
```

in which res1 and res2 are the responses of the services, to obtain information about the parts in collision. Plus, if we want to visualize the collision, we use

```c++
    visual_tools.publishRobotState(state);
```
Before sending a request to the server, we update the state of the robot to the one we want to check. 


```c++
    state.setJointPositions(fmt::format("{}_shoulder_pan_joint" , robot_1),  &req.joint_config_right[i].data[0]);
    state.setJointPositions(fmt::format("{}_shoulder_lift_joint", robot_1),  &req.joint_config_right[i].data[1]);
    state.setJointPositions(fmt::format("{}_elbow_joint"        , robot_1),  &req.joint_config_right[i].data[2]);
    state.setJointPositions(fmt::format("{}_wrist_1_joint"      , robot_1),  &req.joint_config_right[i].data[3]);
    state.setJointPositions(fmt::format("{}_wrist_2_joint"      , robot_1),  &req.joint_config_right[i].data[4]);
    state.setJointPositions(fmt::format("{}_wrist_3_joint"      , robot_1),  &req.joint_config_right[i].data[5]);
    state.setJointPositions(fmt::format("{}_bl_to_leftFinger"   , robot_1),  &req.gripper_config[0]);   
    state.setJointPositions(fmt::format("{}_leftFinger_to_rightFinger", robot_1), &req.gripper_config[1]);   
    state.setJointPositions(fmt::format("{}_shoulder_pan_joint" , robot_2), &req.joint_config_left[i].data[0]);
    state.setJointPositions(fmt::format("{}_shoulder_lift_joint", robot_2), &req.joint_config_left[i].data[1]);
    state.setJointPositions(fmt::format("{}_elbow_joint"        , robot_2), &req.joint_config_left[i].data[2]);
    state.setJointPositions(fmt::format("{}_wrist_1_joint"      , robot_2), &req.joint_config_left[i].data[3]);
    state.setJointPositions(fmt::format("{}_wrist_2_joint"      , robot_2), &req.joint_config_left[i].data[4]);
    state.setJointPositions(fmt::format("{}_wrist_3_joint"      , robot_2), &req.joint_config_left[i].data[5]);
    state.setJointPositions(fmt::format("{}_bl_to_leftFinger"   , robot_2), &req.gripper_config[0]);   
    state.setJointPositions(fmt::format("{}_leftFinger_to_rightFinger", robot_2), &req.gripper_config[1]);      
    state.update();
```
