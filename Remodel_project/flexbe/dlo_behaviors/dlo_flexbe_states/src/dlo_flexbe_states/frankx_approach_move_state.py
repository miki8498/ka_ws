#! /usr/bin/env python3

from flexbe_core import EventState
from flexbe_core.proxy import ProxyActionClient
from frankx_ros.msg import FrankxTrajectoryAction, FrankxTrajectoryActionGoal

from geometry_msgs.msg import Pose, Point, Quaternion, PoseArray
import copy

class FrankxApproachMove(EventState):
    def __init__(self, topic="/panda_1/frankx_ros/frankx_trajectory_action", z_offset=0.0, clamp_z=False, clamp_value=0.0):
        super(FrankxApproachMove, self).__init__(outcomes=["done", "failed"], input_keys=["pose"])
        self._z_offset = z_offset
        self._clamp_z = clamp_z
        self._clamp_value = clamp_value
        self._topic = topic
        self._client = ProxyActionClient({self._topic: FrankxTrajectoryAction})

    def list_to_pose(self, pose):
        return Pose(
            Point(pose[0], pose[1], pose[2]),
            Quaternion(pose[3], pose[4], pose[5], pose[6]),
        )

    def execute(self, userdata):
        if self._client.has_result(self._topic):
            result = self._client.get_result(self._topic)
            
            if not result.success:
                return "failed"
            else:
                return "done"


    def on_enter(self, userdata):

        target_pose = self.list_to_pose(userdata.pose)
        target_pose.position.z += 0.008 # accounting fingers

        pose_top = copy.deepcopy(target_pose)
        pose_top.position.z += 0.3

        pose_approach = copy.deepcopy(target_pose)
        pose_approach.position.z += self._z_offset      

        pose_grasp = copy.deepcopy(target_pose)
        if self._clamp_z:
            pose_grasp.position.z = max(pose_grasp.position.z, self._clamp_value)

        # Create the goal.
        pose_array = PoseArray()
        pose_array.poses.append(pose_top)
        pose_array.poses.append(pose_approach)
        pose_array.poses.append(pose_grasp)

        # Send the goal.
        msg = FrankxTrajectoryActionGoal()
        msg.goal.trajectory_goal = pose_array
        msg.goal.blend_max_distance.data = 0.01

        self._client.send_goal(self._topic, msg.goal)

        

    def on_exit(self, userdata):
        pass
