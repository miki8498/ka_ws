#! /usr/bin/env python3

from flexbe_core import EventState
from flexbe_core.proxy import ProxyActionClient
from frankx_ros.msg import FrankxTrajectoryAction, FrankxTrajectoryActionGoal

from geometry_msgs.msg import Pose, Point, Quaternion, PoseArray
import copy
import numpy as np

PLANE_ANGLE = 3.0
Z_CLAMP = -0.005

class FrankxMove(EventState):
    def __init__(self, topic="/panda_1/frankx_ros/frankx_trajectory_action", z_offset=0.0, clamp_z=False):
        super(FrankxMove, self).__init__(outcomes=["done", "failed"], input_keys=["pose"])
        self._z_offset = z_offset
        self._clamp_z = clamp_z
        self._topic = topic
        self._client = ProxyActionClient({self._topic: FrankxTrajectoryAction})

    def list_to_pose(self, pose):
        return Pose(
            Point(pose[0], pose[1], pose[2] + self._z_offset),
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

        pose1 = copy.deepcopy(target_pose)
        pose1.position.z += self._z_offset      

        if self._clamp_z:
            if pose1.position.x > 0.0:
                z_offset = np.sin(np.deg2rad(PLANE_ANGLE)) * pose1.position.x
            else:
                z_offset = 0.0
            z_limit = Z_CLAMP + z_offset
            print("clamp: ", z_limit, z_offset, pose1.position.z)
            pose1.position.z = max(pose1.position.z, z_limit)

        # Create the goal.
        pose_array = PoseArray()
        pose_array.poses.append(pose1)

            
        # Send the goal.
        msg = FrankxTrajectoryActionGoal()
        msg.goal.trajectory_goal = pose_array
        msg.goal.blend_max_distance.data = 0.0

        self._client.send_goal(self._topic, msg.goal)

        

    def on_exit(self, userdata):
        pass
