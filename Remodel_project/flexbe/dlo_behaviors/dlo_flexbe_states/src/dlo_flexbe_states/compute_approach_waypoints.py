#! /usr/bin/env python3

from flexbe_core import EventState
import copy 

class ComputeApproachWaypoints(EventState):
    def __init__(self, z_offset=0.0, z_clamp=True, clamp_value=0.0):
        super(ComputeApproachWaypoints, self).__init__(outcomes=["done", "failed"], input_keys=["pose"], output_keys=["waypoints"])
        self._z_offset = z_offset
        self._z_clamp = z_clamp
        self._clamp_value = clamp_value

    def execute(self, userdata):
        return "done"

    def on_enter(self, userdata):
        self.waypoints = []

        # approach
        approach_pose = copy.deepcopy(userdata.pose)
        approach_pose[2] += self._z_offset

        # grasp
        grasp_pose = copy.deepcopy(userdata.pose)
        if self._z_clamp:
            grasp_pose[2] = max(grasp_pose[2], self._clamp_value)

        # output
        self.waypoints = [approach_pose, grasp_pose]

    def on_exit(self, userdata):
        userdata.waypoints = self.waypoints
