#! /usr/bin/env python3

from flexbe_core import EventState
import copy 

class ComputeDepartureWaypoints(EventState):
    def __init__(self, z_offset=0.0, z_clamp=True, clamp_value=0.0):
        super(ComputeDepartureWaypoints, self).__init__(outcomes=["done", "failed"], input_keys=["pose"], output_keys=["waypoints"])
        self._z_offset = z_offset
        self._z_clamp = z_clamp
        self._clamp_value = clamp_value

    def execute(self, userdata):
        return "done"

    def on_enter(self, userdata):
        self.waypoints = []

        # approach
        release_pose = copy.deepcopy(userdata.pose)
        if self._z_clamp:
            release_pose[2] = max(release_pose[2], self._clamp_value)

        # grasp
        departure_pose = copy.deepcopy(userdata.pose)
        departure_pose[2] += self._z_offset
        
        # output
        self.waypoints = [release_pose, departure_pose]

    def on_exit(self, userdata):
        userdata.waypoints = self.waypoints
