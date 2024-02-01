#! /usr/bin/env python3

from flexbe_core import EventState
from flexbe_core.proxy import ProxyServiceCaller
from piltz_single_arm.srv import MovePose, MovePoseRequest

from geometry_msgs.msg import Pose, Point, Quaternion


class RobotMovePose(EventState):
    def __init__(self, srv_name="/panda_1/mover/move_pose", z_offset=0.0, clamp_z=False):
        super(RobotMovePose, self).__init__(outcomes=["done", "failed"], input_keys=["pose"])
        self._z_offset = z_offset
        self._clamp_z = clamp_z
        self._srv_name = srv_name
        self._srv = ProxyServiceCaller({self._srv_name: MovePose})

    def list_to_pose(self, pose):
        return Pose(
            Point(pose[0], pose[1], pose[2] + self._z_offset),
            Quaternion(pose[3], pose[4], pose[5], pose[6]),
        )

    def execute(self, userdata):
        if self.res is not None:
            if self.res.success == False:
                return "failed"
            else:
                return "done"

        return "failed"

    def on_enter(self, userdata):
        req = MovePoseRequest()
        req.pose = self.list_to_pose(userdata.pose)

        if self._clamp_z:
            req.pose.position.z = max(req.pose.position.z, 0.002)
        
        self.res = self._srv.call(self._srv_name, req)

    def on_exit(self, userdata):
        pass
