#! /usr/bin/env python3

from flexbe_core import EventState
from flexbe_core.proxy import ProxyServiceCaller
from piltz_single_arm.srv import MoveJoints, MoveJointsRequest


class RobotMoveJoints(EventState):
    def __init__(self, srv_name="/panda_1/mover/move_joint"):
        super(RobotMoveJoints, self).__init__(outcomes=["done", "failed"], input_keys=["joint_values"])
        self._srv_name = srv_name
        self._srv = ProxyServiceCaller({self._srv_name: MoveJoints})

    def execute(self, userdata):
        if self.res is not None:
            if self.res.success == False:
                return "failed"
            else:
                return "done"

        return "failed"

    def on_enter(self, userdata):
        req = MoveJointsRequest()
        req.joint_values = userdata.joint_values
        self.res = self._srv.call(self._srv_name, req)

    def on_exit(self, userdata):
        pass
