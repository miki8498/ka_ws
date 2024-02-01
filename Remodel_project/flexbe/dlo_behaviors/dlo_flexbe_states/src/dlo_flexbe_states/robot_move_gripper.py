#! /usr/bin/env python3

from flexbe_core import EventState
from flexbe_core.proxy import ProxyServiceCaller
from piltz_single_arm.srv import MoveGripper, MoveGripperRequest


class RobotMoveGripper(EventState):
    def __init__(self, srv_name="/panda_1/mover/move_gripper"):
        super(RobotMoveGripper, self).__init__(outcomes=["done", "failed"], input_keys=["width", "force"])
        self._srv_name = srv_name
        self._srv = ProxyServiceCaller({self._srv_name: MoveGripper})

    def execute(self, userdata):
        if self.res is not None:
            if self.res.success == False:
                return "failed"
            else:
                return "done"

        return "failed"

    def on_enter(self, userdata):
        req = MoveGripperRequest()
        req.width = userdata.width
        req.force = userdata.force
        self.res = self._srv.call(self._srv_name, req)

    def on_exit(self, userdata):
        pass
