#! /usr/bin/env python3

from flexbe_core import EventState
from flexbe_core.proxy import ProxyServiceCaller
from piltz_single_arm.srv import MoveHome, MoveHomeRequest


class RobotMoveHome(EventState):
    def __init__(self, srv_name="/panda_1/mover/move_home"):
        super(RobotMoveHome, self).__init__(outcomes=["done", "failed"])
        self._srv_name = srv_name
        self._srv = ProxyServiceCaller({self._srv_name: MoveHome})

    def execute(self, userdata):
        if self.res is not None:
            if self.res.success == False:
                return "failed"
            else:
                return "done"

        return "failed"

    def on_enter(self, userdata):
        self.res = self._srv.call(self._srv_name, MoveHomeRequest())

    def on_exit(self, userdata):
        pass
