#! /usr/bin/env python3

from flexbe_core import EventState
from flexbe_core.proxy import ProxyServiceCaller
from frankx_ros.srv import FrankxHoming, FrankxHomingRequest

class FrankxMoveHome(EventState):
    def __init__(self, srv_name="/panda_1/frankx_ros/homing"):
        super(FrankxMoveHome, self).__init__(outcomes=["done", "failed"])
        self._srv_name = srv_name
        self._client = ProxyServiceCaller({self._srv_name: FrankxHoming})
        

    def execute(self, userdata):
        if self.res is not None:
            if self.res.success == False:
                return "failed"
            else:
                return "done"
            
        return "failed"

    def on_enter(self, userdata):
        self.res = None
        self.res = self._client.call(self._srv_name, FrankxHomingRequest())

    def on_exit(self, userdata):
        pass
