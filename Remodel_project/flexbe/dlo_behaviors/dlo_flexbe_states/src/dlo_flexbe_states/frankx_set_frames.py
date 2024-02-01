#! /usr/bin/env python3

from flexbe_core import EventState
from flexbe_core.proxy import ProxyServiceCaller
from frankx_ros.srv import SetFrame, SetFrameRequest

class FrankxSetFrames(EventState):
    def __init__(self, 
                srv_ee="/panda_1/frankx_ros/set_end_effector_frame",
                srv_world="/panda_1/frankx_ros/set_world_frame"):
        super(FrankxSetFrames, self).__init__(outcomes=["done", "failed"], input_keys=["ee_frame", "world_frame"])
        self._srv_ee = srv_ee
        self._srv_world = srv_world
        self._srv = ProxyServiceCaller({self._srv_ee: SetFrame, self._srv_world: SetFrame})

    def execute(self, userdata):
        if self.rv_ee is not None and self.rv_world is not None:
            if self.rv_ee.success == False or self.rv_world.success == False:
                return "failed"
            else:
                return "done"

        return "failed"

    def on_enter(self, userdata):
        self.rv_world = None
        self.rv_ee = None

        req = SetFrameRequest()
        req.frame.data = userdata.world_frame
        self.rv_world = self._srv.call(self._srv_world, req)

        req = SetFrameRequest()
        req.frame.data = userdata.ee_frame
        self.rv_ee = self._srv.call(self._srv_ee, req)

    def on_exit(self, userdata):
        pass


