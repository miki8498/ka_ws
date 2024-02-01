#! /usr/bin/env python3

import rospy
from flexbe_core import EventState
from flexbe_core.proxy import ProxyServiceCaller
from dlo_state_planner.srv import FindParams, FindParamsRequest

class FindDloParams(EventState):
    def __init__(self, srv_name="dlo_state_planner/find_params"):
        super(FindDloParams, self).__init__(
            outcomes=["done", "failed"], input_keys=["indices", "mass"], output_keys=["kd", "kb"]
        )
        # self._srv_name = srv_name
        # self._srv = ProxyServiceCaller({self._srv_name: FindParams})

    def execute(self, userdata):
        if self.res is not None:
            if self.res.rv == False:
                return "failed"

            self.kd = self.res.kd
            self.kb = self.res.kb
            rospy.loginfo(f"found parameters: KD {self.kd:.3f}, KB {self.kb:.3f}")
            return "done"
            

    def on_enter(self, userdata):
        self.res = None

        self.kd = None
        self.kb = None

        req = FindParamsRequest()
        req.indices = userdata.indices
        req.mass = userdata.mass
        self.res = self._srv.call(self._srv_name, req)


    def on_exit(self, userdata):
        userdata.kd = self.kd
        userdata.kb = self.kb

    