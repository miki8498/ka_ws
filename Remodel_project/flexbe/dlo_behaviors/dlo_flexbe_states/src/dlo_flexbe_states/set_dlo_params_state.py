#! /usr/bin/env python3

from flexbe_core import EventState
from flexbe_core.proxy import ProxyServiceCaller
from dlo_state_planner.srv import SetDloStateParams, SetDloStateParamsRequest


class SetDloParams(EventState):
    def __init__(self, srv_name="/dlo_state_planner/set_dlo_params"):
        super(SetDloParams, self).__init__(outcomes=["done", "failed"], input_keys=["kb", "kd", "m"])

        # self._srv_name = srv_name
        # self._srv = ProxyServiceCaller({self._srv_name: SetDloStateParams})

    def execute(self, userdata):
        if self.res is not None:
            if self.res.rv == False:
                return "failed"
            else:
                return "done"

        return "failed"

    def on_enter(self, userdata):
        self.res = None

        req = SetDloStateParamsRequest()
        req.kb = userdata.kb
        req.kd = userdata.kd
        req.m = userdata.m
        self.res = self._srv.call(self._srv_name, req)

    def on_exit(self, userdata):
        pass
