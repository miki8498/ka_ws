#! /usr/bin/env python3

from flexbe_core import EventState
from flexbe_core.proxy import ProxyServiceCaller
from dlo_state_planner.srv import GetDloState, GetDloStateRequest

from dlo_state_planner.conversions import multiarray_to_numpy

class GetDloStateCall(EventState):
    def __init__(self, srv_name="/dlo_state_planner/get_dlo_state"):
        super(GetDloStateCall, self).__init__(outcomes=["done", "failed"], output_keys=["dlo"])

        self._srv_name = srv_name
        self._srv = ProxyServiceCaller({self._srv_name: GetDloState})

    def execute(self, userdata):
        if self.res is not None:
            if self.res.rv == False:
                return "failed"
            else:
                return "done"

        return "failed"

    def on_enter(self, userdata):
        self.res = None
        self.res = self._srv.call(self._srv_name, GetDloStateRequest())

    def on_exit(self, userdata):
        userdata.dlo = multiarray_to_numpy(self.res.dlo)
