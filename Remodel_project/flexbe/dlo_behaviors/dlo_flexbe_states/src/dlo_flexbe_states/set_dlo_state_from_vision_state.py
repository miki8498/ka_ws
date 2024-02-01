#! /usr/bin/env python3

from flexbe_core import EventState
from flexbe_core.proxy import ProxyServiceCaller
from dlo_state_planner.srv import SetDloStateFromVision, SetDloStateFromVisionRequest


class SetDloStateFromVisionS(EventState):
    def __init__(self, srv_name="/dlo_state_planner/set_dlo_state_from_vision"):
        super(SetDloStateFromVisionS, self).__init__(outcomes=["done", "failed"])

        # self._srv_name = srv_name
        # self._srv = ProxyServiceCaller({self._srv_name: SetDloStateFromVision})

    def execute(self, userdata):
        if self.res is not None:
            if self.res.rv == False:
                return "failed"
            else:
                return "done"

        return "failed"

    def on_enter(self, userdata):
        self.res = None
        self.res = self._srv.call(self._srv_name, SetDloStateFromVisionRequest())

    def on_exit(self, userdata):
        pass
