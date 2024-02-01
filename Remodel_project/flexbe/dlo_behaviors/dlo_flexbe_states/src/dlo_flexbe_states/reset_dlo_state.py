#! /usr/bin/env python3

from flexbe_core import EventState
from flexbe_core.proxy import ProxyServiceCaller
from dlo_state_planner.srv import ResetDloState, ResetDloStateRequest


class ResetDloStateAndTarget(EventState):
    def __init__(self, srv_name="/dlo_state_planner/reset_dlo_state", reset_state=True, reset_target=True, reset_pred=True):
        super(ResetDloStateAndTarget, self).__init__(outcomes=["done", "failed"])
        # self.reset_state = reset_state
        # self.reset_target = reset_target
        # self.reset_pred = reset_pred
        # self._srv_name = srv_name
        # self._srv = ProxyServiceCaller({self._srv_name: ResetDloState})

    def execute(self, userdata):
        if self.res is not None:
            if self.res.rv == False:
                return "failed"
            else:
                return "done"

        return "failed"

    def on_enter(self, userdata):
        self.res = None

        req = ResetDloStateRequest()
        req.reset_state = self.reset_state
        req.reset_target = self.reset_target
        req.reset_pred = self.reset_pred
        self.res = self._srv.call(self._srv_name, req)

    def on_exit(self, userdata):
        pass
