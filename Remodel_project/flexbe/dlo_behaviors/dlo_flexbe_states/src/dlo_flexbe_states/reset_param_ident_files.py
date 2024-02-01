#! /usr/bin/env python3

from flexbe_core import EventState
from flexbe_core.proxy import ProxyServiceCaller
from dlo_state_planner.srv import ResetParamIdent, ResetParamIdentRequest


class ResetParamIdentFiles(EventState):
    def __init__(self, srv_name="/dlo_state_planner/reset_param_ident"):
        super(ResetParamIdentFiles, self).__init__(outcomes=["done", "failed"])
        self._srv_name = srv_name
        self._srv = ProxyServiceCaller({self._srv_name: ResetParamIdent})

    def execute(self, userdata):
        if self.res is not None:
            if self.res.rv == False:
                return "failed"
            else:
                return "done"

        return "failed"

    def on_enter(self, userdata):
        self.res = None
        self.res = self._srv.call(self._srv_name, ResetParamIdentRequest())

    def on_exit(self, userdata):
        pass
