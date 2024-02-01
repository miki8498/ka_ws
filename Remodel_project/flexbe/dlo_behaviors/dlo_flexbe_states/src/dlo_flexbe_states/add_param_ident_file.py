#! /usr/bin/env python3

from flexbe_core import EventState
from flexbe_core.proxy import ProxyServiceCaller
from dlo_state_planner.srv import AddParamIdentFile, AddParamIdentFileRequest


class AddParamIdent(EventState):
    def __init__(self, srv_name="/dlo_state_planner/add_param_ident_file"):
        super(AddParamIdent, self).__init__(outcomes=["done", "failed"], input_keys=["identifier"])

        # self._srv_name = srv_name
        # self._srv = ProxyServiceCaller({self._srv_name: AddParamIdentFile})

    def execute(self, userdata):
        if self.res is not None:
            if self.res.rv == False:
                return "failed"
            else:
                return "done"

        return "failed"

    def on_enter(self, userdata):
        self.res = None

        req = AddParamIdentFileRequest()
        req.identifier.data = userdata.identifier
        self.res = self._srv.call(self._srv_name, req)

    def on_exit(self, userdata):
        pass
