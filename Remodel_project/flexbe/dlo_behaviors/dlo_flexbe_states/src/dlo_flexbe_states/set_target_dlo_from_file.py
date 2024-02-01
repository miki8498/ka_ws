#! /usr/bin/env python3

from flexbe_core import EventState
from flexbe_core.proxy import ProxyServiceCaller
from dlo_state_planner.srv import SetDloState, SetDloStateRequest

import rospkg, os
import numpy as np
from dlo_state_planner.conversions import numpy_to_multiarray



class SetTargetDloFromFile(EventState):
    def __init__(self, srv_name="/dlo_state_planner/set_target_dlo_state", file_name=""):
        super(SetTargetDloFromFile, self).__init__(outcomes=["done", "failed"])

        # self._srv_name = srv_name
        # self._srv = ProxyServiceCaller({self._srv_name: SetDloState})

        # pkg_path = rospkg.RosPack().get_path('dlo_state_planner')
        # self._states_path = os.path.join(pkg_path, "dlo_states")

        # self._file_name = file_name

    def execute(self, userdata):
        if self.res is not None:
            if self.res.rv == False:
                return "failed"
            else:
                return "done"

        return "failed"

    def on_enter(self, userdata):

        # Read the file
        path = os.path.join(self._states_path, self._file_name)
        dlo = np.loadtxt(path)

        dlo_msg = numpy_to_multiarray(dlo)


        # Create the request
        req = SetDloStateRequest()
        req.dlo = dlo_msg

        self.res = None
        self.res = self._srv.call(self._srv_name, req)

    def on_exit(self, userdata):
        pass
