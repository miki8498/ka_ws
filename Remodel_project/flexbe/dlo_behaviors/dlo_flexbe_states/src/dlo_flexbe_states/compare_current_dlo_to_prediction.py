#! /usr/bin/env python3

from flexbe_core import EventState
from flexbe_core.proxy import ProxyServiceCaller
from dlo_state_planner.srv import GetDloState, GetDloStateRequest

from dlo_state_planner.conversions import multiarray_to_numpy
import numpy as np

class CompareDLOtoPred(EventState):
    def __init__(self, srv_name="dlo_state_planner/get_dlo_state"):
        super(CompareDLOtoPred, self).__init__(
            outcomes=["done", "failed"], input_keys=["prediction"], 
        )
        self._srv_name = srv_name
        self._srv = ProxyServiceCaller({self._srv_name: GetDloState})


    def execute(self, userdata):
        if self.res is not None:
            dlo_state = multiarray_to_numpy(self.res.dlo)

            dist = np.linalg.norm(dlo_state[0,:] - self.prediction[0,:])
            dist_flip = np.linalg.norm(dlo_state[-1,:] - self.prediction[0,:])

            if dist_flip < dist:
                dlo_state = dlo_state[::-1,:]
                            
            # error
            err = np.linalg.norm(dlo_state - self.prediction, axis=1)
            print(f"Error: mean {np.mean(err)}, max {np.max(err)}")

            return "done"
        
        return "failed"

    def on_enter(self, userdata):
        self.res = self._srv.call(self._srv_name, GetDloStateRequest())
        self.prediction = np.array(userdata.prediction)

    def on_exit(self, userdata):
        pass