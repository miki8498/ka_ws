#! /usr/bin/env python3

from flexbe_core import EventState
from flexbe_core.proxy import ProxyServiceCaller
from dlo_state_planner.srv import GetDloAllStates, GetDloAllStatesRequest

from dlo_state_planner.conversions import multiarray_to_numpy
import numpy as np

from termcolor import cprint

class ErrorFromTargetShape(EventState):
    def __init__(self, srv_name="dlo_state_planner/get_all_dlo_states", _error_threshold=0.01):
        super(ErrorFromTargetShape, self).__init__(outcomes=["done", "finished", "failed"])
        # self._srv_name = srv_name
        # self._error_threshold = _error_threshold
        # self._srv = ProxyServiceCaller({self._srv_name: GetDloAllStates})


    # flip the dlo state if the end is closer to the target than the start
    # then compute the error
    def flip_dlo_state(self, dlo_reference, other_dlo):
        if np.linalg.norm(dlo_reference[0,:] - other_dlo[0,:]) < np.linalg.norm(dlo_reference[-1,:] - other_dlo[0,:]):
            return other_dlo
        else:
            return other_dlo[::-1,:]



    def execute(self, userdata):
        if self.res is not None:
            dlo_curr = multiarray_to_numpy(self.res.dlo_curr)
            dlo_prev = multiarray_to_numpy(self.res.dlo_prev)
            dlo_target = multiarray_to_numpy(self.res.dlo_target)

            dlo_curr = self.flip_dlo_state(dlo_target, dlo_curr)
            dlo_prev = self.flip_dlo_state(dlo_target, dlo_prev)

            err_curr = np.linalg.norm(dlo_curr - dlo_target, axis=1)
            err_prev = np.linalg.norm(dlo_prev - dlo_target, axis=1)
          
            # error
            cprint(f"err_curr: mean {np.mean(err_curr)}, max {np.max(err_curr)}", "yellow")
            cprint(f"err_prev: mean {np.mean(err_prev)}, max {np.max(err_prev)}", "yellow")

            if np.mean(err_curr) < self._error_threshold:
                return "finished"
            else:
                return "done"
        
        return "failed"

    def on_enter(self, userdata):
        self.res = self._srv.call(self._srv_name, GetDloAllStatesRequest())

    def on_exit(self, userdata):
        pass