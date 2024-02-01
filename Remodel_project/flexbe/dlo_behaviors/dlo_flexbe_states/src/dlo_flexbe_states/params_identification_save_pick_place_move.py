#! /usr/bin/env python3

from flexbe_core import EventState
from flexbe_core.proxy import ProxyServiceCaller
from dlo_state_planner.srv import GetDloAllStates, GetDloAllStatesRequest, GetDloStateParams, GetDloStateParamsRequest

import os, pickle, rospy, rospkg
import numpy as np
from termcolor import cprint

from dlo_state_planner.conversions import multiarray_to_numpy


class ParamsIdentSave(EventState):
    def __init__(
        self, srv_name="/dlo_state_planner/get_all_dlo_states", srv_param_name="/dlo_state_planner/get_dlo_params"
    ):
        super(ParamsIdentSave, self).__init__(
            outcomes=["done", "failed"],
            input_keys=["pick_pose", "place_pose", "idx", "dx", "dy", "dtheta", "identifier"],
        )

        # self._srv_name = srv_name
        # self._srv_param_name = srv_param_name
        # self._srv = ProxyServiceCaller({self._srv_name: GetDloAllStates, self._srv_param_name: GetDloStateParams})

        # dlo_state_planner_path = rospkg.RosPack().get_path("dlo_state_planner")
        # self.log_path = os.path.join(dlo_state_planner_path, "logs")
        # os.makedirs(self.log_path, exist_ok=True)

    def execute(self, userdata):
        if self.dlo_curr is not None:
            self.save_on_disk()

            return "done"

        return "failed"

    def on_enter(self, userdata):
        self.res = self._srv.call(self._srv_name, GetDloAllStatesRequest())

        if self.res.rv == True:
            self.dlo_curr = self.res.dlo_curr
            self.dlo_prev = self.res.dlo_prev
        else:
            self.dlo_curr = None
            self.dlo_prev = None

        self.res_params = self._srv.call(self._srv_param_name, GetDloStateParamsRequest())

        if self.res_params.rv == True:
            self.kd = self.res_params.kd
            self.kb = self.res_params.kb
            self.m = self.res_params.m
        else:
            self.kd = None
            self.kb = None
            self.m = None

        self.pick_pose = userdata.pick_pose
        self.place_pose = userdata.place_pose
        self.idx = userdata.idx
        self.dx = userdata.dx
        self.dy = userdata.dy
        self.dtheta = userdata.dtheta
        self.identifier = userdata.identifier
        # self.prediction = userdata.prediction

    def on_exit(self, userdata):
        pass

    def compute_error(self, a, b):
        return np.sum(np.linalg.norm(a - b, axis=1))

    def compute_length(self, a):
        return np.sum(np.linalg.norm(a[1:] - a[:-1], axis=1))

    def save_on_disk(self):
        if self.dlo_curr is None or self.dlo_prev is None:
            print("dlo_curr or dlo_prev is None")
            return

        self.dlo_curr = multiarray_to_numpy(self.dlo_curr)
        self.dlo_prev = multiarray_to_numpy(self.dlo_prev)

        dlo_curr_rev = np.flip(self.dlo_curr, axis=0)
        if self.compute_error(self.dlo_prev, dlo_curr_rev) < self.compute_error(self.dlo_prev, self.dlo_curr):
            self.dlo_curr = dlo_curr_rev

        """
        if len(self.prediction) > 0:
            dlo_pred_rev = np.flip(self.prediction, axis=0)
            if self.compute_error(self.dlo_curr, dlo_pred_rev) < self.compute_error(self.dlo_curr, self.prediction):
                self.prediction = dlo_pred_rev
        """

        dlo_curr_len = self.compute_length(self.dlo_curr)
        dlo_prev_len = self.compute_length(self.dlo_prev)

        if np.fabs(dlo_curr_len - dlo_prev_len) > 0.02:
            cprint("dlo_curr_len and dlo_prev_len are different", "yellow")
            print("dlo_curr_len: ", dlo_curr_len)
            print("dlo_prev_len: ", dlo_prev_len)
            # return

        file_path = os.path.join(self.log_path, self.identifier + "_identification_log.pickle")
        with open(file_path, "wb") as f:
            data = {
                "sys_after": self.dlo_curr,
                "sys_before": self.dlo_prev,
                "pick": self.pick_pose,
                "place": self.place_pose,
                "idx": self.idx,
                "action_dx": self.dx,
                "action_dy": self.dy,
                "action_dtheta": self.dtheta,
                "kd": self.kd,
                "kb": self.kb,
                "m": self.m,
                # "prediction": self.prediction,
            }
            pickle.dump(data, f)

        rospy.loginfo("Saved log file: {}".format(file_path))
