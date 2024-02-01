#! /usr/bin/env python3

from flexbe_core import EventState
from flexbe_core.proxy import ProxyServiceCaller
from dlo_state_planner.srv import GetDloAllStates, GetDloAllStatesRequest

import os, pickle, rospy
import numpy as np

from dlo_state_planner.conversions import multiarray_to_numpy

import rospkg


class SaveAllPickPlaceMoveDlo(EventState):
    def __init__(self, output_folder_name="output"):
        super(SaveAllPickPlaceMoveDlo, self).__init__(outcomes=["done", "failed"], 
                                                      input_keys=["pick_pose", "place_pose", "idx", "dx", "dy", "dtheta", "identifier"])

        # self._srv_name="/dlo_state_planner/get_all_dlo_states"
        # self._srv = ProxyServiceCaller({self._srv_name: GetDloAllStates})

        # dlo_state_planner_path = rospkg.RosPack().get_path("dlo_state_planner")
        # self.log_path = os.path.join(dlo_state_planner_path, "logs")
        # os.makedirs(self.log_path , exist_ok=True)

    def execute(self, userdata):
        if self.res is not None:
            if self.res.rv == False:
                return "failed"
            else:
                self.dlo_curr = self.res.dlo_curr
                self.dlo_prev = self.res.dlo_prev
                self.dlo_target = self.res.dlo_target
                self.dlo_pred = self.res.dlo_pred

                self.save_on_disk()

                return "done"

        return "failed"

    def on_enter(self, userdata):
        self.res = None

        self.dlo_curr = None
        self.dlo_prev = None
        self.dlo_target = None
        self.dlo_pred = None

        self.pick_pose = userdata.pick_pose
        self.place_pose = userdata.place_pose
        self.idx = userdata.idx
        self.dx = userdata.dx
        self.dy = userdata.dy
        self.dtheta = userdata.dtheta
        self.identifier = userdata.identifier

        self.res = self._srv.call(self._srv_name, GetDloAllStatesRequest())


    def on_exit(self, userdata):
        pass


    def compute_length(self, a):
        return np.sum(np.linalg.norm(a[1:] - a[:-1], axis=1))

    def save_on_disk(self):


        self.dlo_curr = multiarray_to_numpy(self.dlo_curr)
        self.dlo_prev = multiarray_to_numpy(self.dlo_prev)
        self.dlo_target = multiarray_to_numpy(self.dlo_target)
        self.dlo_pred = multiarray_to_numpy(self.dlo_pred)

        file_path = os.path.join(self.log_path, self.identifier + "_behavior_log.pickle")
        with open(file_path, "wb") as f:
            data = {
                "sys_after": self.dlo_curr,
                "sys_before": self.dlo_prev,
                "target": self.dlo_target,
                "pred": self.dlo_pred,
                "pick": self.pick_pose,
                "place": self.place_pose,
                "idx": int(self.idx),
                "dx": float(self.dx),
                "dy": float(self.dy),
                "dtheta": float(self.dtheta),
            }
            pickle.dump(data, f)
    
        rospy.loginfo("Saved log to: {}".format(file_path))
