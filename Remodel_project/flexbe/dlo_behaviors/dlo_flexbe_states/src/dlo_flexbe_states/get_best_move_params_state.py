#! /usr/bin/env python3

from flexbe_core import EventState
from flexbe_core.proxy import ProxyServiceCaller
from dlo_state_planner.srv import ParamsBestMove, ParamsBestMoveRequest

from dlo_state_planner.conversions import multiarray_to_numpy

class ComputeBestMoveParams(EventState):
    def __init__(self, srv_name="dlo_state_planner/params_best_move"):
        super(ComputeBestMoveParams, self).__init__(
            outcomes=["done", "failed"], input_keys=["save_log"], 
            output_keys=["pick_pose", "place_pose", "idx", "dx", "dy", "dtheta", "prediction", "identifier"]
        )
        self._srv_name = srv_name
        self._srv = ProxyServiceCaller({self._srv_name: ParamsBestMove})

    def pose_to_list(self, pose):
        return [
            pose.position.x,
            pose.position.y,
            pose.position.z,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ]

    def execute(self, userdata):
        if self.res is not None:
            self.pick_pose = self.pose_to_list(self.res.pick)
            self.place_pose = self.pose_to_list(self.res.place)
            self.node_idx = self.res.idx
            self.dx = self.res.dx
            self.dy = self.res.dy
            self.dtheta = self.res.dtheta
            self.prediction = multiarray_to_numpy(self.res.prediction)
            self.identifier = self.res.identifier.data

            return "done"
        return "failed"

    def on_enter(self, userdata):
        req = ParamsBestMoveRequest()
        req.save_log = userdata.save_log
        self.res = self._srv.call(self._srv_name, req)

        self.pick_pose = None
        self.place_pose = None
        self.node_idx = None
        self.dx = None
        self.dy = None
        self.dtheta = None
        self.identifier = None
        self.prediction = None

    def on_exit(self, userdata):
        userdata.pick_pose = self.pick_pose
        userdata.place_pose = self.place_pose
        userdata.idx = self.node_idx
        userdata.dx = self.dx
        userdata.dy = self.dy
        userdata.dtheta = self.dtheta
        userdata.prediction = self.prediction
        userdata.identifier = self.identifier
