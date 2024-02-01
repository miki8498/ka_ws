#! /usr/bin/env python3

from flexbe_core import EventState
from flexbe_core.proxy import ProxyServiceCaller
from dlo_state_planner.srv import GetBendingMove, GetBendingMoveRequest


class ComputeBendingMove(EventState):
    def __init__(self, srv_name="dlo_state_planner/params_random_bending_move"):
        super(ComputeBendingMove, self).__init__(
            outcomes=["done", "failed"], input_keys=["max_range", "min_range", "dtheta_range"], output_keys=["pick_pose", "place_pose", "idx", "dx", "dy", "dtheta", "identifier"]
        )
        self._srv_name = srv_name
        self._srv = ProxyServiceCaller({self._srv_name: GetBendingMove})

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
            self.identifier = self.res.identifier.data

            return "done"
        return "failed"

    def on_enter(self, userdata):
        req = GetBendingMoveRequest()
        req.max_range = userdata.max_range
        req.min_range = userdata.min_range
        req.dtheta_range = userdata.dtheta_range
        self.res = self._srv.call(self._srv_name, req)

        self.pick_pose = None
        self.place_pose = None
        self.node_idx = None
        self.dx = None
        self.dy = None
        self.dtheta = None
        self.identifier = None

    def on_exit(self, userdata):
        userdata.pick_pose = self.pick_pose
        userdata.place_pose = self.place_pose
        userdata.idx = self.node_idx
        userdata.dx = self.dx
        userdata.dy = self.dy
        userdata.dtheta = self.dtheta
        userdata.identifier = self.identifier
