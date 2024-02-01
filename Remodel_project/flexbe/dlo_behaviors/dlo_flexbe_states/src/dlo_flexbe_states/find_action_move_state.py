#! /usr/bin/env python3

from flexbe_core import EventState
from flexbe_core.proxy import ProxyServiceCaller
from dlo_state_planner.srv import FindAction, FindActionRequest

class FindActionMove(EventState):
    def __init__(self, srv_name="dlo_state_planner/find_action", error_th = 0.02, save_log=True):
        super(FindActionMove, self).__init__(
            outcomes=["done", "perform_action", "failed"], output_keys=["pick_pose", "place_pose", "idx", "dx", "dy", "dtheta", "identifier"]
        )
        # self._error_th = error_th
        # self._save_log = save_log
        # self._srv_name = srv_name
        # self._srv = ProxyServiceCaller({self._srv_name: FindAction})

    def pose_to_list(self, pose):
        return [pose.position.x, pose.position.y, pose.position.z, pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]

    def execute(self, userdata):
        if self.res is not None:

            if self.res.error_below_threshold:
                return "done"
            else:
                self.node_idx = self.res.idx
                self.dx = self.res.dx
                self.dy = self.res.dy
                self.dtheta = self.res.dtheta
                self.identifier = self.res.identifier.data
                self.pick_pose = self.pose_to_list(self.res.pick)
                self.place_pose = self.pose_to_list(self.res.place)
                return "perform_action"
            

    def on_enter(self, userdata):
        self.res = None

        self.pick_pose = None
        self.place_pose = None
        self.node_idx = None
        self.dx = None
        self.dy = None
        self.dtheta = None
        self.identifier = None

        req = FindActionRequest()
        req.error_threshold = self._error_th
        req.save_log = self._save_log
        self.res = self._srv.call(self._srv_name, req)


    def on_exit(self, userdata):
        userdata.identifier = self.identifier
        userdata.idx = self.node_idx
        userdata.dx = self.dx
        userdata.dy = self.dy
        userdata.dtheta = self.dtheta
        userdata.pick_pose = self.pick_pose
        userdata.place_pose = self.place_pose

        print("pick pose: ", self.pick_pose)