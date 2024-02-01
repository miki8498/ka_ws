#! /usr/bin/env python3

from flexbe_core import EventState
from flexbe_core.proxy import ProxyActionClient
from frankx_ros.msg import PlanPoseTrajectoryAction, PlanPoseTrajectoryActionGoal

from geometry_msgs.msg import Pose, Point, Quaternion
import copy

class MoveitPosePlanMove(EventState):
    def __init__(self, topic="plan_posearray_trajectory_action", group_name="panda_arm_1"):
        super(MoveitPosePlanMove, self).__init__(outcomes=["done", "failed"], input_keys=["pose"], output_keys=["trajectory_points"])
        self._group_name = group_name
        self._topic = topic
        self._client = ProxyActionClient({self._topic: PlanPoseTrajectoryAction})

    def list_to_pose(self, pose):
        return Pose(
            Point(pose[0], pose[1], pose[2]),
            Quaternion(pose[3], pose[4], pose[5], pose[6]),
        )

    def execute(self, userdata):
        if self._client.has_result(self._topic):
            result = self._client.get_result(self._topic)
            self.trajectory_points = []
            for point in result.trajectory_goal.points:
                print(point)
                self.trajectory_points.append(copy.deepcopy(point.positions))

            if not result.success:
                return "failed"
            else:
                return "done"


    def on_enter(self, userdata):
        self.trajectory_points = []

        ################
        # PLANNER
        ################
        msg_plan = PlanPoseTrajectoryActionGoal()
        msg_plan.goal.group_name.data = self._group_name
        msg_plan.goal.goal = self.list_to_pose(w)

        self._client.send_goal(self._topic, msg_plan.goal)

    def on_exit(self, userdata):
        userdata.trajectory_points = self.trajectory_points
        pass
