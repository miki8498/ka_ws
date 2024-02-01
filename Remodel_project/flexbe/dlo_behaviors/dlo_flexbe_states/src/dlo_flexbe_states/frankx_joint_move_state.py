#! /usr/bin/env python3

from flexbe_core import EventState
from flexbe_core.proxy import ProxyActionClient
from frankx_ros.msg import FrankxJointTrajectoryAction, FrankxJointTrajectoryActionGoal

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class FrankxJointMove(EventState):
    def __init__(self, topic="/panda_1/frankx_ros/frankx_joint_trajectory_action", z_offset=0.0, clamp_z=False):
        super(FrankxJointMove, self).__init__(outcomes=["done", "failed"], input_keys=["trajectory_points"])
        self._z_offset = z_offset
        self._clamp_z = clamp_z
        self._topic = topic
        self._client = ProxyActionClient({self._topic: FrankxJointTrajectoryAction})

    def execute(self, userdata):
        if self._client.has_result(self._topic):
            result = self._client.get_result(self._topic)
            
            if not result.success:
                return "failed"
            else:
                return "done"

    def on_enter(self, userdata):
        
        traj_msg = JointTrajectory()
        for point in userdata.trajectory_points:
            traj_point = JointTrajectoryPoint()
            traj_point.positions = point
            traj_msg.points.append(traj_point)

        
        
        msg = FrankxJointTrajectoryActionGoal()
        msg.goal.trajectory_goal = traj_msg
        self._client.send_goal(self._topic, msg.goal)

    def on_exit(self, userdata):
        pass
