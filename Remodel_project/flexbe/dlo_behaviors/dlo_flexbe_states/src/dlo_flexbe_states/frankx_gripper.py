#! /usr/bin/env python3

from flexbe_core import EventState
from flexbe_core.proxy import ProxyActionClient

from control_msgs.msg import GripperCommandAction, GripperCommandActionGoal

class FrankxGripper(EventState):
    def __init__(self, topic="/panda_1/franka_gripper/gripper_action"):
        super(FrankxGripper, self).__init__(outcomes=["done", "failed"], input_keys=["width", "force"])
        self._topic = topic
        self._client = ProxyActionClient({self._topic: GripperCommandAction})

    def execute(self, userdata):
        if self._client.has_result(self._topic):
            result = self._client.get_result(self._topic)
            print(result)
            return "done"

            if not result.reached_goal:
                return "failed"
            else:
                return "done"


    def on_enter(self, userdata):
        msg = GripperCommandActionGoal()
        msg.goal.command.position = userdata.width
        msg.goal.command.max_effort = userdata.force
        self._client.send_goal(self._topic, msg.goal)


    def on_exit(self, userdata):
        pass

