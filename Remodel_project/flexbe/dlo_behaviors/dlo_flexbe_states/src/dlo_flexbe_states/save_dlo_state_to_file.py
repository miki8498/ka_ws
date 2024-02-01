#! /usr/bin/env python3

from flexbe_core import EventState
import rospkg, os
import numpy as np



class SaveDloStateToFile(EventState):
    def __init__(self, file_name=""):
        super(SaveDloStateToFile, self).__init__(outcomes=["done", "failed"], input_keys=["dlo"])

        pkg_path = rospkg.RosPack().get_path('dlo_state_planner')
        self._states_path = os.path.join(pkg_path, "dlo_states")
        self._file_name = file_name

    def execute(self, userdata):
        return "done"

    def on_enter(self, userdata):
        self.dlo = userdata.dlo

    def on_exit(self, userdata):
        # Save to file
        path = os.path.join(self._states_path, self._file_name)
        np.savetxt(path, self.dlo)

