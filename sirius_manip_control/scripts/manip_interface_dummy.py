import time

from manip_interface import ManipInterface, ManipParams
from ik import ManipJointState


class DummyManipInterface(ManipInterface):

    def __init__(self):
        self.jointstate = None
        self._params = None
        self._time_scale = 0

    def get_jointstate(self) -> ManipJointState:
        return self.jointstate

    def set_jointstate(self, jointstate: ManipJointState):
        self.jointstate = jointstate

    def sleep(self, t):
        if self._time_scale > 0:
            time.sleep(t * self._time_scale)

    def get_manip_params(self) -> ManipParams:
        return self._params

    def set_manip_params(self, params: ManipParams):
        self._params = params

    def set_time_scale(self, value):
        assert value >= 0
        self._time_scale = value
