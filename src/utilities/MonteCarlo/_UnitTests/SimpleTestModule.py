
from Basilisk.architecture import sysModel
class SimpleTestModule(sysModel.SysModel):

    def __init__(self):
        super(SimpleTestModule, self).__init__()

        self._ticker = 0
        self.moduleTag = "testModule"

    def UpdateState(self, current_sim_nanos):
        self._ticker += 1

    def GetTicker(self):
        return self._ticker

    def Reset(self, current_sim_nanos):
        pass
