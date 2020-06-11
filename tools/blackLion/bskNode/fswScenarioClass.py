import sys, os, inspect
filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
sys.path.append(path + '/../../fswAuto/fswExamples/')
from desktopFswSim import DesktopFSW
import bskRouter


class fswScenario:
    def __init__(self, theSimClass):
        self.theSim = theSimClass()
        # Create Router process on top of the Sim
        fswRoutingRate = int(5E8)
        self.theSim.RouterProcessName = "RouterProcess"
        self.theSim.routerProc = self.theSim.CreateNewPythonProcess(self.theSim.RouterProcessName, 10)
        fswTarget = bskRouter.TargetProcess_class(fswRoutingRate, self.theSim.fswProc.Name)
        self.theSim.RouterClass = bskRouter.BSK_router_class(self.theSim, fswTarget)


if __name__ == "__main__":
    scenario = fswScenario(DesktopFSW)

