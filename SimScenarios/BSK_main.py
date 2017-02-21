import sys, os, inspect

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
bskName = 'Basilisk'
splitPath = path.split(bskName)
bskPath = splitPath[0] + '/' + bskName + '/'
sys.path.append(bskPath + 'modules')
sys.path.append(bskPath + 'PythonModules')

import SimulationBaseClass
import sim_model
import message_router



import BSK_DKE
import BSK_FSW


class BSKSim(SimulationBaseClass.SimBaseClass):
    def __init__(self):
        # Create a sim module as an empty container
        SimulationBaseClass.SimBaseClass.__init__(self)

        # Create simulation process names
        self.DynamicsProcessName = "DynamicsProcess"
        self.FSWProcessName = "FSWProcess"

        # Create processes
        self.dynProc = self.CreateNewProcess(self.DynamicsProcessName)
        self.fswProc = self.CreateNewProcess(self.FSWProcessName)

        # Define process message interfaces.
        self.dyn2FSWInterface = sim_model.SysInterface()
        self.fsw2DynInterface = sim_model.SysInterface()

        # Crate Dynamics and FSW classes
        self.DynClass = BSK_DKE.DynamicsClass(self)
        self.FSWClass = BSK_FSW.FSWClass(self)

        # Define TCP/IP interfaces
        self.DynamicsServer = False
        self.FSWClient = False

        # Create Synchronization task
        self.dynProc.addTask(self.CreateNewTask("SynchTask", int(5E8)),)
        self.fswProc.addTask(self.CreateNewTask("SynchTask", int(5E8)),)
        self.disableTask("SynchTask")

        if (self.DynamicsServer):
            self.enableTask('SynchTask')
            self.router = message_router.MessageRouter("DynamicsProcess", "FSWProcess")
            self.router.runAsServer = True
            self.router.linkProcesses()
            self.dyn2FSWInterface.addNewInterface(self.router)
            self.AddModelToTask("SynchTask", self.router)
            self.routIn = message_router.MessageRouter(self.router.getConnection())
            self.routIn.linkProcesses()
            self.fsw2DynInterface.addNewInterface(self.routIn)
            self.dynProc.addInterfaceRef(self.fsw2DynInterface)

        elif (self.FSWClient):
            self.enableTask('SynchTask')
            self.router = message_router.MessageRouter()
            self.router.linkProcesses()
            self.dyn2FSWInterface.addNewInterface(self.router)
            self.fswProc.addInterfaceRef(self.dyn2FSWInterface)
            fromString = "FSWProcess"
            toString = "DynamicsProcess"
            connString = fromString + "2" + toString + "Interface"
            self.routOut = message_router.MessageRouter(fromString, toString, connString, self.router.getConnection())
            self.routOut.runAsServer = True
            self.routOut.linkProcesses()
            self.fsw2DynInterface.addNewInterface(self.routOut)
            self.AddModelToTask("SynchTask", self.routOut)

        else:
            self.dyn2FSWInterface.addNewInterface(self.DynamicsProcessName, self.FSWProcessName)
            self.fsw2DynInterface.addNewInterface(self.FSWProcessName, self.DynamicsProcessName)
            self.dynProc.addInterfaceRef(self.dyn2FSWInterface)
            self.fswProc.addInterfaceRef(self.fsw2DynInterface)


#TheBSKSim = BSKSim()
