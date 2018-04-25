''' '''
'''
 ISC License

 Copyright (c) 2016-2018, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

 Permission to use, copy, modify, and/or distribute this software for any
 purpose with or without fee is hereby granted, provided that the above
 copyright notice and this permission notice appear in all copies.

 THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

'''

# Import architectural modules
from Basilisk.utilities import SimulationBaseClass, macros
from Basilisk.simulation import sim_model

# Get current file path
import sys, os, inspect
filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

# Import Dynamics and FSW models
sys.path.append(path + '/models')
import BSK_Dynamics
import BSK_Fsw


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
        self.DynModels = BSK_Dynamics.BSKDynamicModels(self)
        self.FSWModels = BSK_Fsw.BSKFswModels(self)

        # Discover interfaces between processes
        self.dyn2FSWInterface.addNewInterface(self.DynamicsProcessName, self.FSWProcessName)
        self.fsw2DynInterface.addNewInterface(self.FSWProcessName, self.DynamicsProcessName)
        self.dynProc.addInterfaceRef(self.dyn2FSWInterface)
        self.fswProc.addInterfaceRef(self.fsw2DynInterface)


class BSKScenario(object):
    def __init__(self, masterSim):
        self.name = "scenario"
        self.masterSim = masterSim

    def configure_initial_conditions(self):
        """
            Developer must override this method in their BSK_Scenario derived subclass.
        """
        pass

    def log_outputs(self):
        """
            Developer must override this method in their BSK_Scenario derived subclass.
        """
        pass

    def pull_outputs(self):
        """
            Developer must override this method in their BSK_Scenario derived subclass.
        """
        pass



