#
#  ISC License
#
#  Copyright (c) 2021, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
#
#  Permission to use, copy, modify, and/or distribute this software for any
#  purpose with or without fee is hereby granted, provided that the above
#  copyright notice and this permission notice appear in all copies.
#
#  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
#  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
#  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
#  ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
#  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
#  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
#  OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
#

# Get current file path
import inspect
import os
import sys

from Basilisk import __path__
from Basilisk.fswAlgorithms import formationBarycenter
# Import architectural modules
from Basilisk.utilities import SimulationBaseClass, macros as mc

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
bskPath = __path__[0]

# Import Dynamics and FSW models
sys.path.append(path + '/models')


class BSKSim(SimulationBaseClass.SimBaseClass):
    """
    Main bskSim simulation class

    Args:
        numberSpacecraft (int): number of spacecraft
        relativeNavigation (bool): whether the chief is the barycenter of the spacecraft formation
        fswRate (float): [s] FSW update rate
        dynRate (float): [s] dynamics update rate
        envRate (float): [s] environment update rate
        relNavRate (float): [s] relative navigation update rate

    """

    def __init__(self, numberSpacecraft, relativeNavigation=False, fswRate=0.1, dynRate=0.1, envRate=0.1, relNavRate=0.1):
        self.dynRate = dynRate
        self.fswRate = fswRate
        self.envRate = envRate
        self.relNavRate = relNavRate
        self.numberSpacecraft = numberSpacecraft

        # Create a sim module as an empty container
        SimulationBaseClass.SimBaseClass.__init__(self)
        self.SetProgressBar(True)

        self.EnvModel = []
        self.DynModels = []
        self.FSWModels = []
        self.EnvProcessName = None
        self.DynamicsProcessName = []
        self.FSWProcessName = []
        self.envProc = None
        self.dynProc = []
        self.fswProc = []

        self.environment_added = False
        self.dynamics_added = False
        self.fsw_added = False

        # Set the formationBarycenter module if the flag is set to True
        if relativeNavigation:
            self.relNavProc = None
            self.relativeNavigationModule = None
            self.relativeNavigationTaskName = None
            self.add_relativeNavigation()

    def get_EnvModel(self):
        assert (self.environment_added is True), "It is mandatory to use an environment model as an argument"
        return self.EnvModel

    def set_EnvModel(self, envModel):
        self.environment_added = True
        self.EnvProcessName = "EnvironmentProcess"
        self.envProc = self.CreateNewProcess(self.EnvProcessName, 300)

        # Add the environment class
        self.EnvModel = envModel.BSKEnvironmentModel(self, self.envRate)

    def get_DynModel(self):
        assert (self.dynamics_added is True), "It is mandatory to use a dynamics model as an argument"
        return self.DynModels

    def set_DynModel(self, dynModel):
        self.dynamics_added = True

        # Add the dynamics classes
        for spacecraftIndex in range(self.numberSpacecraft):
            self.DynamicsProcessName.append("DynamicsProcess" + str(spacecraftIndex))  # Create simulation process name
            self.dynProc.append(self.CreateNewProcess(self.DynamicsProcessName[spacecraftIndex], 200))  # Create process
            self.DynModels.append(dynModel[spacecraftIndex].BSKDynamicModels(self, self.dynRate, spacecraftIndex))

    def get_FswModel(self):
        assert (self.fsw_added is True), "A flight software model has not been added yet"
        return self.FSWModels

    def set_FswModel(self, fswModel):
        self.fsw_added = True

        # Add the FSW classes
        for spacecraftIndex in range(self.numberSpacecraft):
            self.FSWProcessName.append("FSWProcess" + str(spacecraftIndex))  # Create simulation process name
            self.fswProc.append(self.CreateNewProcess(self.FSWProcessName[spacecraftIndex], 100))  # Create process
            self.FSWModels.append(fswModel[spacecraftIndex].BSKFswModels(self, self.fswRate, spacecraftIndex))

    def add_relativeNavigation(self):
        processName = "RelativeNavigation"
        processTasksTimeStep = mc.sec2nano(self.relNavRate)
        self.relativeNavigationTaskName = "relativeNavigationTask"

        # Create an independt process for barycenter calculation
        self.relNavProc = self.CreateNewProcess(processName, 150)
        self.relNavProc.addTask(self.CreateNewTask(self.relativeNavigationTaskName, processTasksTimeStep), 20)

        # Add the formationBarycenter module
        self.relativeNavigationModule = formationBarycenter.FormationBarycenter()
        self.relativeNavigationModule.ModelTag = "RelativeNavigation"
        self.AddModelToTask(self.relativeNavigationTaskName, self.relativeNavigationModule, 0)

class BSKScenario(object):
    def __init__(self):
        self.name = "scenario"

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

    def pull_outputs(self, showPlots, spacecraftIndex):
        """
            Developer must override this method in their BSK_Scenario derived subclass.
        """
        pass
