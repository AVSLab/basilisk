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

    def __init__(
        self,
        numberSpacecraft,
        relativeNavigation=False,
        fswRate=0.1,  # [s]
        dynRate=0.1,  # [s]
        envRate=0.1,  # [s]
        relNavRate=0.1,  # [s]
    ):
        self.dynRate = dynRate
        self.fswRate = fswRate
        self.envRate = envRate
        self.relNavRate = relNavRate
        self.numberSpacecraft = numberSpacecraft

        # Create a sim module as an empty container
        SimulationBaseClass.SimBaseClass.__init__(self)
        self.SetProgressBar(True)

        self._envModel = None
        self._dynModels = None
        self._fswModels = None
        self._environmentSetupAttempted = False
        self._dynamicsSetupAttempted = False
        self._fswSetupAttempted = False
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

    @property
    def EnvModel(self):
        """Return the configured environment model.

        :raises RuntimeError: If an environment model has not been added.
        """
        if self._envModel is None:
            raise RuntimeError("An environment model has not been added yet")
        return self._envModel

    def get_EnvModel(self):
        """Return the configured environment model.

        :raises RuntimeError: If an environment model has not been added.
        """
        return self.EnvModel

    def set_EnvModel(self, envModel):
        """Construct and store the environment model.

        :param envModel: Module containing the ``BSKEnvironmentModel`` class.
        :raises RuntimeError: If environment setup has already been attempted.
        """
        if self._environmentSetupAttempted:
            if self._envModel is None:
                raise RuntimeError("A previous environment model setup attempt failed")
            raise RuntimeError("An environment model has already been added")
        self._environmentSetupAttempted = True
        self.EnvProcessName = "EnvironmentProcess"
        self.envProc = self.CreateNewProcess(self.EnvProcessName, 300)

        # Add the environment class
        environmentModel = envModel.BSKEnvironmentModel(self, self.envRate)
        self._envModel = environmentModel
        self.environment_added = True

    @property
    def DynModels(self):
        """Return the configured dynamics models.

        :raises RuntimeError: If dynamics models have not been added.
        """
        if self._dynModels is None:
            raise RuntimeError("Dynamics models have not been added yet")
        return self._dynModels

    def get_DynModel(self):
        """Return the configured dynamics models.

        :raises RuntimeError: If dynamics models have not been added.
        """
        return self.DynModels

    def set_DynModel(self, dynModel):
        """Construct and store the dynamics models.

        :param dynModel: Modules containing the ``BSKDynamicModels`` class.
        :raises RuntimeError: If the environment is missing or dynamics setup was already attempted.
        """
        if self._dynamicsSetupAttempted:
            if self._dynModels is None:
                raise RuntimeError("A previous dynamics model setup attempt failed")
            raise RuntimeError("Dynamics models have already been added")
        if self._envModel is None:
            raise RuntimeError("An environment model must be added before the dynamics models")
        self._dynamicsSetupAttempted = True

        # Add the dynamics classes
        dynamicsModels = []
        for spacecraftIndex in range(self.numberSpacecraft):
            self.DynamicsProcessName.append("DynamicsProcess" + str(spacecraftIndex))  # Create simulation process name
            self.dynProc.append(self.CreateNewProcess(self.DynamicsProcessName[spacecraftIndex], 200))  # Create process
            dynamicsModels.append(dynModel[spacecraftIndex].BSKDynamicModels(self, self.dynRate, spacecraftIndex))
        self._dynModels = dynamicsModels
        self.dynamics_added = True

    @property
    def FSWModels(self):
        """Return the configured flight software models.

        :raises RuntimeError: If flight software models have not been added.
        """
        if self._fswModels is None:
            raise RuntimeError("Flight software models have not been added yet")
        return self._fswModels

    def get_FswModel(self):
        """Return the configured flight software models.

        :raises RuntimeError: If flight software models have not been added.
        """
        return self.FSWModels

    def set_FswModel(self, fswModel):
        """Construct and store the flight software models.

        :param fswModel: Modules containing the ``BSKFswModels`` class.
        :raises RuntimeError: If dynamics is missing or FSW setup was already attempted.
        """
        if self._fswSetupAttempted:
            if self._fswModels is None:
                raise RuntimeError("A previous flight software model setup attempt failed")
            raise RuntimeError("Flight software models have already been added")
        if self._dynModels is None:
            raise RuntimeError("Dynamics models must be added before the flight software models")
        self._fswSetupAttempted = True

        # Add the FSW classes
        flightSoftwareModels = []
        for spacecraftIndex in range(self.numberSpacecraft):
            self.FSWProcessName.append("FSWProcess" + str(spacecraftIndex))  # Create simulation process name
            self.fswProc.append(self.CreateNewProcess(self.FSWProcessName[spacecraftIndex], 100))  # Create process
            flightSoftwareModels.append(
                fswModel[spacecraftIndex].BSKFswModels(self, self.fswRate, spacecraftIndex)
            )
        self._fswModels = flightSoftwareModels
        self.fsw_added = True

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
