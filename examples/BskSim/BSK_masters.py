#
#  ISC License
#
#  Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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

# Import architectural modules
from Basilisk.utilities import SimulationBaseClass

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

# Import Dynamics and FSW models
sys.path.append(path + '/models')


class BSKSim(SimulationBaseClass.SimBaseClass):
    """Main bskSim simulation class"""

    def __init__(self, fswRate=0.1, dynRate=0.1):  # [s]
        self.dynRate = dynRate
        self.fswRate = fswRate
        # Create a sim module as an empty container
        SimulationBaseClass.SimBaseClass.__init__(self)

        self._dynModels = None
        self._fswModels = None
        self._dynamicsSetupAttempted = False
        self._fswSetupAttempted = False
        self.DynamicsProcessName = None
        self.FSWProcessName = None
        self.dynProc = None
        self.fswProc = None

        self.oneTimeRWFaultFlag = 0
        self.oneTimeFaultTime = -1
        self.repeatRWFaultFlag = 0

        self.dynamics_added = False
        self.fsw_added = False

    @property
    def DynModels(self):
        """Return the configured dynamics model.

        :raises RuntimeError: If a dynamics model has not been added.
        """
        if self._dynModels is None:
            raise RuntimeError("A dynamics model has not been added yet")
        return self._dynModels

    def get_DynModel(self):
        """Return the configured dynamics model.

        :raises RuntimeError: If a dynamics model has not been added.
        """
        return self.DynModels

    def set_DynModel(self, dynModel):
        """Construct and store the dynamics model.

        :param dynModel: Module containing the ``BSKDynamicModels`` class.
        :raises RuntimeError: If dynamics setup has already been attempted.
        """
        if self._dynamicsSetupAttempted:
            if self._dynModels is None:
                raise RuntimeError("A previous dynamics model setup attempt failed")
            raise RuntimeError("A dynamics model has already been added")
        self._dynamicsSetupAttempted = True
        self.DynamicsProcessName = 'DynamicsProcess'  # Create simulation process name
        self.dynProc = self.CreateNewProcess(self.DynamicsProcessName)  # Create process
        dynModels = dynModel.BSKDynamicModels(self, self.dynRate)  # Create dynamics class
        self._dynModels = dynModels
        self.dynamics_added = True

    @property
    def FSWModels(self):
        """Return the configured flight software model.

        :raises RuntimeError: If a flight software model has not been added.
        """
        if self._fswModels is None:
            raise RuntimeError("A flight software model has not been added yet")
        return self._fswModels

    def get_FswModel(self):
        """Return the configured flight software model.

        :raises RuntimeError: If a flight software model has not been added.
        """
        return self.FSWModels

    def set_FswModel(self, fswModel):
        """Construct and store the flight software model.

        :param fswModel: Module containing the ``BSKFswModels`` class.
        :raises RuntimeError: If dynamics has not been added or FSW setup has already been attempted.
        """
        if self._fswSetupAttempted:
            if self._fswModels is None:
                raise RuntimeError("A previous flight software model setup attempt failed")
            raise RuntimeError("A flight software model has already been added")
        if self._dynModels is None:
            raise RuntimeError("A dynamics model must be added before the flight software model")
        self._fswSetupAttempted = True
        self.FSWProcessName = "FSWProcess"  # Create simulation process name
        self.fswProc = self.CreateNewProcess(self.FSWProcessName)  # Create process
        fswModels = fswModel.BSKFswModels(self, self.fswRate)  # Create FSW class
        self._fswModels = fswModels
        self.fsw_added = True


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

    def pull_outputs(self):
        """
            Developer must override this method in their BSK_Scenario derived subclass.
        """
        pass
