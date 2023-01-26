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

    def __init__(self, fswRate=0.1, dynRate=0.1):
        self.dynRate = dynRate
        self.fswRate = fswRate
        # Create a sim module as an empty container
        SimulationBaseClass.SimBaseClass.__init__(self)

        self.DynModels = []
        self.FSWModels = []
        self.DynamicsProcessName = None
        self.FSWProcessName = None
        self.dynProc = None
        self.fswProc = None
        
        self.oneTimeRWFaultFlag = 0
        self.oneTimeFaultTime = -1
        self.repeatRWFaultFlag = 0

        self.dynamics_added = False
        self.fsw_added = False

    def get_DynModel(self):
        assert (self.dynamics_added is True), "It is mandatory to use a dynamics model as an argument"
        return self.DynModels

    def set_DynModel(self, dynModel):
        self.dynamics_added = True
        self.DynamicsProcessName = 'DynamicsProcess'  # Create simulation process name
        self.dynProc = self.CreateNewProcess(self.DynamicsProcessName)  # Create process
        self.DynModels = dynModel.BSKDynamicModels(self, self.dynRate)  # Create Dynamics and FSW classes

    def get_FswModel(self):
        assert (self.fsw_added is True), "A flight software model has not been added yet"
        return self.FSWModels

    def set_FswModel(self, fswModel):
        self.fsw_added = True
        self.FSWProcessName = "FSWProcess"  # Create simulation process name
        self.fswProc = self.CreateNewProcess(self.FSWProcessName)  # Create process
        self.FSWModels = fswModel.BSKFswModels(self, self.fswRate)  # Create Dynamics and FSW classes


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
