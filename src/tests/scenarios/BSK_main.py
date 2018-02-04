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
import sys, os, inspect

from Basilisk.utilities import SimulationBaseClass
from Basilisk.simulation import sim_model
from Basilisk.simulation import message_router

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
