#
#  ISC License
#
#  Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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

import os
from Basilisk.simulation import(
    mujoco,
    MJSystemCoM,
    MJSystemMassMatrix,
    MJJointReactionForces,
    svIntegrators,
)
from Basilisk.utilities import macros as mc

XML_PATH = os.path.join(os.path.dirname(__file__), "..", "sat_w_thruster_arms.xml")


class BSKMujocoDynamicsModels():
    """
    bskSim simulation class that sets up the MuJoCo spacecraft dynamics models.
    """
    def __init__(self, SimBase, dynRate):
        # Define process name, task name and task time-step
        self.processName = SimBase.DynamicsProcessName
        self.taskName = "DynamicsTask"
        self.processTasksTimeStep = mc.sec2nano(dynRate)

        # Create task
        SimBase.dynProc.addTask(SimBase.CreateNewTask(self.taskName, self.processTasksTimeStep))

        # Instantiate Dyn modules as objects
        self.scene = mujoco.MJScene.fromFile(XML_PATH)
        self.systemCoM = MJSystemCoM.MJSystemCoM()
        self.systemMassMatrix = MJSystemMassMatrix.MJSystemMassMatrix()
        self.jointReactionForces = MJJointReactionForces.MJJointReactionForces()
        self.integrator = svIntegrators.svIntegratorRKF45(self.scene)

        # Initialize all modules
        self.InitAllDynObjects()

        # Assign all initialized modules to tasks
        SimBase.AddModelToTask(self.taskName, self.scene, 200)
        SimBase.AddModelToTask(self.taskName, self.systemCoM, 199)
        SimBase.AddModelToTask(self.taskName, self.systemMassMatrix, 198)
        SimBase.AddModelToTask(self.taskName, self.jointReactionForces, 197)


    # ------------------------------------------------------------------------------------------- #
    # These are module-initialization methods

    def setScene(self):
        self.scene.extraEoMCall = True
        self.scene.ModelTag = "mujocoScene"
        self.scene.setIntegrator(self.integrator)

    def setSystemCoM(self):
        self.systemCoM.ModelTag = "systemCoM"
        self.systemCoM.scene = self.scene

    def setSystemMassMatrix(self):
        self.systemMassMatrix.ModelTag = "systemMassMatrix"
        self.systemMassMatrix.scene = self.scene

    def setJointReactionForces(self):
        self.jointReactionForces.ModelTag = "jointReactionForces"
        self.jointReactionForces.scene = self.scene


    # Global call to initialize every module
    def InitAllDynObjects(self):
        self.setScene()
        self.setSystemCoM()
        self.setSystemMassMatrix()
        self.setJointReactionForces()
