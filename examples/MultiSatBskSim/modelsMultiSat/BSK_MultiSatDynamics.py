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

import numpy as np
from Basilisk.utilities import (macros as mc, unitTestSupport as sp, RigidBodyKinematics as rbk,
                                simIncludeRW, simIncludeThruster)
from Basilisk.simulation import (spacecraft, simpleNav, reactionWheelStateEffector, thrusterDynamicEffector)

from Basilisk import __path__

bskPath = __path__[0]


class BSKDynamicModels:
    """
    Defines the Dynamics class.
    """
    def __init__(self, SimBase, dynRate, spacecraftIndex):
        self.I_sc = None
        self.numRW = 4
        self.numThr = 2
        self.spacecraftIndex = spacecraftIndex

        # Define process name, task name and task time-step
        self.taskName = "DynamicsTask" + str(spacecraftIndex)
        self.processTasksTimeStep = mc.sec2nano(dynRate)

        # Create task
        SimBase.dynProc[spacecraftIndex].addTask(SimBase.CreateNewTask(self.taskName, self.processTasksTimeStep))

        # Instantiate Dyn modules as objects
        self.scObject = spacecraft.Spacecraft()
        self.simpleNavObject = simpleNav.SimpleNav()
        self.rwStateEffector = reactionWheelStateEffector.ReactionWheelStateEffector()
        self.rwFactory = simIncludeRW.rwFactory()
        self.thrusterEffector = thrusterDynamicEffector.ThrusterDynamicEffector()
        self.thrusterFactory = simIncludeThruster.thrusterFactory()

        # Initialize all modules and write init one-time messages
        self.InitAllDynObjects(SimBase)

        # Assign initialized modules to tasks
        SimBase.AddModelToTask(self.taskName, self.scObject, None, 100)
        SimBase.AddModelToTask(self.taskName, self.simpleNavObject, None, 100)
        SimBase.AddModelToTask(self.taskName, self.rwStateEffector, None, 100)
        SimBase.AddModelToTask(self.taskName, self.thrusterEffector, None, 100)

    # ------------------------------------------------------------------------------------------- #
    # These are module-initialization methods

    def SetSpacecraftHub(self):
        """
        Defines the spacecraft object properties.
        """
        self.scObject.ModelTag = "spacecraft" + str(self.spacecraftIndex)
        self.I_sc = [900., 0., 0.,
                     0., 800., 0.,
                     0., 0., 600.]
        self.scObject.hub.mHub = 750.0  # kg - spacecraft mass
        self.scObject.hub.r_BcB_B = [[0.0], [0.0], [0.0]]  # m - position vector of body-fixed point B relative to CM
        self.scObject.hub.IHubPntBc_B = sp.np2EigenMatrix3d(self.I_sc)

    def SetGravityBodies(self, SimBase):
        """
        Specify what gravitational bodies to include in the simulation
        """
        # Attach the gravity body
        self.scObject.gravField.gravBodies = spacecraft.GravBodyVector(list(SimBase.EnvModel.gravFactory.gravBodies.values()))

    def SetSimpleNavObject(self):
        """
        Defines the navigation module.
        """
        self.simpleNavObject.ModelTag = "SimpleNavigation" + str(self.spacecraftIndex)
        self.simpleNavObject.scStateInMsg.subscribeTo(self.scObject.scStateOutMsg)

    def SetReactionWheelDynEffector(self):
        """
        Defines the RW state effector.
        """
        # specify RW momentum capacity
        maxRWMomentum = 50.  # Nms

        # Define orthogonal RW pyramid
        # -- Pointing directions
        rwElAngle = np.array([40.0, 40.0, 40.0, 40.0]) * mc.D2R
        rwAzimuthAngle = np.array([45.0, 135.0, 225.0, 315.0]) * mc.D2R
        rwPosVector = [[0.8, 0.8, 1.79070],
                       [0.8, -0.8, 1.79070],
                       [-0.8, -0.8, 1.79070],
                       [-0.8, 0.8, 1.79070]]

        for elAngle, azAngle, posVector in zip(rwElAngle, rwAzimuthAngle, rwPosVector):
            gsHat = (rbk.Mi(-azAngle, 3).dot(rbk.Mi(elAngle, 2))).dot(np.array([1, 0, 0]))
            self.rwFactory.create('Honeywell_HR16',
                                  gsHat,
                                  maxMomentum=maxRWMomentum,
                                  rWB_B=posVector)

        self.rwFactory.addToSpacecraft("RWArray" + str(self.spacecraftIndex), self.rwStateEffector, self.scObject)

    def SetThrusterDynEffector(self):
        """
        Defines the thruster state effector.
        """
        location = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
        direction = [[1.0, 1.0, 1.0], [1.0, 1.0, 1.0]]

        # create the thruster devices by specifying the thruster type and its location and direction
        for pos_B, dir_B in zip(location, direction):
            self.thrusterFactory.create('MOOG_Monarc_445', pos_B, dir_B, useMinPulseTime=False)

        # create thruster object container and tie to spacecraft object
        self.thrusterFactory.addToSpacecraft("thrusterFactory", self.thrusterEffector, self.scObject)

    def SetGroundLocations(self, SimBase):
        """
        Adds the spacecraft to the ground location module.
        """
        SimBase.EnvModel.groundStation.addSpacecraftToModel(self.scObject.scStateOutMsg)

    # Global call to initialize every module
    def InitAllDynObjects(self, SimBase):
        """
        Initializes all dynamic objects.
        """
        self.SetSpacecraftHub()
        self.SetGravityBodies(SimBase)
        self.SetReactionWheelDynEffector()
        self.SetThrusterDynEffector()
        self.SetSimpleNavObject()
        self.SetGroundLocations(SimBase)
