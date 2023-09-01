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

import numpy as np
from Basilisk import __path__
from Basilisk.simulation import (spacecraft, extForceTorque, simpleNav,
                                 reactionWheelStateEffector)
from Basilisk.utilities import RigidBodyKinematics as rbk
from Basilisk.utilities import macros as mc
from Basilisk.utilities import simIncludeRW, simIncludeGravBody
from Basilisk.utilities import unitTestSupport as sp

bskPath = __path__[0]



class BSKDynamicModels():
    def __init__(self, SimBase, dynRate):
        # Define process name, task name and task time-step
        self.processName = SimBase.DynamicsProcessName
        self.taskName = "DynamicsTask"
        self.taskName2 = "DynamicsTask2"
        self.processTasksTimeStep = mc.sec2nano(dynRate)

        # Create task
        SimBase.dynProc.addTask(SimBase.CreateNewTask(self.taskName, self.processTasksTimeStep))
        SimBase.dynProc.addTask(SimBase.CreateNewTask(self.taskName2, self.processTasksTimeStep))

        # Instantiate Dyn modules as objects
        self.scObject = spacecraft.Spacecraft()
        self.simpleNavObject = simpleNav.SimpleNav()
        self.rwStateEffector = reactionWheelStateEffector.ReactionWheelStateEffector()
        self.rwFactory = simIncludeRW.rwFactory()

        self.scObject2 = spacecraft.Spacecraft()
        self.simpleNavObject2 = simpleNav.SimpleNav()
        self.rwStateEffector2 = reactionWheelStateEffector.ReactionWheelStateEffector()
        self.extForceTorqueObject2 = extForceTorque.ExtForceTorque()
        self.rwFactory2 = simIncludeRW.rwFactory()

        # Create gravity body
        self.gravFactory = simIncludeGravBody.gravBodyFactory()
        planet = self.gravFactory.createEarth()
        planet.isCentralBody = True          # ensure this is the central gravitational body
        self.scObject.gravField.gravBodies = spacecraft.GravBodyVector(list(self.gravFactory.gravBodies.values()))
        self.scObject2.gravField.gravBodies = spacecraft.GravBodyVector(list(self.gravFactory.gravBodies.values()))

        # Initialize all modules and write init one-time messages
        self.InitAllDynObjects()

        # Assign initialized modules to tasks
        SimBase.AddModelToTask(self.taskName, self.scObject, 201)
        SimBase.AddModelToTask(self.taskName, self.simpleNavObject, 109)
        SimBase.AddModelToTask(self.taskName, self.rwStateEffector, 301)

        SimBase.AddModelToTask(self.taskName2, self.scObject2, 201)
        SimBase.AddModelToTask(self.taskName2, self.simpleNavObject2, 109)
        SimBase.AddModelToTask(self.taskName2, self.rwStateEffector2, 301)
        SimBase.AddModelToTask(self.taskName2, self.extForceTorqueObject2, 300)

    # ------------------------------------------------------------------------------------------- #
    # These are module-initialization methods

    def SetSpacecraftHub(self):
        self.scObject.ModelTag = "chief"
        self.I_sc = [900., 0., 0.,
                     0., 800., 0.,
                     0., 0., 600.]
        self.scObject.hub.mHub = 750.0  # kg - spacecraft mass
        self.scObject.hub.r_BcB_B = [[0.0], [0.0], [0.0]]  # m - position vector of body-fixed point B relative to CM
        self.scObject.hub.IHubPntBc_B = sp.np2EigenMatrix3d(self.I_sc)

        self.scObject2.ModelTag = "deputy"
        self.I_sc2 = [900., 0., 0.,
                      0., 800., 0.,
                      0., 0., 600.]
        self.scObject2.hub.mHub = 750.0  # kg - spacecraft mass
        self.scObject2.hub.r_BcB_B = [[0.0], [0.0], [0.0]]  # m - position vector of body-fixed point B relative to CM
        self.scObject2.hub.IHubPntBc_B = sp.np2EigenMatrix3d(self.I_sc2)

    def SetSimpleNavObject(self):
        self.simpleNavObject.ModelTag = "SimpleNavigation_chief"
        self.simpleNavObject.scStateInMsg.subscribeTo(self.scObject.scStateOutMsg)

        self.simpleNavObject2.ModelTag = "SimpleNavigation_deputy"
        self.simpleNavObject2.scStateInMsg.subscribeTo(self.scObject2.scStateOutMsg)

    def SetReactionWheelDynEffector(self):
        # Make a fresh RW factory instance, this is critical to run multiple times

        # specify RW momentum capacity
        maxRWMomentum = 50.  # Nms

        # Define orthogonal RW pyramid
        # -- Pointing directions
        rwElAngle = np.array([40.0, 40.0, 40.0, 40.0])*mc.D2R
        rwAzimuthAngle = np.array([45.0, 135.0, 225.0, 315.0])*mc.D2R
        rwPosVector = [[0.8, 0.8, 1.79070],
                       [0.8, -0.8, 1.79070],
                       [-0.8, -0.8, 1.79070],
                       [-0.8, 0.8, 1.79070]
                       ]

        for elAngle, azAngle, posVector in zip(rwElAngle, rwAzimuthAngle, rwPosVector):
            gsHat = (rbk.Mi(-azAngle,3).dot(rbk.Mi(elAngle,2))).dot(np.array([1,0,0]))
            self.rwFactory.create('Honeywell_HR16',
                                  gsHat,
                                  maxMomentum=maxRWMomentum,
                                  rWB_B=posVector)
            self.rwFactory2.create('Honeywell_HR16',
                                   gsHat,
                                   maxMomentum=maxRWMomentum,
                                   rWB_B =posVector)

        self.rwFactory.addToSpacecraft("RW_chief", self.rwStateEffector, self.scObject)
        self.rwFactory2.addToSpacecraft("RW_deputy", self.rwStateEffector2, self.scObject2)

    def SetExternalForceTorqueObject(self):
        self.extForceTorqueObject2.ModelTag = "externalDisturbance"
        self.scObject2.addDynamicEffector(self.extForceTorqueObject2)

    # Global call to initialize every module
    def InitAllDynObjects(self):
        self.SetSpacecraftHub()
        self.SetSimpleNavObject()
        self.SetReactionWheelDynEffector()
        self.SetExternalForceTorqueObject()

