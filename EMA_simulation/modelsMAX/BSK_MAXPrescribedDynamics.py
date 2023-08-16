#
#  ISC License
#
#  Copyright (c) 2022, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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
from Basilisk.utilities import (macros as mc, unitTestSupport as sp, RigidBodyKinematics as rbk, simIncludeRW)
from Basilisk.simulation import (spacecraft, simpleNav, simpleMassProps, reactionWheelStateEffector,
                                 ReactionWheelPower, boreAngCalc, prescribedMotionStateEffector)


from Basilisk import __path__

bskPath = __path__[0]


class BSKDynamicModels:
    """
    Defines the Dynamics class.
    """

    def __init__(self, SimBase, dynRate, spacecraftIndex):
        self.I_sc = None
        self.solarPanelAxis = None
        self.numRW = 4
        self.numThr = None
        self.numRSA = 2
        self.spacecraftIndex = spacecraftIndex

        # Define process name, task name and task time-step
        self.taskName = "DynamicsTask" + str(spacecraftIndex)
        self.processTasksTimeStep = mc.sec2nano(dynRate)

        # Create task
        SimBase.dynProc[spacecraftIndex].addTask(SimBase.CreateNewTask(self.taskName, self.processTasksTimeStep))

        # Instantiate Dyn modules as objects
        self.scObject = spacecraft.Spacecraft()
        self.simpleNavObject = simpleNav.SimpleNav()
        self.simpleMassPropsObject = simpleMassProps.SimpleMassProps()
        self.rwStateEffector = reactionWheelStateEffector.ReactionWheelStateEffector()
        self.rwFactory = simIncludeRW.rwFactory()
        self.earthBoresight = boreAngCalc.BoreAngCalc()
        self.inertialBoresight = boreAngCalc.BoreAngCalc()
        self.sensitiveBoresight = boreAngCalc.BoreAngCalc()
        self.platform = prescribedMotionStateEffector.PrescribedMotionStateEffector()

        self.rwPowerList = []
        for item in range(self.numRW):
            self.rwPowerList.append(ReactionWheelPower.ReactionWheelPower())

        self.RSAList = []
        self.sunBoresightList = []
        for item in range(self.numRSA):
            self.RSAList.append(prescribedMotionStateEffector.PrescribedMotionStateEffector())
            self.sunBoresightList.append(boreAngCalc.BoreAngCalc())

        # Initialize all modules and write init one-time messages
        self.InitAllDynObjects(SimBase)

        # Assign initialized modules to tasks
        SimBase.AddModelToTask(self.taskName, self.scObject, None, 100)
        SimBase.AddModelToTask(self.taskName, self.simpleNavObject, None, 100)
        SimBase.AddModelToTask(self.taskName, self.simpleMassPropsObject, None, 100)
        SimBase.AddModelToTask(self.taskName, self.rwStateEffector, None, 100)
        SimBase.AddModelToTask(self.taskName, self.platform, None, 100)
        SimBase.AddModelToTask(self.taskName, self.earthBoresight, None, 100)
        SimBase.AddModelToTask(self.taskName, self.sensitiveBoresight, None, 100)
        # SimBase.AddModelToTask(self.taskName, self.inertialBoresight, None, 100)

        for item in range(self.numRSA):
            SimBase.AddModelToTask(self.taskName, self.RSAList[item], None, 100)
            SimBase.AddModelToTask(self.taskName, self.sunBoresightList[item], None, 100)

    # ------------------------------------------------------------------------------------------- #
    # These are module-initialization methods

    def SetSpacecraftHub(self):
        """
        Defines the spacecraft object properties
        """
        self.scObject.ModelTag = "sat-" + str(self.spacecraftIndex)
        self.I_sc = [900., 0., 0.,
                     0., 800., 0.,
                     0., 0., 600.]
        self.scObject.hub.mHub = 750.0  # kg - spacecraft mass
        self.scObject.hub.r_BcB_B = [[0.1], [0.2], [0.0]]  # m - position vector of body-fixed point B relative to CM
        self.scObject.hub.IHubPntBc_B = sp.np2EigenMatrix3d(self.I_sc)

    def SetGravityBodies(self, SimBase):
        """
        Specify what gravitational bodies to include in the simulation
        """
        # Attach the gravity body
        self.scObject.gravField.gravBodies = spacecraft.GravBodyVector(list(SimBase.EnvModel.gravFactory.gravBodies.values()))

    def SetSimpleNavObject(self, SimBase):
        """
        Defines the navigation module
        """
        self.simpleNavObject.ModelTag = "SimpleNavigation" + str(self.spacecraftIndex)
        self.simpleNavObject.scStateInMsg.subscribeTo(self.scObject.scStateOutMsg)
        self.simpleNavObject.sunStateInMsg.subscribeTo(SimBase.EnvModel.gravFactory.spiceObject.planetStateOutMsgs[SimBase.EnvModel.sun])

    def SetSimpleMassPropsObject(self):
        """
        Defines the mass properties module
        """
        self.simpleMassPropsObject.ModelTag = "SimpleMassProperties" + str(self.spacecraftIndex)
        self.simpleMassPropsObject.scMassPropsInMsg.subscribeTo(self.scObject.scMassOutMsg)

    def SetReactionWheelDynEffector(self):
        """
        Defines the reaction wheels effector
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
            self.rwFactory.create('Honeywell_HR16', gsHat, maxMomentum=maxRWMomentum, rWB_B=posVector)

        self.numRW = self.rwFactory.getNumOfDevices()
        self.rwFactory.addToSpacecraft("RWArray" + str(self.spacecraftIndex), self.rwStateEffector, self.scObject)

    def SetPrescribedSolarArrays(self):
        """
        Defines the prescribed solar arrays
        """
        # Define the first solar array
        self.RSAList[0].IPntFc_F = [[17.5, 0, 0], [0, 35, 0], [0, 0, 17.5]]
        self.RSAList[0].mass = 5
        self.RSAList[0].r_MB_B = [0.75, 0, 0.75]
        self.RSAList[0].r_FcF_F = [3.7, 0, 0]
        self.RSAList[0].ModelTag = "solarArray0"
        self.scObject.addStateEffector(self.RSAList[0])

        # Define the second solar array
        self.RSAList[1].IPntFc_F = [[17.5, 0, 0], [0, 35, 0], [0, 0, 17.5]]
        self.RSAList[1].mass = 5
        self.RSAList[1].r_MB_B = [-0.75, 0, 0.75]
        self.RSAList[1].r_FcF_F = [3.7, 0, 0]
        self.RSAList[1].sigma_MB = [0, 1, 0]
        self.RSAList[1].ModelTag = "solarArray1"
        self.scObject.addStateEffector(self.RSAList[1])

    def SetEarthBoresight(self, SimBase):
        """Sets up the boresight calc module"""
        self.earthBoresight.ModelTag = "earthBoresight"
        self.earthBoresight.scStateInMsg.subscribeTo(self.scObject.scStateOutMsg)
        self.earthBoresight.celBodyInMsg.subscribeTo(SimBase.EnvModel.gravFactory.spiceObject.planetStateOutMsgs[SimBase.EnvModel.earth])
        self.earthBoresight.boreVec_B = [0, 1, 0]

    def SetSunBoresight(self, SimBase):
        """Sets up the boresight calc module"""
        self.sunBoresightList[0].ModelTag = "boresight"
        self.sunBoresightList[0].scStateInMsg.subscribeTo(self.RSAList[0].prescribedMotionConfigLogOutMsg)
        self.sunBoresightList[0].celBodyInMsg.subscribeTo(SimBase.EnvModel.gravFactory.spiceObject.planetStateOutMsgs[SimBase.EnvModel.sun])
        self.sunBoresightList[0].boreVec_B = [0, 1, 0]

        self.sunBoresightList[1].ModelTag = "boresight"
        self.sunBoresightList[1].scStateInMsg.subscribeTo(self.RSAList[1].prescribedMotionConfigLogOutMsg)
        self.sunBoresightList[1].celBodyInMsg.subscribeTo(SimBase.EnvModel.gravFactory.spiceObject.planetStateOutMsgs[SimBase.EnvModel.sun])
        self.sunBoresightList[1].boreVec_B = [0, 1, 0]

    def SetInertialBoresight(self, SimBase):
        """Sets up the boresight calc module"""
        self.inertialBoresight.ModelTag = "inertialBoresight"
        self.inertialBoresight.scStateInMsg.subscribeTo(self.platform.prescribedMotionConfigLogOutMsg)
        self.inertialBoresight.boreVec_B = [0, 0, 1]
        self.inertialBoresight.inertialHeadingVec_N = [1, 0, 0]

    def SetSensitiveBoresight(self, SimBase):
        """Sets up the boresight calc module"""
        self.sensitiveBoresight.ModelTag = "sensitivePlatformBoresight"
        self.sensitiveBoresight.scStateInMsg.subscribeTo(self.scObject.scStateOutMsg)
        self.sensitiveBoresight.celBodyInMsg.subscribeTo(SimBase.EnvModel.gravFactory.spiceObject.planetStateOutMsgs[SimBase.EnvModel.sun])
        self.sensitiveBoresight.boreVec_B = [0, -1, 0]

    def SetPrescribedPlatform(self):
        """
        Sets up the platform using prescribed dynamics
        """
        self.platform.IPntFc_F = [[4, 0, 0], [0, 6, 0], [0, 0, 8]]
        self.platform.mass = 10
        self.platform.r_MB_B = [0, 0, -1.6]
        self.platform.ModelTag = "platform"
        self.scObject.addStateEffector(self.platform)

    # Global call to initialize every module
    def InitAllDynObjects(self, SimBase):
        """
        Initializes all dynamic objects.
        """
        self.SetSpacecraftHub()
        self.SetGravityBodies(SimBase)
        self.SetReactionWheelDynEffector()
        self.SetPrescribedSolarArrays()
        self.SetSimpleNavObject(SimBase)
        self.SetSimpleMassPropsObject()
        self.SetPrescribedPlatform()
        self.SetEarthBoresight(SimBase)
        self.SetSunBoresight(SimBase)
        self.SetInertialBoresight(SimBase)
        self.SetSensitiveBoresight(SimBase)
