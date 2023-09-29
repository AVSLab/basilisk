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
from Basilisk.utilities import (macros as mc, unitTestSupport as sp, RigidBodyKinematics as rbk, simIncludeRW)
from Basilisk.simulation import (spacecraft, simpleNav, simpleMassProps, reactionWheelStateEffector,
                                 ReactionWheelPower, boreAngCalc, spinningBodyOneDOFStateEffector, 
                                 spinningBodyTwoDOFStateEffector, thrusterStateEffector, facetSRPDynamicEffector)
from Basilisk.architecture import messaging

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
        thrDir_F = np.array([0.02, -0.025, 1])
        self.thrDir_F = thrDir_F / np.linalg.norm(thrDir_F)

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
        self.sensitiveBoresight = boreAngCalc.BoreAngCalc()
        self.inertialBoresight1 = boreAngCalc.BoreAngCalc()
        self.inertialBoresight2 = boreAngCalc.BoreAngCalc()
        self.platform1 = spinningBodyTwoDOFStateEffector.SpinningBodyTwoDOFStateEffector()
        self.platform2 = spinningBodyTwoDOFStateEffector.SpinningBodyTwoDOFStateEffector()
        self.SEPThruster1 = thrusterStateEffector.ThrusterStateEffector()
        self.SEPThruster2 = thrusterStateEffector.ThrusterStateEffector()
        self.newSRP = facetSRPDynamicEffector.FacetSRPDynamicEffector()


        self.rwPowerList = []
        for item in range(self.numRW):
            self.rwPowerList.append(ReactionWheelPower.ReactionWheelPower())

        self.RSAList = []
        self.sunBoresightList = []
        for item in range(self.numRSA):
            self.RSAList.append(spinningBodyOneDOFStateEffector.SpinningBodyOneDOFStateEffector())
            self.sunBoresightList.append(boreAngCalc.BoreAngCalc())

        # Initialize all modules and write init one-time messages
        self.InitAllDynObjects(SimBase)

        # Assign initialized modules to tasks
        SimBase.AddModelToTask(self.taskName, self.scObject, None, 100)
        SimBase.AddModelToTask(self.taskName, self.simpleNavObject, None, 100)
        SimBase.AddModelToTask(self.taskName, self.simpleMassPropsObject, None, 100)
        SimBase.AddModelToTask(self.taskName, self.rwStateEffector, None, 100)
        SimBase.AddModelToTask(self.taskName, self.platform1, None, 100)
        SimBase.AddModelToTask(self.taskName, self.platform2, None, 100)
        SimBase.AddModelToTask(self.taskName, self.earthBoresight, None, 100)
        SimBase.AddModelToTask(self.taskName, self.sensitiveBoresight, None, 100)
        SimBase.AddModelToTask(self.taskName, self.inertialBoresight1, None, 100)
        SimBase.AddModelToTask(self.taskName, self.inertialBoresight2, None, 100)
        SimBase.AddModelToTask(self.taskName, self.SEPThruster1, None, 100)
        SimBase.AddModelToTask(self.taskName, self.SEPThruster2, None, 100)
        SimBase.AddModelToTask(self.taskName, self.newSRP, None, 100)


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
        self.I_sc = [ 1725.697021321552,    -5.098169104856651,  -11.96633789093479,
                        -5.098169104856651,  5524.699699357305,   42.83593479126124,
                       -11.96633789093479,   42.83593479126124,   4809.231017715462]
        self.scObject.hub.mHub = 2049.51816  # kg - spacecraft mass
        self.scObject.hub.r_BcB_B = [[7.84713085978795 / 1000], [-9.966952366 / 1000], [1214.854881 / 1000]]  # [m] - position vector of hub CM relative to the body-fixed point B
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
        maxRWMomentum = 100.  # Nms

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
            self.rwFactory.create('Honeywell_HR16', gsHat, maxMomentum=maxRWMomentum, rWB_B=posVector, Omega=0.)

        self.numRW = self.rwFactory.getNumOfDevices()
        self.rwFactory.addToSpacecraft("RWArray" + str(self.spacecraftIndex), self.rwStateEffector, self.scObject)

    def SetRotatingSolarArrays(self):
        """
        Defines the rotating solar arrays
        """
        # Define the first solar array
        self.RSAList[0].r_SB_B = [0.5 * 1.53, 0.0, 0.44]
        self.RSAList[0].r_ScS_S = [-0.036, 2.827, -0.469]
        self.RSAList[0].sHat_S = [0, 1, 0]
        self.RSAList[0].dcm_S0B = [[0, 0, -1], 
                                   [1, 0, 0], 
                                   [0, -1, 0]]
        self.RSAList[0].IPntSc_S = [[319.0, 0.0, 0.0],
                                    [0.0, 185.0, 0.0],
                                    [0.0, 0.0, 495.0]]
        self.RSAList[0].mass = 82.79
        self.RSAList[0].k = 0
        self.RSAList[0].c = 0
        self.RSAList[0].thetaInit = 0
        self.RSAList[0].thetaDotInit = 0
        self.RSAList[0].ModelTag = "solarArray0"
        self.scObject.addStateEffector(self.RSAList[0])

        # Define the second solar array
        self.RSAList[1].r_SB_B = [-0.5 * 1.53, 0.0, 0.44]
        self.RSAList[1].r_ScS_S = [2.827, -0.036, -0.469]
        self.RSAList[1].sHat_S = [0, 1, 0]
        self.RSAList[1].dcm_S0B = [[0, 0, 1], [-1, 0, 0], [0, -1, 0]]
        self.RSAList[1].IPntSc_S = [[319.0, 0.0, 0.0],
                                    [0.0, 185.0, 0.0],
                                    [0.0, 0.0, 495.0]]
        self.RSAList[1].mass = 82.79
        self.RSAList[1].k = 0
        self.RSAList[1].c = 0
        self.RSAList[1].thetaInit = 0
        self.RSAList[1].thetaDotInit = 0
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
        self.sunBoresightList[0].scStateInMsg.subscribeTo(self.RSAList[0].spinningBodyConfigLogOutMsg)
        self.sunBoresightList[0].celBodyInMsg.subscribeTo(SimBase.EnvModel.gravFactory.spiceObject.planetStateOutMsgs[SimBase.EnvModel.sun])
        self.sunBoresightList[0].boreVec_B = [0, 1, 0]

        self.sunBoresightList[1].ModelTag = "boresight"
        self.sunBoresightList[1].scStateInMsg.subscribeTo(self.RSAList[1].spinningBodyConfigLogOutMsg)
        self.sunBoresightList[1].celBodyInMsg.subscribeTo(SimBase.EnvModel.gravFactory.spiceObject.planetStateOutMsgs[SimBase.EnvModel.sun])
        self.sunBoresightList[1].boreVec_B = [0, 1, 0]

    def SetInertialBoresight1(self, SimBase):
        """Sets up the boresight calc module"""
        self.inertialBoresight1.ModelTag = "inertialBoresight1"
        self.inertialBoresight1.scStateInMsg.subscribeTo(self.platform1.spinningBodyConfigLogOutMsgs[1])
        self.inertialBoresight1.boreVec_B = self.thrDir_F
        self.inertialBoresight1.inertialHeadingVec_N = [1, 0, 0]

    def SetInertialBoresight2(self, SimBase):
        """Sets up the boresight calc module"""
        self.inertialBoresight2.ModelTag = "inertialBoresight2"
        self.inertialBoresight2.scStateInMsg.subscribeTo(self.platform2.spinningBodyConfigLogOutMsgs[1])
        self.inertialBoresight2.boreVec_B = self.thrDir_F
        self.inertialBoresight2.inertialHeadingVec_N = [1, 0, 0]

    def SetSensitiveBoresight(self, SimBase):
        """Sets up the boresight calc module"""
        self.sensitiveBoresight.ModelTag = "sensitivePlatformBoresight"
        self.sensitiveBoresight.scStateInMsg.subscribeTo(self.scObject.scStateOutMsg)
        self.sensitiveBoresight.celBodyInMsg.subscribeTo(SimBase.EnvModel.gravFactory.spiceObject.planetStateOutMsgs[SimBase.EnvModel.sun])
        self.sensitiveBoresight.boreVec_B = [0, -1, 0]

    def SetPlatform1(self):
        """
        Defines the rotating platform
        """
        t = np.tan(15*mc.D2R)
        c = np.cos(15*mc.D2R)
        s = np.sin(15*mc.D2R)
        self.platform1.theta1Init = 0
        self.platform1.theta1DotInit = 0
        self.platform1.theta2Init = 0
        self.platform1.theta2DotInit = 0
        self.platform1.mass1 = 0
        self.platform1.mass2 = 10
        self.platform1.k1 = 0
        self.platform1.k2 = 0
        self.platform1.r_S1B_B = [0, 1.43*t, -1.43]
        self.platform1.r_S2S1_S1 = [0, 0, 0]
        self.platform1.r_Sc1S1_S1 = [0, 0, 0]
        self.platform1.r_Sc2S2_S2 = [0, 0, 0]
        self.platform1.s1Hat_S1 = [1, 0, 0]
        self.platform1.s2Hat_S2 = [0, 1, 0]
        self.platform1.IS1PntSc1_S1 = [[2, 0, 0], [0, 3, 0], [0, 0, 4]]
        self.platform1.IS2PntSc2_S2 = [[2, 0, 0], [0, 3, 0], [0, 0, 4]]
        self.platform1.dcm_S10B = [[1, 0, 0], [0, c, s], [0, -s, c]]
        self.platform1.dcm_S20S1 = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
        self.platform1.ModelTag = "platform1"
        self.scObject.addStateEffector(self.platform1)

    def SetPlatform2(self):
        """
        Defines the rotating platform
        """
        t = np.tan(15*mc.D2R)
        c = np.cos(15*mc.D2R)
        s = np.sin(15*mc.D2R)
        self.platform2.theta1Init = 0
        self.platform2.theta1DotInit = 0
        self.platform2.theta2Init = 0
        self.platform2.theta2DotInit = 0
        self.platform2.mass1 = 0
        self.platform2.mass2 = 10
        self.platform2.k1 = 0
        self.platform2.k2 = 0
        self.platform2.r_S1B_B = [0, -1.43*t, -1.43]
        self.platform2.r_S2S1_S1 = [0, 0, 0]
        self.platform2.r_Sc1S1_S1 = [0, 0, 0]
        self.platform2.r_Sc2S2_S2 = [0, 0, 0]
        self.platform2.s1Hat_S1 = [1, 0, 0]
        self.platform2.s2Hat_S2 = [0, 1, 0]
        self.platform2.IS1PntSc1_S1 = [[2, 0, 0], [0, 3, 0], [0, 0, 4]]
        self.platform2.IS2PntSc2_S2 = [[2, 0, 0], [0, 3, 0], [0, 0, 4]]
        self.platform2.dcm_S10B = [[1, 0, 0], [0, c, -s], [0, s, c]]
        self.platform2.dcm_S20S1 = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
        self.platform2.ModelTag = "platform2"
        self.scObject.addStateEffector(self.platform2)

    def SetSEPTrusterStateEffectors(self):
        """
        Defines the thruster effectors
        """
        thruster = thrusterStateEffector.THRSimConfig()
        thruster.thrLoc_B = [0, 0, 0]  # Parametrized location for thruster
        thruster.thrDir_B = self.thrDir_F
        thruster.MaxThrust = 0.27
        thruster.steadyIsp = 1600
        thruster.MinOnTime = 0.006
        thruster.cutoffFrequency = 5

        self.SEPThruster1.addThruster(thruster, self.platform1.spinningBodyConfigLogOutMsgs[1])
        self.SEPThruster1.kappaInit = messaging.DoubleVector([0.0])
        self.SEPThruster1.ModelTag = "SEPThruster1"
        self.scObject.addStateEffector(self.SEPThruster1)
        
        self.SEPThruster2.addThruster(thruster, self.platform2.spinningBodyConfigLogOutMsgs[1])
        self.SEPThruster2.kappaInit = messaging.DoubleVector([0.0])
        self.SEPThruster2.ModelTag = "SEPThruster2"
        self.scObject.addStateEffector(self.SEPThruster2)

    def SetFacetSRPDynamicEffector(self, SimBase):
        """
        Defines the facet SRP effector
        """
        # Define the spacecraft geometry for populating the FacetedSRPSpacecraftGeometryData structure in the SRP module
        # Define the facet surface areas
        lenXHub = 1.53  # [m]
        lenYHub = 1.8  # [m]
        lenZHub = 2.86  # [m]
        area2 = np.pi*(0.5 * 7.262)*(0.5 * 7.262)  # [m^2]
        facetAreas = [lenYHub * lenZHub, lenXHub * lenZHub, lenYHub * lenZHub, lenXHub * lenZHub, lenXHub * lenYHub, lenXHub * lenYHub, area2, area2, area2, area2]

        # Define the facet normals in B frame components
        facetNormal1 = np.array([1.0, 0.0, 0.0])
        facetNormal2 = np.array([0.0, 1.0, 0.0])
        facetNormal3 = np.array([-1.0, 0.0, 0.0])
        facetNormal4 = np.array([0.0, -1.0, 0.0])
        facetNormal5 = np.array([0.0, 0.0, 1.0])
        facetNormal6 = np.array([0.0, 0.0, -1.0])
        facetNormal7 = np.array([0.0, 1.0, 0.0])
        facetNormal8 = np.array([0.0, -1.0, 0.0])
        facetNormal9 = np.array([0.0, 1.0, 0.0])
        facetNormal10 = np.array([0.0, -1.0, 0.0])
        normals_B = [facetNormal1, facetNormal2, facetNormal3, facetNormal4, facetNormal5, facetNormal6, facetNormal7, facetNormal8, facetNormal9, facetNormal10]

        # Define the facet center of pressure locations with respect to point B in B frame components
        facetLoc1 = np.array([0.5 * lenXHub, 0.0, 0.5 * lenZHub])  # [m]
        facetLoc2 = np.array([0.0, 0.5 * lenYHub, 0.5 * lenZHub])  # [m]
        facetLoc3 = np.array([-0.5 * lenXHub, 0.0, 0.5 * lenZHub])  # [m]
        facetLoc4 = np.array([0.0, -0.5 * lenYHub, 0.5 * lenZHub])  # [m]
        facetLoc5 = np.array([0.0, 0.0, lenZHub])  # [m]
        facetLoc6 = np.array([0.0, 0.0, 0.0])  # [m]
        facetLoc7 = np.array([3.75 + 0.5 * lenXHub, 0.544, 0.44])  # [m]
        facetLoc8 = np.array([3.75 + 0.5 * lenXHub, 0.544, 0.44])  # [m]
        facetLoc9 = np.array([-(3.75 + 0.5 * lenXHub), 0.544, 0.44])  # [m]
        facetLoc10 = np.array([-(3.75 + 0.5 * lenXHub), 0.544, 0.44])  # [m]
        locationsPntB_B = [facetLoc1, facetLoc2, facetLoc3, facetLoc4, facetLoc5, facetLoc6, facetLoc7, facetLoc8, facetLoc9, facetLoc10]

        # Define the facet optical coefficients
        specCoeff = np.array([0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9])
        diffCoeff = np.array([0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1])

        # Populate the scGeometry structure with the facet information
        for i in range(len(facetAreas)):
            self.newSRP.addFacet(facetAreas[i], specCoeff[i], diffCoeff[i], normals_B[i], locationsPntB_B[i])

        self.newSRP.ModelTag = "FacetSRP"
        self.newSRP.sunInMsg.subscribeTo(SimBase.EnvModel.gravFactory.spiceObject.planetStateOutMsgs[SimBase.EnvModel.sun])
        self.scObject.addDynamicEffector(self.newSRP)


    # Global call to initialize every module
    def InitAllDynObjects(self, SimBase):
        """
        Initializes all dynamic objects.
        """
        self.SetSpacecraftHub()
        self.SetGravityBodies(SimBase)
        self.SetReactionWheelDynEffector()
        self.SetRotatingSolarArrays()
        self.SetSimpleNavObject(SimBase)
        self.SetSimpleMassPropsObject()
        self.SetPlatform1()
        self.SetPlatform2()
        self.SetEarthBoresight(SimBase)
        self.SetSunBoresight(SimBase)
        self.SetSensitiveBoresight(SimBase)
        self.SetInertialBoresight1(SimBase)
        self.SetInertialBoresight2(SimBase)
        self.SetSEPTrusterStateEffectors()
        self.SetFacetSRPDynamicEffector(SimBase)

