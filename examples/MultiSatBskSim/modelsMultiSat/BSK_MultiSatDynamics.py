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
from Basilisk import __path__
from Basilisk.simulation import (spacecraft, simpleNav, simpleMassProps, reactionWheelStateEffector,
                                 thrusterDynamicEffector, simpleSolarPanel, simplePowerSink, simpleBattery, fuelTank,
                                 ReactionWheelPower)
from Basilisk.utilities import (macros as mc, unitTestSupport as sp, RigidBodyKinematics as rbk,
                                simIncludeRW, simIncludeThruster)

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
        self.tankModel = None
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
        self.thrusterDynamicEffector = thrusterDynamicEffector.ThrusterDynamicEffector()
        self.thrusterFactory = simIncludeThruster.thrusterFactory()
        self.solarPanel = simpleSolarPanel.SimpleSolarPanel()
        self.powerSink = simplePowerSink.SimplePowerSink()
        self.powerMonitor = simpleBattery.SimpleBattery()
        self.fuelTankStateEffector = fuelTank.FuelTank()

        self.rwPowerList = []
        for item in range(self.numRW):
            self.rwPowerList.append(ReactionWheelPower.ReactionWheelPower())

        # Initialize all modules and write init one-time messages
        self.InitAllDynObjects(SimBase)

        # Assign initialized modules to tasks
        SimBase.AddModelToTask(self.taskName, self.scObject, 100)
        SimBase.AddModelToTask(self.taskName, self.simpleNavObject, 100)
        SimBase.AddModelToTask(self.taskName, self.simpleMassPropsObject, 99)
        SimBase.AddModelToTask(self.taskName, self.rwStateEffector, 100)
        SimBase.AddModelToTask(self.taskName, self.thrusterDynamicEffector, 100)
        SimBase.AddModelToTask(self.taskName, self.solarPanel, 100)
        SimBase.AddModelToTask(self.taskName, self.powerSink, 100)
        SimBase.AddModelToTask(self.taskName, self.powerMonitor, 100)
        SimBase.AddModelToTask(self.taskName, self.fuelTankStateEffector, 100)

        for item in range(self.numRW):
            SimBase.AddModelToTask(self.taskName, self.rwPowerList[item], 100)

    # ------------------------------------------------------------------------------------------- #
    # These are module-initialization methods

    def SetSpacecraftHub(self):
        """
        Defines the spacecraft object properties.
        """
        self.scObject.ModelTag = "sat-" + str(self.spacecraftIndex)
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
        SimBase.EnvModel.gravFactory.addBodiesTo(self.scObject)

    def SetGroundLocations(self, SimBase):
        """
        Adds the spacecraft to the ground location module.
        """
        SimBase.EnvModel.groundStation.addSpacecraftToModel(self.scObject.scStateOutMsg)

    def SetEclipseObject(self, SimBase):
        """
        Adds the spacecraft to the eclipse module.
        """
        SimBase.EnvModel.eclipseObject.addSpacecraftToModel(self.scObject.scStateOutMsg)

    def SetSimpleNavObject(self):
        """
        Defines the navigation module.
        """
        self.simpleNavObject.ModelTag = "SimpleNavigation" + str(self.spacecraftIndex)
        self.simpleNavObject.scStateInMsg.subscribeTo(self.scObject.scStateOutMsg)

    def SetSimpleMassPropsObject(self):
        """
        Defines the navigation module.
        """
        self.simpleMassPropsObject.ModelTag = "SimpleMassProperties" + str(self.spacecraftIndex)
        self.simpleMassPropsObject.scMassPropsInMsg.subscribeTo(self.scObject.scMassOutMsg)

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

        self.numRW = self.rwFactory.getNumOfDevices()
        self.rwFactory.addToSpacecraft("RWArray" + str(self.spacecraftIndex), self.rwStateEffector, self.scObject)

    def SetThrusterDynEffector(self):
        """
        Defines the thruster state effector.
        """
        location = [[0.0, 0.0, 0.0]]
        direction = [[0.0, 0.0, 1.0]]

        # create the thruster devices by specifying the thruster type and its location and direction
        for pos_B, dir_B in zip(location, direction):
            self.thrusterFactory.create('MOOG_Monarc_90HT', pos_B, dir_B, useMinPulseTime=False)

        self.numThr = self.thrusterFactory.getNumOfDevices()

        # create thruster object container and tie to spacecraft object
        self.thrusterFactory.addToSpacecraft("thrusterFactory", self.thrusterDynamicEffector, self.scObject)

    def SetFuelTank(self):
        """
        Defines the fuel tank for the thrusters.
        """
        # Define the tank
        self.fuelTankStateEffector.setTankModel(fuelTank.TANK_MODEL_UNIFORM_BURN)
        self.tankModel = fuelTank.cvar.FuelTankModelUniformBurn
        self.tankModel.propMassInit = 50.0
        self.tankModel.maxFuelMass = 75.0
        self.tankModel.r_TcT_TInit = [[0.0], [0.0], [0.0]]
        self.fuelTankStateEffector.r_TB_B = [[0.0], [0.0], [0.0]]
        self.tankModel.radiusTankInit = 1
        self.tankModel.lengthTank = 1
        
        # Add the tank and connect the thrusters
        self.scObject.addStateEffector(self.fuelTankStateEffector)
        self.fuelTankStateEffector.addThrusterSet(self.thrusterDynamicEffector)

    def SetReactionWheelPower(self):
        """Sets the reaction wheel power parameters"""
        for item in range(self.numRW):
            self.rwPowerList[item].ModelTag = self.scObject.ModelTag + "RWPower" + str(item)
            self.rwPowerList[item].basePowerNeed = 5.  # baseline power draw, Watt
            self.rwPowerList[item].rwStateInMsg.subscribeTo(self.rwStateEffector.rwOutMsgs[item])
            self.rwPowerList[item].mechToElecEfficiency = 0.5

    def SetSolarPanel(self, SimBase):
        """Sets the solar panel"""
        self.solarPanel.ModelTag = "solarPanel"
        self.solarPanel.stateInMsg.subscribeTo(self.scObject.scStateOutMsg)
        self.solarPanel.sunEclipseInMsg.subscribeTo(SimBase.EnvModel.eclipseObject.eclipseOutMsgs[0])  # choose the earth message
        self.solarPanel.sunInMsg.subscribeTo(SimBase.EnvModel.gravFactory.spiceObject.planetStateOutMsgs[SimBase.EnvModel.sun])
        self.solarPanelAxis = [0, 0, 1]
        self.solarPanel.setPanelParameters(self.solarPanelAxis,  # panel normal vector in the body frame
                                           0.4 * 0.4 * 2 + 0.2 * 0.4 * 2,  # area, m^2
                                           0.35)  # efficiency

    def SetPowerSink(self):
        """Defines the energy sink parameters"""
        self.powerSink.ModelTag = "powerSink"
        self.powerSink.nodePowerOut = -2.  # Watt

    def SetBattery(self):
        """Sets up the battery with all the power components"""
        self.powerMonitor.ModelTag = "powerMonitor"
        self.powerMonitor.storageCapacity = 2 * 60.0 * 3600.0  # Convert from W-hr to Joule
        self.powerMonitor.storedCharge_Init = self.powerMonitor.storageCapacity * 0.6  # 40% depletion

        # attach the sources/sinks to the battery
        self.powerMonitor.addPowerNodeToModel(self.solarPanel.nodePowerOutMsg)
        self.powerMonitor.addPowerNodeToModel(self.powerSink.nodePowerOutMsg)
        for item in range(self.numRW):
            self.powerMonitor.addPowerNodeToModel(self.rwPowerList[item].nodePowerOutMsg)

    # Global call to initialize every module
    def InitAllDynObjects(self, SimBase):
        """
        Initializes all dynamic objects.
        """
        self.SetSpacecraftHub()
        self.SetGravityBodies(SimBase)
        self.SetReactionWheelDynEffector()
        self.SetThrusterDynEffector()
        self.SetFuelTank()
        self.SetSimpleNavObject()
        self.SetSimpleMassPropsObject()
        self.SetGroundLocations(SimBase)
        self.SetReactionWheelPower()
        self.SetEclipseObject(SimBase)
        self.SetSolarPanel(SimBase)
        self.SetPowerSink()
        self.SetBattery()
