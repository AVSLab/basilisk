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
from Basilisk.architecture import messaging
from Basilisk.simulation import (spacecraft, simpleNav, simpleMassProps, reactionWheelStateEffector,
                                 thrusterDynamicEffector, simpleSolarPanel, simplePowerSink, simpleBattery, fuelTank,
                                 ReactionWheelPower, magnetometer, MtbEffector, dragDynamicEffector, simpleInstrument,
                                 partitionedStorageUnit, spaceToGroundTransmitter, simpleAntenna, antennaPower)
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
        self.mtbConfigParams = None
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
        self.tam = magnetometer.Magnetometer() # Three Axis Magnetometer (measuring magnetic field)
        self.mtbEff = MtbEffector.MtbEffector() # Magnet Torquer
        self.dragEff = dragDynamicEffector.DragDynamicEffector()
        self.instrument = simpleInstrument.SimpleInstrument()
        self.dataMonitor = partitionedStorageUnit.PartitionedStorageUnit()
        self.transmitter = spaceToGroundTransmitter.SpaceToGroundTransmitter()

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
        SimBase.AddModelToTask(self.taskName, self.tam, 100)
        SimBase.AddModelToTask(self.taskName, self.mtbEff, 100)
        SimBase.AddModelToTask(self.taskName, self.dragEff, 100)
        SimBase.AddModelToTask(self.taskName, self.instrument, 90)
        SimBase.AddModelToTask(self.taskName, self.dataMonitor, 89)
        SimBase.AddModelToTask(self.taskName, self.transmitter, 99)

        for item in range(self.numRW):
            SimBase.AddModelToTask(self.taskName, self.rwPowerList[item], 100)

    # ------------------------------------------------------------------------------------------- #
    # These are module-initialization methods

    def SetSpacecraftHub(self):
        """
        Defines the spacecraft object properties.
        """
        self.l_cube = 0.1  # m - length of cubesat side (10 cm)
        m_solar = 4.0  # kg - mass of solar panels (structure + solarCells + GNSS-R antennas) | Based on NanoAvionics M12P
        self.scObject.ModelTag = "sat-" + str(self.spacecraftIndex)
        self.scObject.hub.mHub = 32.0  # kg - spacecraft mass | Based on NanoAvionics M12P max spacecraft mass
        #Calculate the inertia tensor assuming a cuboid with 2 solar panels attached (NanoAvionics M12P, mass homogeneously distributed)
        Ix = 1/12 * (self.scObject.hub.mHub - 2*m_solar) * ((2*self.l_cube)**2 + (2*self.l_cube)**2) \
                        + 1/12 * m_solar * (2*self.l_cube)**2*2 \
                        +(np.sqrt((2*self.l_cube)**2 + self.l_cube**2))*m_solar *2  # kg*m^2 - spacecraft inertia about x-axis | Based on NanoAvionics M12P
        Iy = 1/12 * (self.scObject.hub.mHub - 2*m_solar) * ((2*self.l_cube)**2 + (3*self.l_cube)**2) \
                        + 1/12 * m_solar * (3*self.l_cube)**2*2 \
                        + self.l_cube**2 * m_solar *2  # kg*m^2 - spacecraft inertia about y-axis | Based on NanoAvionics M12P
        Iz = 1/12 * (self.scObject.hub.mHub - 2*m_solar) * ((2*self.l_cube)**2 + (3*self.l_cube)**2) \
                        + 1/12 * m_solar * ((2*self.l_cube)**2 + (3*self.l_cube)**2)  \
                            +(2*self.l_cube)**2 * m_solar*2  # kg*m^2 - spacecraft inertia about z-axis | Based on NanoAvionics M12P
        self.I_sc = [Ix, 0., 0.,
                     0., Iy, 0.,
                     0., 0., Iz]

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
        SimBase.EnvModel.groundStationBar.addSpacecraftToModel(self.scObject.scStateOutMsg)
        SimBase.EnvModel.groundStationSval.addSpacecraftToModel(self.scObject.scStateOutMsg)

    def SetEclipseObject(self, SimBase):
        """
        Adds the spacecraft to the eclipse module.
        """
        SimBase.EnvModel.eclipseObject.addSpacecraftToModel(self.scObject.scStateOutMsg)

    def SetWmmObject(self, SimBase):
        """
        Adds the spacecraft to the WMM magnetic field module.
        """
        SimBase.EnvModel.magneticField.addSpacecraftToModel(self.scObject.scStateOutMsg)

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
        # Define orthogonal RW pyramid
        # -- Pointing directions
        rwpyramidAngle = np.arccos(np.sqrt(1.0 / 3.0)) * mc.R2D
        rwElAngle = np.array([rwpyramidAngle, rwpyramidAngle, rwpyramidAngle, rwpyramidAngle]) * mc.D2R # Guesstimate
        rwAzimuthAngle = np.array([45.0, 135.0, 225.0, 315.0]) * mc.D2R                                 # Guesstimate
        rwPosVector = [[0.8, 0.8, 1.79070],                                                             # Guesstimate
                       [0.8, -0.8, 1.79070],
                       [-0.8, -0.8, 1.79070],
                       [-0.8, 0.8, 1.79070]]

        for elAngle, azAngle, posVector in zip(rwElAngle, rwAzimuthAngle, rwPosVector):
            gsHat = (rbk.Mi(-azAngle, 3).dot(rbk.Mi(elAngle, 2))).dot(np.array([1, 0, 0]))
            self.rwFactory.create('NanoAvionics_RW0',
                                  gsHat,
                                  rWB_B=posVector)

        self.numRW = self.rwFactory.getNumOfDevices()
        self.rwFactory.addToSpacecraft("RWArray" + str(self.spacecraftIndex), self.rwStateEffector, self.scObject)

    def SetThrusterDynEffector(self):
        #ALTERNATIVE -> ENPULSION IFM Nano thruster FEEP electric propulsion (https://www.enpulsion.com/ifm-nano/)
        """
        Defines the thruster state effector. | EPSS C2 thruster model
        """
        location = [[0.0, 0.0, 0.0]]
        direction = [[0.0, 0.0, 1.0]]

        # create the thruster devices by specifying the thruster type and its location and direction
        for pos_B, dir_B in zip(location, direction):
            self.thrusterFactory.create('EPSS_C2', pos_B, dir_B, useMinPulseTime=False)

        self.numThr = self.thrusterFactory.getNumOfDevices()

        # create thruster object container and tie to spacecraft object
        self.thrusterFactory.addToSpacecraft("thrusterFactory", self.thrusterDynamicEffector, self.scObject)

    def SetFuelTank(self):
        """
        Defines the fuel tank for the thrusters.
        """
        # Define the tank
        self.tankModel = fuelTank.FuelTankModelUniformBurn() #|https://satcatalog.s3.amazonaws.com/components/901/SatCatalog_-_NanoAvionics_-_EPSS_C2_-_Datasheet.pdf
        self.fuelTankStateEffector.setTankModel(self.tankModel)
        self.tankModel.propMassInit = 0.8                          # Initial fuel mass according to Nano Avionics M12P specs (1 kg)
        self.tankModel.maxFuelMass = 0.8                           # Fuel tank at maximum capacity at t=0 (1 kg)
        self.tankModel.r_TcT_TInit = [[0.0], [0.0], [0.0]]         # TODO update according to Nano Avionics M12P | position vector from tank CM to tank connection point
        self.fuelTankStateEffector.r_TB_B = [[0.0], [0.0], [0.0]]  # TODO update according to Nano Avionics M12P | position vector from spacecraft body frame origin to tank connection point
        self.tankModel.radiusTankInit = 1
        self.tankModel.lengthTank = 1

        # Add the tank and connect the thrusters
        self.scObject.addStateEffector(self.fuelTankStateEffector)
        self.fuelTankStateEffector.addThrusterSet(self.thrusterDynamicEffector)

    def SetReactionWheelPower(self):
        """Sets the reaction wheel power parameters"""
        for item in range(self.numRW):
            self.rwPowerList[item].ModelTag = self.scObject.ModelTag + "RWPower" + str(item)
            self.rwPowerList[item].basePowerNeed = 0.045  # baseline power draw, Watt # TODO Question: Is this idle power?
            self.rwPowerList[item].rwStateInMsg.subscribeTo(self.rwStateEffector.rwOutMsgs[item])
            self.rwPowerList[item].mechToElecEfficiency = 0.5 # TODO What is this? update according to Nano Avionics M12P

    def SetSolarPanel(self, SimBase):
        """Sets the solar panel"""
        self.solarPanel.ModelTag = "solarPanel"
        self.solarPanel.stateInMsg.subscribeTo(self.scObject.scStateOutMsg)
        self.solarPanel.sunEclipseInMsg.subscribeTo(SimBase.EnvModel.eclipseObject.eclipseOutMsgs[0])  # choose the earth message
        self.solarPanel.sunInMsg.subscribeTo(SimBase.EnvModel.gravFactory.spiceObject.planetStateOutMsgs[SimBase.EnvModel.gravBodyList.index('sun')])
        self.solarPanelAxis = [0, 0, 1]
        self.solarPanel.setPanelParameters(self.solarPanelAxis,  # panel normal vector in the body frame
                                           2*self.l_cube * 3*self.l_cube * 3,  # area, m^2 # Solar pannel area of NanoAvionics M12P
                                           0.295)  # efficiency according to NanoAvionics GaAs Solar Panels

    def SetPowerSink(self):
        """Defines the energy sink parameters"""
        self.powerSink.ModelTag = "powerSink"
        self.powerSink.nodePowerOut = -2.  # Watt

    def SetBattery(self):
        """Sets up the battery with all the power components

            The battery used is the NanoAvionics 4S6P battery pack
        """
        self.powerMonitor.ModelTag = "powerMonitor"
        self.powerMonitor.storageCapacity = 161 * 3600.0  # Convert from W-hr to Joule
        self.powerMonitor.storedCharge_Init = self.powerMonitor.storageCapacity * 0.6  # 40% depletion

        # attach the sources/sinks to the battery
        self.powerMonitor.addPowerNodeToModel(self.solarPanel.nodePowerOutMsg)
        self.powerMonitor.addPowerNodeToModel(self.powerSink.nodePowerOutMsg)
        self.powerMonitor.addPowerNodeToModel(self.simpleAntennaPower.nodePowerOutMsg)
        for item in range(self.numRW):
            self.powerMonitor.addPowerNodeToModel(self.rwPowerList[item].nodePowerOutMsg)

    def SetTam(self, SimBase):
        """Sets up the magnetometer"""
        self.tam.ModelTag = "ThreeAxisMagnetometer" + str(self.spacecraftIndex)
        # specify the optional tam variables
        self.tam.scaleFactor = 1.0               # TODO confirm
        self.tam.senNoiseStd = [0.0,  0.0, 0.0]  # TODO confirm

        # Add the spacecraft to the magnetic field model
        self.tam.stateInMsg.subscribeTo(self.scObject.scStateOutMsg)
        self.tam.magInMsg.subscribeTo(SimBase.EnvModel.magneticField.envOutMsgs[0])

    def SetMtbEffector(self, SimBase): # TODO: Is this already 3 orthogonally aligned MTBs?
        """Sets up the magnetorquer"""
        self.mtbEff.ModelTag = "MtbEff" + str(self.spacecraftIndex)
        # Add the the magnetic torquer to the spacecraft
        self.scObject.addDynamicEffector(self.mtbEff)

        # Connect MTB msgs / setup MTB cofiguration
        self.mtbConfigParams = messaging.MTBArrayConfigMsgPayload()
        self.mtbConfigParams.numMTB = 4
        # row major toque bar alignments
        self.mtbConfigParams.GtMatrix_B = [
        1., 0., 0., 0.70710678,
        0., 1., 0., 0.70710678,
        0., 0., 1., 0.]
        maxDipole = 0.1
        self.mtbConfigParams.maxMtbDipoles = [maxDipole]*self.mtbConfigParams.numMTB

        self.mtbParamsInMsg = messaging.MTBArrayConfigMsg()
        self.mtbParamsInMsg.write(self.mtbConfigParams) # TODO is this in the right place? (moved to FSW)

        self.mtbEff.magInMsg.subscribeTo(SimBase.EnvModel.magneticField.envOutMsgs[0])

    def SetDragEffector(self, SimBase):
        """Sets up the drag force effector"""
        self.dragEff.ModelTag = "DragEff"
        projArea = 2*self.l_cube * 3*self.l_cube * 3  # drag area in m^2 # TODO update according to Nano Avionics M12P
        dragCoeff = 2.1  # drag coefficient # TODO update according to Nano Avionics M12P (drag coeff for a "box")

        self.dragEff.coreParams.projectedArea = projArea
        self.dragEff.coreParams.dragCoeff = dragCoeff

        # Add spacecraft to the Atmosphere model
        SimBase.EnvModel.atmosphere.addSpacecraftToModel(self.scObject.scStateOutMsg)

        # Add the drag effector to the spacecraft
        self.scObject.addDynamicEffector(self.dragEff)

        # Connect the drag effector to the atmosphere model
        self.dragEff.atmoDensInMsg.subscribeTo(SimBase.EnvModel.atmosphere.envOutMsgs[self.spacecraftIndex])

        # Force logging (optional)
        # forceLog = dragEffector.logger("forceExternal_B", samplingTime)

    def SetInstrument(self):
        """Sets up the simple instrument"""
        self.instrument.ModelTag = "SimpleInstrument" + str(self.spacecraftIndex)
        self.instrument.nodeBaudRate = 1e6  # 1 Mbps | TODO update according to GNSS-R instrument specs
        self.instrument.nodeDataName = "GNSS-R_Data" + str(self.spacecraftIndex)

    def SetTransmitter(self, SimBase):
        """Sets up the transmitter (simple antenna + link budget)"""
        self.transmitter.ModelTag = "SpaceToGroundTransmitter" + str(self.spacecraftIndex)
        self.transmitter.nodeBaudRate = -1e6  # 1 Mbps | According to NanoAvionics M12P specs: Uplink 2025-2110 MHz Up to 1Mbps / Downlink 2200-2290 MHz Up to 2Mbps
        self.transmitter.nodeDataName = "SpaceToGround_Data" + str(self.spacecraftIndex)
        self.transmitter.packetSize = -8e6  # bits
        self.transmitter.numBuffers = 2
        self.transmitter.addAccessMsgToTransmitter(SimBase.EnvModel.groundStationSval.accessOutMsgs[-1]) # TODO Check this | self.transmitter.addStorageUnitToTransmitter()
        self.transmitter.addStorageUnitToTransmitter(self.dataMonitor.storageUnitDataOutMsg)

    def SetDataMonitor(self):
        """Sets up the data monitor (partitioned storage unit)"""
        self.dataMonitor.ModelTag = "GnssDataMonitor" + str(self.spacecraftIndex)
        self.dataMonitor.storageCapacity = 32e9  # 32 GB | Nano Avionics M12P max data storage capacity is 32 GB NAND
        self.dataMonitor.addDataNodeToModel(self.instrument.nodeDataOutMsg)
#        self.dataMonitor.addDataNodeToModel(self.transmitter.nodeDataOutMsg) # TODO add transmitter when available
        self.dataMonitor.addPartition("GNSS-R Partition 1")
        self.dataMonitor.addPartition("GNSS-R Partition 2")

    def SetSimpleAntenna(self, SimBase):
        """Sets up the simple antenna"""
        self.simpleAntenna = simpleAntenna.SimpleAntenna()
        self.simpleAntenna.setAntennaName("spaceSimpleAntenna" + str(self.spacecraftIndex))
        self.simpleAntenna.setAntennaDirectivity_dB(10.0)                # [dBi] 10 dBi, Guesstimate for smallSat S-Band antenna
        self.simpleAntenna.setAntennaFrequency(2.1e9)                    # [Hz]  2.1GHz, S-Band according to Nano Avionics M12P specs
        self.simpleAntenna.setAntennaBandwidth(2.0e6)                    # [Hz]  2.0MHz, S-Band (typical S-Band bandwidth)
        self.simpleAntenna.setAntennaHpbwRatio(1.0)                      # [-]   Symetrical antenna beam.
        self.simpleAntenna.setAntennaP_Tx(6.0)                           # [W]   6W, according to Nano Avionics M12P specs
        self.simpleAntenna.setAntennaP_Rx(0.1)                           # [W]   0.1W, Guesstimate
        self.simpleAntenna.setAntennaRadEfficiency(0.7)                  # [-]   Guesstimate (typical antenna efficiency)
        self.simpleAntenna.setAntennaEquivalentNoiseTemp(50)             # [K]   Guesstimate, noise temperature of the antenna
        self.simpleAntenna.setAntennaPositionBodyFrame([0.5, 0.0, 0.0])  # [m]   body fixed position
        self.simpleAntenna.scStateInMsg.subscribeTo(self.scObject.scStateOutMsg)
        self.simpleAntenna.sunInMsg.subscribeTo(SimBase.EnvModel.gravFactory.spiceObject.planetStateOutMsgs[SimBase.EnvModel.gravBodyList.index('sun')])
        self.simpleAntenna.addPlanetToModel(SimBase.EnvModel.gravFactory.spiceObject.planetStateOutMsgs[SimBase.EnvModel.gravBodyList.index('earth')])
        self.simpleAntenna.addPlanetToModel(SimBase.EnvModel.gravFactory.spiceObject.planetStateOutMsgs[SimBase.EnvModel.gravBodyList.index('moon')])

    def setSimpleAntennaPower(self):
        """Sets up the simple antenna power consumption"""
        self.simpleAntennaPower = antennaPower.AntennaPower()
        self.simpleAntennaPower.ModelTag = "AntennaPower" + str(self.spacecraftIndex)
        self.simpleAntennaPower.antennaSetStateInMsg.subscribeTo(self.simpleAntenna.antennaOutMsg)
        self.simpleAntennaPower.basePowerNeed = 0.0  # Watt

    # Global call to initialize every module
    def InitAllDynObjects(self, SimBase):
        """
        Initializes all dynamic objects.
        """
        # Initialize SC Hub (SC mass and balance)
        self.SetSpacecraftHub()
        # Connect the spacecraft to the environment modules
        self.SetGravityBodies(SimBase) # Gravity Bodys
        self.SetEclipseObject(SimBase) # Eclipse (SC in shadow / light)
        self.SetWmmObject(SimBase) # World magnetic model
        # Set ground locations
        self.SetGroundLocations(SimBase)
        # Initialize SC modules
        self.SetReactionWheelDynEffector()
        self.SetReactionWheelPower()
        self.SetThrusterDynEffector()
        self.SetFuelTank()
        self.SetSimpleNavObject()
        self.SetSimpleMassPropsObject()
        self.SetSolarPanel(SimBase)
        self.SetPowerSink()
        self.SetSimpleAntenna(SimBase)
        self.setSimpleAntennaPower()
        self.SetBattery()
        self.SetTam(SimBase)
        self.SetMtbEffector(SimBase)
        self.SetDragEffector(SimBase)
        self.SetInstrument()
        self.SetTransmitter(SimBase)
        self.SetDataMonitor()
#        self.SetLinkBudget()
