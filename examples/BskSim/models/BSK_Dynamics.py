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
from Basilisk.simulation import ephemerisConverter
from Basilisk.simulation import (spacecraft, extForceTorque, simpleNav,
                                 reactionWheelStateEffector, coarseSunSensor, eclipse)
from Basilisk.simulation import thrusterDynamicEffector
from Basilisk.utilities import RigidBodyKinematics as rbk
from Basilisk.utilities import macros as mc
from Basilisk.utilities import simIncludeRW, simIncludeGravBody
from Basilisk.utilities import simIncludeThruster
from Basilisk.utilities import unitTestSupport as sp

bskPath = __path__[0]


class BSKDynamicModels():
    """
    General bskSim simulation class that sets up the spacecraft simulation configuration.

    """
    def __init__(self, SimBase, dynRate):
        # define empty class variables
        self.sun = None
        self.earth = None
        self.moon = None
        self.epochMsg = None
        self.RW1 = None
        self.RW2 = None
        self.RW3 = None
        self.RW4 = None

        # Define process name, task name and task time-step
        self.processName = SimBase.DynamicsProcessName
        self.taskName = "DynamicsTask"
        self.processTasksTimeStep = mc.sec2nano(dynRate)

        # Create task
        SimBase.dynProc.addTask(SimBase.CreateNewTask(self.taskName, self.processTasksTimeStep))

        # Instantiate Dyn modules as objects
        self.scObject = spacecraft.Spacecraft()
        self.gravFactory = simIncludeGravBody.gravBodyFactory()
        self.rwFactory = simIncludeRW.rwFactory()
        self.extForceTorqueObject = extForceTorque.ExtForceTorque()
        self.simpleNavObject = simpleNav.SimpleNav()
        self.eclipseObject = eclipse.Eclipse()
        self.CSSConstellationObject = coarseSunSensor.CSSConstellation()
        self.rwStateEffector = reactionWheelStateEffector.ReactionWheelStateEffector()
        self.thrustersDynamicEffector = thrusterDynamicEffector.ThrusterDynamicEffector()
        self.EarthEphemObject = ephemerisConverter.EphemerisConverter()

        # Initialize all modules and write init one-time messages
        self.InitAllDynObjects()

        # Assign initialized modules to tasks
        SimBase.AddModelToTask(self.taskName, self.scObject, 201)
        SimBase.AddModelToTask(self.taskName, self.simpleNavObject, 109)
        SimBase.AddModelToTask(self.taskName, self.gravFactory.spiceObject, 200)
        SimBase.AddModelToTask(self.taskName, self.EarthEphemObject, 199)
        SimBase.AddModelToTask(self.taskName, self.CSSConstellationObject, 108)
        SimBase.AddModelToTask(self.taskName, self.eclipseObject, 204)
        SimBase.AddModelToTask(self.taskName, self.rwStateEffector, 301)
        SimBase.AddModelToTask(self.taskName, self.extForceTorqueObject, 300)
        
        SimBase.createNewEvent("addOneTimeRWFault", self.processTasksTimeStep, True,
            ["self.TotalSim.CurrentNanos>=self.oneTimeFaultTime and self.oneTimeRWFaultFlag==1"],
            ["self.DynModels.AddRWFault('friction',0.05,1, self.TotalSim.CurrentNanos)", "self.oneTimeRWFaultFlag=0"])

        
        SimBase.createNewEvent("addRepeatedRWFault", self.processTasksTimeStep, True,
            ["self.repeatRWFaultFlag==1"],
            ["self.DynModels.PeriodicRWFault(1./3000,'friction',0.005,1, self.TotalSim.CurrentNanos)", "self.setEventActivity('addRepeatedRWFault',True)"])

    # ------------------------------------------------------------------------------------------- #
    # These are module-initialization methods

    def SetSpacecraftHub(self):
        """
        Specify the spacecraft hub parameters.
        """
        self.scObject.ModelTag = "bskSat"
        # -- Crate a new variable for the sim sc inertia I_sc. Note: this is currently accessed from FSWClass
        self.I_sc = [900., 0., 0.,
                     0., 800., 0.,
                     0., 0., 600.]
        self.scObject.hub.mHub = 750.0  # kg - spacecraft mass
        self.scObject.hub.r_BcB_B = [[0.0], [0.0], [0.0]]  # m - position vector of body-fixed point B relative to CM
        self.scObject.hub.IHubPntBc_B = sp.np2EigenMatrix3d(self.I_sc)

    def SetGravityBodies(self):
        """
        Specify what gravitational bodies to include in the simulation
        """
        timeInitString = "2012 MAY 1 00:28:30.0"
        gravBodies = self.gravFactory.createBodies(['sun', 'earth', 'moon'])
        gravBodies['earth'].isCentralBody = True
        self.sun = 0
        self.earth = 1
        self.moon = 2

        self.scObject.gravField.gravBodies = spacecraft.GravBodyVector(list(self.gravFactory.gravBodies.values()))
        self.gravFactory.createSpiceInterface(bskPath + '/supportData/EphemerisData/',
                                              timeInitString,
                                              epochInMsg=True)
        self.epochMsg = self.gravFactory.epochMsg

        self.gravFactory.spiceObject.zeroBase = 'Earth'

        self.EarthEphemObject.addSpiceInputMsg(self.gravFactory.spiceObject.planetStateOutMsgs[self.earth])

    def SetEclipseObject(self):
        """
        Specify what celestial object is causing an eclipse message.
        """
        self.eclipseObject.ModelTag = "eclipseObject"
        self.eclipseObject.sunInMsg.subscribeTo(self.gravFactory.spiceObject.planetStateOutMsgs[self.sun])
        # add all celestial objects in spiceObjects except for the sun (0th object)
        for c in range(1, len(self.gravFactory.spiceObject.planetStateOutMsgs)):
            self.eclipseObject.addPlanetToModel(self.gravFactory.spiceObject.planetStateOutMsgs[c])
        self.eclipseObject.addSpacecraftToModel(self.scObject.scStateOutMsg)

    def SetExternalForceTorqueObject(self):
        """Set the external force and torque object."""
        self.extForceTorqueObject.ModelTag = "externalDisturbance"
        self.scObject.addDynamicEffector(self.extForceTorqueObject)

    def SetSimpleNavObject(self):
        """Set the navigation sensor object."""
        self.simpleNavObject.ModelTag = "SimpleNavigation"
        self.simpleNavObject.scStateInMsg.subscribeTo(self.scObject.scStateOutMsg)

    def SetReactionWheelDynEffector(self):
        """Set the 4 reaction wheel devices."""
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

        gsHat = (rbk.Mi(-rwAzimuthAngle[0], 3).dot(rbk.Mi(rwElAngle[0], 2))).dot(np.array([1, 0, 0]))
        self.RW1 = self.rwFactory.create('Honeywell_HR16',
                                         gsHat,
                                         maxMomentum=maxRWMomentum,
                                         rWB_B=rwPosVector[0])
        
        gsHat = (rbk.Mi(-rwAzimuthAngle[1], 3).dot(rbk.Mi(rwElAngle[1], 2))).dot(np.array([1, 0, 0]))
        self.RW2 = self.rwFactory.create('Honeywell_HR16',
                                         gsHat,
                                         maxMomentum=maxRWMomentum,
                                         rWB_B=rwPosVector[1])

        gsHat = (rbk.Mi(-rwAzimuthAngle[2], 3).dot(rbk.Mi(rwElAngle[2], 2))).dot(np.array([1, 0, 0]))
        self.RW3 = self.rwFactory.create('Honeywell_HR16',
                                         gsHat,
                                         maxMomentum=maxRWMomentum,
                                         rWB_B=rwPosVector[2])
            
        gsHat = (rbk.Mi(-rwAzimuthAngle[3], 3).dot(rbk.Mi(rwElAngle[3], 2))).dot(np.array([1, 0, 0]))
        self.RW4 = self.rwFactory.create('Honeywell_HR16',
                                         gsHat,
                                         maxMomentum=maxRWMomentum,
                                         rWB_B=rwPosVector[3])

        self.rwFactory.addToSpacecraft("RWA", self.rwStateEffector, self.scObject)

    def SetThrusterStateEffector(self):
        """Set the 8 ACS thrusters."""
        # Make a fresh TH factory instance, this is critical to run multiple times
        thFactory = simIncludeThruster.thrusterFactory()

        # 8 thrusters are modeled that act in pairs to provide the desired torque
        thPos = [
            [825.5/1000.0, 880.3/1000.0, 1765.3/1000.0],
            [825.5/1000.0, 880.3/1000.0, 260.4/1000.0],
            [880.3/1000.0, 825.5/1000.0, 1765.3/1000.0],
            [880.3/1000.0, 825.5/1000.0, 260.4/1000.0],
            [-825.5/1000.0, -880.3/1000.0, 1765.3/1000.0],
            [-825.5/1000.0, -880.3/1000.0, 260.4/1000.0],
            [-880.3/1000.0, -825.5/1000.0, 1765.3/1000.0],
            [-880.3/1000.0, -825.5/1000.0, 260.4/1000.0]
                 ]
        thDir = [
            [0.0, -1.0, 0.0],
            [0.0, -1.0, 0.0],
            [-1.0, 0.0, 0.0],
            [-1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 1.0, 0.0],
            [1.0, 0.0, 0.0],
            [1.0, 0.0, 0.0]
        ]
        for pos_B, dir_B in zip(thPos, thDir):
            thFactory.create(
                'MOOG_Monarc_1'
                , pos_B
                , dir_B
            )
        # create thruster object container and tie to spacecraft object
        thFactory.addToSpacecraft("ACS Thrusters",
                                  self.thrustersDynamicEffector,
                                  self.scObject)

    def SetCSSConstellation(self):
        """Set the 8 CSS sensors"""
        self.CSSConstellationObject.ModelTag = "cssConstellation"

        def setupCSS(cssDevice):
            cssDevice.fov = 80. * mc.D2R         # half-angle field of view value
            cssDevice.scaleFactor = 2.0
            cssDevice.sunInMsg.subscribeTo(self.gravFactory.spiceObject.planetStateOutMsgs[self.sun])
            cssDevice.stateInMsg.subscribeTo(self.scObject.scStateOutMsg)
            cssDevice.sunEclipseInMsg.subscribeTo(self.eclipseObject.eclipseOutMsgs[0])
            cssDevice.this.disown()

        # setup CSS sensor normal vectors in body frame components
        nHat_B_List = [
            [0.0, 0.707107, 0.707107],
            [0.707107, 0., 0.707107],
            [0.0, -0.707107, 0.707107],
            [-0.707107, 0., 0.707107],
            [0.0, -0.965926, -0.258819],
            [-0.707107, -0.353553, -0.612372],
            [0., 0.258819, -0.965926],
            [0.707107, -0.353553, -0.612372]
        ]
        numCSS = len(nHat_B_List)

        # store all
        cssList = []
        for nHat_B, i in zip(nHat_B_List, list(range(1,numCSS+1))):
            CSS = coarseSunSensor.CoarseSunSensor()
            setupCSS(CSS)
            CSS.ModelTag = "CSS" + str(i)
            CSS.nHat_B = np.array(nHat_B)
            cssList.append(CSS)

        # assign the list of CSS devices to the CSS array class
        self.CSSConstellationObject.sensorList = coarseSunSensor.CSSVector(cssList)

    # Method for adding reaction wheel faults
    def PeriodicRWFault(self, probability, faultType, fault, faultRW, currentTime):
        """
        Adds a fault periodically. Probability is the chance of the fault occurring per update.
        """
        if np.random.uniform() < probability:
            self.AddRWFault(faultType, fault, faultRW, currentTime)
        
        
    
    def AddRWFault(self, faultType, fault, faultRW, currentTime):
        """
        Adds a static friction fault to the reaction wheel.
        """
        self.RWFaultLog.append([faultType, fault, faultRW, currentTime*mc.NANO2MIN])
        if faultType == "friction":
            if faultRW == 1:
                self.RW1.fCoulomb += fault
            elif faultRW == 2:
                self.RW2.fCoulomb += fault
            elif faultRW == 3:
                self.RW3.fCoulomb += fault
            elif faultRW == 4:
                self.RW4.fCoulomb += fault
        else:
            print("Invalid fault type. No fault added.")

    # Global call to initialize every module
    def InitAllDynObjects(self):
        """
        Initialize all the dynamics objects.
        """
        self.SetSpacecraftHub()
        self.SetGravityBodies()
        self.SetExternalForceTorqueObject()
        self.SetSimpleNavObject()
        self.SetEclipseObject()
        self.SetCSSConstellation()

        self.SetReactionWheelDynEffector()
        self.SetThrusterStateEffector()

