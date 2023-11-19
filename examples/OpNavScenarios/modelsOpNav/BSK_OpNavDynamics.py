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
r"""
Overview
--------

``OpNavScenarios/models/BSK_OpNavDynamics.py`` is similar to the :ref:`Folder_BskSim` versions seen previously.
The main additions are
the instantiation of :ref:`vizInterface`, and the camera module.


"""


import inspect
import math
import os

import numpy as np
from Basilisk import __path__
from Basilisk.simulation import (spacecraft, extForceTorque, simpleNav,
                                 reactionWheelStateEffector, coarseSunSensor, eclipse,
                                 thrusterDynamicEffector, ephemerisConverter, vizInterface,
                                 camera)
from Basilisk.topLevelModules import pyswice
from Basilisk.utilities import RigidBodyKinematics as rbk
from Basilisk.utilities import macros as mc
from Basilisk.utilities import simIncludeThruster, simIncludeRW, simIncludeGravBody
from Basilisk.utilities import unitTestSupport
from Basilisk.utilities import vizSupport

bskPath = __path__[0]
filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

class BSKDynamicModels():
    """
    BSK Dynamics model for the op nav simulations
    """
    def __init__(self, SimBase, dynRate):
        # define empty class variables
        self.cameraRate = None
        self.cameraSize = None
        self.cameraFocal = None
        self.sun = None
        self.earth = None
        self.mars = None
        self.jupiter = None

        # Define process name, task name and task time-step
        self.processName = SimBase.DynamicsProcessName
        self.taskName = "DynamicsTask"
        self.taskCamera = "CameraTask"
        self.processTasksTimeStep = mc.sec2nano(dynRate)

        # Create task
        SimBase.dynProc.addTask(SimBase.CreateNewTask(self.taskName, self.processTasksTimeStep), 1000)
        SimBase.dynProc.addTask(SimBase.CreateNewTask(self.taskCamera, mc.sec2nano(60)), 999)

        # Instantiate Dyn modules as objects
        self.simBasePath = bskPath
        self.cameraMRP_CB =[]
        self.cameraRez = []

        self.rwFactory = simIncludeRW.rwFactory()
        self.scObject = spacecraft.Spacecraft()
        self.gravFactory = simIncludeGravBody.gravBodyFactory()
        self.extForceTorqueObject = extForceTorque.ExtForceTorque()
        self.SimpleNavObject = simpleNav.SimpleNav()
        self.vizInterface = vizInterface.VizInterface()
        self.eclipseObject = eclipse.Eclipse()
        self.CSSConstellationObject = coarseSunSensor.CSSConstellation()
        self.rwStateEffector = reactionWheelStateEffector.ReactionWheelStateEffector()
        self.thrustersDynamicEffector = thrusterDynamicEffector.ThrusterDynamicEffector()
        self.cameraMod = camera.Camera()
        self.cameraMod2 = camera.Camera()
        self.ephemObject = ephemerisConverter.EphemerisConverter()

        # Initialize all modules
        self.InitAllDynObjects(SimBase)

        # Assign initialized modules to tasks
        SimBase.AddModelToTask(self.taskName, self.scObject, 201)
        SimBase.AddModelToTask(self.taskName, self.SimpleNavObject, 109)
        SimBase.AddModelToTask(self.taskName, self.gravFactory.spiceObject, 200)
        SimBase.AddModelToTask(self.taskName, self.ephemObject, 199)
        SimBase.AddModelToTask(self.taskName, self.CSSConstellationObject, 299)
        SimBase.AddModelToTask(self.taskName, self.eclipseObject, 204)
        SimBase.AddModelToTask(self.taskName, self.rwStateEffector, 301)
        SimBase.AddModelToTask(self.taskName, self.extForceTorqueObject, 300)
        SimBase.AddModelToTask(self.taskName, self.vizInterface, 100)
        SimBase.AddModelToTask(self.taskCamera, self.cameraMod, 99)
        SimBase.AddModelToTask(self.taskCamera, self.cameraMod2, 99)

    # ------------------------------------------------------------------------------------------- #
    # These are module-initialization methods

    def SetCamera(self):
        self.cameraMod.imageInMsg.subscribeTo(self.vizInterface.opnavImageOutMsgs[0])
        self.cameraMod.ModelTag = "instrument"
        self.cameraMod.cameraIsOn = 1
        self.cameraMod.cameraID = 1
        self.cameraMod.saveImages = 0
        # Note, the `saveDir` variable is a path to a file.  This file name is then
        # appended with the frame number.
        imgFolder = os.path.abspath(os.path.dirname(__file__)) + "/img/"
        imgFileName = "cameraImages"
        self.cameraMod.saveDir = imgFolder + imgFileName
        if self.cameraMod.saveImages:
            if not os.path.exists(imgFolder):
                os.makedirs(imgFolder)
            print("Saving camera ID:" + str(self.cameraMod.cameraID) + " Images to: " + self.cameraMod.saveDir)

        # Noise parameters
        # self.cameraMod.gaussian = 2
        # self.cameraMod.darkCurrent = 0
        # self.cameraMod.saltPepper = 0.5
        # self.cameraMod.cosmicRays = 1
        self.cameraMod.blurParam = 3

        # Camera config
        self.cameraRate = 60
        self.cameraMod.renderRate = int(mc.sec2nano(self.cameraRate))  # in
        self.cameraMRP_CB = [0., 0., 0.]  # Arbitrary camera orientation
        self.cameraMod.sigma_CB = self.cameraMRP_CB
        self.cameraMod.cameraPos_B = [0., 0.2, 2.2]  # in meters
        self.cameraRez = [512, 512]  # [1024,1024] # in pixels
        self.cameraSize = [10.*1E-3, self.cameraRez[1]/self.cameraRez[0]*10.*1E-3]  # in m
        self.cameraMod.resolution = self.cameraRez
        self.cameraMod.fieldOfView = np.deg2rad(55)
        self.cameraMod.parentName = self.scObject.ModelTag
        self.cameraMod.skyBox = 'black'
        self.cameraFocal = self.cameraSize[1]/2./np.tan(self.cameraMod.fieldOfView/2.)  # in m

    def SetCamera2(self):
        # this 2nd camera is setup, but not used in the FSW image processing
        self.cameraMod2.imageInMsg.subscribeTo(self.vizInterface.opnavImageOutMsgs[1])
        self.cameraMod2.ModelTag = "cam2"
        self.cameraMod2.cameraIsOn = 1
        self.cameraMod2.cameraID = 3
        self.cameraMod2.saveImages = 0

        self.cameraMod2.UpdateState(0)

        imgFolder = os.path.abspath(os.path.dirname(__file__)) + "/imgTest/"
        imgFileName = "instr2Images"
        self.cameraMod2.saveDir = imgFolder + imgFileName
        if self.cameraMod2.saveImages:
            if not os.path.exists(imgFolder):
                os.makedirs(imgFolder)
            print("Saving camera ID:" + str(self.cameraMod2.cameraID) + " Images to: " + self.cameraMod2.saveDir)

        self.cameraMod2.blurParam = 3

        # Camera config
        self.cameraMod2.renderRate = int(mc.sec2nano(self.cameraRate))  # in
        self.cameraMod2.sigma_CB = [0., 0.5, 0.]
        self.cameraMod2.cameraPos_B = [0., 0.2, 2.2]  # in meters
        self.cameraMod2.resolution = self.cameraRez
        self.cameraMod2.fieldOfView = np.deg2rad(55)
        self.cameraMod2.parentName = self.scObject.ModelTag
        self.cameraMod2.skyBox = 'black'

    def SetVizInterface(self, SimBase):
        self.vizInterface = vizSupport.enableUnityVisualization(
            SimBase, self.taskName, [self.scObject]
            # , saveFile=__file__
            , rwEffectorList=[self.rwStateEffector]
            )
        # setup OpNav behavior by connecting camera module config message
        self.vizInterface.addCamMsgToModule(self.cameraMod.cameraConfigOutMsg)
        self.vizInterface.addCamMsgToModule(self.cameraMod2.cameraConfigOutMsg)
        self.vizInterface.opNavMode = 2
        self.vizInterface.settings.skyBox = "black"
        self.vizInterface.settings.ambient = 0.5

    def SetSpacecraftHub(self):
        self.scObject.ModelTag = "bskSat"
        # -- Crate a new variable for the sim sc inertia I_sc. Note: this is currently accessed from FSWClass
        self.I_sc = [900., 0., 0.,
                     0., 800., 0.,
                     0., 0., 600.]
        self.scObject.hub.mHub = 750.0  # kg - spacecraft mass
        self.scObject.hub.r_BcB_B = [[0.0], [0.0], [0.0]]  # m - position vector of body-fixed point B relative to CM
        self.scObject.hub.IHubPntBc_B = unitTestSupport.np2EigenMatrix3d(self.I_sc)

    def SetGravityEffector(self):
        """
        Specify what gravitational bodies to include in the simulation
        """

        timeInitString = "2019 DECEMBER 12 18:00:00.0"
        gravBodies = self.gravFactory.createBodies(['sun', 'earth', 'mars barycenter', 'jupiter barycenter'])
        gravBodies['mars barycenter'].isCentralBody = True
        self.sun = 0
        self.earth = 1
        self.mars = 2
        self.jupiter = 3

        gravBodies['mars barycenter'].useSphericalHarmonicsGravityModel(
            bskPath + '/supportData/LocalGravData/GGM2BData.txt', 2)

        self.scObject.gravField.gravBodies = spacecraft.GravBodyVector(list(self.gravFactory.gravBodies.values()))
        self.gravFactory.createSpiceInterface(bskPath + '/supportData/EphemerisData/',
                                              timeInitString,
                                              epochInMsg=True)

        self.gravFactory.spiceObject.referenceBase = "J2000"
        self.gravFactory.spiceObject.zeroBase = 'mars barycenter'

        pyswice.furnsh_c(self.gravFactory.spiceObject.SPICEDataPath + 'de430.bsp')  # solar system bodies
        pyswice.furnsh_c(self.gravFactory.spiceObject.SPICEDataPath + 'naif0012.tls')  # leap second file
        pyswice.furnsh_c(self.gravFactory.spiceObject.SPICEDataPath + 'de-403-masses.tpc')  # solar system masses
        pyswice.furnsh_c(self.gravFactory.spiceObject.SPICEDataPath + 'pck00010.tpc')  # generic Planetary Constants

    def SetEclipseObject(self):
        self.eclipseObject.sunInMsg.subscribeTo(self.gravFactory.spiceObject.planetStateOutMsgs[self.sun])
        for c in range(1, len(self.gravFactory.spiceObject.planetStateOutMsgs)):
            self.eclipseObject.addPlanetToModel(self.gravFactory.spiceObject.planetStateOutMsgs[c])
        self.eclipseObject.addSpacecraftToModel(self.scObject.scStateOutMsg)

    def SetExternalForceTorqueObject(self):
        self.extForceTorqueObject.ModelTag = "externalDisturbance"
        self.scObject.addDynamicEffector(self.extForceTorqueObject)

    def SetSimpleNavObject(self):
        self.SimpleNavObject.ModelTag = "SimpleNavigation"
        posSigma = 10.0
        velSigma = 0.001
        attSigma = 1.0 / 36000.0 * math.pi / 180.0
        attRateSig = 0.00005 * math.pi / 180.0
        sunSig = 0.1 * math.pi / 180.0
        DVsig = 0.005
        PMatrix = np.diag(
            [posSigma, posSigma, posSigma, velSigma, velSigma, velSigma, attSigma, attSigma, attSigma, attRateSig,
             attRateSig, attRateSig, sunSig, sunSig, sunSig, DVsig, DVsig, DVsig])
        errorBounds = [100000.0, 100000.0, 100000.0,  # Position
                       0.1, 0.1, 0.1,  # Velocity
                       1E-18 * math.pi / 180.0, 1E-18 * math.pi / 180.0, 1E-18 * math.pi / 180.0,  # Attitude
                       1E-18 * math.pi / 180.0, 1E-18 * math.pi / 180.0, 1E-18 * math.pi / 180.0,  # Attitude Rate
                       5.0 * math.pi / 180.0, 5.0 * math.pi / 180.0, 5.0 * math.pi / 180.0,  # Sun vector
                       0.5, 0.5, 0.5]  # Accumulated DV
        # PMatrix = np.zeros_like(np.eye(18))
        # errorBounds = [0.0] * 18  # Accumulated DV
        self.SimpleNavObject.walkBounds = np.array(errorBounds)
        self.SimpleNavObject.PMatrix = PMatrix
        self.SimpleNavObject.crossTrans = True
        self.SimpleNavObject.crossAtt = False

        self.SimpleNavObject.scStateInMsg.subscribeTo(self.scObject.scStateOutMsg)

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

        for elAngle, azAngle, posVector in zip(rwElAngle, rwAzimuthAngle, rwPosVector):
            gsHat = (rbk.Mi(-azAngle,3).dot(rbk.Mi(elAngle,2))).dot(np.array([1,0,0]))
            self.rwFactory.create('Honeywell_HR16',
                                  gsHat,
                                  maxMomentum=maxRWMomentum,
                                  rWB_B=posVector)

        self.rwFactory.addToSpacecraft("RWStateEffector", self.rwStateEffector, self.scObject)

    def SetACSThrusterStateEffector(self):
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
        thFactory.addToSpacecraft("Thrusters",
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
        for nHat_B, i in zip(nHat_B_List, list(range(1, numCSS+1))):
            CSS = coarseSunSensor.CoarseSunSensor()
            setupCSS(CSS)
            CSS.ModelTag = "CSS" + str(i)
            CSS.nHat_B = np.array(nHat_B)
            cssList.append(CSS)

        # assign the list of CSS devices to the CSS array class
        self.CSSConstellationObject.sensorList = coarseSunSensor.CSSVector(cssList)

    def SetEphemConvert(self):
        # Initialize the ephemeris module
        self.ephemObject.ModelTag = 'EphemData'
        self.ephemObject.addSpiceInputMsg(self.gravFactory.spiceObject.planetStateOutMsgs[self.mars])

    def SetSimpleGrav(self):
        planet = self.gravFactory.createMarsBarycenter()
        planet.isCentralBody = True
        self.scObject.gravField.gravBodies = \
            spacecraft.GravBodyVector(list(self.gravFactory.gravBodies.values()))

    # Global call to initialize every module
    def InitAllDynObjects(self, SimBase):
        self.SetSpacecraftHub()
        self.SetGravityEffector()
        # self.SetSimpleGrav()
        self.SetEclipseObject()
        self.SetExternalForceTorqueObject()
        self.SetSimpleNavObject()
        self.SetReactionWheelDynEffector()
        self.SetACSThrusterStateEffector()
        self.SetCSSConstellation()
        self.SetVizInterface(SimBase)
        self.SetEphemConvert()
        self.SetCamera()
        self.SetCamera2()


