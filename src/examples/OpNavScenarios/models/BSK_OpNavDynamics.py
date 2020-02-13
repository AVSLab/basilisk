''' '''
'''
 ISC License

 Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

import numpy as np
import math, sys, os, inspect
from Basilisk.utilities import macros as mc
from Basilisk.utilities import unitTestSupport as sp
from Basilisk.simulation import (spacecraftPlus, gravityEffector, extForceTorque, simple_nav, spice_interface,
                                 reactionWheelStateEffector, coarse_sun_sensor, eclipse, alg_contain, bore_ang_calc,
                                 thrusterDynamicEffector, ephemeris_converter, vizInterface,
                                 camera)
from Basilisk.utilities import simIncludeThruster, simIncludeRW, simIncludeGravBody, unitTestSupport
from Basilisk.utilities import RigidBodyKinematics as rbk
from Basilisk import pyswice
from Basilisk import __path__

from Basilisk.fswAlgorithms import attTrackingError

bskPath = __path__[0]
filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

class BSKDynamicModels():
    def __init__(self, SimBase, dynRate):
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

        self.SpiceObject = spice_interface.SpiceInterface()
        self.scObject = spacecraftPlus.SpacecraftPlus()
        self.gravFactory = simIncludeGravBody.gravBodyFactory()
        self.extForceTorqueObject = extForceTorque.ExtForceTorque()
        self.SimpleNavObject = simple_nav.SimpleNav()
        self.TruthNavObject = simple_nav.SimpleNav()
        self.vizInterface = vizInterface.VizInterface()
        self.instrumentSunBore = bore_ang_calc.BoreAngCalc()
        self.eclipseObject = eclipse.Eclipse()
        self.CSSConstellationObject = coarse_sun_sensor.CSSConstellation()
        self.rwStateEffector = reactionWheelStateEffector.ReactionWheelStateEffector()
        self.thrustersDynamicEffector = thrusterDynamicEffector.ThrusterDynamicEffector()
        self.cameraMod = camera.Camera()

        self.ephemObject = ephemeris_converter.EphemerisConverter()

        # Initialize all modules and write init one-time messages
        self.InitAllDynObjects()
        # self.WriteInitDynMessages(SimBase)

        self.truthRefErrors = attTrackingError.attTrackingErrorConfig()
        self.truthRefErrorsWrap = alg_contain.AlgContain(self.truthRefErrors,
                                                         attTrackingError.Update_attTrackingError,
                                                         attTrackingError.SelfInit_attTrackingError,
                                                         attTrackingError.CrossInit_attTrackingError,
                                                         attTrackingError.Reset_attTrackingError)
        self.truthRefErrorsWrap.ModelTag = "truthRefErrors"

        self.InitAllAnalysisObjects()

        # Assign initialized modules to tasks
        SimBase.AddModelToTask(self.taskName, self.scObject, None, 201)
        SimBase.AddModelToTask(self.taskName, self.SimpleNavObject, None, 109)
        SimBase.AddModelToTask(self.taskName, self.TruthNavObject, None, 110)
        SimBase.AddModelToTask(self.taskName, self.SpiceObject, 200)
        SimBase.AddModelToTask(self.taskName, self.ephemObject, 199)
        SimBase.AddModelToTask(self.taskName, self.CSSConstellationObject, None, 299)
        SimBase.AddModelToTask(self.taskName, self.eclipseObject, None, 204)
        SimBase.AddModelToTask(self.taskName, self.rwStateEffector, None, 301)
        SimBase.AddModelToTask(self.taskName, self.extForceTorqueObject, None, 300)
        SimBase.AddModelToTask(self.taskName, self.vizInterface, None, 100)
        SimBase.AddModelToTask(self.taskCamera, self.cameraMod, None, 99)

    # ------------------------------------------------------------------------------------------- #
    # These are module-initialization methods

    def SetCamera(self):
        self.cameraMod.imageInMsgName = "unity_image"
        self.cameraMod.imageOutMsgName = "opnav_image"
        self.cameraMod.cameraOutMsgName = "camera_config_data"
        self.cameraMod.saveImages = 0
        self.cameraMod.saveDir = 'Test/'

        # Noise parameters
        # self.cameraMod.gaussian = 2
        # self.cameraMod.darkCurrent = 0
        # self.cameraMod.saltPepper = 0.5
        # self.cameraMod.cosmicRays = 1
        self.cameraMod.blurParam = 3

        # Camera config
        self.cameraMod.cameraIsOn = 1
        self.cameraMod.cameraID = 1
        self.cameraRate = 60
        self.cameraMod.renderRate = int(mc.sec2nano(self.cameraRate))  # in
        self.cameraMRP_CB = [0.,0.,0.] # Arbitrary camera orientation
        self.cameraMod.sigma_CB = self.cameraMRP_CB
        self.cameraMod.cameraPos_B = [0., 0.2, 0.2]  # in meters
        self.cameraRez = [512, 512]  #[1024,1024] # in pixels
        self.cameraSize = [10.*1E-3, self.cameraRez[1]/self.cameraRez[0]*10.*1E-3]  # in m
        self.cameraMod.sensorSize = self.cameraSize
        self.cameraMod.resolution = self.cameraRez
        self.cameraMod.fieldOfView = np.deg2rad(55)  # in degrees
        self.cameraMod.focalLength = self.cameraMod.sensorSize[1]/2./np.tan(self.cameraMod.fieldOfView/2.) #in m
        self.cameraMod.parentName = 'inertial'
        self.cameraMod.skyBox = 'black'
        self.cameraFocal = self.cameraMod.focalLength


    def SetVizInterface(self):
        fileName = os.path.splitext(sys.argv[0])[0] + '_UnityViz.bin'
        home = os.path.dirname(fileName)
        if len(home) != 0:
            home += '/'
        namePath, name = os.path.split(fileName)
        if not os.path.isdir(home + '_VizFiles'):
            os.mkdir(home + '_VizFiles')
        fileName = home + '_VizFiles/' + name

        scData = vizInterface.VizSpacecraftData()
        scData.spacecraftName = 'inertial'
        self.vizInterface.scData.push_back(scData)
        self.vizInterface.opNavMode = 2
        self.vizInterface.opnavImageOutMsgName = "unity_image"#"opnav_image"#
        self.vizInterface.spiceInMsgName = vizInterface.StringVector(["earth_planet_data",
                                                                "mars barycenter_planet_data",
                                                                "sun_planet_data",
                                                                "jupiter barycenter_planet_data"
                                                                ])
        self.vizInterface.planetNames = vizInterface.StringVector(
            ["earth", "mars barycenter", "sun", "jupiter barycenter"])

        # self.vizInterface.spiceInMsgName = vizInterface.StringVector(["mars barycenter_planet_data"])
        # self.vizInterface.planetNames = vizInterface.StringVector(["mars barycenter"])
        # vizMessager.numRW = 4
        self.vizInterface.protoFilename = fileName

    def SetSpacecraftHub(self):
        self.scObject.ModelTag = "spacecraftBody"
        # -- Crate a new variable for the sim sc inertia I_sc. Note: this is currently accessed from FSWClass
        self.I_sc = [900., 0., 0.,
                     0., 800., 0.,
                     0., 0., 600.]
        self.scObject.hub.mHub = 750.0  # kg - spacecraft mass
        self.scObject.hub.r_BcB_B = [[0.0], [0.0], [0.0]]  # m - position vector of body-fixed point B relative to CM
        self.scObject.hub.IHubPntBc_B = sp.np2EigenMatrix3d(self.I_sc)
        self.scObject.scStateOutMsgName = "inertial_state_output"


    def SetTruthErrorsData(self):
        self.truthRefErrors.inputRefName = "att_ref_output"
        self.truthRefErrors.inputNavName = "truth_att_nav_output"
        self.truthRefErrors.outputDataName = "truth_att_errors"
    
    
    def SetgravityEffector(self):
        self.earthGravBody = gravityEffector.GravBodyData()
        self.earthGravBody.bodyInMsgName = "earth_planet_data"
        self.earthGravBody.mu = 0.3986004415E+15  # meters!
        self.earthGravBody.isCentralBody = False
        self.earthGravBody.useSphericalHarmParams = False

        self.sunGravBody = gravityEffector.GravBodyData()
        self.sunGravBody.bodyInMsgName = "sun_planet_data"
        self.sunGravBody.mu = 1.32712440018E20  # meters!
        self.sunGravBody.isCentralBody = False
        self.sunGravBody.useSphericalHarmParams = False

        self.marsGravBody = gravityEffector.GravBodyData()
        self.marsGravBody.bodyInMsgName = "mars barycenter_planet_data"
        self.marsGravBody.mu = 4.2828371901284001E+13  # meters!
        self.marsGravBody.isCentralBody = True
        self.marsGravBody.useSphericalHarmParams = True
        gravityEffector.loadGravFromFile(
            self.simBasePath + '/supportData/LocalGravData/GGM2BData.txt',
                                         self.marsGravBody.spherHarm, 2)

        self.jupiterGravBody = gravityEffector.GravBodyData()
        self.jupiterGravBody.bodyInMsgName = "jupiter barycenter_planet_data"
        self.jupiterGravBody.mu = 1.266865349093058E17  # meters!
        self.jupiterGravBody.isCentralBody = False
        self.jupiterGravBody.useSphericalHarmParams = False

        self.scObject.gravField.gravBodies = spacecraftPlus.GravBodyVector([self.earthGravBody,
                                                                                self.sunGravBody, self.jupiterGravBody,
                                                                               self.marsGravBody])
        # self.scObject.gravField.gravBodies = spacecraftPlus.GravBodyVector([self.marsGravBody])

    def SetEclipseObject(self):
        self.eclipseObject.sunInMsgName = 'sun_planet_data'
        self.eclipseObject.addPlanetName('mars barycenter')
        self.eclipseObject.addPositionMsgName(self.scObject.scStateOutMsgName)

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

    def SetTruthNavObject(self):
        self.TruthNavObject.ModelTag = "TruthNavigation"
        PMatrix = np.zeros_like(np.eye(18))
        errorBounds = [0.0] * 18  # Accumulated DV
        self.TruthNavObject.walkBounds = np.array(errorBounds)
        self.TruthNavObject.PMatrix = PMatrix
        self.TruthNavObject.outputAttName = "truth_att_output"
        self.TruthNavObject.outputTransName = "truth_trans_output"

    def SetReactionWheelDynEffector(self):
        # Make a fresh RW factory instance, this is critical to run multiple times
        rwFactory = simIncludeRW.rwFactory()

        # specify RW momentum capacity
        maxRWMomentum = 50. # Nms

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
            rwFactory.create('Honeywell_HR16',
                             gsHat,
                             maxMomentum=maxRWMomentum,
                             rWB_B=posVector)

        rwFactory.addToSpacecraft("RWStateEffector", self.rwStateEffector, self.scObject)

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
        self.CSSConstellationObject.ModelTag = "cssConstellation"
        self.CSSConstellationObject.outputConstellationMessage = "CSSConstellation_output"

        # define single CSS element
        CSS_default = coarse_sun_sensor.CoarseSunSensor()
        CSS_default.fov = 80. * mc.D2R         # half-angle field of view value
        CSS_default.scaleFactor = 2.0

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
        for nHat_B, i in zip(nHat_B_List,range(1,numCSS+1)):
            CSS = coarse_sun_sensor.CoarseSunSensor(CSS_default)
            CSS.ModelTag = "CSS" + str(i) + "_sensor"
            CSS.cssDataOutMsgName = "CSS" + str(i) + "_output"
            CSS.nHat_B = np.array(nHat_B)
            CSS.sunEclipseInMsgName = "eclipse_data_0"
            cssList.append(CSS)

        # assign the list of CSS devices to the CSS array class
        self.CSSConstellationObject.sensorList = coarse_sun_sensor.CSSVector(cssList)

    def SetEphemConvert(self):
        # Initialize the ephermis module
        self.ephemObject.ModelTag = 'EphemData'
        planets = ["mars barycenter"]
        messageMap = {}
        for planet in planets:
            messageMap[planet + '_planet_data'] = planet + '_ephemeris_data'
        self.ephemObject.messageNameMap = ephemeris_converter.map_string_string(messageMap)

    def SetinstrumentSunBore(self):
        self.instrumentSunBore.ModelTag = "instrumentBoreSun"
        self.instrumentSunBore.StateString = "inertial_state_output"
        self.instrumentSunBore.celBodyString = "sun_planet_data"
        self.instrumentSunBore.OutputDataString = "instrument_sun_bore"
        self.instrumentSunBore.boreVec_B = [0.0, 1.0, 0.0]

    def SetSimpleGrav(self):
        # clear prior gravitational body and SPICE setup definitions
        self.marsGravBody = gravityEffector.GravBodyData()
        self.marsGravBody.bodyInMsgName = "mars barycenter_planet_data"
        self.marsGravBody.mu = 4.2828371901284001E+13  # meters!
        self.marsGravBody.isCentralBody = True
        self.marsGravBody.useSphericalHarmParams = False

        # attach gravity model to spaceCraftPlus
        self.scObject.gravField.gravBodies = spacecraftPlus.GravBodyVector([self.marsGravBody])

    def SetSpiceObject(self):
        self.SpiceObject.ModelTag = "SpiceInterfaceData"
        self.SpiceObject.SPICEDataPath = self.simBasePath + '/supportData/EphemerisData/'
        self.SpiceObject.UTCCalInit = "2019 DECEMBER 12 18:00:00.0"
        self.SpiceObject.outputBufferCount = 2
        self.SpiceObject.planetNames = spice_interface.StringVector(["earth", "mars barycenter", "sun", "jupiter barycenter"])
        # self.SpiceObject.planetNames = spice_interface.StringVector(["mars barycenter"])
        self.SpiceObject.referenceBase = "J2000"
        self.SpiceObject.zeroBase = "mars barycenter"

        pyswice.furnsh_c(self.SpiceObject.SPICEDataPath + 'de430.bsp')  # solar system bodies
        pyswice.furnsh_c(self.SpiceObject.SPICEDataPath + 'naif0012.tls')  # leap second file
        pyswice.furnsh_c(self.SpiceObject.SPICEDataPath + 'de-403-masses.tpc')  # solar system masses
        pyswice.furnsh_c(self.SpiceObject.SPICEDataPath + 'pck00010.tpc')  # generic Planetary Constants Kernel

    # Global call to initialize every module
    def InitAllDynObjects(self):
        self.SetSpacecraftHub()
        # self.SetgravityEffector()
        self.SetSimpleGrav()
        self.SetEclipseObject()
        self.SetExternalForceTorqueObject()
        self.SetSimpleNavObject()
        self.SetReactionWheelDynEffector()
        self.SetACSThrusterStateEffector()
        self.SetCSSConstellation()
        self.SetVizInterface()
        self.SetEphemConvert()
        self.SetCamera()

        self.SetSpiceObject()


    def InitAllAnalysisObjects(self):
        self.SetTruthNavObject()
        self.SetTruthErrorsData()
        self.SetinstrumentSunBore()

    # Global call to create every required one-time message
    def WriteInitDynMessages(self, SimBase):
        msgName = 'mars barycenter_planet_data'
        ephemData = spice_interface.SpicePlanetStateSimMsg()
        ephemData.J2000Current = 0.0
        ephemData.PositionVector = [0.0, 0.0, 0.0]
        ephemData.VelocityVector = [0.0, 0.0, 0.0]
        ephemData.J20002Pfix = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
        ephemData.J20002Pfix_dot = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
        ephemData.PlanetName = 'mars barycenter'
        # setting the msg structure name is required below to all the planet msg to be logged
        unitTestSupport.setMessage(SimBase.TotalSim, self.processName, msgName,
                                   ephemData, "SpicePlanetStateSimMsg")
        return


