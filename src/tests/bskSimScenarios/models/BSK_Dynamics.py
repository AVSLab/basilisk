''' '''
'''
ISC License

Copyright (c) 2016-2017, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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
from Basilisk.utilities import macros as mc
from Basilisk.utilities import unitTestSupport as sp
from Basilisk.simulation import (spacecraftPlus, gravityEffector, extForceTorque, simple_nav, spice_interface,
                                 reactionWheelStateEffector, coarse_sun_sensor, eclipse, imu_sensor)
from Basilisk.utilities import simIncludeRW, simIncludeGravBody
from Basilisk.utilities import RigidBodyKinematics as rbk
from Basilisk import pyswice
bskPath = '/Users/johnmartin/Basilisk'



class BSKDynamicModels():
    def __init__(self, SimBase):
        # Define process name, task name and task time-step
        self.processName = SimBase.DynamicsProcessName
        self.taskName = "DynamicsTask"
        self.processTasksTimeStep = mc.sec2nano(0.1)
        
        # Create task
        SimBase.dynProc.addTask(SimBase.CreateNewTask(self.taskName, self.processTasksTimeStep))
        
        # Instantiate Dyn modules as objects
        self.scObject = spacecraftPlus.SpacecraftPlus()
        self.ephemerisSPICEObject = spice_interface.SpicePlanetStateSimMsg()
        self.ephemerisSunSPICEObject = spice_interface.SpicePlanetStateSimMsg()
        self.earthGravBody = gravityEffector.GravBodyData()
        self.gravFactory = simIncludeGravBody.gravBodyFactory()
        self.extForceTorqueObject = extForceTorque.ExtForceTorque()
        self.simpleNavObject = simple_nav.SimpleNav()
        self.eclipseObject = eclipse.Eclipse()
        self.CSSObject = coarse_sun_sensor.CoarseSunSensor()
        self.CSSConstellationObject = coarse_sun_sensor.CSSConstellation()
        self.imuObject = imu_sensor.ImuSensor()
        self.rwStateEffector = reactionWheelStateEffector.ReactionWheelStateEffector()
        
        # Initialize all modules and write init one-time messages
        self.InitAllDynObjects()
        self.WriteInitDynMessages(SimBase)
        
        # Assign initialized modules to tasks
        SimBase.AddModelToTask(self.taskName, self.scObject, None, 201)
        SimBase.AddModelToTask(self.taskName, self.simpleNavObject, None, 109)
        SimBase.AddModelToTask(self.taskName, self.gravFactory.spiceObject, 200)
        SimBase.AddModelToTask(self.taskName, self.CSSObject, None, 202)
        SimBase.AddModelToTask(self.taskName, self.CSSConstellationObject, None, 203)
        SimBase.AddModelToTask(self.taskName, self.eclipseObject, None, 204)
        SimBase.AddModelToTask(self.taskName, self.imuObject, None, 205)
        SimBase.AddModelToTask(self.taskName, self.rwStateEffector, None, 301)
        SimBase.AddModelToTask(self.taskName, self.extForceTorqueObject, None, 300)

    
    # ------------------------------------------------------------------------------------------- #
    # These are module-initialization methods
    
    def SetSpacecraftHub(self):
        self.scObject.ModelTag = "spacecraftBody"
        # -- Crate a new variable for the sim sc inertia I_sc. Note: this is currently accessed from FSWClass
        self.I_sc = [900., 0., 0.,
                     0., 800., 0.,
                     0., 0., 600.]
        self.scObject.hub.mHub = 750.0  # kg - spacecraft mass
        self.scObject.hub.r_BcB_B = [[0.0], [0.0], [0.0]]  # m - position vector of body-fixed point B relative to CM
        self.scObject.hub.IHubPntBc_B = sp.np2EigenMatrix3d(self.I_sc)

    def SetSpacecraftDynObject(self):
        self.scObject.addDynamicEffector(self.extForceTorqueObject)
        self.scObject.scStateOutMsgName = "inertial_state_output"
    
    
    def SetGravityBodies(self):
        timeInitString = "2012 MAY 1 00:28:30.0"
        gravBodies = self.gravFactory.createBodies(['earth', 'sun', 'moon'])
        gravBodies['earth'].isCentralBody = True

        self.scObject.gravField.gravBodies = spacecraftPlus.GravBodyVector(self.gravFactory.gravBodies.values())
        self.gravFactory.createSpiceInterface(bskPath + '/supportData/EphemerisData/', timeInitString)
        self.gravFactory.spiceObject.zeroBase = 'Earth'

        pyswice.furnsh_c(self.gravFactory.spiceObject.SPICEDataPath + 'de430.bsp')  # solar system bodies
        pyswice.furnsh_c(self.gravFactory.spiceObject.SPICEDataPath + 'naif0012.tls')  # leap second file
        pyswice.furnsh_c(self.gravFactory.spiceObject.SPICEDataPath + 'de-403-masses.tpc')  # solar system masses
        pyswice.furnsh_c(self.gravFactory.spiceObject.SPICEDataPath + 'pck00010.tpc')  # generic Planetary Constants Kernel

    def SetEclipseObject(self):
        self.eclipseObject.sunInMsgName = 'sun_planet_data'
        self.eclipseObject.addPlanetName('earth')
        self.eclipseObject.addPositionMsgName(self.scObject.scStateOutMsgName)
    
    def SetExternalForceTorqueObject(self):
        self.extForceTorqueObject.ModelTag = "externalDisturbance"
    
    def SetSimpleNavObject(self):
        self.simpleNavObject.ModelTag = "SimpleNavigation"
    
    def SetCSSObject(self):
        self.CSSObject.cssDataOutMsgName = "singleCssOut"
        self.CSSObject.nHat_B = np.array([1., 0., 0.])
        self.CSSObject.sunEclipseInMsgName = "eclipse_data_0"
        self.CSSObject.sunInMsgName = "sun_planet_data"
        self.CSSObject.stateInMsgName = self.scObject.scStateOutMsgName
    
    def SetCSSConstellation(self):
        cssP11 = coarse_sun_sensor.CoarseSunSensor(self.CSSObject)
        cssP12 = coarse_sun_sensor.CoarseSunSensor(self.CSSObject)
        cssP13 = coarse_sun_sensor.CoarseSunSensor(self.CSSObject)
        cssP14 = coarse_sun_sensor.CoarseSunSensor(self.CSSObject)
        cssP11.cssDataOutMsgName = "cssP11Out"
        cssP12.cssDataOutMsgName = "cssP12Out"
        cssP13.cssDataOutMsgName = "cssP13Out"
        cssP14.cssDataOutMsgName = "cssP14Out"
        cssP11.nHat_B = [1. / np.sqrt(2.), 0., -1. / np.sqrt(2.)]
        cssP12.nHat_B = [1. / np.sqrt(2.), 1. / np.sqrt(2.), 0.]
        cssP13.nHat_B = [1. / np.sqrt(2.), 0., 1. / np.sqrt(2)]
        cssP14.nHat_B = [1. / np.sqrt(2.), -1. / np.sqrt(2.), 0.]
        constellationList = [cssP11, cssP12, cssP13, cssP14]
        self.CSSConstellationObject.ModelTag = "cssConstellation"
        self.CSSConstellationObject.sensorList = coarse_sun_sensor.CSSVector(constellationList)
        self.CSSConstellationObject.outputConstellationMessage = "CSSConstellation_output"
    
    def SetImuSensor(self):
        self.imuObject.InputStateMsg = self.scObject.scStateOutMsgName
        self.imuObject.OutputDataMsg = "imu_sensor_output"
    
    
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


    def SetSpiceData(self, SimBase):
        '''
        ephemerisMessageName = "earth_planet_data"
        self.ephemerisSPICEObject.J2000Current = 0.0
        self.ephemerisSPICEObject.PositionVector = [0.0, 0.0, 0.0]
        self.ephemerisSPICEObject.VelocityVector = [0.0, 0.0, 0.0]
        self.ephemerisSPICEObject.J20002Pfix = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
        self.ephemerisSPICEObject.J20002Pfix_dot = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
        self.ephemerisSPICEObject.PlanetName = ephemerisMessageName[:-12]
        ephemerisMessageSize = self.ephemerisSPICEObject.getStructSize()
        SimBase.TotalSim.CreateNewMessage(self.processName, ephemerisMessageName, ephemerisMessageSize, 2)
        SimBase.TotalSim.WriteMessageData(ephemerisMessageName, ephemerisMessageSize, 0, self.ephemerisSPICEObject)
        
        ephemerisSunMessageName = "sun_planet_data"
        self.ephemerisSunSPICEObject.J2000Current = 0.0
        self.ephemerisSunSPICEObject.PositionVector = [1.0 * om.AU * 1000.0, 0.0, 0.0]
        self.ephemerisSunSPICEObject.VelocityVector = [0.0, 0.0, 0.0]
        self.ephemerisSunSPICEObject.J20002Pfix = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
        self.ephemerisSunSPICEObject.J20002Pfix_dot = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
        self.ephemerisSunSPICEObject.PlanetName = ephemerisSunMessageName[:-12]
        ephemerisSunMessageSize = self.ephemerisSunSPICEObject.getStructSize()
        SimBase.TotalSim.CreateNewMessage(self.processName, ephemerisSunMessageName, ephemerisSunMessageSize, 2)
        SimBase.TotalSim.WriteMessageData(ephemerisSunMessageName, ephemerisSunMessageSize, 0, self.ephemerisSunSPICEObject)
        '''
    
    # Global call to initialize every module
    def InitAllDynObjects(self):
        self.SetSpacecraftHub()
        self.SetGravityBodies()
        self.SetExternalForceTorqueObject()
        self.SetSimpleNavObject()
        self.SetEclipseObject()
        self.SetCSSObject()
        self.SetCSSConstellation()
        self.SetImuSensor()
        
        self.SetReactionWheelDynEffector()
        self.SetSpacecraftDynObject()

    # Global call to create every required one-time message
    def WriteInitDynMessages(self, SimBase):
        self.SetSpiceData(SimBase)
