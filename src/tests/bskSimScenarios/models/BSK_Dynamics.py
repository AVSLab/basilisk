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
from Basilisk.utilities import macros as mc
from Basilisk.utilities import unitTestSupport as sp
from Basilisk.simulation import (spacecraftPlus, gravityEffector, extForceTorque, simple_nav, spice_interface,
                                 reactionWheelStateEffector, coarse_sun_sensor, eclipse, imu_sensor)
from Basilisk.simulation import thrusterDynamicEffector
from Basilisk.utilities import simIncludeThruster
from Basilisk.utilities import simIncludeRW, simIncludeGravBody
from Basilisk.utilities import RigidBodyKinematics as rbk
from Basilisk import pyswice
from Basilisk import __path__
bskPath = __path__[0]



class BSKDynamicModels():
    def __init__(self, SimBase, dynRate):
        # Define process name, task name and task time-step
        self.processName = SimBase.DynamicsProcessName
        self.taskName = "DynamicsTask"
        self.processTasksTimeStep = mc.sec2nano(dynRate)
        
        # Create task
        SimBase.dynProc.addTask(SimBase.CreateNewTask(self.taskName, self.processTasksTimeStep))
        
        # Instantiate Dyn modules as objects
        self.scObject = spacecraftPlus.SpacecraftPlus()
        self.gravFactory = simIncludeGravBody.gravBodyFactory()
        self.extForceTorqueObject = extForceTorque.ExtForceTorque()
        self.simpleNavObject = simple_nav.SimpleNav()
        self.eclipseObject = eclipse.Eclipse()
        self.CSSConstellationObject = coarse_sun_sensor.CSSConstellation()
        self.rwStateEffector = reactionWheelStateEffector.ReactionWheelStateEffector()
        self.thrustersDynamicEffector = thrusterDynamicEffector.ThrusterDynamicEffector()

        # Initialize all modules and write init one-time messages
        self.InitAllDynObjects()
        self.WriteInitDynMessages(SimBase)
        
        # Assign initialized modules to tasks
        SimBase.AddModelToTask(self.taskName, self.scObject, None, 201)
        SimBase.AddModelToTask(self.taskName, self.simpleNavObject, None, 109)
        SimBase.AddModelToTask(self.taskName, self.gravFactory.spiceObject, 200)
        SimBase.AddModelToTask(self.taskName, self.CSSConstellationObject, None, 108)
        SimBase.AddModelToTask(self.taskName, self.eclipseObject, None, 204)
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
        self.scObject.addDynamicEffector(self.extForceTorqueObject)

    def SetSimpleNavObject(self):
        self.simpleNavObject.ModelTag = "SimpleNavigation"
    
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

    def SetThrusterStateEffector(self):
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



    # Global call to initialize every module
    def InitAllDynObjects(self):
        self.SetSpacecraftHub()
        self.SetGravityBodies()
        self.SetExternalForceTorqueObject()
        self.SetSimpleNavObject()
        self.SetEclipseObject()
        self.SetCSSConstellation()
        
        self.SetReactionWheelDynEffector()
        self.SetThrusterStateEffector()

    # Global call to create every required one-time message
    def WriteInitDynMessages(self, SimBase):
        return


