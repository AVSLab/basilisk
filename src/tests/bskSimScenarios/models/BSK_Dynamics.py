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
import math
import numpy as np
from Basilisk.utilities import macros as mc
from Basilisk.utilities import unitTestSupport as sp
from Basilisk.simulation import (spacecraftPlus, gravityEffector, extForceTorque, simple_nav, spice_interface,
                                 reactionWheelStateEffector)


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
        self.earthGravBody = gravityEffector.GravBodyData()
        self.extForceTorqueObject = extForceTorque.ExtForceTorque()
        self.simpleNavObject = simple_nav.SimpleNav()
        self.rwStateEffector = reactionWheelStateEffector.ReactionWheelStateEffector()

        # Initialize all modules and write init one-time messages
        self.InitAllDynObjects()
        self.WriteInitDynMessages(SimBase)

        # Assign initialized modules to tasks
        SimBase.AddModelToTask(self.taskName, self.scObject, None, 201)
        SimBase.AddModelToTask(self.taskName, self.simpleNavObject, None, 109)
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
        self.scObject.hub.useTranslation = True
        self.scObject.hub.useRotation = True

    def SetSpacecraftDynObject(self):
        self.scObject.addDynamicEffector(self.extForceTorqueObject)
        self.scObject.addStateEffector(self.rwStateEffector)
        self.scObject.scStateOutMsgName = "inertial_state_output"


    def SetGravityBodies(self):
        self.gravBodyList = []
        self.spicePlanetNames = []

        self.earthGravBody.bodyInMsgName = "earth_planet_data"
        self.earthGravBody.outputMsgName = "earth_display_frame_data"
        self.earthGravBody.mu = 0.3986004415E+15  # meters^3/s^2
        self.earthGravBody.radEquator = 6378136.6  # meters
        self.earthGravBody.isCentralBody = True
        self.earthGravBody.useSphericalHarmParams = False

        self.gravBodyList.append(self.earthGravBody)
        self.spicePlanetNames.append(self.earthGravBody.bodyInMsgName[:-12])
        # -- Attach gravity model to spaceCraftPlus
        self.scObject.gravField.gravBodies = spacecraftPlus.GravBodyVector(self.gravBodyList)

    def SetExternalForceTorqueObject(self):
        self.extForceTorqueObject.ModelTag = "externalDisturbance"

    def SetSimpleNavObject(self):
        self.simpleNavObject.ModelTag = "SimpleNavigation"

    def SetReactionWheelDynEffector(self):
        self.rwStateEffector.ModelTag = "RWStateEffector"
        self.rwStateEffector.InputCmds = "reactionwheel_cmds"
        # Define common parameters for the wheels if non-zero
        rwMaxTorque = 0.2
        rwMinTorque = 0.004  # verbal spec for min-torque from HI
        rwStaticFrictionTorque = 0.0005  # arbitrary
        rwStaticImbalance = 8.5E-6  # kg-m, Honeywell HR-16 100 Nms standard balance option EOL
        rwDynamicImbalance = 28.3E-7  # kg-m^2, Honeywell HR-16 100 Nms standard balance option EOL
        rwJs = 1.0 / (6000.0 * math.pi * 2.0) #  spinning axis inertia
        # Define orthogonal RW pyramid
        # -- Pointing directions
        rwElAngle = 40.0 * math.pi / 180.0
        rwClockAngle = 45.0 * math.pi / 180.0
        RWGsList = []
        RWGsList.append([math.sin(rwElAngle) * math.sin(rwClockAngle), math.sin(rwElAngle) * math.cos(rwClockAngle),
                         -math.cos(rwElAngle)])
        rwClockAngle += 180.0 * math.pi / 180.0
        RWGsList.append([math.sin(rwElAngle) * math.sin(rwClockAngle), -math.sin(rwElAngle) * math.cos(rwClockAngle),
                         math.cos(rwElAngle)])
        rwClockAngle += 90.0 * math.pi / 180.0
        RWGsList.append([math.sin(rwElAngle) * math.sin(rwClockAngle), math.sin(rwElAngle) * math.cos(rwClockAngle),
                         -math.cos(rwElAngle)])
        rwClockAngle -= 1800.0 * math.pi / 180.0
        RWGsList.append([math.sin(rwElAngle) * math.sin(rwClockAngle), -math.sin(rwElAngle) * math.cos(rwClockAngle),
                         math.cos(rwElAngle)])
        # -- Wheel locations
        r_BList = [[0.8, 0.8, 1.79070],
                   [0.8, -0.8, 1.79070],
                   [-0.8, -0.8, 1.79070],
                   [-0.8, 0.8, 1.79070]]
        # Instantiate the RW State Effectors with the parameters defined previously
        for i in range(len(RWGsList)):
            RW = reactionWheelStateEffector.RWConfigSimMsg()
            RW.Js = rwJs
            RW.Jt = 0.5 * RW.Js
            RW.Jg = RW.Jt
            RW.U_s = rwStaticImbalance
            RW.U_d = rwDynamicImbalance
            RW.gsHat_B = np.array(RWGsList[i]).reshape(3, 1).tolist()
            w2Hat0_B = np.cross(RWGsList[i], [1, 0, 0])
            norm = np.linalg.norm(w2Hat0_B)
            if norm < 0.01:
                print "ERROR: Your spin-axis orthogonal vector is no-good!  Please re-do RWA ICs!!!"
            w2Hat0_B = w2Hat0_B / norm
            w3Hat0_B = np.cross(RWGsList[i], w2Hat0_B)
            w3Hat0_B /= np.linalg.norm(w3Hat0_B)
            RW.w2Hat0_B = w2Hat0_B.reshape(3, 1).tolist()
            RW.w3Hat0_B = w3Hat0_B.reshape(3, 1).tolist()
            RW.rWB_B = np.array(r_BList[i]).reshape(3, 1).tolist()
            RW.Omega = 0.0
            RW.theta = 0.0
            RW.u_f = rwStaticFrictionTorque
            RW.u_max = rwMaxTorque
            RW.u_min = rwMinTorque
            self.rwStateEffector.addReactionWheel(RW)

    def SetSpiceData(self, SimBase):
        ephemerisMessageName = self.earthGravBody.bodyInMsgName
        self.ephemerisSPICEObject.J2000Current = 0.0
        self.ephemerisSPICEObject.PositionVector = [0.0, 0.0, 0.0]
        self.ephemerisSPICEObject.VelocityVector = [0.0, 0.0, 0.0]
        self.ephemerisSPICEObject.J20002Pfix = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
        self.ephemerisSPICEObject.J20002Pfix_dot = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
        self.ephemerisSPICEObject.PlanetName = ephemerisMessageName
        ephemerisMessageSize = self.ephemerisSPICEObject.getStructSize()
        SimBase.TotalSim.CreateNewMessage(self.processName, ephemerisMessageName, ephemerisMessageSize, 2)
        SimBase.TotalSim.WriteMessageData(ephemerisMessageName, ephemerisMessageSize, 0, self.ephemerisSPICEObject)

    # Global call to initialize every module
    def InitAllDynObjects(self):
        self.SetSpacecraftHub()
        self.SetGravityBodies()
        self.SetExternalForceTorqueObject()
        self.SetSimpleNavObject()

        self.SetReactionWheelDynEffector()
        self.SetSpacecraftDynObject()

    # Global call to create every required one-time message
    def WriteInitDynMessages(self, SimBase):
        self.SetSpiceData(SimBase)
