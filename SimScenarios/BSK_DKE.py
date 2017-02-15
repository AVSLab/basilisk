import sys, os, inspect

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
bskName = 'Basilisk'
splitPath = path.split(bskName)
bskPath = splitPath[0] + '/' + bskName + '/'
sys.path.append(bskPath + 'modules')
sys.path.append(bskPath + 'PythonModules')

import macros as mc
import unitTestSupport as sp

import spacecraftPlus
import gravityEffector
import ExtForceTorque
import simple_nav
import spice_interface


class DynamicsClass():
    def __init__(self, SimBase):
        # Define process name, task name and task time-step
        self.processName = SimBase.DynamicsProcessName
        self.taskName = "DynamicsTask"
        self.taskTimeStep = mc.sec2nano(0.1)

        # Create task
        SimBase.dynProc.addTask(SimBase.CreateNewTask(self.taskName, self.taskTimeStep))

        # Instantiate Dyn modules as objects
        self.scObject = spacecraftPlus.SpacecraftPlus()
        self.extForceTorqueObject = ExtForceTorque.ExtForceTorque()
        self.simpleNavObject = simple_nav.SimpleNav()

        # Initialize all modules and write init one-time messages
        self.InitAllDynObjects()
        self.WriteInitDynMessages(SimBase)

        # Assign initialized modules to tasks
        SimBase.AddModelToTask(self.taskName, self.scObject)
        SimBase.AddModelToTask(self.taskName, self.extForceTorqueObject)
        SimBase.AddModelToTask(self.taskName, self.simpleNavObject)


    # ------------------------------------------------------------------------------------------- #
    # These are module-initialization methods
    def SetSpacecraftObject(self):
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
        return

    def SetGravityBodies(self):
        self.gravBodyList = []
        self.spicePlanetNames = []

        self.earthGravBody = gravityEffector.GravBodyData()
        self.earthGravBody.bodyInMsgName = "earth_planet_data"
        self.earthGravBody.outputMsgName = "earth_display_frame_data"
        self.earthGravBody.mu = 0.3986004415E+15  # meters^3/s^2
        self.earthGravBody.radEquator = 6378136.6  # meters
        self.earthGravBody.isCentralBody = True
        self.earthGravBody.useSphericalHarmParams = False

        self.gravBodyList.append(self.earthGravBody)
        self.spicePlanetNames.append(self.earthGravBody.bodyInMsgName[:-12])
        self.mu = self.earthGravBody.mu
        # -- Attach gravity model to spaceCraftPlus
        self.scObject.gravField.gravBodies = spacecraftPlus.GravBodyVector(self.gravBodyList)
        return

    def SetExternalForceTorqueObject(self):
        self.extForceTorqueObject.ModelTag = "externalDisturbance"
        self.scObject.addDynamicEffector(self.extForceTorqueObject)
        return

    def SetSimpleNavObject(self):
        self.simpleNavObject.ModelTag = "SimpleNavigation"
        return

    def SetSpiceData(self, SimBase):
        ephemerisMessageName = self.earthGravBody.bodyInMsgName
        self.ephemerisSPICEObject = spice_interface.SpicePlanetState()
        self.ephemerisSPICEObject.J2000Current = 0.0
        self.ephemerisSPICEObject.PositionVector = [0.0, 0.0, 0.0]
        self.ephemerisSPICEObject.VelocityVector = [0.0, 0.0, 0.0]
        self.ephemerisSPICEObject.J20002Pfix = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
        self.ephemerisSPICEObject.J20002Pfix_dot = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
        self.ephemerisSPICEObject.PlanetName = ephemerisMessageName
        ephemerisMessageSize = self.ephemerisSPICEObject.getStructSize()
        SimBase.TotalSim.CreateNewMessage(self.processName, ephemerisMessageName, ephemerisMessageSize, 2)
        SimBase.TotalSim.WriteMessageData(ephemerisMessageName, ephemerisMessageSize, 0, self.ephemerisSPICEObject)
        return

    # Global call to initialize every module
    def InitAllDynObjects(self):
        self.SetSpacecraftObject()
        self.SetGravityBodies()
        self.SetExternalForceTorqueObject()
        self.SetSimpleNavObject()

    # Global call to create every required one-time message
    def WriteInitDynMessages(self, SimBase):
        self.SetSpiceData(SimBase)

