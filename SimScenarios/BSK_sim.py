import sys, os, inspect
import numpy as np
import matplotlib.pyplot as plt

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
bskName = 'Basilisk'
splitPath = path.split(bskName)
bskPath = splitPath[0] + '/' + bskName + '/'
sys.path.append(bskPath + 'modules')
sys.path.append(bskPath + 'PythonModules')

import SimulationBaseClass
import sim_model
import RigidBodyKinematics as rbk
import macros as mc
import unitTestSupport as sp

import spacecraftPlus
import gravityEffector
import ExtForceTorque
import simple_nav
import spice_interface
import orbitalMotion

import vehicleConfigData
import hillPoint
import attTrackingError
import MRP_Feedback

class BSKSim(SimulationBaseClass.SimBaseClass):
    def __init__(self):
        # ------------------------------------------ARCHITECTURE---------------------------------------------------- #
        # Create a sim module as an empty container
        SimulationBaseClass.SimBaseClass.__init__(self)

        # self.TotalSim.terminateSimulation()
        # simulationTime = mc.min2nano(10.)

        # Create simulation process names
        self.DynamicsProcessName = "DynamicsProcess"
        self.FSWProcessName = "FSWProcess"
        # Create processes
        self.dynProc = self.CreateNewProcess(self.DynamicsProcessName)
        self.fswProc = self.CreateNewProcess(self.FSWProcessName)
        # Process message interfaces.
        self.dyn2FSWInterface = sim_model.SysInterface()
        self.fsw2DynInterface = sim_model.SysInterface()
        self.dyn2FSWInterface.addNewInterface(self.DynamicsProcessName, self.FSWProcessName)
        self.fsw2DynInterface.addNewInterface(self.FSWProcessName, self.DynamicsProcessName)
        self.dynProc.addInterfaceRef(self.dyn2FSWInterface)
        self.fswProc.addInterfaceRef(self.fsw2DynInterface)

        # Create simulation task names
        DynamicsTaskName = "DynamicsTask"
        # Add the tasks to the corresponding processes and specify the integration update time
        dynTimeStep = mc.sec2nano(0.1)
        self.dynProc.addTask(self.CreateNewTask(DynamicsTaskName, dynTimeStep))
        fswTimeStep = mc.sec2nano(0.5)
        self.fswProc.addTask(self.CreateNewTask("hillPointTask", fswTimeStep), 20)
        self.fswProc.addTask(self.CreateNewTask("mrpFeedbackTask", fswTimeStep), 10)



        # -------------------------------------------DKE SIM MODULES------------------------------------------------ #
        self.scObject = spacecraftPlus.SpacecraftPlus()
        self.extForceTorqueObject = ExtForceTorque.ExtForceTorque()
        self.simpleNavObject = simple_nav.SimpleNav()

        self.InitAllDynObjects()

        self.AddModelToTask(DynamicsTaskName, self.scObject)
        self.AddModelToTask(DynamicsTaskName, self.extForceTorqueObject)
        self.AddModelToTask(DynamicsTaskName, self.simpleNavObject)



        # -------------------------------------------FSW MODULES------------------------------------------------ #

        # Set up hillPoint guidance module
        self.hillPointData = hillPoint.hillPointConfig()
        self.hillPointWrap = self.setModelDataWrap(self.hillPointData)
        self.hillPointWrap.ModelTag = "hillPoint"

        # Set up the attitude tracking error evaluation module
        self.trackingErrorData = attTrackingError.attTrackingErrorConfig()
        self.trackingErrorWrap = self.setModelDataWrap(self.trackingErrorData)
        self.trackingErrorWrap.ModelTag = "trackingError"

        # Set up the MRP Feedback control module
        self.mrpFeedbackData = MRP_Feedback.MRP_FeedbackConfig()
        self.mrpFeedbackWrap = self.setModelDataWrap(self.mrpFeedbackData)
        self.mrpFeedbackWrap.ModelTag = "mrpFeedback"


        self.InitAllFSWObjects()

        self.AddModelToTask("hillPointTask", self.hillPointWrap, self.hillPointData, 20)
        self.AddModelToTask("hillPointTask", self.trackingErrorWrap, self.trackingErrorData, 19)
        self.AddModelToTask("mrpFeedbackTask", self.mrpFeedbackWrap, self.mrpFeedbackData, 10)

        # Disable all tasks in the FSW process
        self.fswProc.disableAllTasks()

        # ----------------------------------------------- GN&C EVENTS ----------------------------------------------- #
        # Define Events:
        # Function: createNewEvent("initiateEvent", int(taskRate), bool(eventActivity), modeRequest, taskList)
        # Parameters:
        #   # "initiateEvent": Give a name for the event (up to you. This name won't be used for anything)
        #   # taskRate: Period at which all the tasks in the event will run [nanoseconds]. Use taskRate = 1E9 as default.
        #   # eventActivity: Set the activity status of the event: True (active), False (inactive). Use True as default.
        #   # modeRequest: IMPORTANT. This is the name you'll use in the integrated scenario to call an event
        #   # taskList: IMPORTANT. List of tasks that will run orderly at each time-step of the event

        # -- Sun safe-capture event.
        self.createNewEvent("initiateHillPoint", fswTimeStep, True, ["self.modeRequest == 'hillPoint'"],
                            ["self.fswProc.disableAllTasks()",
                             "self.enableTask('hillPointTask')",
                             "self.enableTask('mrpFeedbackTask')"])


    # -------------------------------------------DKE INITIALIZATION------------------------------------------------ #

    def SetSpacecraftObject(self):
        self.scObject.ModelTag = "spacecraftBody"
        # -- Define the simulation inertia
        I = [900., 0., 0.,
             0., 800., 0.,
             0., 0., 600.]
        self.scObject.hub.mHub = 750.0  # kg - spacecraft mass
        self.scObject.hub.r_BcB_B = [[0.0], [0.0], [0.0]]  # m - position vector of body-fixed point B relative to CM
        self.scObject.hub.IHubPntBc_B = sp.np2EigenMatrix3d(I)
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
    
    def SetSpiceData(self):
        # SPICE sim message
        ephemerisMessageName = self.earthGravBody.bodyInMsgName
        self.ephemerisSPICEObject = spice_interface.SpicePlanetState()
        self.ephemerisSPICEObject.J2000Current = 0.0
        self.ephemerisSPICEObject.PositionVector = [0.0, 0.0, 0.0]
        self.ephemerisSPICEObject.VelocityVector = [0.0, 0.0, 0.0]
        self.ephemerisSPICEObject.J20002Pfix = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
        self.ephemerisSPICEObject.J20002Pfix_dot = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
        self.ephemerisSPICEObject.PlanetName = ephemerisMessageName
        ephemerisMessageSize =  self.ephemerisSPICEObject.getStructSize()
        self.TotalSim.CreateNewMessage(self.DynamicsProcessName, ephemerisMessageName, ephemerisMessageSize, 2)
        self.TotalSim.WriteMessageData(ephemerisMessageName, ephemerisMessageSize, 0, self.ephemerisSPICEObject)
        return

    # -------------------------------------------FSW INITIALIZATION------------------------------------------------ #

    def SetHillPointGuidance(self):
        self.hillPointData.inputNavDataName = self.simpleNavObject.outputTransName
        self.hillPointData.inputCelMessName = self.earthGravBody.outputMsgName
        self.hillPointData.outputDataName = "referenceOut"
        return

    def SetAttitudeTrackingError(self):
        self.trackingErrorData.inputNavName = self.simpleNavObject.outputAttName
        self.trackingErrorData.inputRefName = "referenceOut"
        self.trackingErrorData.outputDataName = "guidanceOut"
        return

    def SetMRPFeedback(self):
        self.mrpFeedbackData.inputGuidName = "guidanceOut"
        self.mrpFeedbackData.vehConfigInMsgName = "vehicleData"
        self.mrpFeedbackData.outputDataName = self.extForceTorqueObject.cmdTorqueInMsgName
        self.mrpFeedbackData.K = 3.5
        self.mrpFeedbackData.Ki = -1  # make value negative to turn off integral feedback
        self.mrpFeedbackData.P = 30.0
        self.mrpFeedbackData.integralLimit = 2. / self.mrpFeedbackData.Ki * 0.1
        return

    def SetVehicleData(self):
        # Vehicle config FSW message
        self.vehicleData = vehicleConfigData.vehicleConfigData()
        self.vehicleData.ISCPntB_B = [
            900., 0., 0.,
            0., 800., 0.,
            0., 0., 600.
        ] # Make sure you use the same inertia as in the simulation side
        vehicleMessageSize = self.vehicleData.getStructSize()
        self.TotalSim.CreateNewMessage(self.FSWProcessName, "vehicleData", vehicleMessageSize, 2)
        self.TotalSim.WriteMessageData("vehicleData", vehicleMessageSize, 0, self.vehicleData)
        return
    

    # -------------------------------------------GLOBAL INIT CALLS------------------------------------------------ #
    def InitAllDynObjects(self):
        self.SetSpacecraftObject()
        self.SetGravityBodies()
        self.SetExternalForceTorqueObject()
        self.SetSimpleNavObject()
        self.SetSpiceData()

    def InitAllFSWObjects(self):
        self.SetHillPointGuidance()
        self.SetAttitudeTrackingError()
        self.SetMRPFeedback()
        self.SetVehicleData()


