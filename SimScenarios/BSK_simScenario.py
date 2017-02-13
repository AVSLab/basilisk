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




def SimScenario():
    TheBSKSim = BSKSim()

    # DATA LOGGING
    simulationTime = mc.min2nano(10.)
    numDataPoints = 100
    samplingTime = simulationTime / (numDataPoints-1)
    TheBSKSim.TotalSim.logThisMessage(TheBSKSim.mrpFeedbackData.outputDataName, samplingTime)
    TheBSKSim.TotalSim.logThisMessage(TheBSKSim.trackingErrorData.outputDataName, samplingTime)
    TheBSKSim.TotalSim.logThisMessage(TheBSKSim.simpleNavObject.outputTransName, samplingTime)
    TheBSKSim.TotalSim.logThisMessage(TheBSKSim.simpleNavObject.outputAttName, samplingTime)

    # Initialize Simulation
    TheBSKSim.InitializeSimulation()
    # The next call ensures that the FSW and Dynamics Message that have the same
    # name are copied over every time the simulation ticks forward.  This function
    # has to be called after the simulation is initialized to ensure that all modules
    # have created their own output/input messages declarations.
    TheBSKSim.dyn2FSWInterface.discoverAllMessages()
    TheBSKSim.fsw2DynInterface.discoverAllMessages()

    # Initialize Spacecraft States within the state manager.
    # This must occur after the initialization
    posRef = TheBSKSim.scObject.dynManager.getStateObject("hubPosition")
    velRef = TheBSKSim.scObject.dynManager.getStateObject("hubVelocity")
    sigmaRef = TheBSKSim.scObject.dynManager.getStateObject("hubSigma")
    omegaRef = TheBSKSim.scObject.dynManager.getStateObject("hubOmega")

    # setup the orbit using classical orbit elements
    oe = orbitalMotion.ClassicElements()
    mu = TheBSKSim.earthGravBody.mu
    oe.a = 10000000.0  # meters
    oe.e = 0.01
    oe.i = 33.3 * mc.D2R
    oe.Omega = 48.2 * mc.D2R
    oe.omega = 347.8 * mc.D2R
    oe.f = 85.3 * mc.D2R
    rN, vN = orbitalMotion.elem2rv(mu, oe)

    posRef.setState(sp.np2EigenVectorXd(rN))  # r_BN_N [m]
    velRef.setState(sp.np2EigenVectorXd(vN))  # r_BN_N [m]
    sigmaRef.setState([[0.1], [0.2], [-0.3]])  # sigma_BN_B
    omegaRef.setState([[0.001], [-0.01], [0.03]])  # omega_BN_B [rad/s]

    # Configure a simulation stop time time and execute the simulation run

    TheBSKSim.modeRequest = 'hillPoint'
    TheBSKSim.ConfigureStopTime(simulationTime)
    TheBSKSim.ExecuteSimulation()

    # Retrieve the logged data
    dataLr = TheBSKSim.pullMessageLogData(TheBSKSim.mrpFeedbackData.outputDataName + ".torqueRequestBody", range(3))
    dataSigmaBR = TheBSKSim.pullMessageLogData(TheBSKSim.trackingErrorData.outputDataName + ".sigma_BR", range(3))
    dataOmegaBR = TheBSKSim.pullMessageLogData(TheBSKSim.trackingErrorData.outputDataName + ".omega_BR_B", range(3))
    dataPos = TheBSKSim.pullMessageLogData(TheBSKSim.simpleNavObject.outputTransName + ".r_BN_N", range(3))
    dataVel = TheBSKSim.pullMessageLogData(TheBSKSim.simpleNavObject.outputTransName + ".v_BN_N", range(3))
    dataSigmaBN = TheBSKSim.pullMessageLogData(TheBSKSim.simpleNavObject.outputAttName + ".sigma_BN", range(3))

    return (dataLr, dataSigmaBR, dataOmegaBR, dataPos, dataVel, dataSigmaBN)


def plotResults(dataLr, dataSigmaBR, dataOmegaBR, dataPos, dataVel, dataSigmaBN):
    # Plot the results
    plt.close("all")        # clears out plots from earlier test runs
    timeLineFSW = dataSigmaBR[:, 0] * mc.NANO2MIN


    plt.figure(1)
    fig = plt.gcf()
    ax = fig.gca()
    vectorData = sp.pullVectorSetFromData(dataSigmaBR)
    sNorm = np.array([np.linalg.norm(v) for v in vectorData])
    plt.plot(timeLineFSW, sNorm, color=sp.getLineColor(1,3),)
    plt.xlabel('Time [min]')
    plt.ylabel('Attitude Error Norm $|\sigma_{B/R}|$')
    ax.set_yscale('log')

    plt.figure(2)
    for idx in range(1,4):
        plt.plot(timeLineFSW, dataLr[:, idx],
                 color=sp.getLineColor(idx,3),
                 label='$L_{r,'+str(idx)+'}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('Control Torque $L_r$ [Nm]')

    plt.figure(3)
    for idx in range(1,4):
        plt.plot(timeLineFSW, dataOmegaBR[:, idx],
                 color=sp.getLineColor(idx,3),
                 label='$\omega_{BR,'+str(idx)+'}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('Rate Tracking Error [rad/s] ')

    timeLineDYN = dataPos[:, 0] * mc.NANO2MIN
    vectorPosData = sp.pullVectorSetFromData(dataPos)
    vectorVelData = sp.pullVectorSetFromData(dataVel)
    vectorMRPData = sp.pullVectorSetFromData(dataSigmaBN)
    data = np.empty([len(vectorPosData),3])
    for idx in range(0,len(vectorPosData)):
        ir = vectorPosData[idx] / np.linalg.norm(vectorPosData[idx])
        hv = np.cross(vectorPosData[idx], vectorVelData[idx])
        ih = hv / np.linalg.norm(hv)
        itheta = np.cross(ih,ir)
        dcmBN = rbk.MRP2C(vectorMRPData[idx])
        data[idx] = [np.dot(ir, dcmBN[0]), np.dot(itheta, dcmBN[1]), np.dot(ih, dcmBN[2])]
    plt.figure(4)
    labelStrings = (r'$\hat\imath_r\cdot \hat b_1$'
                    , r'${\hat\imath}_{\theta}\cdot \hat b_2$'
                    , r'$\hat\imath_h\cdot \hat b_3$')
    for idx in range(0,3):
        plt.plot(timeLineDYN, data[:, idx],
                 color=sp.getLineColor(idx+1,3),
                 label=labelStrings[idx])
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('Orientation Illustration')


    plt.show()
    plt.close("all")
    return

(dataLr, dataSigmaBR, dataOmegaBR, dataPos, dataVel, dataSigmaBN) = SimScenario()
plotResults(dataLr, dataSigmaBR, dataOmegaBR, dataPos, dataVel, dataSigmaBN)
