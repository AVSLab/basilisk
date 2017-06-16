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

#
# Basilisk Scenario Script and Integrated Test
#
# Purpose:  Integrated test of the spacecraftPlus(), RWs, simpleNav() and
#           MRP_PD() modules.  Illustrates a 6-DOV spacecraft detumbling in orbit
#           while using the RWs to do the attitude control actuation.
# Author:   Hanspeter Schaub
# Creation Date:  Jan. 7, 2017
#



import pytest
import sys, os, inspect
import matplotlib
import numpy as np
import ctypes
import math
import csv
import logging

# @cond DOXYGEN_IGNORE
filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
bskName = 'Basilisk'
splitPath = path.split(bskName)
bskPath = splitPath[0] + '/' + bskName + '/'
# if this script is run from a custom folder outside of the Basilisk folder, then uncomment the
# following line and specify the absolute bath to the Basilisk folder
#bskPath = '/Users/hp/Documents/Research/' + bskName + '/'
sys.path.append(bskPath + 'modules')
sys.path.append(bskPath + 'PythonModules')
# @endcond

# import general simulation support files
import SimulationBaseClass
import unitTestSupport                  # general support file with common unit test functions
import matplotlib.pyplot as plt
import macros
import orbitalMotion
import sim_model

# import simulation related support
import spacecraftPlus
import simIncludeGravity
import simIncludeRW
import simple_nav
import reactionWheelStateEffector
import rwVoltageInterface

# import FSW Algorithm related support
import MRP_PD
import inertial3D
import attTrackingError
import rwMotorTorque
import fswSetupRW
import rwMotorVoltage
import simulationArchTypes

# import message declarations
import fswMessages


# ------------------------------------- Interface Functions ------------------------------------- #
def GetStructSize(messageStructName):
    return  eval(messageStructName + '()').getStructSize()

def CreateNewMessage(self, messageName, messageStructName):
    messageID = sim_model.SystemMessaging_GetInstance().CreateNewMessage(
        messageName, GetStructSize(messageStructName), 2, messageStructName.split('.')[1], self.moduleID)
    return messageID

def SubscribeToMessage(self, messageName, messageStructName):
    messageID = sim_model.SystemMessaging_GetInstance().subscribeToMessage(
        messageName, GetStructSize(messageStructName), self.moduleID)
    return messageID

def ReadMessage(self, messageID, messageStructName):
    localHeader = sim_model.SingleMessageHeader()
    localData = eval(messageStructName + '()')
    sim_model.SystemMessaging_GetInstance().ReadMessage(messageID, localHeader, localData.getStructSize(), localData, self.moduleID)
    return localData

def WriteMessage(self, messageID, currentTime, messageStruct):
    sim_model.SystemMessaging_GetInstance().WriteMessage(
        messageID, currentTime, messageStruct.getStructSize(), messageStruct, self.moduleID)
    return
# -------------------------------------------------------------------------- #

class PythonMRPPD(simulationArchTypes.PythonModelClass):
    def __init__(self, modelName, modelActive = True, modelPriority = -1):
        self.modelName = modelName
        self.modelActive = modelActive
        self.modelPriority = -1
        self.moduleID = sim_model.SystemMessaging_GetInstance().checkoutModuleID()

        self.K = 0
        self.P = 0
        self.inputGuidName =""
        self.inputVehicleConfigDataName = ""
        self.outputDataName = ""
        self.inputGuidID = -1
        self.inputVehConfigID = -1
        self.outputDataID = -1


    def selfInit(self):
        self.outputDataID = CreateNewMessage(self, self.outputDataName, 'MRP_PD.CmdTorqueBodyIntMsg')
        return
    def crossInit(self):
        self.inputGuidID = SubscribeToMessage(self, self.inputGuidName, 'MRP_PD.AttGuidFswMsg')
        self.inputVehConfigID = SubscribeToMessage(self, self.inputVehicleConfigDataName, 'MRP_PD.VehicleConfigFswMsg')
        return
    def reset(self, currentTime):
        return
    def updateState(self, currentTime):
        localAttErr = ReadMessage(self, self.inputGuidID, 'MRP_PD.AttGuidFswMsg')
        lrCmd = np.array(localAttErr.sigma_BR) * self.K + np.array(localAttErr.omega_BR_B)*self.P
        outTorque = MRP_PD.CmdTorqueBodyIntMsg()
        outTorque.torqueRequestBody = (-lrCmd).tolist()
        WriteMessage(self, self.outputDataID, currentTime, outTorque)

        def print_output():
            print currentTime*1.0E-9
            print outTorque.torqueRequestBody
            print localAttErr.sigma_BR
            print localAttErr.omega_BR_B
        
        return





    # uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail(True)

# The following 'parametrize' function decorator provides the parameters and expected results for each
#   of the multiple test runs for this test.
@pytest.mark.parametrize("useJitterSimple, useRWVoltageIO", [
      (False, False)
])

# provide a unique test method name, starting with test_
def test_bskAttitudeFeedbackRW(show_plots, useJitterSimple, useRWVoltageIO):
    '''This function is called by the py.test environment.'''
    # each test method requires a single assert method to be called
    [testResults, testMessage] = runRegularTask( True,
            show_plots, useJitterSimple, useRWVoltageIO)
    assert testResults < 1, testMessage

def runRegularTask(doUnitTests, show_plots, useJitterSimple, useRWVoltageIO):
    '''Call this routine directly to run the tutorial scenario.'''
    testFailCount = 0                       # zero unit test result counter
    testMessages = []                       # create empty array to store test log messages

    #
    #  From here on there scenario python code is found.  Above this line the code is to setup a
    #  unitTest environment.  The above code is not critical if learning how to code BSK.
    #
    simulationTimeStep = macros.sec2nano(.1)
    #  Create a sim module as an empty container
    scSim = SimulationBaseClass.SimBaseClass()
    scSim.TotalSim.terminateSimulation()

    # Create simulation variable names
    scSim.simTaskPreControlName = "simTaskPreControl"
    scSim.simTaskControlName = "simTaskControl"
    scSim.simTaskPostControlName = "simTaskPostControl"
    scSim.simPreControlProc = "simPreControl"
    scSim.simControlProc = "simControlProc"
    scSim.simPostControlProc = "simPostControlProc"

    #
    #  create the simulation process
    #
    scSim.dynProcessFirst = scSim.CreateNewProcess(scSim.simPreControlProc, 3)
    scSim.dynProcessSecond = scSim.CreateNewProcess(scSim.simControlProc, 2)
    scSim.dynProcessThird = scSim.CreateNewProcess(scSim.simPostControlProc, 1)

    scSim.dynProcessSecond.addTask(scSim.CreateNewTask(scSim.simTaskControlName, simulationTimeStep))
    
    # setup the MRP Feedback control module
    mrpControlConfig = MRP_PD.MRP_PDConfig()
    mrpControlWrap = scSim.setModelDataWrap(mrpControlConfig)
    mrpControlWrap.ModelTag = "MRP_PD"
    scSim.AddModelToTask(scSim.simTaskControlName, mrpControlWrap, mrpControlConfig)
    mrpControlConfig.inputGuidName  = "attErrorInertial3DMsg"
    mrpControlConfig.inputVehicleConfigDataName  = "vehicleConfigName"
    mrpControlConfig.outputDataName = "LrRequested"
    mrpControlConfig.K  =   3.5
    mrpControlConfig.P  = 30.0

    dataUsReqBase, dataSigmaBRBase, dataOmegaBRBase, dataPosBase, dataOmegaRWBase, dataRWBase = \
        executeMainSimRun(scSim, show_plots, useJitterSimple, useRWVoltageIO)

    scSimPy = SimulationBaseClass.SimBaseClass()
    scSimPy.TotalSim.terminateSimulation()
    
    # Create simulation variable names
    scSimPy.simTaskPreControlName = "simTaskPreControl"
    scSimPy.simTaskControlName = "simTaskControlPy"
    scSimPy.simTaskPostControlName = "simTaskPostControl"
    scSimPy.simPreControlProc = "simPreControl"
    scSimPy.simControlProc = "simControlProcPy"
    scSimPy.simPostControlProc = "simPostControlProc"
    
    scSimPy.dynProcessFirst = scSimPy.CreateNewProcess(scSimPy.simPreControlProc, 3)
    scSimPy.dynProcessSecond = scSimPy.CreateNewPythonProcess(scSimPy.simControlProc, 2)
    scSimPy.dynProcessThird = scSimPy.CreateNewProcess(scSimPy.simPostControlProc, 1)
    
    scSimPy.dynProcessSecond.createPythonTask(scSimPy.simTaskControlName, simulationTimeStep, True, -1)
    
    scSimPy.pyMRPPD = PythonMRPPD("pyMRP_PD", True, 100)
    scSimPy.pyMRPPD.inputGuidName = mrpControlConfig.inputGuidName
    scSimPy.pyMRPPD.inputVehicleConfigDataName = mrpControlConfig.inputVehicleConfigDataName
    scSimPy.pyMRPPD.outputDataName = mrpControlConfig.outputDataName
    scSimPy.pyMRPPD.K = mrpControlConfig.K
    scSimPy.pyMRPPD.P = mrpControlConfig.P
    
    
    scSimPy.dynProcessSecond.addModelToTask(scSimPy.simTaskControlName, scSimPy.pyMRPPD)
    
    dataUsReq, dataSigmaBR, dataOmegaBR, dataPos, dataOmegaRW, dataRW = executeMainSimRun(scSimPy, show_plots, useJitterSimple, useRWVoltageIO)
    
    numRW = simIncludeRW.getNumOfDevices()

    #
    #   retrieve the logged data
    #

    np.set_printoptions(precision=16)
    #
    #   plot the results
    #
    fileNameString = filename[len(path)+6:-3]
    timeData = dataUsReq[:, 0] * macros.NANO2SEC
    timeDataBase = dataUsReqBase[:, 0] * macros.NANO2SEC
    plt.close("all")  # clears out plots from earlier test runs
    plt.figure(1)
    for idx in range(1,4):
        plt.plot(timeData, dataSigmaBR[:, idx],
                 timeDataBase, dataSigmaBRBase[:,idx], '--')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('Attitude Error $\sigma_{B/R}$')
    if doUnitTests:     # only save off the figure if doing a unit test run
        unitTestSupport.saveScenarioFigure(
            fileNameString+"1"+str(int(useJitterSimple))+str(int(useRWVoltageIO))
            , plt, path)

    plt.figure(2)
    for idx in range(1,4):
        plt.plot(timeData, dataSigmaBR[:, idx] - dataSigmaBRBase[:,idx])
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('Attitude Difference $\sigma_{B/R}$')

    plt.figure(3)
    for idx in range(1,4):
        plt.plot(timeData, dataUsReq[:, idx] - dataUsReqBase[:,idx])
    plt.xlabel('Time [min]')
    plt.ylabel('RW tau diff [Nm] ')


    if show_plots:
        plt.show()

    # close the plots being saved off to avoid over-writing old and new figures
    plt.close("all")

    # each test method requires a single assert method to be called
    # this check below just makes sure no sub-test failures were found
    return [testFailCount, ''.join(testMessages)]

def executeMainSimRun(scSim, show_plots, useJitterSimple, useRWVoltageIO):

    # set the simulation time variable used later on
    simulationTime = macros.min2nano(10.)
    #simulationTime = macros.min2nano(0.1/60)
    scSim.pre2ContInterface = sim_model.SysInterface()
    scSim.cont2PostInterface = sim_model.SysInterface()
    scSim.post2PreInterface = sim_model.SysInterface()
    scSim.pre2ContInterface.addNewInterface(scSim.simPreControlProc, scSim.simControlProc)
    scSim.cont2PostInterface.addNewInterface(scSim.simControlProc, scSim.simPostControlProc)
    scSim.post2PreInterface.addNewInterface(scSim.simPostControlProc, scSim.simPreControlProc)
    scSim.dynProcessFirst.addInterfaceRef(scSim.post2PreInterface)
    scSim.dynProcessSecond.addInterfaceRef(scSim.pre2ContInterface)
    scSim.dynProcessThird.addInterfaceRef(scSim.cont2PostInterface)

    # create the dynamics task and specify the integration update time
    simulationTimeStep = macros.sec2nano(.1)
    scSim.dynProcessFirst.addTask(scSim.CreateNewTask(scSim.simTaskPreControlName, simulationTimeStep))
    scSim.dynProcessThird.addTask(scSim.CreateNewTask(scSim.simTaskPostControlName, simulationTimeStep))

    #
    #   setup the simulation tasks/objects
    #

    # initialize spacecraftPlus object and set properties
    scObject = spacecraftPlus.SpacecraftPlus()
    scObject.ModelTag = "spacecraftBody"
    # define the simulation inertia
    I = [900., 0., 0.,
         0., 800., 0.,
         0., 0., 600.]
    scObject.hub.mHub = 750.0                   # kg - spacecraft mass
    scObject.hub.r_BcB_B = [[0.0], [0.0], [0.0]] # m - position vector of body-fixed point B relative to CM
    scObject.hub.IHubPntBc_B = unitTestSupport.np2EigenMatrix3d(I)
    scObject.hub.useTranslation = True
    scObject.hub.useRotation = True

    # add spacecraftPlus object to the simulation process
    scSim.AddModelToTask(scSim.simTaskPreControlName, scObject, None, 1)

    # clear prior gravitational body and SPICE setup definitions
    simIncludeGravity.clearSetup()

    # setup Earth Gravity Body
    simIncludeGravity.addEarth()
    simIncludeGravity.gravBodyList[-1].isCentralBody = True          # ensure this is the central gravitational body
    mu = simIncludeGravity.gravBodyList[-1].mu

    # attach gravity model to spaceCraftPlus
    scObject.gravField.gravBodies = spacecraftPlus.GravBodyVector(simIncludeGravity.gravBodyList)


    # add RW devices
    # The clearRWSetup() is critical if the script is to run multiple times
    simIncludeRW.clearSetup()
    # the Honeywell HR16 comes in three momentum configuration, 100, 75 and 50 Nms
    simIncludeRW.options.maxMomentum = 50
    if useJitterSimple:
        simIncludeRW.options.RWModel = simIncludeRW.JitterSimple
    # create each RW by specifying the RW type, the spin axis gsHat and the initial wheel speed Omega
    simIncludeRW.create(
            'Honeywell_HR16',
            [1, 0, 0],              # gsHat_B
            100.0                     # RPM
            )
    simIncludeRW.create(
            'Honeywell_HR16',
            [0, 1, 0],              # gsHat_B
            200.0                     # RPM
            )
    simIncludeRW.create(
            'Honeywell_HR16',
            [0, 0, 1],              # gsHat_B
            300.0,                    # RPM
            [0.5,0.5,0.5]           # r_B (optional argument)
            )
    numRW = simIncludeRW.getNumOfDevices()

    # create RW object container and tie to spacecraft object
    rwStateEffector = reactionWheelStateEffector.ReactionWheelStateEffector()
    simIncludeRW.addToSpacecraft("ReactionWheels", rwStateEffector, scObject)

    # add RW object array to the simulation process
    scSim.AddModelToTask(scSim.simTaskPreControlName, rwStateEffector, None, 2)

    if useRWVoltageIO:
        rwVoltageIO = rwVoltageInterface.RWVoltageInterface()
        rwVoltageIO.ModelTag = "rwVoltageInterface"

        # set module parameters(s)
        rwVoltageIO.voltage2TorqueGain = 0.2/10.  # [Nm/V] conversion gain

        # Add test module to runtime call list
        scSim.AddModelToTask(scSim.simTaskPreControlName, rwVoltageIO)


    # add the simple Navigation sensor module.  This sets the SC attitude, rate, position
    # velocity navigation message
    sNavObject = simple_nav.SimpleNav()
    sNavObject.ModelTag = "SimpleNavigation"
    scSim.AddModelToTask(scSim.simTaskPreControlName, sNavObject)



    #
    #   setup the FSW algorithm tasks
    #

    # setup inertial3D guidance module
    inertial3DConfig = inertial3D.inertial3DConfig()
    inertial3DWrap = scSim.setModelDataWrap(inertial3DConfig)
    inertial3DWrap.ModelTag = "inertial3D"
    scSim.AddModelToTask(scSim.simTaskPreControlName, inertial3DWrap, inertial3DConfig)
    inertial3DConfig.sigma_R0N = [0., 0., 0.]       # set the desired inertial orientation
    inertial3DConfig.outputDataName = "guidanceInertial3D"

    # setup the attitude tracking error evaluation module
    attErrorConfig = attTrackingError.attTrackingErrorConfig()
    attErrorWrap = scSim.setModelDataWrap(attErrorConfig)
    attErrorWrap.ModelTag = "attErrorInertial3D"
    scSim.AddModelToTask(scSim.simTaskPreControlName, attErrorWrap, attErrorConfig)
    attErrorConfig.outputDataName = "attErrorInertial3DMsg"
    attErrorConfig.inputRefName = inertial3DConfig.outputDataName
    attErrorConfig.inputNavName = sNavObject.outputAttName
    
    # setup the MRP Feedback control module (left for variable connectivity.  Note that it isn't added to task...)
    mrpControlConfig = MRP_PD.MRP_PDConfig()
    mrpControlWrap = scSim.setModelDataWrap(mrpControlConfig)
    mrpControlWrap.ModelTag = "MRP_PD"
    mrpControlConfig.inputGuidName  = attErrorConfig.outputDataName
    mrpControlConfig.inputVehicleConfigDataName  = "vehicleConfigName"
    mrpControlConfig.outputDataName = "LrRequested"
    mrpControlConfig.K  =   3.5
    mrpControlConfig.P  = 30.0

    # add module that maps the Lr control torque into the RW motor torques
    rwMotorTorqueConfig = rwMotorTorque.rwMotorTorqueConfig()
    rwMotorTorqueWrap = scSim.setModelDataWrap(rwMotorTorqueConfig)
    rwMotorTorqueWrap.ModelTag = "rwMotorTorque"
    scSim.AddModelToTask(scSim.simTaskPostControlName, rwMotorTorqueWrap, rwMotorTorqueConfig)
    # Initialize the test module msg names
    if useRWVoltageIO:
        rwMotorTorqueConfig.outputDataName = "rw_torque_Lr"
    else:
        rwMotorTorqueConfig.outputDataName = rwStateEffector.InputCmds
    rwMotorTorqueConfig.inputVehControlName = mrpControlConfig.outputDataName
    rwMotorTorqueConfig.rwParamsInMsgName = "rwa_config_data_parsed"
    # Make the RW control all three body axes
    controlAxes_B = [
             1,0,0
            ,0,1,0
            ,0,0,1
        ]
    rwMotorTorqueConfig.controlAxes_B = controlAxes_B

    if useRWVoltageIO:
        fswRWVoltageConfig = rwMotorVoltage.rwMotorVoltageConfig()
        fswRWVoltageWrap = scSim.setModelDataWrap(fswRWVoltageConfig)
        fswRWVoltageWrap.ModelTag = "rwMotorVoltage"

        # Add test module to runtime call list
        scSim.AddModelToTask(scSim.simTaskPostControlName, fswRWVoltageWrap, fswRWVoltageConfig)

        # Initialize the test module configuration data
        fswRWVoltageConfig.torqueInMsgName = rwMotorTorqueConfig.outputDataName
        fswRWVoltageConfig.rwParamsInMsgName = mrpControlConfig.rwParamsInMsgName
        fswRWVoltageConfig.voltageOutMsgName = rwVoltageIO.rwVoltageInMsgName

        # set module parameters
        fswRWVoltageConfig.VMin = 0.0  # Volts
        fswRWVoltageConfig.VMax = 10.0  # Volts


    #
    #   Setup data logging before the simulation is initialized
    #
    numDataPoints = 10000
    samplingTime = simulationTime / (numDataPoints-1)
    scSim.TotalSim.logThisMessage(rwMotorTorqueConfig.outputDataName, samplingTime)
    scSim.TotalSim.logThisMessage(attErrorConfig.outputDataName, samplingTime)
    scSim.TotalSim.logThisMessage(sNavObject.outputTransName, samplingTime)
    scSim.TotalSim.logThisMessage(rwStateEffector.OutputDataString, samplingTime)
    rwOutName = ["rw_config_0_data", "rw_config_1_data", "rw_config_2_data"]
    for item in rwOutName:
        scSim.TotalSim.logThisMessage(item, samplingTime)
    if useRWVoltageIO:
        scSim.TotalSim.logThisMessage(fswRWVoltageConfig.voltageOutMsgName, samplingTime)

    #
    # create simulation messages
    #
    simIncludeGravity.addDefaultEphemerisMsg(scSim.TotalSim, scSim.simPreControlProc)

    # create the FSW vehicle configuration message
    vehicleConfigOut = fswMessages.VehicleConfigFswMsg()
    vehicleConfigOut.ISCPntB_B = I  # use the same inertia in the FSW algorithm as in the simulation
    unitTestSupport.setMessage(scSim.TotalSim,
                               scSim.simControlProc,
                               mrpControlConfig.inputVehicleConfigDataName,
                               vehicleConfigOut)
    unitTestSupport.setMessage(scSim.TotalSim,
                               scSim.simPostControlProc,
                               mrpControlConfig.inputVehicleConfigDataName,
                               vehicleConfigOut)
    unitTestSupport.setMessage(scSim.TotalSim,
                               scSim.simPreControlProc,
                               mrpControlConfig.inputVehicleConfigDataName,
                               vehicleConfigOut)

    # FSW RW configuration message
    # use the same RW states in the FSW algorithm as in the simulation
    fswSetupRW.clearSetup()
    for rw in simIncludeRW.rwList:
        fswSetupRW.create(unitTestSupport.EigenVector3d2np(rw.gsHat_B), rw.Js, 0.2)
    fswSetupRW.writeConfigMessage(rwMotorTorqueConfig.rwParamsInMsgName, scSim.TotalSim, scSim.simPreControlProc)
    fswSetupRW.writeConfigMessage(rwMotorTorqueConfig.rwParamsInMsgName, scSim.TotalSim, scSim.simControlProc)
    fswSetupRW.writeConfigMessage(rwMotorTorqueConfig.rwParamsInMsgName, scSim.TotalSim, scSim.simPostControlProc)

    #
    #   initialize Spacecraft States with intitialization variables
    #

    # setup the orbit using classical orbit elements
    oe = orbitalMotion.ClassicElements()
    oe.a     = 10000000.0           # meters
    oe.e     = 0.01
    oe.i     = 33.3*macros.D2R
    oe.Omega = 48.2*macros.D2R
    oe.omega = 347.8*macros.D2R
    oe.f     = 85.3*macros.D2R
    rN, vN = orbitalMotion.elem2rv(mu, oe)

    scObject.hub.r_CN_NInit = unitTestSupport.np2EigenVectorXd(rN)  # m  - r_CN_N
    scObject.hub.v_CN_NInit = unitTestSupport.np2EigenVectorXd(vN)  # m/s - v_CN_N
    scObject.hub.sigma_BNInit = [[0.1], [0.2], [-0.3]]       # sigma_BN_B
    scObject.hub.omega_BN_BInit = [[0.001], [-0.01], [0.03]]   # rad/s - omega_BN_B

    #
    #   initialize Simulation
    #
    scSim.InitializeSimulationAndDiscover()

    #
    #   configure a simulation stop time time and execute the simulation run
    #
    scSim.ConfigureStopTime(simulationTime)
    scSim.ExecuteSimulation()

    dataUsReq = scSim.pullMessageLogData(rwMotorTorqueConfig.outputDataName+".motorTorque", range(numRW))
    dataSigmaBR = scSim.pullMessageLogData(attErrorConfig.outputDataName+".sigma_BR", range(3))
    dataOmegaBR = scSim.pullMessageLogData(attErrorConfig.outputDataName+".omega_BR_B", range(3))
    dataPos = scSim.pullMessageLogData(sNavObject.outputTransName+".r_BN_N", range(3))
    dataOmegaRW = scSim.pullMessageLogData(rwStateEffector.OutputDataString+".wheelSpeeds", range(numRW))
    dataRW = []
    for i in range(0,numRW):
        dataRW.append(scSim.pullMessageLogData(rwOutName[i]+".u_current", range(1)))
    if useRWVoltageIO:
        dataVolt = scSim.pullMessageLogData(fswRWVoltageConfig.voltageOutMsgName+".voltage", range(numRW))

    return dataUsReq, dataSigmaBR, dataOmegaBR, dataPos, dataOmegaRW, dataRW

#
# This statement below ensures that the unit test scrip can be run as a
# stand-along python script
#
if __name__ == "__main__":
    runRegularTask(  False        # do unit tests
        , True         # show_plots
        , False        # useJitterSimple
        , False        # useRWVoltageIO
       )

