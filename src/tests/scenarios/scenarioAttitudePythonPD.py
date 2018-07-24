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


#
# Basilisk Scenario Script and Integrated Test
#
# Purpose:  Integrated test of the spacecraftPlus(), RWs, simpleNav() and
#           a Python implementation of the MRP_PD module.  Illustrates
#           a 6-DOV spacecraft detumbling in orbit
#           while using the RWs to do the attitude control actuation.  The main
#           purpose of this module is to illustrate how to use python processes.
# Author:   Scott Piggott
# Creation Date:  Jun. 28, 2017
#

import pytest
import os
import numpy as np

# import general simulation support files
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import unitTestSupport  # general support file with common unit test functions
import matplotlib.pyplot as plt
from Basilisk.utilities import macros
from Basilisk.utilities import orbitalMotion
from Basilisk.simulation import sim_model

# import simulation related support
from Basilisk.simulation import spacecraftPlus
from Basilisk.utilities import simIncludeGravBody
from Basilisk.utilities import simIncludeRW
from Basilisk.simulation import simple_nav
from Basilisk.simulation import reactionWheelStateEffector
from Basilisk.simulation import rwVoltageInterface

# import FSW Algorithm related support
from Basilisk.fswAlgorithms import MRP_PD
from Basilisk.fswAlgorithms import inertial3D
from Basilisk.fswAlgorithms import attTrackingError
from Basilisk.fswAlgorithms import rwMotorTorque
from Basilisk.utilities import fswSetupRW
from Basilisk.fswAlgorithms import rwMotorVoltage
from Basilisk.utilities import simulationArchTypes

# import message declarations
from Basilisk.fswAlgorithms import fswMessages


## \defgroup Tutorials_2_0_3
##   @{
## Demonstrates how to stabilize the tumble of a spacecraft orbiting the
# Earth that is initially tumbling, but uses 3 separate threads, one of them python.
#
# Attitude Detumbling Simulation in a Three Process Simulation Setup {#scenarioAttitudePythonPD}
# ====
#
# Scenario Description
# -----
# This script sets up a 6-DOF spacecraft which is orbiting the Earth. This setup
# is similar to the [scenarioAttitudeFeedback.py](@ref scenarioAttitudeFeedback),
# but here the dynamics
# simulation and the Flight Software (FSW) algorithms are run at different time steps.
# The scenario runs two different cases.  The first is with the nominal MRP_PD (c-code), the
# second is with the same module coded into a python process.  Identical results should be
# obtained.
#
# To run the default scenario 1., call the python script through
#
#       python scenarioAttitudePythonPD.py
#
# When the simulation completes 3 plots are shown for the MRP attitude history, the differences
# between the attitude history, and the differences between the RWA commands.  The attitude history
# should show a clean overlay, and there should be zero differences.
#
# It is assumed at this point that you are familiar with how to set up simulations in Basilisk and that
# you have a good understanding of processes/Tasks/models/etc.  This document tries to call out the
# unique changes that are needed for python processes.
#
# The simulation layout is broken up into three different processes.  In the nominal sim (with MRP_PD c-code),
# the processes contain the following contents:
# -# Dynamics and control leading up to MRP_PD
# -# MRP_PD (c-code)
# -# RWA control based on MRP_PD outputs
#
# For the simulation with the python task, the scenario is broken up as follows:
# -# Dynamics and control leading up to MRP_PD
# -# MRP_PD (python-code)
# -# RWA control based on MRP_PD outputs
#
# The process, task, and model settings for the standard models follow a procedure identical to the
# other tutorials.  Then for Python processes, the spin up procedure closely mirrors that of the
# regular processes.
#
# For the python processes, the creation step is almost identical to the creation step for standard
# C/C++ processes.  Instead of:
# ~~~~~~~~~~~~~~{.py}
#       scSim.dynProcessSecond = scSim.CreateNewProcess(scSim.simControlProc, 2)
# ~~~~~~~~~~~~~~
# We use:
# ~~~~~~~~~~~~~~{.py}
#       scSimPy.dynProcessSecond = scSimPy.CreateNewPythonProcess(scSimPy.simControlProc, 2)
# ~~~~~~~~~~~~~~
# With this process, we've embedded Py in most of the naming, but that is an arbitrary user
# choice, the only thing that matters is the CreateNewPythonProcess instead of CreateNewProcess.
# Note that the prioritization is the same procedure as is used for standard tasks.
#
# Then, models are created using model classes that have been defined (more on this later).  These
# models are then attached to the python task which has been attached to the python process.  Again
# the same prioritization procedures are used for the python tasks as are used for the standard tasks.
#
##  @}




def runRegularTask(show_plots, useJitterSimple, useRWVoltageIO):
    '''Call this routine directly to run the tutorial scenario.'''

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
    mrpControlConfig.inputGuidName = "attErrorInertial3DMsg"
    mrpControlConfig.inputVehicleConfigDataName = "vehicleConfigName"
    mrpControlConfig.outputDataName = "LrRequested"
    mrpControlConfig.K = 3.5
    mrpControlConfig.P = 30.0

    dataUsReqBase, dataSigmaBRBase, dataOmegaBRBase, dataPosBase, dataOmegaRWBase, dataRWBase = \
        executeMainSimRun(scSim, show_plots, useJitterSimple, useRWVoltageIO)

    scSimPy = SimulationBaseClass.SimBaseClass()
    scSimPy.TotalSim.terminateSimulation()

    # Create simulation variable names
    ## For the python process, the name used includes Py at the end for this tutorial.
    # However there is no magic to that, they can be named arbitrarily like any other process
    # or task
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

    dataUsReq, dataSigmaBR, dataOmegaBR, dataPos, dataOmegaRW, dataRW = executeMainSimRun(scSimPy, show_plots,
                                                                                          useJitterSimple,
                                                                                          useRWVoltageIO)

    #
    #   retrieve the logged data
    #

    np.set_printoptions(precision=16)
    #
    #   plot the results
    #
    fileName = os.path.basename(os.path.splitext(__file__)[0])

    timeData = dataUsReq[:, 0] * macros.NANO2SEC
    timeDataBase = dataUsReqBase[:, 0] * macros.NANO2SEC
    plt.close("all")  # clears out plots from earlier test runs
    plt.figure(1)
    for idx in range(1, 4):
        plt.plot(timeData, dataSigmaBR[:, idx],
                 timeDataBase, dataSigmaBRBase[:, idx], '--')
    plt.xlabel('Time [min]')
    plt.ylabel('Attitude Error $\sigma_{B/R}$')
    figureList = {}
    pltName = fileName + "1" + str(int(useJitterSimple)) + str(int(useRWVoltageIO))
    figureList[pltName] = plt.figure(1)

    plt.figure(2)
    for idx in range(1, 4):
        plt.plot(timeData, dataSigmaBR[:, idx] - dataSigmaBRBase[:, idx])
    plt.xlabel('Time [min]')
    plt.ylabel('Attitude Difference $\sigma_{B/R}$')

    plt.figure(3)
    for idx in range(1, 4):
        plt.plot(timeData, dataUsReq[:, idx] - dataUsReqBase[:, idx])
    plt.xlabel('Time [min]')
    plt.ylabel('RW tau diff [Nm] ')


    if show_plots:
        plt.show()

    # close the plots being saved off to avoid over-writing old and new figures
    plt.close("all")

    return dataSigmaBR, dataUsReq, dataSigmaBRBase, dataUsReqBase, figureList


def executeMainSimRun(scSim, show_plots, useJitterSimple, useRWVoltageIO):
    # set the simulation time variable used later on
    simulationTime = macros.min2nano(10.)
    # simulationTime = macros.min2nano(0.1/60)

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
    scObject.hub.mHub = 750.0  # kg - spacecraft mass
    scObject.hub.r_BcB_B = [[0.0], [0.0], [0.0]]  # m - position vector of body-fixed point B relative to CM
    scObject.hub.IHubPntBc_B = unitTestSupport.np2EigenMatrix3d(I)

    # add spacecraftPlus object to the simulation process
    scSim.AddModelToTask(scSim.simTaskPreControlName, scObject, None, 1)

    # clear prior gravitational body and SPICE setup definitions
    gravFactory = simIncludeGravBody.gravBodyFactory()

    # setup Earth Gravity Body
    earth = gravFactory.createEarth()
    earth.isCentralBody = True  # ensure this is the central gravitational body
    mu = earth.mu

    # attach gravity model to spaceCraftPlus
    scObject.gravField.gravBodies = spacecraftPlus.GravBodyVector(gravFactory.gravBodies.values())

    #
    # add RW devices
    #
    # Make a fresh RW factory instance, this is critical to run multiple times
    rwFactory = simIncludeRW.rwFactory()

    # store the RW dynamical model type
    varRWModel = rwFactory.BalancedWheels
    if useJitterSimple:
        varRWModel = rwFactory.JitterSimple

    # create each RW by specifying the RW type, the spin axis gsHat, plus optional arguments
    RW1 = rwFactory.create('Honeywell_HR16'
                           , [1, 0, 0]
                           , maxMomentum=50.
                           , Omega=100.  # RPM
                           , RWModel=varRWModel
                           )
    RW2 = rwFactory.create('Honeywell_HR16'
                           , [0, 1, 0]
                           , maxMomentum=50.
                           , Omega=200.  # RPM
                           , RWModel=varRWModel
                           )

    RW3 = rwFactory.create('Honeywell_HR16'
                           , [0, 0, 1]
                           , maxMomentum=50.
                           , Omega=300.  # RPM
                           , rWB_B=[0.5, 0.5, 0.5]  # meters
                           , RWModel=varRWModel
                           )

    numRW = rwFactory.getNumOfDevices()

    # create RW object container and tie to spacecraft object
    rwStateEffector = reactionWheelStateEffector.ReactionWheelStateEffector()
    rwFactory.addToSpacecraft("ReactionWheels", rwStateEffector, scObject)

    # add RW object array to the simulation process
    scSim.AddModelToTask(scSim.simTaskPreControlName, rwStateEffector, None, 2)

    if useRWVoltageIO:
        rwVoltageIO = rwVoltageInterface.RWVoltageInterface()
        rwVoltageIO.ModelTag = "rwVoltageInterface"

        # set module parameters(s)
        rwVoltageIO.voltage2TorqueGain = 0.2 / 10.  # [Nm/V] conversion gain

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
    inertial3DConfig.sigma_R0N = [0., 0., 0.]  # set the desired inertial orientation
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
    mrpControlConfig.inputGuidName = attErrorConfig.outputDataName
    mrpControlConfig.inputVehicleConfigDataName = "vehicleConfigName"
    mrpControlConfig.outputDataName = "LrRequested"
    mrpControlConfig.K = 3.5
    mrpControlConfig.P = 30.0

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
        1, 0, 0,
        0, 1, 0,
        0, 0, 1
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
    samplingTime = simulationTime / (numDataPoints - 1)
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
    for key, rw in rwFactory.rwList.iteritems():
        fswSetupRW.create(unitTestSupport.EigenVector3d2np(rw.gsHat_B), rw.Js, 0.2)
    fswSetupRW.writeConfigMessage(rwMotorTorqueConfig.rwParamsInMsgName, scSim.TotalSim, scSim.simPreControlProc)
    fswSetupRW.writeConfigMessage(rwMotorTorqueConfig.rwParamsInMsgName, scSim.TotalSim, scSim.simControlProc)
    fswSetupRW.writeConfigMessage(rwMotorTorqueConfig.rwParamsInMsgName, scSim.TotalSim, scSim.simPostControlProc)

    #
    #   initialize Spacecraft States with intitialization variables
    #

    # setup the orbit using classical orbit elements
    oe = orbitalMotion.ClassicElements()
    oe.a = 10000000.0  # meters
    oe.e = 0.01
    oe.i = 33.3 * macros.D2R
    oe.Omega = 48.2 * macros.D2R
    oe.omega = 347.8 * macros.D2R
    oe.f = 85.3 * macros.D2R
    rN, vN = orbitalMotion.elem2rv(mu, oe)

    scObject.hub.r_CN_NInit = unitTestSupport.np2EigenVectorXd(rN)  # m  - r_CN_N
    scObject.hub.v_CN_NInit = unitTestSupport.np2EigenVectorXd(vN)  # m/s - v_CN_N
    scObject.hub.sigma_BNInit = [[0.1], [0.2], [-0.3]]  # sigma_BN_B
    scObject.hub.omega_BN_BInit = [[0.001], [-0.01], [0.03]]  # rad/s - omega_BN_B

    #
    #   initialize Simulation
    #
    scSim.InitializeSimulationAndDiscover()

    #
    #   configure a simulation stop time time and execute the simulation run
    #
    scSim.ConfigureStopTime(simulationTime)
    scSim.ExecuteSimulation()

    dataUsReq = scSim.pullMessageLogData(rwMotorTorqueConfig.outputDataName + ".motorTorque", range(numRW))
    dataSigmaBR = scSim.pullMessageLogData(attErrorConfig.outputDataName + ".sigma_BR", range(3))
    dataOmegaBR = scSim.pullMessageLogData(attErrorConfig.outputDataName + ".omega_BR_B", range(3))
    dataPos = scSim.pullMessageLogData(sNavObject.outputTransName + ".r_BN_N", range(3))
    dataOmegaRW = scSim.pullMessageLogData(rwStateEffector.OutputDataString + ".wheelSpeeds", range(numRW))
    dataRW = []
    for i in range(0, numRW):
        dataRW.append(scSim.pullMessageLogData(rwOutName[i] + ".u_current", range(1)))
    if useRWVoltageIO:
        dataVolt = scSim.pullMessageLogData(fswRWVoltageConfig.voltageOutMsgName + ".voltage", range(numRW))

    return dataUsReq, dataSigmaBR, dataOmegaBR, dataPos, dataOmegaRW, dataRW


## \addtogroup Tutorials_2_0_3
##   @{
# ====
#
# PythonMRPPD module implementation
# -----
#
# This class inherits from the PythonModelClass available in the simulationArchTypes module.
# The PythonModelClass is the parent class which your Python BSK modules must inherit.
# The class uses the following
# virtual functions:
# -# selfInit: The method that creates all of the messages that will be written by the
#    python model that is implemented in your class.
# -# crossInit: The method that will subscribe to all of the input messages that your class
#    needs in order to perform its function.
# -# reset: The method that will initialize any persistent data in your model to a common
#    "ready to run" state (e.g. filter states, integral control sums, etc).
# -# updateState: The method that will be called at the rate specified
#    in the PythonTask that was created in the input file.
#
# Additionally, your class should ensure that in the __init__ method, your call the super
# __init__ method for the class so that the base class' constructor also gets called to
# initialize the model-name, activity, moduleID, and other important class members:
# ~~~~~~~~~~~~~~{.py}
#       super(PythonMRPPD, self).__init__(modelName, modelActive, modelPriority)
# ~~~~~~~~~~~~~~
# You class must implement the above four functions. Beyond these four funcitons you class
# can complete any other computations you need (Numpy, matplotlib, vision processing
# AI, whatever).
class PythonMRPPD(simulationArchTypes.PythonModelClass):
    def __init__(self, modelName, modelActive=True, modelPriority=-1):
        super(PythonMRPPD, self).__init__(modelName, modelActive, modelPriority)

        # Proportional gain term used in control
        self.K = 0
        # Derivative gain term used in control
        self.P = 0
        # Input guidance structure message name
        self.inputGuidName = ""
        # Input vehicle configuration structure message name
        self.inputVehicleConfigDataName = ""
        # Output body torque message name
        self.outputDataName = ""
        # Input message ID (initialized to -1 to break messaging if unset)
        self.inputGuidID = -1
        # Input message ID (initialized to -1 to break messaging if unset)
        self.inputVehConfigID = -1
        # Output message ID (initialized to -1 to break messaging if unset)
        self.outputDataID = -1
        # Output Lr torque structure instantiation.  Note that this creates the whole class for interation
        # with the messaging system
        self.outputMessageData = MRP_PD.CmdTorqueBodyIntMsg()
        # Input guidance error structure instantiation.  Note that this creates the whole class for interation
        # with the messaging system
        self.inputGuidMsg = MRP_PD.AttGuidFswMsg()
        # Input vehicle configuration structure instantiation.  Note that this creates the whole class for interation
        # with the messaging system
        self.inputConfigMsg = MRP_PD.VehicleConfigFswMsg()

    # The selfInit method is used to initialze all of the output messages of a class.
    # It is important that ALL outputs are initialized here so that other models can
    # subscribe to these messages in their crossInit method.
    def selfInit(self):
        self.outputDataID = simulationArchTypes.CreateNewMessage(self.outputDataName, self.outputMessageData,
                                                                 self.moduleID)
        return

    # The crossInit method is used to initialize all of the input messages of a class.
    #  This subscription assumes that all of the other models present in a given simulation
    #  instance have initialized their messages during the selfInit step.
    def crossInit(self):
        self.inputGuidID = simulationArchTypes.SubscribeToMessage(self.inputGuidName, self.inputGuidMsg, self.moduleID)
        self.inputVehConfigID = simulationArchTypes.SubscribeToMessage(self.inputVehicleConfigDataName,
                                                                       self.inputConfigMsg, self.moduleID)
        return

    # The reset method is used to clear out any persistent variables that need to get changed
    #  when a task is restarted.  This method is typically only called once after selfInit/crossInit,
    #  but it should be written to allow the user to call it multiple times if necessary.
    def reset(self, currentTime):
        return

    # The updateState method is the cyclical worker method for a given Basilisk class.  It
    # will get called periodically at the rate specified in the Python task that the model is
    # attached to.  It persists and anything can be done inside of it.  If you have realtime
    # requirements though, be careful about how much processing you put into a Python updateState
    # method.  You could easily detonate your sim's ability to run in realtime.
    def updateState(self, currentTime):
        simulationArchTypes.ReadMessage(self.inputGuidID, self.inputGuidMsg, self.moduleID)
        lrCmd = np.array(self.inputGuidMsg.sigma_BR) * self.K + np.array(self.inputGuidMsg.omega_BR_B) * self.P
        self.outputMessageData.torqueRequestBody = (-lrCmd).tolist()
        simulationArchTypes.WriteMessage(self.outputDataID, currentTime, self.outputMessageData, self.moduleID)

        def print_output():
            print currentTime * 1.0E-9
            print outTorque.torqueRequestBody
            print localAttErr.sigma_BR
            print localAttErr.omega_BR_B

        return


##  @}
#
# This statement below ensures that the unit test scrip can be run as a
# stand-along python script
#
if __name__ == "__main__":
    runRegularTask(
        True,  # show_plots
        False,  # useJitterSimple
        False  # useRWVoltageIO
    )
