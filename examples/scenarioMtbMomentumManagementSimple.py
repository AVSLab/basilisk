#
#  ISC License
#
#  Copyright (c) 2021, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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

Demonstrates how to use magnetic torque bars to drive the net angular momentum of the RW cluster
to zero rather than driving
the individual wheels speeds to desired values as in :ref:`scenarioMtbMomentumManagement`.  As in this
scenario, the spacecraft is initialized with a small tumble and the :ref:`inertial3D` module is used to stabilize
to and hold a desired orientation.

In this script a series of modules are used
to control the momentum of the reaction wheels and interface with magnetic torque
bars: :ref:`mtbMomentumManagementSimple`, :ref:`torque2Dipole`, :ref:`dipoleMapping`, and :ref:`mtbFeedforward`.
Four magnetic torque bars (MTBs) (see :ref:`MtbEffector`)
are included to provide a magnetic torque. It’s important to point out that driving the net momentum
of the reaction wheels to zero does not necessarily mean driving the individual reaction wheel speeds to zero
because the wheels can be spun up in their null space, if it exists, and still have a net momentum of zero.
As a result, the :ref:`rwNullSpace` module is used to control
the null space of the wheels. The basic spacecraft setup with RWs is similar to that seen
in :ref:`scenarioAttitudeFeedbackRW`.  A magnetic field is simulated, and a three-axis magnetometer (TAM)
sensor device is added. The spacecraft is setup to stabilize
and point in a fixed inertial direction while this RW momentum control is engaged.

The script is found in the folder ``basilisk/examples`` and executed by using::

      python3 scenarioMtbMomentumManagementSimple.py

Illustration of Simulation Results
----------------------------------

::

    show_plots = True

The first plot illustrates that the :ref:`Inertial3D` module is able to achieve a stable inertial pointing.

.. image:: /_images/Scenarios/scenarioMtbMomentumManagementSimple1.svg
   :align: center

The next plots illustrate the RW states.  The motor torque are initially large to stabilize the
spacecraft orientation.  After this they return to small values that are compensating for the
magnetic momentum dumping.  Finally they settle to zero as the momentum dumping is done and the RW null motion
module has driven the RW speeds as close as it could to the desired values.


.. image:: /_images/Scenarios/scenarioMtbMomentumManagementSimple2.svg
   :align: center

.. image:: /_images/Scenarios/scenarioMtbMomentumManagementSimple3.svg
   :align: center

The following plot illustrates the MTB commanded dipoles.  They first are active to reduce the net RW
momentum and then settle to zero.

.. image:: /_images/Scenarios/scenarioMtbMomentumManagementSimple7.svg
   :align: center

"""

#
# Basilisk Scenario Script and Integrated Test
#
# Purpose:  Integrated test of the spacecraft with RWs, TAMs and MTBs to perform RW momentum dumping.
# Author:   Henry Macanas and Hanspeter Schaub
# Creation Date:  June 22, 2021
#

import os

import matplotlib.pyplot as plt
import numpy as np
# The path to the location of Basilisk
# Used to get the location of supporting data.
from Basilisk import __path__
from Basilisk.architecture import messaging
from Basilisk.fswAlgorithms import (mrpFeedback, attTrackingError,
                                    inertial3D, rwMotorTorque,
                                    tamComm, mtbMomentumManagementSimple,
                                    torque2Dipole, dipoleMapping,
                                    mtbFeedforward, rwNullSpace)
from Basilisk.simulation import (reactionWheelStateEffector,
                                 simpleNav,
                                 magneticFieldWMM, magnetometer, MtbEffector,
                                 spacecraft)
from Basilisk.utilities import (SimulationBaseClass, macros,
                                orbitalMotion, simIncludeGravBody,
                                simIncludeRW, unitTestSupport, vizSupport)

bskPath = __path__[0]
fileName = os.path.basename(os.path.splitext(__file__)[0])


# Plotting functions
def plot_attitude_error(timeData, dataSigmaBR):
    """Plot the attitude errors."""
    plt.figure(1)
    for idx in range(3):
        plt.plot(timeData, dataSigmaBR[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label=r'$\sigma_' + str(idx) + '$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel(r'Attitude Error $\sigma_{B/R}$')
    plt.grid(True)

def plot_rw_cmd_torque(timeData, dataUsReq, numRW):
    """Plot the RW command torques."""
    plt.figure(2)
    for idx in range(3):
        plt.plot(timeData, dataUsReq[:, idx],
                 '--',
                 color=unitTestSupport.getLineColor(idx, numRW),
                 label=r'$\hat u_{s,' + str(idx) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('RW Motor Torque [Nm]')
    plt.grid(True)
    
def plot_rw_motor_torque(timeData, dataUsReq, dataRW, numRW):
    """Plot the RW actual motor torques."""
    plt.figure(2)
    for idx in range(numRW):
        plt.plot(timeData, dataUsReq[:, idx],
                 '--',
                 color=unitTestSupport.getLineColor(idx, numRW),
                 label=r'$\hat u_{s,' + str(idx) + '}$')
        plt.plot(timeData, dataRW[idx],
                 color=unitTestSupport.getLineColor(idx, numRW),
                 label='$u_{s,' + str(idx) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('RW Motor Torque [Nm]')
    plt.grid(True)

def plot_rate_error(timeData, dataOmegaBR):
    """Plot the body angular velocity rate tracking errors."""
    plt.figure(3)
    for idx in range(3):
        plt.plot(timeData, dataOmegaBR[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label=r'$\omega_{BR,' + str(idx) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('Rate Tracking Error [rad/s] ')
    plt.grid(True)
    
def plot_rw_speeds(timeData, dataOmegaRW, numRW):
    """Plot the RW spin rates."""
    plt.figure(4)
    for idx in range(numRW):
        plt.plot(timeData, dataOmegaRW[:, idx] / macros.RPM,
                 color=unitTestSupport.getLineColor(idx, numRW),
                 label=r'$\Omega_{' + str(idx) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('RW Speed [RPM] ')
    plt.grid(True)
    
def plot_magnetic_field(timeData, dataMagField):
    """Plot the magnetic field."""
    plt.figure(5)
    for idx in range(3):
        plt.plot(timeData, dataMagField[:, idx] * 1e9,
                 color=unitTestSupport.getLineColor(idx, 3),
                 label=r'$B\_N_{' + str(idx) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('Magnetic Field [nT]')
    plt.grid(True)

def plot_data_tam(timeData, dataTam):
    """Plot the magnetic field."""
    plt.figure(6)
    for idx in range(3):
        plt.plot(timeData, dataTam[:, idx] * 1e9,
                 color=unitTestSupport.getLineColor(idx, 3),
                 label=r'$TAM\_S_{' + str(idx) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('Magnetic Field [nT]')
    plt.grid(True)
    
def plot_data_tam_comm(timeData, dataTamComm):
    """Plot the magnetic field."""
    plt.figure(7)
    for idx in range(3):
        plt.plot(timeData, dataTamComm[:, idx] * 1e9,
                 color=unitTestSupport.getLineColor(idx, 3),
                 label=r'$TAM\_B_{' + str(idx) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('Magnetic Field [nT]')
    plt.grid(True)
    
def plot_data_mtb_momentum_management(timeData, dataMtbMomentumManegement, numMTB):
    """Plot the magnetic field."""
    plt.figure(8)
    for idx in range(numMTB):
        plt.plot(timeData, dataMtbMomentumManegement[:, idx],
                 color=unitTestSupport.getLineColor(idx, numMTB),
                 label=r'$MTB\_T_{' + str(idx) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('Torque Rod Dipoles [A-m2]')
    plt.grid(True)
    

def plot_data_rw_motor_torque_desired(dataUsReq, tauRequested_W, numRW):
    """Plot the RW desired motor torques."""
    plt.figure(9)
    for idx in range(numRW):
        plt.plot(tauRequested_W[idx],
                 color=unitTestSupport.getLineColor(idx, numRW),
                 label='$u_{s,' + str(idx) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('RW Motor Torque (Nm)')
    plt.grid(True)
    
def run(show_plots):
    """
    The scenarios can be run with the followings setups parameters:

    Args:
        show_plots (bool): Determines if the script should display plots

    """

    # Create simulation variable names
    simTaskName = "simTask"
    simProcessName = "simProcess"

    #  Create a sim module as an empty container
    scSim = SimulationBaseClass.SimBaseClass()

    # set the simulation time variable used later on
    simulationTime = macros.min2nano(120.)

    #
    #  create the simulation process
    #
    dynProcess = scSim.CreateNewProcess(simProcessName)

    # create the dynamics task and specify the integration update time
    simulationTimeStep = macros.sec2nano(0.5)
    dynProcess.addTask(scSim.CreateNewTask(simTaskName, simulationTimeStep))

    #
    #   setup the simulation tasks/objects
    #

    # initialize spacecraft object and set properties
    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "bsk-Sat"
    # define the simulation inertia
    I = [0.02 / 3, 0., 0.,
         0., 0.1256 / 3, 0.,
         0., 0., 0.1256 / 3]
    scObject.hub.mHub = 10.0  # kg - spacecraft mass
    scObject.hub.r_BcB_B = [[0.0], [0.0], [0.0]]  # m - position vector of body-fixed point B relative to CM
    scObject.hub.IHubPntBc_B = unitTestSupport.np2EigenMatrix3d(I)

    # add spacecraft object to the simulation process
    scSim.AddModelToTask(simTaskName, scObject, 1)

    # clear prior gravitational body and SPICE setup definitions
    gravFactory = simIncludeGravBody.gravBodyFactory()

    # setup Earth Gravity Body
    earth = gravFactory.createEarth()
    earth.isCentralBody = True  # ensure this is the central gravitational body
    mu = earth.mu

    # attach gravity model to spacecraft
    scObject.gravField.gravBodies = spacecraft.GravBodyVector(list(gravFactory.gravBodies.values()))

    #
    # add RW devices
    #
    # Make a fresh RW factory instance, this is critical to run multiple times
    rwFactory = simIncludeRW.rwFactory()

    # store the RW dynamical model type
    varRWModel = messaging.BalancedWheels

    beta = 52. * np.pi / 180.
    Gs = np.array([
            [0.,            0.,             np.cos(beta), -np.cos(beta)],
            [np.cos(beta),  np.sin(beta),  -np.sin(beta), -np.sin(beta)],
            [np.sin(beta), -np.cos(beta),   0.,             0.]])

    # create each RW by specifying the RW type, the spin axis gsHat, plus optional arguments
    RW1 = rwFactory.create('BCT_RWP015', Gs[:, 0], Omega_max=5000.  # RPM
                           , RWModel=varRWModel, useRWfriction=False,
                           )
    RW2 = rwFactory.create('BCT_RWP015', Gs[:, 1], Omega_max=5000.  # RPM
                           , RWModel=varRWModel, useRWfriction=False,
                           )
    RW3 = rwFactory.create('BCT_RWP015', Gs[:, 2], Omega_max=5000.  # RPM
                           , RWModel=varRWModel, useRWfriction=False,
                           )
    
    RW4 = rwFactory.create('BCT_RWP015', Gs[:, 3], Omega_max=5000.  # RPM
                           , RWModel=varRWModel, useRWfriction=False,
                           )
    
    # In this simulation the RW objects RW1, RW2 or RW3 are not modified further.  However, you can over-ride
    # any values generate in the `.create()` process using for example RW1.Omega_max = 100. to change the
    # maximum wheel speed.

    numRW = rwFactory.getNumOfDevices()

    # create RW object container and tie to spacecraft object
    rwStateEffector = reactionWheelStateEffector.ReactionWheelStateEffector()
    rwStateEffector.ModelTag = "RW_cluster"
    rwFactory.addToSpacecraft(scObject.ModelTag, rwStateEffector, scObject)

    # add RW object array to the simulation process.  This is required for the UpdateState() method
    # to be called which logs the RW states
    scSim.AddModelToTask(simTaskName, rwStateEffector, 2)


    # add the simple Navigation sensor module.  This sets the SC attitude, rate, position
    # velocity navigation message
    sNavObject = simpleNav.SimpleNav()
    sNavObject.ModelTag = "SimpleNavigation"
    scSim.AddModelToTask(simTaskName, sNavObject)

    # create magnetic field module
    magModule = magneticFieldWMM.MagneticFieldWMM()
    magModule.ModelTag = "WMM"
    magModule.dataPath = bskPath + '/supportData/MagneticField/'
    epochMsg = unitTestSupport.timeStringToGregorianUTCMsg('2019 June 27, 10:23:0.0 (UTC)')
    magModule.epochInMsg.subscribeTo(epochMsg)
    magModule.addSpacecraftToModel(scObject.scStateOutMsg)  # this command can be repeated if multiple
    scSim.AddModelToTask(simTaskName, magModule)
    
    # add magnetic torque bar effector
    mtbEff = MtbEffector.MtbEffector()
    mtbEff.ModelTag = "MtbEff"
    scObject.addDynamicEffector(mtbEff)
    scSim.AddModelToTask(simTaskName, mtbEff)
    
    
    #
    #   setup the FSW algorithm tasks
    #

    # setup inertial3D guidance module
    inertial3DObj = inertial3D.inertial3D()
    inertial3DObj.ModelTag = "inertial3D"
    scSim.AddModelToTask(simTaskName, inertial3DObj)
    inertial3DObj.sigma_R0N = [0., 0., 0.]  # set the desired inertial orientation

    # setup the attitude tracking error evaluation module
    attError = attTrackingError.attTrackingError()
    attError.ModelTag = "attErrorInertial3D"
    scSim.AddModelToTask(simTaskName, attError)

    # setup the MRP Feedback control module
    mrpControl = mrpFeedback.mrpFeedback()
    mrpControl.ModelTag = "mrpFeedback"
    scSim.AddModelToTask(simTaskName, mrpControl)
    
    mrpControl.Ki = -1  # make value negative to turn off integral feedback
    mrpControl.K = 0.0001
    mrpControl.P = 0.002
    mrpControl.integralLimit = 2. / mrpControl.Ki * 0.1

    
    # create the minimal TAM module
    TAM = magnetometer.Magnetometer()
    TAM.ModelTag = "TAM_sensor"
    # specify the optional TAM variables
    TAM.scaleFactor = 1.0
    TAM.senNoiseStd = [0.0,  0.0, 0.0]
    scSim.AddModelToTask(simTaskName, TAM)
    
    # setup tamComm module
    tamCommObj = tamComm.tamComm()
    tamCommObj.dcm_BS = [1., 0., 0., 0., 1., 0., 0., 0., 1.]
    tamCommObj.ModelTag = "tamComm"
    scSim.AddModelToTask(simTaskName, tamCommObj)
    
    # setup mtbMomentumManagement module
    mtbMomentumManagementSimpleObj = mtbMomentumManagementSimple.mtbMomentumManagementSimple()
    mtbMomentumManagementSimpleObj.Kp = 0.003
    mtbMomentumManagementSimpleObj.ModelTag = "mtbMomentumManagementSimple"          
    scSim.AddModelToTask(simTaskName, mtbMomentumManagementSimpleObj)
    
    # setup torque2Dipole module
    torque2DipoleObj = torque2Dipole.torque2Dipole()
    torque2DipoleObj.ModelTag = "torque2Dipole"
    scSim.AddModelToTask(simTaskName, torque2DipoleObj)
    
    # mtbConfigData message
    mtbConfigParams = messaging.MTBArrayConfigMsgPayload()
    mtbConfigParams.numMTB = 4
    
    # row major toque bar alignments
    mtbConfigParams.GtMatrix_B =[
        1., 0., 0., 0.70710678,
        0., 1., 0., 0.70710678,
        0., 0., 1., 0.]
    maxDipole = 0.1
    mtbConfigParams.maxMtbDipoles = [maxDipole]*mtbConfigParams.numMTB
    mtbParamsInMsg = messaging.MTBArrayConfigMsg().write(mtbConfigParams)
    
    # setup dipoleMapping module
    dipoleMappingObj = dipoleMapping.dipoleMapping()
    
    # row major toque bar alignment inverse
    dipoleMappingObj.steeringMatrix = [0.75, -0.25, 0.,
                                          -0.25, 0.75, 0.,
                                           0., 0., 1.,
                                           0.35355339, 0.35355339, 0.]
    dipoleMappingObj.ModelTag = "dipoelMapping"
    scSim.AddModelToTask(simTaskName, dipoleMappingObj)
    
    # setup mtbFeedforward module
    mtbFeedforwardObj = mtbFeedforward.mtbFeedforward()
    mtbFeedforwardObj.ModelTag = "mtbFeedforward"
    scSim.AddModelToTask(simTaskName, mtbFeedforwardObj)
    
    # add module that maps the Lr control torque into the RW motor torques
    rwMotorTorqueObj = rwMotorTorque.rwMotorTorque()
    rwMotorTorqueObj.ModelTag = "rwMotorTorque"
    scSim.AddModelToTask(simTaskName, rwMotorTorqueObj)

    # Make the RW control all three body axes
    controlAxes_B = [
        1, 0, 0, 0, 1, 0, 0, 0, 1
    ]
    rwMotorTorqueObj.controlAxes_B = controlAxes_B
    
    # setup rwNullSpace module
    rwNullSpaceObj = rwNullSpace.rwNullSpace()
    rwNullSpaceObj.OmegaGain = 0.0000003
    rwNullSpaceObj.ModelTag = "rwNullSpace"
    scSim.AddModelToTask(simTaskName, rwNullSpaceObj)
    
    # setup RWConstellationMsgPayload
    rwConstellationConfig = messaging.RWConstellationMsgPayload()
    rwConstellationConfig.numRW = numRW
    rwConfigElementList = list()
    rwConfigElementMsg1 = messaging.RWConfigElementMsgPayload()
    rwConfigElementMsg1.gsHat_B = Gs[:, 0]
    rwConfigElementMsg1.Js = RW1.Js
    rwConfigElementMsg1.uMax = RW1.u_max
    rwConfigElementList.append(rwConfigElementMsg1)
    rwConfigElementMsg2 = messaging.RWConfigElementMsgPayload()
    rwConfigElementMsg2.gsHat_B = Gs[:, 1]
    rwConfigElementMsg2.Js = RW2.Js
    rwConfigElementMsg2.uMax = RW2.u_max
    rwConfigElementList.append(rwConfigElementMsg2)
    rwConfigElementMsg3 = messaging.RWConfigElementMsgPayload()
    rwConfigElementMsg3.gsHat_B = Gs[:, 2]
    rwConfigElementMsg3.Js = RW3.Js
    rwConfigElementMsg3.uMax = RW3.u_max
    rwConfigElementList.append(rwConfigElementMsg3)
    rwConfigElementMsg4 = messaging.RWConfigElementMsgPayload()
    rwConfigElementMsg4.gsHat_B = Gs[:, 3]
    rwConfigElementMsg4.Js = RW4.Js
    rwConfigElementMsg4.uMax = RW4.u_max
    rwConfigElementList.append(rwConfigElementMsg4)
    rwConstellationConfig.reactionWheels = rwConfigElementList
    rwConstellationConfigInMsg = messaging.RWConstellationMsg().write(rwConstellationConfig)
    
    # setup RWSpeedMsgPayload for the rwNullSpace
    desiredOmega = [1000 * macros.rpm2radsec]*numRW
    rwNullSpaceWheelSpeedBias = messaging.RWSpeedMsgPayload()
    rwNullSpaceWheelSpeedBias.wheelSpeeds = desiredOmega
    rwNullSpaceWheelSpeedBiasInMsg = messaging.RWSpeedMsg().write(rwNullSpaceWheelSpeedBias)
    
    #
    #   Setup data logging before the simulation is initialized
    #
    numDataPoints = 200
    samplingTime = unitTestSupport.samplingTime(simulationTime, simulationTimeStep, numDataPoints)
    rwMotorLog = rwMotorTorqueObj.rwMotorTorqueOutMsg.recorder(samplingTime)
    attErrorLog = attError.attGuidOutMsg.recorder(samplingTime)
    magLog = magModule.envOutMsgs[0].recorder(samplingTime)
    tamLog = TAM.tamDataOutMsg.recorder(samplingTime)
    tamCommLog = tamCommObj.tamOutMsg.recorder(samplingTime)
    mtbDipoleCmdsLog = dipoleMappingObj.dipoleRequestMtbOutMsg.recorder(samplingTime)
    
    scSim.AddModelToTask(simTaskName, rwMotorLog)
    scSim.AddModelToTask(simTaskName, attErrorLog)
    scSim.AddModelToTask(simTaskName, magLog)
    scSim.AddModelToTask(simTaskName, tamLog)
    scSim.AddModelToTask(simTaskName, tamCommLog)
    scSim.AddModelToTask(simTaskName, mtbDipoleCmdsLog)
    
    # To log the RW information, the following code is used:
    mrpLog = rwStateEffector.rwSpeedOutMsg.recorder(samplingTime)
    scSim.AddModelToTask(simTaskName, mrpLog)
    
    # A message is created that stores an array of the \f$\Omega\f$ wheel speeds.  This is logged
    # here to be plotted later on.  However, RW specific messages are also being created which
    # contain a wealth of information.  The vector of messages is ordered as they were added.  This
    # allows us to log RW specific information such as the actual RW motor torque being applied.
    rwLogs = []
    for item in range(numRW):
        rwLogs.append(rwStateEffector.rwOutMsgs[item].recorder(samplingTime))
        scSim.AddModelToTask(simTaskName, rwLogs[item])
        
    #
    # create simulation messages
    #

    # create the FSW vehicle configuration message
    vehicleConfigOut = messaging.VehicleConfigMsgPayload()
    vehicleConfigOut.ISCPntB_B = I  # use the same inertia in the FSW algorithm as in the simulation
    vcMsg = messaging.VehicleConfigMsg().write(vehicleConfigOut)

    # Setup the FSW RW configuration to be the same as the simulated RW configuration
    fswRwParamMsg = rwFactory.getConfigMessage()

    #
    #   set initial Spacecraft States
    #
    # setup the orbit using classical orbit elements
    oe = orbitalMotion.ClassicElements()
    oe.a = 6778.14 * 1000.  # meters
    oe.e = 0.00
    oe.i = 45. * macros.D2R
    oe.Omega = 60. * macros.D2R
    oe.omega = 0. * macros.D2R
    oe.f = 0. * macros.D2R
    rN, vN = orbitalMotion.elem2rv(mu, oe)
    scObject.hub.r_CN_NInit = rN  # m   - r_CN_N
    scObject.hub.v_CN_NInit = vN  # m/s - v_CN_N
    scObject.hub.sigma_BNInit = [[0.1], [0.2], [-0.3]]  # sigma_CN_B
    scObject.hub.omega_BN_BInit = [[0.001], [-0.01], [0.03]]  # rad/s - omega_CN_B

    # if this scenario is to interface with the BSK Viz, uncomment the following lines
    viz = vizSupport.enableUnityVisualization(scSim, simTaskName, scObject
                                              # , saveFile=fileName
                                              , rwEffectorList=rwStateEffector
                                              )

    # link messages
    sNavObject.scStateInMsg.subscribeTo(scObject.scStateOutMsg)
    attError.attNavInMsg.subscribeTo(sNavObject.attOutMsg)
    attError.attRefInMsg.subscribeTo(inertial3DObj.attRefOutMsg)
    mrpControl.guidInMsg.subscribeTo(attError.attGuidOutMsg)
    mrpControl.vehConfigInMsg.subscribeTo(vcMsg)
    mrpControl.rwParamsInMsg.subscribeTo(fswRwParamMsg)
    mrpControl.rwSpeedsInMsg.subscribeTo(rwStateEffector.rwSpeedOutMsg)

    TAM.stateInMsg.subscribeTo(scObject.scStateOutMsg)
    TAM.magInMsg.subscribeTo(magModule.envOutMsgs[0])
    tamCommObj.tamInMsg.subscribeTo(TAM.tamDataOutMsg)
    
    mtbMomentumManagementSimpleObj.rwParamsInMsg.subscribeTo(fswRwParamMsg)
    mtbMomentumManagementSimpleObj.rwSpeedsInMsg.subscribeTo(rwStateEffector.rwSpeedOutMsg)
    
    torque2DipoleObj.tauRequestInMsg.subscribeTo(mtbMomentumManagementSimpleObj.tauMtbRequestOutMsg)
    torque2DipoleObj.tamSensorBodyInMsg.subscribeTo(tamCommObj.tamOutMsg)
    
    dipoleMappingObj.dipoleRequestBodyInMsg.subscribeTo(torque2DipoleObj.dipoleRequestOutMsg)
    dipoleMappingObj.mtbArrayConfigParamsInMsg.subscribeTo(mtbParamsInMsg)
    
    mtbFeedforwardObj.vehControlInMsg.subscribeTo(mrpControl.cmdTorqueOutMsg)
    mtbFeedforwardObj.dipoleRequestMtbInMsg.subscribeTo(dipoleMappingObj.dipoleRequestMtbOutMsg)
    mtbFeedforwardObj.tamSensorBodyInMsg.subscribeTo(tamCommObj.tamOutMsg)
    mtbFeedforwardObj.mtbArrayConfigParamsInMsg.subscribeTo(mtbParamsInMsg)
    
    rwMotorTorqueObj.rwParamsInMsg.subscribeTo(fswRwParamMsg)
    rwMotorTorqueObj.vehControlInMsg.subscribeTo(mtbFeedforwardObj.vehControlOutMsg)
    
    rwNullSpaceObj.rwMotorTorqueInMsg.subscribeTo(rwMotorTorqueObj.rwMotorTorqueOutMsg)
    rwNullSpaceObj.rwSpeedsInMsg.subscribeTo(rwStateEffector.rwSpeedOutMsg)
    rwNullSpaceObj.rwConfigInMsg.subscribeTo(rwConstellationConfigInMsg)
    rwNullSpaceObj.rwDesiredSpeedsInMsg.subscribeTo(rwNullSpaceWheelSpeedBiasInMsg)
    
    rwStateEffector.rwMotorCmdInMsg.subscribeTo(rwNullSpaceObj.rwMotorTorqueOutMsg)
    
    mtbEff.mtbCmdInMsg.subscribeTo(dipoleMappingObj.dipoleRequestMtbOutMsg)
    mtbEff.mtbParamsInMsg.subscribeTo(mtbParamsInMsg)
    mtbEff.magInMsg.subscribeTo(magModule.envOutMsgs[0])
    
    
    # initialize configure and execute sim
    scSim.InitializeSimulation()
    scSim.ConfigureStopTime(simulationTime)
    scSim.ExecuteSimulation()

    #   retrieve the logged data
    dataUsReq = rwMotorLog.motorTorque
    dataSigmaBR = attErrorLog.sigma_BR
    dataOmegaBR = attErrorLog.omega_BR_B
    dataOmegaRW = mrpLog.wheelSpeeds
    dataRW = []
    for i in range(numRW):
        dataRW.append(rwLogs[i].u_current)
    dataMagField = magLog.magField_N
    dataTam = tamLog.tam_S
    dataTamComm = tamCommLog.tam_B
    dataMtbDipoleCmds = mtbDipoleCmdsLog.mtbDipoleCmds
    
    np.set_printoptions(precision=16)

    #   plot the results
    timeData = rwMotorLog.times() * macros.NANO2MIN
    plt.close("all")  # clears out plots from earlier test runs

    plot_attitude_error(timeData, dataSigmaBR)
    figureList = {}
    pltName = fileName + "1"
    figureList[pltName] = plt.figure(1)

    plot_rw_motor_torque(timeData, dataUsReq, dataRW, numRW)
    pltName = fileName + "2"
    figureList[pltName] = plt.figure(2)

    plot_rate_error(timeData, dataOmegaBR)
    plot_rw_speeds(timeData, dataOmegaRW, numRW)
    pltName = fileName + "3"
    figureList[pltName] = plt.figure(4)

    plot_magnetic_field(timeData, dataMagField)
    pltName = fileName + "4"
    figureList[pltName] = plt.figure(5)
    
    plot_data_tam(timeData, dataTam)
    pltName = fileName + "5"
    figureList[pltName] = plt.figure(6)
    
    plot_data_tam_comm(timeData, dataTamComm)
    pltName = fileName + "6"
    figureList[pltName] = plt.figure(7)
    
    plot_data_mtb_momentum_management(timeData, dataMtbDipoleCmds, mtbConfigParams.numMTB)
    pltName = fileName + "7"
    figureList[pltName] = plt.figure(8)

    if show_plots:
        plt.show()

    # close the plots being saved off to avoid over-writing old and new figures
    plt.close("all")

    return figureList


#
# This statement below ensures that the unit test scrip can be run as a
# stand-along python script
#
if __name__ == "__main__":
    run(
        True,  # show_plots
    )
