#
#  ISC License
#
#  Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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
It's recommended to review the following scenario(s) first (and any
recommended scenario(s) that they may have):

#. ``examples/scenarioMomentumDumping.py``
#. ``examples/mujoco/scenarioAttitudeFeedbackRW.py``

This script demonstrates how to run a reaction wheel momentum dumping
scenario, stranslated from the classic Basilisk ``scenarioMomentumDumping.py``
example, using MuJoCo dynamics via :ref:`MJScene<MJScene>` instead of the
traditional hub-centric Basilisk :ref:`spacecraft` dynamics.

The multi-body system is created programmatically as a MuJoCo XML string.
It consists of a free-floating spacecraft bus ("hub") carrying 4 reaction
wheel rigid bodies ("rw1Spin" ... "rw4Spin") mounted at canted spin
axes, and 8 thrusters attached directly to the hub via fixed sites.
Each wheel spin DOF is driven by a MuJoCo single-input torque actuator, and
each thruster site has a MuJoCo motor actuator that applies force along the
site's z-axis.

A standard Basilisk FSW stack is used to point the hub at a fixed inertial
attitude, offload control effort onto the reaction wheels, and dump excess
wheel momentum with the ACS thrusters once it exceeds a threshold:

#. ``inertial3D`` generates a fixed inertial attitude reference.
#. ``attTrackingError`` computes the attitude and rate tracking errors.
#. ``mrpFeedback`` computes the commanded body control torque.
#. ``rwMotorTorque`` maps the commanded body torque into individual wheel
   motor torque commands.
#. ``thrMomentumManagement`` monitors the total wheel momentum and computes
   a delta-H command once it exceeds a specified maximum.
#. ``thrForceMapping`` maps the desaturation delta-H command into individual
   thruster force commands.
#. ``thrMomentumDumping`` converts the thruster force commands into
   discrete thruster on-time pulses.

   ** See ``scenarioMomentumDumping.py`` for more info on this.

Several small adapter modules bridge Basilisk messaging to MuJoCo objects:

#. ``RWTorqueDistributor`` splits the single ``ArrayMotorTorqueMsg`` from
   ``rwMotorTorque`` into individual ``SingleActuatorMsg`` commands, one per
   MuJoCo wheel actuator.
#. ``RWSpeedCombiner`` reads each wheel's spin rate directly from its MuJoCo
   joint state and republishes them as a single ``RWSpeedMsg`` for the
   momentum-management chain.
#. ``thrOnTimeToForce`` converts the thruster on-time pulse commands from
   ``thrMomentumDumping`` into instantaneous force commands for each MuJoCo
   thruster actuator.

Earth gravity is configured using :ref:`NBodyGravity<NBodyGravity>` with a
:ref:`pointMassGravityModel<pointMassGravityModel>` as the central body,
applied to the hub only.

The spacecraft is placed on a near-circular LEO orbit, and the wheels are
initialized above the momentum-dumping threshold. The simulation briefly
coasts before the momentum management module is reset (since desaturation
cannot begin exactly at t = 0), then runs for 5 minutes while the wheels
provide attitude control and the thrusters intermittently fire to dump
excess wheel momentum.

Attitude error, rate tracking error, individual and total wheel momenta,
dumped momentum, wheel speeds, thruster impulse requests, thruster on-time
requests, and delivered thruster forces are plotted at the end.
"""

import os
import matplotlib.pyplot as plt
import numpy as np

from Basilisk.utilities import SimulationBaseClass, macros, orbitalMotion, simHelpers, simIncludeRW, simIncludeThruster
from Basilisk.simulation import NBodyGravity, mujoco, pointMassGravityModel, simpleNav, thrOnTimeToForce
from Basilisk.fswAlgorithms import mrpFeedback, inertial3D, attTrackingError, rwMotorTorque, thrMomentumManagement, thrForceMapping, thrMomentumDumping
from Basilisk.architecture import messaging, sysModel

from Basilisk import __path__

# Used to tag saved figs with name of file
fileName = os.path.basename(os.path.splitext(__file__)[0])

# -------------------------------------------------------------------------
# PLOTTING FUNCTIONS
# -------------------------------------------------------------------------
def plot_attitude_error(timeData, dataSigmaBR):
    """Plot the attitude errors."""
    fig = plt.figure(num = 1, clear = True)
    ax = fig.gca()
    ax.ticklabel_format(useOffset = False, style = 'plain')
    for idx in range(3):
        plt.plot(timeData, dataSigmaBR[:, idx],
                 color = simHelpers.getLineColor(idx, 3),
                 label = r'$\sigma_' + str(idx) + r'$')
    plt.legend(loc = 'lower right')
    plt.xlabel('Time [min]')
    plt.ylabel(r'Attitude Error $\sigma_{B/R}$')

    return fig


def plot_rate_error(timeData, dataOmegaBR):
    """Plot the body angular velocity rate tracking errors."""
    fig = plt.figure(num = 2, clear = True)
    ax = fig.gca()
    ax.ticklabel_format(useOffset = False, style = 'plain')
    for idx in range(3):
        plt.plot(timeData, dataOmegaBR[:, idx],
                 color = simHelpers.getLineColor(idx, 3),
                 label = r'$\omega_{BR,' + str(idx+1) + r'}$')
    plt.legend(loc = 'lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('Rate Tracking Error (rad/s) ')

    return fig


def plot_rw_momenta(timeData, dataOmegaRw, RW, numRW):
    """Plot the RW momenta."""
    # Total momentum norm found by projecting each wheel's momentum onto spin axis and summing
    totMomentumNorm = []
    for j in range(len(timeData)):
        totMomentum = np.array([0,0,0])
        for idx in range(numRW):
            for k in range(3):
                totMomentum[k] = totMomentum[k] + dataOmegaRw[j, idx] * RW[idx].Js * RW[idx].gsHat_B[k][0]
        totMomentumNorm.append(np.linalg.norm(totMomentum))

    fig = plt.figure(num = 3, clear = True)
    ax = fig.gca()
    ax.ticklabel_format(useOffset = False, style = 'plain')
    for idx in range(numRW):
        plt.plot(timeData, dataOmegaRw[:, idx] * RW[idx].Js,
                 color = simHelpers.getLineColor(idx, numRW),
                 label = r'$H_{' + str(idx+1) + r'}$')
    plt.plot(timeData, totMomentumNorm, '--',
             label = r'$\|H\|$')
    plt.legend(loc = 'lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('RW Momentum (Nms)')

    return fig


def plot_DH(timeData, dataDH):
    """Plot the body angular velocity rate tracking errors."""
    fig = plt.figure(num = 4, clear = True)
    ax = fig.gca()
    ax.ticklabel_format(useOffset = False, style = 'plain')
    for idx in range(3):
        plt.plot(timeData, dataDH[:, idx],
                 color = simHelpers.getLineColor(idx, 3),
                 label = r'$\Delta H_{' + str(idx+1) + r'}$')
    plt.legend(loc = 'lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('Dumped momentum (Nms) ')

    return fig


def plot_rw_speeds(timeData, dataOmegaRW, numRW):
    """Plot the RW spin rates."""
    fig = plt.figure(num = 5, clear = True)
    ax = fig.gca()
    ax.ticklabel_format(useOffset = False, style = 'plain')
    plt.figure(5)
    for idx in range(numRW):
        plt.plot(timeData, dataOmegaRW[:, idx] / macros.RPM,
                 color = simHelpers.getLineColor(idx, numRW),
                 label = r'$\Omega_{' + str(idx+1) + r'}$')
    plt.legend(loc = 'lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('RW Speed (RPM) ')

    return fig


def plot_thrImpulse(timeDataFSW, dataMap, numTh):
    """Plot the Thruster force values."""
    fig = plt.figure(num = 6, clear = True)
    ax = fig.gca()
    ax.ticklabel_format(useOffset = False, style = 'plain')
    for idx in range(numTh):
        plt.plot(timeDataFSW, dataMap[:, idx],
                 color = simHelpers.getLineColor(idx, numTh),
                 label = r'$thrImpulse_{' + str(idx+1) + r'}$')
    plt.legend(loc = 'lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('Impulse requested [Ns]')

    return fig


def plot_OnTimeRequest(timeData, dataOnTime, numTh):
    """Plot the thruster on time requests."""
    fig = plt.figure(num = 7, clear = True)
    ax = fig.gca()
    ax.ticklabel_format(useOffset = False, style = 'plain')
    for idx in range(numTh):
        plt.plot(timeData, dataOnTime[:, idx],
                 color = simHelpers.getLineColor(idx, numTh),
                 label = r'$OnTimeRequest_{' + str(idx+1) + r'}$')
    plt.legend(loc = 'lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('OnTimeRequest [sec]')

    return fig


def plot_thrForce(timeDataFSW, dataThr, numTh):
    """Plot the Thruster force values."""
    fig = plt.figure(num = 8, clear = True)
    ax = fig.gca()
    ax.ticklabel_format(useOffset = False, style = 'plain')
    for idx in range(numTh):
        plt.plot(timeDataFSW, dataThr[idx],
                 color = simHelpers.getLineColor(idx, numTh),
                 label = r'$thrForce_{' + str(idx+1) + r'}$')
    plt.legend(loc = 'lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('Thruster force [N]')

    return fig


# addRWsXML: supporter function for XML constructor function to generate RW bodies within
#   hub body, generalized for any number of RWs
#   INPUTS:
#       - rwPos: list of [x, y, z] positions, one per wheel
#       - rwAxes: list of [x, y, z] unit spin-axis vectors, one per wheel
#       - rwFactory: simIncludeRW.rwFactory instance used to create the RW config objects
#       - maxMomentum: maximum momentum spec [Nms]
#       - baseIndent: initial num of indents (tabs) necessary for proper XML formatting
def addRWsXML(rwPos: list, 
              rwAxes: list,
              rwFactory: simIncludeRW, 
              maxMomentum: float = 100.,
              baseIndent: int = 3):

    numRW = len(rwPos)
    pad = "\t" * baseIndent

    rwTags, actTags, RWs = [], [], []

    for idx in range(numRW):
        n = idx + 1
        pos = rwPos[idx]
        rwAxis = rwAxes[idx]

        # Register wheel with the RW factory to get correct mass/inertia/torque properties
        rw = rwFactory.create('Honeywell_HR16', rwAxis, maxMomentum = maxMomentum)
        RWs.append(rw)

        # XML string defining RW body: single revolute joint about spin axis, inertial props
        # taken from the factory-created wheel (Jt, Jt, Js), simple cylinder geom for visuals only
        rwTags.append(
f"""{pad}<body name = "rw{n}Spin" pos = "{pos[0]} {pos[1]} {pos[2]}" zaxis = "{rwAxis[0]} {rwAxis[1]} {rwAxis[2]}">
{pad}\t<joint name = "rw{n}Joint" pos = "0 0 0" axis = "0 0 1" ref = "0"/>
{pad}\t<inertial pos = "0 0 0" mass = "{rw.mass}" diaginertia = "{rw.Jt} {rw.Jt} {rw.Js}"/>
{pad}\t<geom name = "rw{n}Geom" type = "cylinder" size = "0.2 0.05" contype = "0" conaffinity = "0"/>
{pad}</body>""")
        # Torque-controlled actuator on the wheel joint, limited to the wheel's max motor torque
        actTags.append(f'\t\t<motor name = "rw{n}Act" joint = "rw{n}Joint" ctrlrange = "{-rw.u_max} {rw.u_max}" ctrllimited = "true"/>')

    return "\n".join(rwTags), "\n".join(actTags), RWs


# addThrustersXML: supporter function for XML constructor function to generate thruster sites
#   and force actuators on the hub body, generalized for any number of thrusters
#   INPUTS:
#       - thrustLocs: list of [x, y, z] thruster mounting positions
#       - thrustDirs: list of [x, y, z] unit thrust-direction vectors, one per thruster
#       - thrFactory: simIncludeThruster.thrusterFactory instance used to create the thruster config objects
#       - maxThrust: maximum thrust force spec [N]
#       - baseIndent: initial number of indents (tabs) necessary for proper XML formatting
def addThrustersXML(thrustLocs: list, 
                    thrustDirs: list,
                    thrFactory: simIncludeThruster,
                    maxThrust: float = 5.0,
                    baseIndent: int = 2):

    numTHRs = len(thrustLocs)
    pad = "\t" * baseIndent

    thrustTags, actTags, THRs = [], [], []

    for idx in range(numTHRs):
        n = idx + 1
        pos = thrustLocs[idx]
        dirVec = thrustDirs[idx]

        # Register thruster with thruster factory to get correct FSW config properties
        thr = thrFactory.create('MOOG_Monarc_5', pos, dirVec, MaxThrust = maxThrust)
        THRs.append(thr)

        # Site marks thruster location/orientation on hub, motor applies force along site z-axis
        thrustTags.append(f'{pad}\t<site name = "thrusterSite{n}" pos = "{pos[0]} {pos[1]} {pos[2]}" zaxis = "{dirVec[0]} {dirVec[1]} {dirVec[2]}"/>')
        actTags.append(f'{pad}<motor name = "thruster{n}" site = "thrusterSite{n}" gear = "0 0 1 0 0 0" ctrlrange = "0 5"/>')
    
    return "\n".join(thrustTags), "\n".join(actTags), THRs


# makeMjXmlString: MuJoCo string constructor, creates MJ model of the hub carrying 4 RWs and
#   8 ACS thrusters, hub properties specified in function
def makeMjXmlString():
    # Hub mass properties
    hubMass = 2500.0
    hubIxx, hubIyy, hubIzz = 1700.0, 1700.0, 1800.0
 
    # All 4 RWs mounted at same offset point, canted along 4 different spin axes
    c = 2**-0.5
    rwPos = [[0.0, 0.0, 1.28]] * 4
    rwAxes = [[c, 0, c], [0, c, c], [-c, 0, c], [0, -c, c]]

    # 8 ACS thrusters at hub corners, each aligned with a principal body axis
    a, b = 1.0, 1.28
    thrustLocs = [
        [-a, -a,  b], [ a, -a, -b], [ a, -a,  b], [ a,  a, -b],
        [ a,  a,  b], [-a,  a, -b], [-a,  a,  b], [-a, -a, -b],
    ]
    thrustDirs = np.array([
        [ 1,  0,  0], [-1,  0,  0], [ 0,  1,  0], [ 0, -1,  0],
        [-1,  0,  0], [ 1,  0,  0], [ 0, -1,  0], [ 0,  1,  0],
    ])
 
    rwFactory = simIncludeRW.rwFactory()
    thrFactory = simIncludeThruster.thrusterFactory()

    # Generating RW and thruster XML chains
    rwBodies, rwActs, RWs = addRWsXML(rwPos, rwAxes, rwFactory)
    thrSites, thrActs, THRs = addThrustersXML(thrustLocs, thrustDirs, thrFactory)
 
    xml = f"""<mujoco model = "busWithRWsAndThrusters">
    <compiler angle = "radian" meshdir = ""/>
 
    <worldbody>
        <body name = "hub" pos = "0 0 0">
            <freejoint name = "busFree"/>
            <inertial pos = "0 0 {b}" mass = "{hubMass}" diaginertia = "{hubIxx} {hubIyy} {hubIzz}"/>
            <geom name = "hubVisual" type = "box" size = "1 1 1.28" rgba = "1 1 1 1"/>

{thrSites}

{rwBodies}

        </body>
    </worldbody>

    <actuator>
{rwActs}

{thrActs}
    </actuator>

</mujoco>"""

    return xml, RWs, THRs, rwFactory, thrFactory


def run(showPlots: bool = False):
    # -------------------------------------------------------------------------
    # 1) Simulation configuration and MJScene dynamics model
    # -------------------------------------------------------------------------
    fswTaskName = "fswTask"
    dynTaskName = "dynTask"
    simProcessName = "simProcess"

    # Initializing simulation time/time-steps for dynamics/fsw task
    simulationTime = macros.min2nano(5)
    simulationTimeStepFsw = macros.sec2nano(1)
    simulationTimeStepDyn = macros.sec2nano(0.1)

    sim = SimulationBaseClass.SimBaseClass()

    dynProcess = sim.CreateNewProcess(simProcessName)
    dynProcess.addTask(sim.CreateNewTask(dynTaskName, simulationTimeStepDyn))
    dynProcess.addTask(sim.CreateNewTask(fswTaskName, simulationTimeStepFsw))

    # Constructing MJ XML string (hub + 4 RWs + 8 thrusters) and loading into MJScene
    xmlString, RWs, THRs, rwFactory, thrFactory = makeMjXmlString()
    scene = mujoco.MJScene(xmlString)
    scene.ModelTag = "mujocoScene"
    scene.extraEoMCall = True  # lets custom sys models step alongside MuJoCo's EoM evaluation
    sim.AddModelToTask(dynTaskName, scene)

    # -------------------------------------------------------------------------
    # 2) Retrieve spacecraft componenets
    # -------------------------------------------------------------------------
    # Pull handles of hub/RW bodies and actuators from XML
    busBody = scene.getBody("hub")
    numRWs = len(RWs)
    numTHRs = len(THRs)

    RWBodies = [scene.getBody(f"rw{i + 1}Spin") for i in range(numRWs)]
    RWJoints = [RWBodies[i].getScalarJoint(f"rw{i + 1}Joint") for i in range(numRWs)]
    RWActuators = [scene.getSingleActuator(f"rw{i + 1}Act") for i in range(numRWs)]
    THRActuators = [scene.getSingleActuator(f"thruster{i + 1}") for i in range(numTHRs)]

    # -------------------------------------------------------------------------
    # 3) Add gravity
    # -------------------------------------------------------------------------
    # Adding N-Body gravity model into MJScene
    gravity = NBodyGravity.NBodyGravity()
    gravity.ModelTag = "gravity"

    muEarth = 0.3986004415e15  # [m^3/s^2]
    earthPm = pointMassGravityModel.PointMassGravityModel()
    earthPm.muBody = muEarth
    gravity.addGravitySource("earth", earthPm, isCentralBody = True)

    # Gravity effects applied to hub only (RWs treated as rigidly attached, negligible mass)
    gravity.addGravityTarget("hub", busBody)

    scene.AddModelToDynamicsTask(gravity)

    # -------------------------------------------------------------------------
    # 4) Initial conditions
    # -------------------------------------------------------------------------
    oe = orbitalMotion.ClassicElements()
    rLEO = 7000. * 1000  # meters
    oe.a = rLEO
    oe.e = 0.0001
    oe.i = 33.3 * macros.D2R
    oe.Omega = 148.2 * macros.D2R
    oe.omega = 347.8 * macros.D2R
    oe.f = 335 * macros.D2R
    rN, vN = orbitalMotion.elem2rv(muEarth, oe)

    # -------------------------------------------------------------------------
    # 5) Navitation and FSW
    # -------------------------------------------------------------------------
    # Reading s/c state and publishing standard navigation output
    simpleNavObj = simpleNav.SimpleNav()
    simpleNavObj.ModelTag = "simpleNav"
    simpleNavObj.scStateInMsg.subscribeTo(busBody.getCenterOfMass().stateOutMsg)
    sim.AddModelToTask(dynTaskName, simpleNavObj)

    # Fixed inertial pointing target
    inertial3DObj = inertial3D.inertial3D()
    inertial3DObj.ModelTag = "inertial3D"
    inertial3DObj.sigma_R0N = [0.0, 0.0, 0.0]
    sim.AddModelToTask(fswTaskName, inertial3DObj)

    # Tracks attitude error of s/c
    attError = attTrackingError.attTrackingError()
    attError.ModelTag = "attErrorInertial3D"
    attError.attNavInMsg.subscribeTo(simpleNavObj.attOutMsg)
    attError.attRefInMsg.subscribeTo(inertial3DObj.attRefOutMsg)
    sim.AddModelToTask(fswTaskName, attError)

    # MRP feedback control law computing the commanded control torque
    mrpControl = mrpFeedback.mrpFeedback()
    mrpControl.ModelTag = "mrpFeedback"
    sim.AddModelToTask(fswTaskName, mrpControl)
    decayTime = 10.0
    xi = 1.0
    I = np.diag([1700, 1700, 1800])
    mrpControl.Ki = -1  # make value negative to turn off integral feedback
    mrpControl.P = 3 * np.max(I) / decayTime
    mrpControl.K = (mrpControl.P/xi) * (mrpControl.P/xi) / np.max(I)
    mrpControl.integralLimit = 2. / mrpControl.Ki * 0.1

    controlAxes_B = [1, 0, 0, 0, 1, 0, 0, 0, 1]

    # Add module that maps the Lr control torque into the RW motor torques
    rwMotorTorqueObj = rwMotorTorque.rwMotorTorque()
    rwMotorTorqueObj.ModelTag = "rwMotorTorque"
    sim.AddModelToTask(fswTaskName, rwMotorTorqueObj)
    # Make the RW control all three body axes
    rwMotorTorqueObj.controlAxes_B = controlAxes_B

    # Momentum dumping configuration
    thrDesatControl = thrMomentumManagement.thrMomentumManagement()
    thrDesatControl.ModelTag = "thrMomentumManagement"
    sim.AddModelToTask(fswTaskName, thrDesatControl)
    thrDesatControl.hs_min = 80   # Nms : maximum wheel momentum

    # Setup the thruster force mapping module
    thrForceMappingObj = thrForceMapping.thrForceMapping()
    thrForceMappingObj.ModelTag = "thrForceMapping"
    sim.AddModelToTask(fswTaskName, thrForceMappingObj)
    thrForceMappingObj.controlAxes_B = controlAxes_B
    thrForceMappingObj.thrForceSign = 1
    thrForceMappingObj.angErrThresh = 3.15 # This needs to be larger than pi (180 deg) for the module to work in the momentum dumping scenario

    # Setup the thruster momentum dumping module
    thrDump = thrMomentumDumping.thrMomentumDumping()
    thrDump.ModelTag = "thrDump"
    sim.AddModelToTask(fswTaskName, thrDump)
    thrDump.maxCounterValue = 100 # Number of control periods (simulationTimeStepFsw) to wait between two subsequent on-times
    thrDump.thrMinFireTime = 0.02       

    # FSW-facing config messages describing all 4 wheels / 8 thrusters
    fswRwParamMsg = rwFactory.getConfigMessage()   
    fswThrParamMsg = thrFactory.getConfigMessage()

    # -------------------------------------------------------------------------
    # 6) Distribute torques/
    # -------------------------------------------------------------------------
    # rwMotorTorque outputs one array message; each MuJoCo RW actuator needs its own single
    # actuator message, so RWTorqueDistributor (see below) fans the array out per wheel
    rwDistributor = RWTorqueDistributor(numRWs)
    rwDistributor.rwMotorTorqueInMsg.subscribeTo(rwMotorTorqueObj.rwMotorTorqueOutMsg)
    sim.AddModelToTask(fswTaskName, rwDistributor)
    for i in range(numRWs):
        RWActuators[i].actuatorInMsg.subscribeTo(rwDistributor.torqueOutMsgs[i])

    # thrOnTimeToForce used to convert on-time commands from FSW stack to instant on/off
    # thruster actuator messages for MuJoCo thrusters
    thrForceConverter = thrOnTimeToForce.ThrOnTimeToForce()
    thrForceConverter.ModelTag = "thrOnTimeToForce"
    for _ in range(numTHRs):
        thrForceConverter.addThruster()
    thrForceConverter.setThrMag([thr.MaxThrust for thr in THRs])

    scene.AddModelToDynamicsTask(thrForceConverter)
    thrForceConverter.onTimeInMsg.subscribeTo(thrDump.thrusterOnTimeOutMsg)

    for i in range(numTHRs):
        THRActuators[i].actuatorInMsg.subscribeTo(thrForceConverter.thrusterForceOutMsgs[i])

    # -------------------------------------------------------------------------
    # 6) Message Linking
    # -------------------------------------------------------------------------
    # Inertia tensor passed into config message, vehicle config created
    vehicleConfigOut = messaging.VehicleConfigMsgPayload(ISCPntB_B = [1700,0,0, 0,1700,0, 0,0,1800])
    vcMsg = messaging.VehicleConfigMsg().write(vehicleConfigOut)
    mrpControl.vehConfigInMsg.subscribeTo(vcMsg)

    mrpControl.guidInMsg.subscribeTo(attError.attGuidOutMsg)
    mrpControl.rwParamsInMsg.subscribeTo(fswRwParamMsg)

    # RWSpeedCombiner (see below) gathers individual per-joint wheel speeds from MuJoCo,
    # republishes them as a single RWSpeedMsg
    speedCombiner = RWSpeedCombiner(RWJoints)
    sim.AddModelToTask(fswTaskName, speedCombiner, 100)
    mrpControl.rwSpeedsInMsg.subscribeTo(speedCombiner.speedOutMsg)

    rwMotorTorqueObj.rwParamsInMsg.subscribeTo(fswRwParamMsg)
    thrForceMappingObj.vehConfigInMsg.subscribeTo(vcMsg)
    rwMotorTorqueObj.vehControlInMsg.subscribeTo(mrpControl.cmdTorqueOutMsg)

    thrDesatControl.rwSpeedsInMsg.subscribeTo(speedCombiner.speedOutMsg)
    thrDesatControl.rwConfigDataInMsg.subscribeTo(fswRwParamMsg)
    thrForceMappingObj.thrConfigInMsg.subscribeTo(fswThrParamMsg)
    thrForceMappingObj.cmdTorqueInMsg.subscribeTo(thrDesatControl.deltaHOutMsg)
    thrDump.thrusterConfInMsg.subscribeTo(fswThrParamMsg)   
    thrDump.deltaHInMsg.subscribeTo(thrDesatControl.deltaHOutMsg)
    thrDump.thrusterImpulseInMsg.subscribeTo(thrForceMappingObj.thrForceCmdOutMsg)

    # -------------------------------------------------------------------------
    # 7) Set up data recording
    # -------------------------------------------------------------------------
    numDataPoints = 5000
    samplingTime = simHelpers.samplingTime(simulationTime, simulationTimeStepDyn, numDataPoints)

    sNavRec = simpleNavObj.attOutMsg.recorder(samplingTime)
    sim.AddModelToTask(dynTaskName, sNavRec)

    dataRec = busBody.getCenterOfMass().stateOutMsg.recorder(samplingTime)
    sim.AddModelToTask(dynTaskName, dataRec)

    rwMotorLog = rwMotorTorqueObj.rwMotorTorqueOutMsg.recorder(samplingTime)
    sim.AddModelToTask(dynTaskName, rwMotorLog)

    attErrorLog = attError.attGuidOutMsg.recorder(samplingTime)
    sim.AddModelToTask(dynTaskName, attErrorLog)

    deltaHLog  = thrDesatControl.deltaHOutMsg.recorder(samplingTime)
    sim.AddModelToTask(dynTaskName, deltaHLog)

    thrMapLog = thrForceMappingObj.thrForceCmdOutMsg.recorder(samplingTime)
    sim.AddModelToTask(dynTaskName, thrMapLog)

    onTimeLog = thrDump.thrusterOnTimeOutMsg.recorder(samplingTime)
    sim.AddModelToTask(dynTaskName, onTimeLog)

    # Wheel speeds live as individual MuJoCo joint states, so log each separately and column-stack later for plotting
    rwSpeedLogs = []
    for i in range(numRWs):
        rwSpeedLogs.append(RWJoints[i].stateDotOutMsg.recorder(samplingTime))
        sim.AddModelToTask(dynTaskName, rwSpeedLogs[i])

    # Delivered force at each thruster actuator, logged individually
    thrForceLogs = []
    for i in range(numTHRs):
        thrForceLogs.append(THRActuators[i].actuatorInMsg.recorder(samplingTime))
        sim.AddModelToTask(dynTaskName, thrForceLogs[i])

    # -------------------------------------------------------------------------
    # 8) Running simulation
    # -------------------------------------------------------------------------
    sim.InitializeSimulation()

    # Setting initial conditions
    busFree = busBody.getFreeJoint()
    busBody.setPosition(rN)
    busFree.setVelocity(vN)

    # Initial wheel spin rates, chosen so the wheels start above the 80 Nms desat threshold
    initialOmegas = [4000., 2000., 3500., 0.]
    for i in range(numRWs):
        omega_radps = initialOmegas[i] * macros.RPM
        RWJoints[i].setVelocity(omega_radps) 

    # Run first 10s before resetting thrDesatControl, since it cannot dump momentum at t = 0
    sim.ConfigureStopTime(macros.sec2nano(10.0))
    sim.ExecuteSimulation()

    # Reset thrDesat module after 10 seconds because momentum cannot be dumped at t = 0
    thrDesatControl.Reset(macros.sec2nano(10.0))

    sim.ConfigureStopTime(simulationTime)
    sim.ExecuteSimulation()

    # -------------------------------------------------------------------------
    # 9) Post processing and plotting
    # -------------------------------------------------------------------------
    dataSigmaBR = attErrorLog.sigma_BR
    dataOmegaBR = attErrorLog.omega_BR_B
    # Column-stack per-wheel speed logs into a single (numSamples x numRWs) array
    dataOmegaRW = np.column_stack([np.squeeze(rwSpeedLogs[i].state) for i in range(numRWs)])
    dataDH = deltaHLog.torqueRequestBody
    dataMap = thrMapLog.thrForce
    dataOnTime = onTimeLog.OnTimeRequest

    # Gather delivered thruster force logs into list of arrays, one per thruster
    dataThr = []
    for i in range(numTHRs):
        dataThr.append(thrForceLogs[i].input)

    np.set_printoptions(precision=16)

    timeData = rwMotorLog.times() * macros.NANO2SEC

    # Generating plots
    plt.close("all")
    figureList = {}
    figureList[fileName + "_attError"] = plot_attitude_error(timeData, dataSigmaBR)
    figureList[fileName + "_rateError"] = plot_rate_error(timeData, dataOmegaBR)
    figureList[fileName + "_rwMomenta"] = plot_rw_momenta(timeData, dataOmegaRW, RWs, numRWs)
    figureList[fileName + "_DH"] = plot_DH(timeData, dataDH)
    figureList[fileName + "_rwSpeeds"] = plot_rw_speeds(timeData, dataOmegaRW, numRWs)
    figureList[fileName + "_thrImpulse"] = plot_thrImpulse(timeData, dataMap, numTHRs)
    figureList[fileName + "_OnTimeReq"] = plot_OnTimeRequest(timeData, dataOnTime, numTHRs)
    figureList[fileName + "_thrForce"] = plot_thrForce(timeData, dataThr, numTHRs)

    if showPlots:
        plt.show()

    return figureList


# -------------------------------------------------------------------------
# RWTorqueDistributor: custom sys model to fan a single array motor-torque message out into
#   one single-actuator message per reaction wheel
#   INPUTS:
#       - numRW: number of reaction wheels to distribute torque commands to
# -------------------------------------------------------------------------
class RWTorqueDistributor(sysModel.SysModel):
    def __init__(self, numRW):
        super().__init__()
        self.ModelTag = "rwTorqueDistributor"
        self.numRW = numRW
        self.rwMotorTorqueInMsg = messaging.ArrayMotorTorqueMsgReader()  # combined torque command
        self.torqueOutMsgs = [messaging.SingleActuatorMsg() for _ in range(numRW)]  # per-wheel outputs

    def UpdateState(self, CurrentSimNanos):
        """Splits the array torque command into individual per-wheel actuator messages"""
        if self.rwMotorTorqueInMsg.isLinked():
            payload = self.rwMotorTorqueInMsg()
            for i in range(self.numRW):
                out = messaging.SingleActuatorMsgPayload()
                out.input = payload.motorTorque[i]
                self.torqueOutMsgs[i].write(out, CurrentSimNanos, self.moduleID)


# -------------------------------------------------------------------------
# RWSpeedCombiner: custom sys model to combine per-joint MuJoCo wheel speeds into a single
#   RWSpeedMsg expected by the FSW momentum-management chain
#   INPUTS:
#       - joints: list of MuJoCo scalar joints, one per reaction wheel
# -------------------------------------------------------------------------
class RWSpeedCombiner(sysModel.SysModel):
    def __init__(self, joints):
        super().__init__()
        self.ModelTag = "rwSpeedCombiner"
        self.joints = joints
        self.speedOutMsg = messaging.RWSpeedMsg()  # combined wheel speed output

    def UpdateState(self, CurrentSimNanos):
        """Reads each wheel joint's current spin rate and republishes as one combined message"""
        speeds = [joint.stateDotOutMsg.read().state for joint in self.joints]
        payload = messaging.RWSpeedMsgPayload()
        payload.wheelSpeeds = speeds
        self.speedOutMsg.write(payload, CurrentSimNanos, self.moduleID)


if __name__ == "__main__":
    # --- XML TESTING ---
    xmlString, *_ = makeMjXmlString()
    xmlPath = os.path.join(os.path.dirname(os.path.abspath(__file__)), "sat_momentumDump.xml")
    with open(xmlPath, "w") as f:
        f.write(xmlString)

    run(showPlots = True)