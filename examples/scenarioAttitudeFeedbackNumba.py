#
#  ISC License
#
#  Copyright (c) 2025, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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

Numba reimplementation of :ref:`scenarioAttitudeFeedback`.

The spacecraft dynamics and effectors stay in C++.  Every other module in the
pipeline -- navigation sensor, guidance reference, tracking-error computation,
and feedback controller -- is reimplemented as a :ref:`numbaModules`.

.. code-block:: text

    [spacecraft C++]
         |  scStateOutMsg
    [NumbaSimpleNav]          (full Gauss-Markov noise model, optional sun pointing)
         |  attOutMsg / transOutMsg
    [NumbaAttTrackingError] <-- [NumbaInertial3D]  attRefOutMsg
         |  attGuidOutMsg
    [NumbaMrpFeedback]        (full RW support, controlLawType flag)
         |  cmdTorqueOutMsg
    [extForceTorque C++]

The Numba modules are 1-to-1 in capability with the corresponding C/C++ modules:

* **NumbaSimpleNav** matches :ref:`simpleNav`: full 18-state Gauss-Markov error
  model (position, velocity, attitude, rate, sun-pointing, accumulated-DV),
  ``crossTrans``/``crossAtt`` coupling flags, optional sun-direction output.
* **NumbaInertial3D** matches :ref:`inertial3D`: fixed inertial attitude reference.
* **NumbaAttTrackingError** matches :ref:`attTrackingError`: MRP tracking error,
  frame-offset correction.
* **NumbaMrpFeedback** matches :ref:`mrpFeedback`: PD/PID feedback with reaction-wheel
  angular-momentum augmentation and ``controlLawType`` flag.

The script is found in the folder ``basilisk/examples`` and executed by using::

    python3 scenarioAttitudeFeedbackNumba.py

"""

import os
import time

import matplotlib.pyplot as plt
import numpy as np
from Basilisk import __path__
from Basilisk.architecture import messaging
from Basilisk.architecture.numbaModel import NumbaModel
from Basilisk.simulation import extForceTorque, spacecraft
from Basilisk.utilities import (SimulationBaseClass, macros, orbitalMotion,
                                 simIncludeGravBody, unitTestSupport, vizSupport)
from Basilisk.utilities.RigidBodyKinematicsNumba import MRP2C, addMRP, subMRP

bskPath = __path__[0]
fileName = os.path.basename(os.path.splitext(__file__)[0])


# =============================================================================
# Numba modules
# =============================================================================

class NumbaSimpleNav(NumbaModel):
    """Navigation sensor with full 18-state Gauss-Markov error model.

    Matches :ref:`simpleNav` in capability:

    * 18 error states: position [0:3], velocity [3:6], attitude [6:9],
      body-rate [9:12], sun-pointing [12:15], accumulated-DV [15:18].
    * ``memory.PMatrix`` (18x18 ndarray): process-noise std-deviation matrix.
    * ``memory.walkBounds`` (18-vector): random-walk bounds per state.
    * ``memory.crossTrans`` / ``memory.crossAtt`` (int32): coupling flags.
    * ``self.RNGSeed``: reproducible noise seed (inherited from SysModel,
      default ``0x1badcad1``).  Set before ``InitializeSimulation()``.
    * Optional sun-direction output when ``sunStateInMsg`` is subscribed.

    Set all ``memory.*`` attributes *before* ``InitializeSimulation()``.
    Defaults are zero -> noise-free pass-through.
    """

    def __init__(self):
        super().__init__()
        self.scStateInMsg  = messaging.SCStatesMsgReader()
        self.sunStateInMsg = messaging.SpicePlanetStateMsgReader()
        self.attOutMsg     = messaging.NavAttMsg()
        self.transOutMsg   = messaging.NavTransMsg()

        self.memory.errors     = np.zeros(18)
        self.memory.PMatrix    = np.zeros((18, 18))
        self.memory.walkBounds = np.zeros(18)
        self.memory.crossTrans = np.int32(0)
        self.memory.crossAtt   = np.int32(0)
        self.memory.prevTime   = np.uint64(0)

    @staticmethod
    def UpdateStateImpl(scStateInMsgPayload,
                        sunStateInMsgPayload, sunStateInMsgIsLinked,
                        attOutMsgPayload, transOutMsgPayload,
                        CurrentSimNanos, memory, rng):
        # Time step
        dt = 0.0 if memory.prevTime == np.uint64(0) else \
             np.float64(CurrentSimNanos - memory.prevTime) * 1e-9

        # Gauss-Markov: new_e = A * old_e + PMatrix @ randn(18)
        old_e  = memory.errors.copy()
        new_e  = old_e.copy()
        if memory.crossTrans:
            new_e[:3]  += dt * old_e[3:6]
        if memory.crossAtt:
            new_e[6:9] += dt * old_e[9:12]
        new_e += np.dot(memory.PMatrix, rng.standardNormal(18))

        # Apply walk bounds
        for k in range(18):
            if memory.walkBounds[k] > 0.0:
                if   new_e[k] >  memory.walkBounds[k]: new_e[k] =  memory.walkBounds[k]
                elif new_e[k] < -memory.walkBounds[k]: new_e[k] = -memory.walkBounds[k]

        memory.errors[:] = new_e

        # Outputs
        timeTag = np.float64(CurrentSimNanos) * 1e-9

        transOutMsgPayload.r_BN_N[:3]     = scStateInMsgPayload.r_BN_N[:3]          + new_e[:3]
        transOutMsgPayload.v_BN_N[:3]     = scStateInMsgPayload.v_BN_N[:3]          + new_e[3:6]
        transOutMsgPayload.vehAccumDV[:3] = scStateInMsgPayload.TotalAccumDVBdy[:3] + new_e[15:18]
        transOutMsgPayload.timeTag        = timeTag

        attOutMsgPayload.sigma_BN[:3]   = addMRP(scStateInMsgPayload.sigma_BN, new_e[6:9])
        attOutMsgPayload.omega_BN_B[:3] = scStateInMsgPayload.omega_BN_B[:3] + new_e[9:12]
        attOutMsgPayload.timeTag        = timeTag

        if sunStateInMsgIsLinked:
            sc2sun = sunStateInMsgPayload.PositionVector - scStateInMsgPayload.r_BN_N
            norm   = np.linalg.norm(sc2sun)
            if norm > 0.0:
                sc2sun /= norm
            attOutMsgPayload.vehSunPntBdy[:3] = np.dot(MRP2C(new_e[12:15]),
                                                        np.dot(MRP2C(scStateInMsgPayload.sigma_BN),
                                                               sc2sun))
        else:
            attOutMsgPayload.vehSunPntBdy[:3] = 0.0

        memory.prevTime = CurrentSimNanos


class NumbaInertial3D(NumbaModel):
    """Guidance: fixed inertial attitude reference with zero rates.

    Matches :ref:`inertial3D`.
    """

    def __init__(self):
        super().__init__()
        self.attRefOutMsg     = messaging.AttRefMsg()
        self.memory.sigma_R0N = np.zeros(3)

    @staticmethod
    def UpdateStateImpl(attRefOutMsgPayload, memory):
        attRefOutMsgPayload.sigma_RN[:3]    = memory.sigma_R0N
        attRefOutMsgPayload.omega_RN_N[:3]  = 0.0
        attRefOutMsgPayload.domega_RN_N[:3] = 0.0


class NumbaAttTrackingError(NumbaModel):
    """FSW: MRP attitude tracking error from navigation and reference messages.

    Matches :ref:`attTrackingError` including the frame-offset correction
    ``memory.sigma_R0R`` (body-fixed offset from reference to control-target
    frame; default zero).
    """

    def __init__(self):
        super().__init__()
        self.attNavInMsg   = messaging.NavAttMsgReader()
        self.attRefInMsg   = messaging.AttRefMsgReader()
        self.attGuidOutMsg = messaging.AttGuidMsg()
        self.memory.sigma_R0R = np.zeros(3)

    @staticmethod
    def UpdateStateImpl(attNavInMsgPayload, attRefInMsgPayload,
                        attGuidOutMsgPayload, memory):
        sigma_RN    = addMRP(attRefInMsgPayload.sigma_RN, -memory.sigma_R0R)
        sigma_BR    = subMRP(attNavInMsgPayload.sigma_BN, sigma_RN)
        C_BN        = MRP2C(attNavInMsgPayload.sigma_BN)
        omega_RN_B  = np.dot(C_BN, attRefInMsgPayload.omega_RN_N)
        domega_RN_B = np.dot(C_BN, attRefInMsgPayload.domega_RN_N)

        attGuidOutMsgPayload.sigma_BR[:3]    = sigma_BR
        attGuidOutMsgPayload.omega_BR_B[:3]  = attNavInMsgPayload.omega_BN_B[:3] - omega_RN_B
        attGuidOutMsgPayload.omega_RN_B[:3]  = omega_RN_B
        attGuidOutMsgPayload.domega_RN_B[:3] = domega_RN_B


class NumbaMrpFeedback(NumbaModel):
    """FSW: MRP PD/PID feedback controller with full reaction-wheel support.

    Matches :ref:`mrpFeedback` including:

    * Reaction-wheel angular-momentum augmentation (``rwParamsInMsg``,
      ``rwSpeedsInMsg``, optional ``rwAvailInMsg``).
    * ``memory.controlLawType``: 0 = reference-frame gyroscopic (default),
      else full-body.
    * Integral feedback (``memory.Ki > 0``) with ``memory.integralLimit`` bound.
    * Feed-forward torque via ``memory.knownTorquePntB_B``.

    Memory parameters (set directly before ``InitializeSimulation()``):
        K                 proportional gain on MRP error
        P                 rate-error feedback gain [N m s]
        Ki                integral gain; negative -> disabled
        integralLimit     wind-up bound [N m]
        knownTorquePntB_B feed-forward torque [N m, 3-vector]
        controlLawType    0 = reference-frame gyroscopic; else full-body
    """

    def __init__(self):
        super().__init__()
        self.guidInMsg       = messaging.AttGuidMsgReader()
        self.vehConfigInMsg  = messaging.VehicleConfigMsgReader()
        self.rwSpeedsInMsg   = messaging.RWSpeedMsgReader()
        self.rwAvailInMsg    = messaging.RWAvailabilityMsgReader()
        self.rwParamsInMsg   = messaging.RWArrayConfigMsgReader()  # Python-only (Reset)
        self.cmdTorqueOutMsg = messaging.CmdTorqueBodyMsg()

        self.memory.K                  = 3.5
        self.memory.P                  = 30.0
        self.memory.Ki                 = -1.0
        self.memory.integralLimit      = 0.0
        self.memory.int_sigma          = np.zeros(3)
        self.memory.knownTorquePntB_B  = np.zeros(3)
        self.memory.priorTime          = np.uint64(0)
        self.memory.controlLawType     = np.int32(0)
        self.memory.numRW              = np.int32(0)
        self.memory.GsMatrix_B         = np.zeros(108)  # up to 36 wheels x 3
        self.memory.JsList             = np.zeros(36)

    def Reset(self, CurrentSimNanos=0):
        super().Reset(CurrentSimNanos)
        self.memory.int_sigma[:]  = 0.0
        self.memory.priorTime     = np.uint64(0)
        self.memory.numRW         = np.int32(0)
        if self.rwParamsInMsg.isLinked():
            cfg = self.rwParamsInMsg()
            self.memory.numRW      = np.int32(cfg.numRW)
            self.memory.GsMatrix_B = np.array(cfg.GsMatrix_B, dtype=np.float64)
            self.memory.JsList     = np.array(cfg.JsList,      dtype=np.float64)

    @staticmethod
    def UpdateStateImpl(guidInMsgPayload, vehConfigInMsgPayload,
                        rwSpeedsInMsgPayload, rwSpeedsInMsgIsLinked,
                        rwAvailInMsgPayload,  rwAvailInMsgIsLinked,
                        cmdTorqueOutMsgPayload, memory, CurrentSimNanos):
        dt = 0.0 if memory.priorTime == np.uint64(0) else \
             np.float64(CurrentSimNanos - memory.priorTime) * 1e-9
        memory.priorTime = CurrentSimNanos

        sigma_BR    = guidInMsgPayload.sigma_BR
        omega_BR_B  = guidInMsgPayload.omega_BR_B
        omega_RN_B  = guidInMsgPayload.omega_RN_B
        domega_RN_B = guidInMsgPayload.domega_RN_B
        omega_BN_B  = omega_BR_B + omega_RN_B

        I = vehConfigInMsgPayload.ISCPntB_B.reshape(3, 3)

        # Integral term
        z = np.zeros(3)
        if memory.Ki > 0.0:
            memory.int_sigma += memory.K * dt * sigma_BR
            memory.int_sigma[:] = np.clip(memory.int_sigma,
                                          -memory.integralLimit, memory.integralLimit)
            z = memory.int_sigma + np.dot(I, omega_BR_B)

        v3_4 = memory.Ki * z

        # Base torque: K*sigma + P*(omega_BR + Ki*z)
        Lr = memory.K * sigma_BR + memory.P * (omega_BR_B + v3_4)

        # Angular momentum: h = I*omega_BN + sum_i Js_i*(omega_BN.Gs_i + Omega_i)*Gs_i
        h = np.dot(I, omega_BN_B)
        if memory.numRW > 0 and rwSpeedsInMsgIsLinked:
            for i in range(memory.numRW):
                if (not rwAvailInMsgIsLinked) or (rwAvailInMsgPayload.wheelAvailability[i] == 0):
                    Gs_i = memory.GsMatrix_B[i*3 : i*3 + 3]
                    h   += memory.JsList[i] * (np.dot(omega_BN_B, Gs_i)
                                               + rwSpeedsInMsgPayload.wheelSpeeds[i]) * Gs_i

        # Gyroscopic + dynamical terms
        v3_8 = omega_RN_B + v3_4 if memory.controlLawType == np.int32(0) else omega_BN_B
        Lr  -= np.cross(v3_8, h)
        Lr  += np.dot(I, np.cross(omega_BN_B, omega_RN_B) - domega_RN_B)
        Lr  += memory.knownTorquePntB_B

        cmdTorqueOutMsgPayload.torqueRequestBody[:3] = -Lr


# =============================================================================
# Scenario
# =============================================================================

def run(show_plots, useUnmodeledTorque, useIntGain, useKnownTorque):
    """
    Args:
        show_plots (bool): display plots when True
        useUnmodeledTorque (bool): apply a constant external disturbance torque
        useIntGain (bool): enable integral feedback in the controller
        useKnownTorque (bool): feed-forward the known external torque
    """
    simTaskName    = "simTask"
    simProcessName = "simProcess"

    scSim = SimulationBaseClass.SimBaseClass()

    simulationTime     = macros.min2nano(10.)
    simulationTimeStep = macros.sec2nano(.1)

    dynProcess = scSim.CreateNewProcess(simProcessName)
    dynProcess.addTask(scSim.CreateNewTask(simTaskName, simulationTimeStep))

    # -------------------------------------------------------------------------
    # Spacecraft dynamics (C++)
    # -------------------------------------------------------------------------
    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "spacecraftBody"
    I = [900., 0., 0.,
         0.,  800., 0.,
         0.,  0.,  600.]
    scObject.hub.mHub        = 750.0
    scObject.hub.r_BcB_B     = [[0.0], [0.0], [0.0]]
    scObject.hub.IHubPntBc_B = unitTestSupport.np2EigenMatrix3d(I)
    scSim.AddModelToTask(simTaskName, scObject)

    gravFactory = simIncludeGravBody.gravBodyFactory()
    earth       = gravFactory.createEarth()
    earth.isCentralBody = True
    mu = earth.mu
    gravFactory.addBodiesTo(scObject)

    extFTObject          = extForceTorque.ExtForceTorque()
    extFTObject.ModelTag = "externalDisturbance"
    if useUnmodeledTorque:
        extFTObject.extTorquePntB_B = [[0.25], [-0.25], [0.1]]
    scObject.addDynamicEffector(extFTObject)
    scSim.AddModelToTask(simTaskName, extFTObject)

    # -------------------------------------------------------------------------
    # Numba navigation (noise-free by default)
    # -------------------------------------------------------------------------
    sNav          = NumbaSimpleNav()
    sNav.ModelTag = "simpleNav"
    # To enable noise:  sNav.memory.PMatrix = <18x18 std-dev matrix>
    #                   sNav.memory.walkBounds = <18-vector>
    scSim.AddModelToTask(simTaskName, sNav)

    # -------------------------------------------------------------------------
    # Numba FSW
    # -------------------------------------------------------------------------
    inertial3DObj          = NumbaInertial3D()
    inertial3DObj.ModelTag = "inertial3D"
    scSim.AddModelToTask(simTaskName, inertial3DObj)

    attError          = NumbaAttTrackingError()
    attError.ModelTag = "attTrackingError"
    scSim.AddModelToTask(simTaskName, attError)

    mrpCtrl          = NumbaMrpFeedback()
    mrpCtrl.ModelTag = "mrpFeedback"
    mrpCtrl.memory.K = 3.5
    mrpCtrl.memory.P = 30.0
    if useIntGain:
        mrpCtrl.memory.Ki            = 0.0002
        mrpCtrl.memory.integralLimit = 2.0 / mrpCtrl.memory.Ki * 0.1
    if useKnownTorque:
        mrpCtrl.memory.knownTorquePntB_B[:] = [0.25, -0.25, 0.1]
    scSim.AddModelToTask(simTaskName, mrpCtrl)

    # -------------------------------------------------------------------------
    # Logging
    # -------------------------------------------------------------------------
    numDataPoints = 50
    samplingTime  = unitTestSupport.samplingTime(simulationTime, simulationTimeStep,
                                                  numDataPoints)
    attGuidLog = attError.attGuidOutMsg.recorder(samplingTime)
    torqueLog  = mrpCtrl.cmdTorqueOutMsg.recorder(samplingTime)
    scSim.AddModelToTask(simTaskName, attGuidLog)
    scSim.AddModelToTask(simTaskName, torqueLog)

    # -------------------------------------------------------------------------
    # Orbital initial conditions
    # -------------------------------------------------------------------------
    oe       = orbitalMotion.ClassicElements()
    oe.a     = 10000e3
    oe.e     = 0.01
    oe.i     = 33.3 * macros.D2R
    oe.Omega = 48.2 * macros.D2R
    oe.omega = 347.8 * macros.D2R
    oe.f     = 85.3 * macros.D2R
    r, v     = orbitalMotion.elem2rv(mu, oe)
    scObject.hub.r_CN_NInit     = r
    scObject.hub.v_CN_NInit     = v
    scObject.hub.sigma_BNInit   = [[0.1], [0.2], [-0.3]]
    scObject.hub.omega_BN_BInit = [[0.001], [-0.01], [0.03]]

    # -------------------------------------------------------------------------
    # Message connections
    # -------------------------------------------------------------------------
    sNav.scStateInMsg.subscribeTo(scObject.scStateOutMsg)
    attError.attNavInMsg.subscribeTo(sNav.attOutMsg)
    attError.attRefInMsg.subscribeTo(inertial3DObj.attRefOutMsg)
    mrpCtrl.guidInMsg.subscribeTo(attError.attGuidOutMsg)
    extFTObject.cmdTorqueInMsg.subscribeTo(mrpCtrl.cmdTorqueOutMsg)

    vehicleConfigOut = messaging.VehicleConfigMsgPayload(ISCPntB_B=I)
    configDataMsg    = messaging.VehicleConfigMsg().write(vehicleConfigOut)
    mrpCtrl.vehConfigInMsg.subscribeTo(configDataMsg)

    vizSupport.enableUnityVisualization(scSim, simTaskName, scObject
                                        # , saveFile=fileName
                                        )

    # -------------------------------------------------------------------------
    # Run
    # -------------------------------------------------------------------------
    scSim.InitializeSimulation()
    scSim.ConfigureStopTime(simulationTime)

    tic = time.time()
    scSim.ExecuteSimulation()
    toc = time.time()
    print(f"Run scenario in {toc - tic:.3f} s")

    # -------------------------------------------------------------------------
    # Plots
    # -------------------------------------------------------------------------
    dataLr      = torqueLog.torqueRequestBody
    dataSigmaBR = attGuidLog.sigma_BR
    timeAxis    = attGuidLog.times()

    plt.close("all")
    plt.figure(1)
    for idx in range(3):
        plt.plot(timeAxis * macros.NANO2MIN, dataSigmaBR[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label=r'$\sigma_' + str(idx) + '$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel(r'Attitude Error $\sigma_{B/R}$')
    figureList = {}
    pltName = fileName + "1" + str(int(useUnmodeledTorque)) + str(int(useIntGain)) + str(int(useKnownTorque))
    figureList[pltName] = plt.figure(1)

    plt.figure(2)
    for idx in range(3):
        plt.plot(timeAxis * macros.NANO2MIN, dataLr[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label='$L_{r,' + str(idx) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('Control Torque $L_r$ [Nm]')
    pltName = fileName + "2" + str(int(useUnmodeledTorque)) + str(int(useIntGain)) + str(int(useKnownTorque))
    figureList[pltName] = plt.figure(2)

    if show_plots:
        plt.show()
    plt.close("all")

    return figureList


if __name__ == "__main__":
    run(
        True,   # show_plots
        False,  # useUnmodeledTorque
        False,  # useIntGain
        False,  # useKnownTorque
    )
