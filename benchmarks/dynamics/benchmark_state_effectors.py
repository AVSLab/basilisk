#!/usr/bin/env python3
#
# ISC License
#
# Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
#
# Permission to use, copy, modify, and/or distribute this software for any
# purpose with or without fee is hereby granted, provided that the above
# copyright notice and this permission notice appear in all copies.
#
# THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
# WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
# ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
# WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
# ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
# OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
#

r"""Benchmark selected Basilisk spacecraft dynamics effectors.

This optional developer benchmark runs compact spacecraft simulations that isolate
selected dynamics effectors.  Each trial builds a fresh
``spacecraft.Spacecraft`` object, attaches the requested effector load, disables
recorders and plotting, runs the default RK4 integration loop, and reports
wall-clock timing.

The benchmark is intended for local before/after comparisons while optimizing
state effectors, dynamic effectors, or the spacecraft integration path.  It is not
a numerical validation test and it is not run by CI.

Running the Benchmark
---------------------

Run the benchmark from the Basilisk repository root directory after building the
Basilisk Python package:

.. code-block:: bash

   cd /path/to/basilisk
   .venv/bin/python benchmarks/dynamics/benchmark_state_effectors.py --case all

Run a single case with a longer measurement:

.. code-block:: bash

   .venv/bin/python benchmarks/dynamics/benchmark_state_effectors.py \
      --case SpinningBodyTwoDOFStateEffector --steps 10000 --trials 7

Use ``--components`` to scale the number of repeated devices in cases that
support repetition.  Use ``--segments`` to scale the per-effector body count in
the NDOF translator and hinged-body cases:

.. code-block:: bash

   .venv/bin/python benchmarks/dynamics/benchmark_state_effectors.py \
      --case LinearTranslationNDOFStateEffector --components 2 --segments 8

Supported Cases
---------------

The ``--case`` option accepts either the canonical benchmark names or the
corresponding effector class-style aliases:

- ``spacecraft``
- ``dual-hinged-rigid-body``
- ``gravity-gradient``
- ``hinged-rigid-body``
- ``linear-spring-mass-damper``
- ``linear-translation-ndof``
- ``linear-translation-one-dof``
- ``n-hinged-rigid-body``
- ``prescribed-motion``
- ``reaction-wheel``
- ``spherical-pendulum``
- ``spinning-body-ndof``
- ``spinning-body-one-dof``
- ``spinning-body-two-dof``
- ``vscmg``

Expected Terminal Output
------------------------

The exact timings will vary by machine.  An abridged representative all-case run
looks like:

.. code-block:: text

   Dynamics effector benchmark
   Cases: all
   Steps per trial: 2000
   Time step: 0.010000 s
   Simulated time per trial: 20.000 s
   Trials: 5
   Warmup steps: 200
   Components per case: 4
   Segments per NDOF case: 4
   Gravity: disabled except gravity-gradient case
   Recorders: disabled
   Spacecraft integrator: RK4 default

   Case                                      median [s]   median us/step      min [s]    min us/step
   spacecraft                                  0.012345            6.173     0.012120          6.060
   hinged-rigid-body                           0.045678           22.839     0.045100         22.550

Interpreting the Columns
------------------------

``median [s]``
   Median wall-clock time across measured trials for one
   ``ExecuteSimulation()`` call.

``median us/step``
   Median wall-clock microseconds per integration step.

``min [s]``
   Fastest measured trial wall-clock time.

``min us/step``
   Fastest measured trial wall-clock microseconds per integration step.
"""

import argparse
from dataclasses import dataclass
import math
import os
import statistics
import tempfile
import time
from typing import Callable

import numpy as np


def _configure_headless_environment():
    """Set plotting-related environment defaults before Basilisk imports."""

    os.environ.setdefault("MPLBACKEND", "Agg")
    if "MPLCONFIGDIR" not in os.environ:
        mplConfigDirectory = os.path.join(tempfile.gettempdir(), "basilisk-mplconfig")
        os.makedirs(mplConfigDirectory, exist_ok=True)
        os.environ["MPLCONFIGDIR"] = mplConfigDirectory


_configure_headless_environment()

from Basilisk.architecture import messaging
from Basilisk.simulation import GravityGradientEffector
from Basilisk.simulation import dualHingedRigidBodyStateEffector
from Basilisk.simulation import gravityEffector
from Basilisk.simulation import hingedRigidBodyStateEffector
from Basilisk.simulation import linearSpringMassDamper
from Basilisk.simulation import linearTranslationNDOFStateEffector
from Basilisk.simulation import linearTranslationOneDOFStateEffector
from Basilisk.simulation import nHingedRigidBodyStateEffector
from Basilisk.simulation import prescribedMotionStateEffector
from Basilisk.simulation import reactionWheelStateEffector
from Basilisk.simulation import spacecraft
from Basilisk.simulation import sphericalPendulum
from Basilisk.simulation import spinningBodyNDOFStateEffector
from Basilisk.simulation import spinningBodyOneDOFStateEffector
from Basilisk.simulation import spinningBodyTwoDOFStateEffector
from Basilisk.simulation import vscmgStateEffector
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros
from Basilisk.utilities import simIncludeRW


@dataclass(frozen=True)
class BenchmarkConfig:
    """Container for dynamics effector benchmark timing controls."""

    selectedCases: tuple[str, ...]
    numberOfSteps: int = 2000
    numberOfTrials: int = 5
    numberOfWarmupSteps: int = 200
    timeStepSec: float = 0.01  # [s]
    numberOfComponents: int = 4
    numberOfSegments: int = 4


@dataclass
class BenchmarkContext:
    """Simulation objects shared by one benchmark case setup."""

    scSim: SimulationBaseClass.SimBaseClass
    scObject: spacecraft.Spacecraft
    taskName: str
    config: BenchmarkConfig
    keepAlive: list


@dataclass(frozen=True)
class CaseDefinition:
    """One benchmark case description and setup callback."""

    displayName: str
    setupFunction: Callable[[BenchmarkContext], None]


def _positive_int(value):
    """Return ``value`` as a positive integer for ``argparse``."""

    parsedValue = int(value)
    if parsedValue < 1:
        raise argparse.ArgumentTypeError("value must be a positive integer")
    return parsedValue


def _nonnegative_int(value):
    """Return ``value`` as a nonnegative integer for ``argparse``."""

    parsedValue = int(value)
    if parsedValue < 0:
        raise argparse.ArgumentTypeError("value must be nonnegative")
    return parsedValue


def _positive_float(value):
    """Return ``value`` as a positive floating-point value for ``argparse``."""

    parsedValue = float(value)
    if parsedValue <= 0.0:
        raise argparse.ArgumentTypeError("value must be positive")
    return parsedValue


def _column(vector):
    """Return ``vector`` as a Basilisk column vector list."""

    return [[value] for value in vector]


def _unit_vector(vector):
    """Return ``vector`` normalized as a Basilisk column vector list."""

    norm = math.sqrt(sum(value * value for value in vector))
    return [[value / norm] for value in vector]


def _rotation_identity():
    """Return a 3-by-3 identity direction cosine matrix."""

    return [[1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0]]


def _set_hub_properties(scObject, useOrbitState=False):
    """Configure the spacecraft hub mass properties and initial state."""

    zeroLengthM = 0.0  # [m]
    zeroSpeedMps = 0.0  # [m/s]
    zeroInertiaKgM2 = 0.0  # [kg*m^2]

    scObject.hub.mHub = 750.0  # [kg]
    scObject.hub.r_BcB_B = [[zeroLengthM], [zeroLengthM], [zeroLengthM]]
    scObject.hub.IHubPntBc_B = [
        [900.0, zeroInertiaKgM2, zeroInertiaKgM2],
        [zeroInertiaKgM2, 800.0, zeroInertiaKgM2],
        [zeroInertiaKgM2, zeroInertiaKgM2, 600.0],
    ]  # [kg*m^2]

    if useOrbitState:
        scObject.hub.r_CN_NInit = [[7000.0e3], [zeroLengthM], [zeroLengthM]]  # [m]
        scObject.hub.v_CN_NInit = [[zeroSpeedMps], [7500.0], [100.0]]  # [m/s]
    else:
        scObject.hub.r_CN_NInit = [[0.1], [-0.2], [0.3]]  # [m]
        scObject.hub.v_CN_NInit = [[0.02], [-0.01], [0.03]]  # [m/s]

    scObject.hub.sigma_BNInit = [[0.01], [-0.02], [0.03]]
    scObject.hub.omega_BN_BInit = [[0.08], [0.01], [-0.03]]  # [rad/s]


def _add_task_model(context, model):
    """Keep ``model`` alive and add it to the benchmark task."""

    context.keepAlive.append(model)
    context.scSim.AddModelToTask(context.taskName, model)


def _add_state_effector(context, effector):
    """Attach ``effector`` to the spacecraft and benchmark task."""

    context.scObject.addStateEffector(effector)
    _add_task_model(context, effector)


def _add_dynamic_effector(context, effector):
    """Attach ``effector`` to the spacecraft and benchmark task."""

    context.scObject.addDynamicEffector(effector)
    _add_task_model(context, effector)


def _setup_spacecraft_only(context):
    """Run the baseline spacecraft dynamics case without extra effectors."""


def _setup_gravity_gradient(context):
    """Attach gravity-gradient dynamic effectors."""

    _set_hub_properties(context.scObject, useOrbitState=True)

    earthGravBody = gravityEffector.GravBodyData()
    earthGravBody.planetName = "earth_planet_data"
    earthGravBody.mu = 0.3986004415e15  # [m^3/s^2]
    earthGravBody.isCentralBody = True
    context.keepAlive.append(earthGravBody)
    context.scObject.gravField.gravBodies = spacecraft.GravBodyVector([earthGravBody])

    for effectorIndex in range(context.config.numberOfComponents):
        ggEffector = GravityGradientEffector.GravityGradientEffector()
        ggEffector.ModelTag = f"gravityGradient{effectorIndex}"
        ggEffector.addPlanetName(earthGravBody.planetName)
        _add_dynamic_effector(context, ggEffector)


def _make_dual_hinged_body(effectorIndex):
    """Create one dual-hinged rigid-body state effector."""

    sideSign = -1.0 if effectorIndex % 2 else 1.0
    panel = dualHingedRigidBodyStateEffector.DualHingedRigidBodyStateEffector()
    panel.ModelTag = f"dualHingedRigidBody{effectorIndex}"
    panel.mass1 = 50.0  # [kg]
    panel.IPntS1_S1 = [[50.0, 0.0, 0.0], [0.0, 25.0, 0.0], [0.0, 0.0, 25.0]]  # [kg*m^2]
    panel.d1 = 0.75  # [m]
    panel.l1 = 1.5  # [m]
    panel.k1 = 100.0  # [N*m/rad]
    panel.c1 = 0.02  # [N*m*s/rad]
    panel.r_H1B_B = [[sideSign * 0.5], [0.0], [1.0]]  # [m]
    panel.dcm_H1B = [[sideSign, 0.0, 0.0], [0.0, sideSign, 0.0], [0.0, 0.0, 1.0]]
    panel.mass2 = 50.0  # [kg]
    panel.IPntS2_S2 = [[50.0, 0.0, 0.0], [0.0, 25.0, 0.0], [0.0, 0.0, 25.0]]  # [kg*m^2]
    panel.d2 = 0.75  # [m]
    panel.k2 = 100.0  # [N*m/rad]
    panel.c2 = 0.02  # [N*m*s/rad]
    panel.theta1Init = sideSign * 5.0 * macros.D2R  # [rad]
    panel.theta1DotInit = 0.02  # [rad/s]
    panel.theta2Init = -sideSign * 3.0 * macros.D2R  # [rad]
    panel.theta2DotInit = -0.01  # [rad/s]
    return panel


def _setup_dual_hinged_rigid_body(context):
    """Attach repeated dual-hinged rigid-body state effectors."""

    for effectorIndex in range(context.config.numberOfComponents):
        _add_state_effector(context, _make_dual_hinged_body(effectorIndex))


def _make_hinged_body(effectorIndex):
    """Create one hinged rigid-body state effector."""

    sideSign = -1.0 if effectorIndex % 2 else 1.0
    panel = hingedRigidBodyStateEffector.HingedRigidBodyStateEffector()
    panel.ModelTag = f"hingedRigidBody{effectorIndex}"
    panel.mass = 100.0  # [kg]
    panel.IPntS_S = [[100.0, 0.0, 0.0], [0.0, 50.0, 0.0], [0.0, 0.0, 50.0]]  # [kg*m^2]
    panel.d = 1.5  # [m]
    panel.k = 25.0  # [N*m/rad]
    panel.c = 0.02  # [N*m*s/rad]
    panel.r_HB_B = [[sideSign * 0.5], [0.0], [1.0]]  # [m]
    panel.dcm_HB = [[sideSign, 0.0, 0.0], [0.0, sideSign, 0.0], [0.0, 0.0, 1.0]]
    panel.thetaInit = sideSign * 4.0 * macros.D2R  # [rad]
    panel.thetaDotInit = 0.02  # [rad/s]
    return panel


def _setup_hinged_rigid_body(context):
    """Attach repeated hinged rigid-body state effectors."""

    for effectorIndex in range(context.config.numberOfComponents):
        _add_state_effector(context, _make_hinged_body(effectorIndex))


def _setup_linear_spring_mass_damper(context):
    """Attach repeated linear spring mass damper state effectors."""

    axes = (
        _unit_vector((1.0, 1.0, 1.0)),
        _unit_vector((1.0, -1.0, -1.0)),
        _unit_vector((-1.0, -1.0, 1.0)),
        _unit_vector((-1.0, 1.0, -1.0)),
    )

    for effectorIndex in range(context.config.numberOfComponents):
        sign = -1.0 if effectorIndex % 2 else 1.0
        particle = linearSpringMassDamper.LinearSpringMassDamper()
        particle.ModelTag = f"linearSpringMassDamper{effectorIndex}"
        particle.k = 100.0  # [N/m]
        particle.c = 0.5  # [N*s/m]
        particle.r_PB_B = [[sign * 0.1], [0.05 * effectorIndex], [-sign * 0.1]]  # [m]
        particle.pHat_B = axes[effectorIndex % len(axes)]
        particle.rhoInit = sign * (0.02 + 0.005 * effectorIndex)  # [m]
        particle.rhoDotInit = 0.01  # [m/s]
        particle.massInit = 10.0 + effectorIndex  # [kg]
        _add_state_effector(context, particle)


def _make_translating_body_one_dof(effectorIndex):
    """Create one linear translation one-DOF state effector."""

    sign = -1.0 if effectorIndex % 2 else 1.0
    translatingBody = linearTranslationOneDOFStateEffector.LinearTranslationOneDOFStateEffector()
    translatingBody.ModelTag = f"linearTranslationOneDOF{effectorIndex}"
    translatingBody.setMass(20.0 + effectorIndex)  # [kg]
    translatingBody.setK(100.0)  # [N/m]
    translatingBody.setC(0.5)  # [N*s/m]
    translatingBody.setRhoInit(sign * 0.5)  # [m]
    translatingBody.setRhoDotInit(0.05)  # [m/s]
    translatingBody.setFHat_B(_unit_vector((3.0, 4.0, 0.0)))
    translatingBody.setR_FcF_F([[-1.0], [1.0], [0.0]])  # [m]
    translatingBody.setR_F0B_B([[sign * -1.0], [0.5 * effectorIndex], [0.3]])  # [m]
    translatingBody.setIPntFc_F([[50.0, 0.0, 0.0],
                                 [0.0, 80.0, 0.0],
                                 [0.0, 0.0, 60.0]])  # [kg*m^2]
    translatingBody.setDCM_FB([[0.0, -1.0, 0.0],
                               [0.0, 0.0, -1.0],
                               [1.0, 0.0, 0.0]])
    return translatingBody


def _setup_linear_translation_one_dof(context):
    """Attach repeated linear translation one-DOF state effectors."""

    for effectorIndex in range(context.config.numberOfComponents):
        _add_state_effector(context, _make_translating_body_one_dof(effectorIndex))


def _make_ndof_translating_body(bodyIndex):
    """Create one body for a linear translation NDOF state effector."""

    sign = -1.0 if bodyIndex % 2 else 1.0
    body = linearTranslationNDOFStateEffector.TranslatingBody()
    body.setMass(8.0 + bodyIndex)  # [kg]
    body.setIPntFc_F([[12.0 + bodyIndex, 0.0, 0.0],
                      [0.0, 14.0 + bodyIndex, 0.0],
                      [0.0, 0.0, 16.0 + bodyIndex]])  # [kg*m^2]
    body.setDCM_FP(_rotation_identity())
    body.setR_FcF_F([[0.05 * sign], [0.02 * bodyIndex], [0.03]])  # [m]
    body.setR_F0P_P([[0.15 * sign], [0.04 * bodyIndex], [0.02]])  # [m]
    body.setFHat_P(_unit_vector((3.0, 4.0, 0.0)) if bodyIndex % 3 else _column((0.0, 0.0, 1.0)))
    body.setRhoInit(sign * (0.2 + 0.02 * bodyIndex))  # [m]
    body.setRhoDotInit(0.02)  # [m/s]
    body.setK(20.0 + bodyIndex)  # [N/m]
    body.setC(0.2)  # [N*s/m]
    return body


def _setup_linear_translation_ndof(context):
    """Attach linear translation NDOF state effectors."""

    for effectorIndex in range(context.config.numberOfComponents):
        effector = linearTranslationNDOFStateEffector.LinearTranslationNDOFStateEffector()
        effector.ModelTag = f"linearTranslationNDOF{effectorIndex}"
        for bodyIndex in range(context.config.numberOfSegments):
            effector.addTranslatingBody(_make_ndof_translating_body(bodyIndex))
        _add_state_effector(context, effector)


def _make_hinged_panel(panelIndex):
    """Create one panel for an N-hinged rigid-body state effector."""

    sign = -1.0 if panelIndex % 2 else 1.0
    panel = nHingedRigidBodyStateEffector.HingedPanel()
    panel.mass = 50.0  # [kg]
    panel.IPntS_S = [[50.0, 0.0, 0.0], [0.0, 25.0, 0.0], [0.0, 0.0, 25.0]]  # [kg*m^2]
    panel.d = 0.75  # [m]
    panel.k = 100.0  # [N*m/rad]
    panel.c = 0.02  # [N*m*s/rad]
    panel.thetaInit = sign * (2.0 + panelIndex) * macros.D2R  # [rad]
    panel.thetaDotInit = 0.01 * sign  # [rad/s]
    panel.theta_0 = 0.0  # [rad]
    return panel


def _setup_n_hinged_rigid_body(context):
    """Attach N-hinged rigid-body state effectors."""

    for effectorIndex in range(context.config.numberOfComponents):
        sideSign = -1.0 if effectorIndex % 2 else 1.0
        effector = nHingedRigidBodyStateEffector.NHingedRigidBodyStateEffector()
        effector.ModelTag = f"nHingedRigidBody{effectorIndex}"
        effector.r_HB_B = [[sideSign * 0.5], [0.0], [1.0]]  # [m]
        effector.dcm_HB = [[sideSign, 0.0, 0.0], [0.0, sideSign, 0.0], [0.0, 0.0, 1.0]]
        for panelIndex in range(context.config.numberOfSegments):
            effector.addHingedPanel(_make_hinged_panel(panelIndex))
        _add_state_effector(context, effector)


def _setup_prescribed_motion(context):
    """Attach prescribed-motion state effectors."""

    for effectorIndex in range(context.config.numberOfComponents):
        sign = -1.0 if effectorIndex % 2 else 1.0
        massKg = 50.0 + effectorIndex  # [kg]
        prescribedBody = prescribedMotionStateEffector.PrescribedMotionStateEffector()
        prescribedBody.ModelTag = f"prescribedMotion{effectorIndex}"
        prescribedBody.setMass(massKg)
        prescribedBody.setIPntPc_P([[5.0, 0.0, 0.0],
                                    [0.0, 4.0, 0.0],
                                    [0.0, 0.0, 3.0]])  # [kg*m^2]
        prescribedBody.setR_MB_B([sign * 0.5, 0.0, 0.0])  # [m]
        prescribedBody.setR_PcP_P([0.5, 0.0, 0.0])  # [m]
        prescribedBody.setR_PM_M([0.0, 0.0, 0.0])  # [m]
        prescribedBody.setRPrime_PM_M(np.array([0.01 * sign, 0.0, 0.0]))  # [m/s]
        prescribedBody.setRPrimePrime_PM_M(np.array([0.001 * sign, 0.0, 0.0]))  # [m/s^2]
        prescribedBody.setOmega_PM_P(np.array([0.0, 0.01 * sign, 0.0]))  # [rad/s]
        prescribedBody.setOmegaPrime_PM_P(np.array([0.0, 0.001 * sign, 0.0]))  # [rad/s^2]
        prescribedBody.setSigma_PM([0.001 * sign, 0.0, 0.0])
        prescribedBody.setSigma_MB([0.0, 0.0, 0.0])
        _add_state_effector(context, prescribedBody)


def _setup_reaction_wheel(context):
    """Attach a reaction wheel state effector with repeated wheels."""

    axes = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0),
            (1.0, 1.0, 0.0), (1.0, 0.0, 1.0), (0.0, 1.0, 1.0))
    rwFactory = simIncludeRW.rwFactory()
    maxMomentumNms = 100.0  # [N*m*s]

    for wheelIndex in range(context.config.numberOfComponents):
        axis = axes[wheelIndex % len(axes)]
        rwFactory.create(
            "Honeywell_HR16",
            [value / math.sqrt(sum(component * component for component in axis)) for value in axis],
            Omega=(500.0 - 75.0 * wheelIndex),  # [RPM]
            rWB_B=[0.05 * wheelIndex, 0.02 * (wheelIndex % 2), 0.1],  # [m]
            maxMomentum=maxMomentumNms,
            RWModel=reactionWheelStateEffector.JitterFullyCoupled,
        )

    rwEffector = reactionWheelStateEffector.ReactionWheelStateEffector()
    rwFactory.addToSpacecraft("reactionWheelBenchmark", rwEffector, context.scObject)

    cmdArray = messaging.ArrayMotorTorqueMsgPayload()
    cmdArray.motorTorque = [
        0.01 * ((-1.0) ** wheelIndex) for wheelIndex in range(context.config.numberOfComponents)
    ]  # [N*m]
    cmdMsg = messaging.ArrayMotorTorqueMsg().write(cmdArray)
    rwEffector.rwMotorCmdInMsg.subscribeTo(cmdMsg)
    context.keepAlive.append(cmdMsg)
    _add_task_model(context, rwEffector)


def _setup_spherical_pendulum(context):
    """Attach repeated spherical pendulum state effectors."""

    for effectorIndex in range(context.config.numberOfComponents):
        sign = -1.0 if effectorIndex % 2 else 1.0
        pendulum = sphericalPendulum.SphericalPendulum()
        pendulum.ModelTag = f"sphericalPendulum{effectorIndex}"
        pendulum.pendulumRadius = 0.3 + 0.02 * effectorIndex  # [m]
        pendulum.d = [[0.1 * sign], [0.1], [0.1]]  # [m]
        pendulum.D = [[0.1, 0.0, 0.0], [0.0, 0.1, 0.0], [0.0, 0.0, 0.1]]  # [N*s/m]
        pendulum.phiDotInit = 0.01 * sign  # [rad/s]
        pendulum.thetaDotInit = 0.05  # [rad/s]
        pendulum.massInit = 20.0 + effectorIndex  # [kg]
        pendulum.pHat_01 = _unit_vector((1.0, 0.0, 1.0))
        pendulum.pHat_02 = [[0.0], [1.0], [0.0]]
        pendulum.pHat_03 = _unit_vector((-1.0, 0.0, 1.0))
        _add_state_effector(context, pendulum)


def _make_spinning_body_ndof(bodyIndex):
    """Create one body for a spinning body NDOF state effector."""

    zeroLengthM = 0.0  # [m]
    zeroInertiaKgM2 = 0.0  # [kg*m^2]
    linkMassKg = 20.0  # [kg]
    linkLengthM = 1.25  # [m]
    linkRadiusM = 0.08  # [m]
    linkCenterM = linkLengthM / 2.0  # [m]
    jointStiffnessNmRad = 2.0  # [N*m/rad]
    jointDampingNmSecRad = 0.05  # [N*m*s/rad]
    initialThetaStepRad = 0.04  # [rad]
    initialThetaDotStepRadSec = 0.01  # [rad/s]

    if bodyIndex == 0:
        parentOffsetM = 1.5  # [m]
    else:
        parentOffsetM = linkLengthM

    transverseInertiaKgM2 = linkMassKg / 12.0 * (
        3.0 * linkRadiusM**2 + linkLengthM**2
    )  # [kg*m^2]
    axialInertiaKgM2 = 0.5 * linkMassKg * linkRadiusM**2  # [kg*m^2]

    sign = -1.0 if bodyIndex % 2 else 1.0
    thetaInitRad = sign * (bodyIndex + 1) * initialThetaStepRad  # [rad]
    thetaDotInitRadSec = sign * initialThetaDotStepRadSec  # [rad/s]

    spinAxes = (
        [[1.0], [0.0], [0.0]],
        [[0.0], [0.0], [1.0]],
        [[0.0], [1.0], [0.0]],
    )

    spinningBody = spinningBodyNDOFStateEffector.SpinningBody()
    spinningBody.setMass(linkMassKg)
    spinningBody.setISPntSc_S(
        [
            [transverseInertiaKgM2, zeroInertiaKgM2, zeroInertiaKgM2],
            [zeroInertiaKgM2, axialInertiaKgM2, zeroInertiaKgM2],
            [zeroInertiaKgM2, zeroInertiaKgM2, transverseInertiaKgM2],
        ]
    )
    spinningBody.setDCM_S0P(_rotation_identity())
    spinningBody.setR_ScS_S([[zeroLengthM], [linkCenterM], [zeroLengthM]])
    spinningBody.setR_SP_P([[zeroLengthM], [parentOffsetM], [zeroLengthM]])
    spinningBody.setSHat_S(spinAxes[bodyIndex % len(spinAxes)])
    spinningBody.setThetaInit(thetaInitRad)
    spinningBody.setThetaDotInit(thetaDotInitRadSec)
    spinningBody.setK(jointStiffnessNmRad)
    spinningBody.setC(jointDampingNmSecRad)

    return spinningBody


def _setup_spinning_body_ndof(context):
    """Attach spinning body NDOF state effectors."""

    for effectorIndex in range(context.config.numberOfComponents):
        effector = spinningBodyNDOFStateEffector.SpinningBodyNDOFStateEffector()
        effector.ModelTag = f"spinningBodyNDOF{effectorIndex}"
        for bodyIndex in range(context.config.numberOfSegments):
            effector.addSpinningBody(_make_spinning_body_ndof(bodyIndex))
        _add_state_effector(context, effector)


def _make_spinning_body_one_dof(effectorIndex):
    """Create one spinning body one-DOF state effector."""

    sign = -1.0 if effectorIndex % 2 else 1.0
    spinningBody = spinningBodyOneDOFStateEffector.SpinningBodyOneDOFStateEffector()
    spinningBody.ModelTag = f"spinningBodyOneDOF{effectorIndex}"
    spinningBody.mass = 50.0  # [kg]
    spinningBody.IPntSc_S = [[50.0, 0.0, 0.0], [0.0, 30.0, 0.0], [0.0, 0.0, 40.0]]  # [kg*m^2]
    spinningBody.dcm_S0B = [[0.0, -1.0, 0.0], [0.0, 0.0, -1.0], [1.0, 0.0, 0.0]]
    spinningBody.r_ScS_S = [[1.0], [0.0], [-1.0]]  # [m]
    spinningBody.r_SB_B = [[0.5 * sign], [-1.5], [-0.5]]  # [m]
    spinningBody.sHat_S = [[0.0], [-1.0], [0.0]]
    spinningBody.thetaInit = sign * 5.0 * macros.D2R  # [rad]
    spinningBody.thetaDotInit = -sign * 1.0 * macros.D2R  # [rad/s]
    spinningBody.k = 100.0  # [N*m/rad]
    spinningBody.c = 0.2  # [N*m*s/rad]
    return spinningBody


def _setup_spinning_body_one_dof(context):
    """Attach repeated spinning body one-DOF state effectors."""

    for effectorIndex in range(context.config.numberOfComponents):
        _add_state_effector(context, _make_spinning_body_one_dof(effectorIndex))


def _make_spinning_body_two_dof(effectorIndex):
    """Create one spinning body two-DOF state effector."""

    sign = -1.0 if effectorIndex % 2 else 1.0
    spinningBody = spinningBodyTwoDOFStateEffector.SpinningBodyTwoDOFStateEffector()
    spinningBody.ModelTag = f"spinningBodyTwoDOF{effectorIndex}"
    spinningBody.mass1 = 100.0  # [kg]
    spinningBody.mass2 = 50.0  # [kg]
    spinningBody.IS1PntSc1_S1 = [[100.0, 0.0, 0.0], [0.0, 50.0, 0.0], [0.0, 0.0, 50.0]]  # [kg*m^2]
    spinningBody.IS2PntSc2_S2 = [[50.0, 0.0, 0.0], [0.0, 30.0, 0.0], [0.0, 0.0, 40.0]]  # [kg*m^2]
    spinningBody.dcm_S10B = [[-1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, 1.0]]
    spinningBody.dcm_S20S1 = [[0.0, -1.0, 0.0], [0.0, 0.0, -1.0], [1.0, 0.0, 0.0]]
    spinningBody.r_Sc1S1_S1 = [[2.0], [-0.5], [0.0]]  # [m]
    spinningBody.r_Sc2S2_S2 = [[1.0], [0.0], [-1.0]]  # [m]
    spinningBody.r_S1B_B = [[-2.0 * sign], [0.5], [-1.0]]  # [m]
    spinningBody.r_S2S1_S1 = [[0.5], [-1.5], [-0.5]]  # [m]
    spinningBody.s1Hat_S1 = [[0.0], [0.0], [1.0]]
    spinningBody.s2Hat_S2 = [[0.0], [-1.0], [0.0]]
    spinningBody.theta1Init = sign * 2.0 * macros.D2R  # [rad]
    spinningBody.theta2Init = sign * 5.0 * macros.D2R  # [rad]
    spinningBody.theta1DotInit = 2.0 * macros.D2R  # [rad/s]
    spinningBody.theta2DotInit = -1.0 * macros.D2R  # [rad/s]
    spinningBody.k1 = 1000.0  # [N*m/rad]
    spinningBody.k2 = 500.0  # [N*m/rad]
    spinningBody.c1 = 0.2  # [N*m*s/rad]
    spinningBody.c2 = 0.1  # [N*m*s/rad]
    return spinningBody


def _setup_spinning_body_two_dof(context):
    """Attach repeated spinning body two-DOF state effectors."""

    for effectorIndex in range(context.config.numberOfComponents):
        _add_state_effector(context, _make_spinning_body_two_dof(effectorIndex))


def _default_vscmg_payload():
    """Create a VSCMG configuration payload with common defaults."""

    vscmgConfig = messaging.VSCMGConfigMsgPayload()
    vscmgConfig.rGB_B = [[0.0], [0.0], [0.0]]  # [m]
    vscmgConfig.gsHat0_B = [[1.0], [0.0], [0.0]]
    vscmgConfig.gtHat0_B = [[0.0], [1.0], [0.0]]
    vscmgConfig.ggHat_B = [[0.0], [0.0], [1.0]]
    vscmgConfig.u_s_max = -1.0  # [N*m]
    vscmgConfig.u_s_min = -1.0  # [N*m]
    vscmgConfig.u_s_f = 0.0  # [N*m]
    vscmgConfig.wheelLinearFrictionRatio = -1.0
    vscmgConfig.u_g_current = 0.0  # [N*m]
    vscmgConfig.u_g_max = -1.0  # [N*m]
    vscmgConfig.u_g_min = -1.0  # [N*m]
    vscmgConfig.u_g_f = 0.0  # [N*m]
    vscmgConfig.gimbalLinearFrictionRatio = -1.0
    vscmgConfig.Omega = 0.0  # [rad/s]
    vscmgConfig.gamma = 0.0  # [rad]
    vscmgConfig.gammaDot = 0.0  # [rad/s]
    vscmgConfig.Omega_max = 6000.0 * macros.RPM  # [rad/s]
    vscmgConfig.gammaDot_max = -1.0  # [rad/s]
    vscmgConfig.IW1 = 100.0 / vscmgConfig.Omega_max  # [kg*m^2]
    vscmgConfig.IW2 = 0.5 * vscmgConfig.IW1  # [kg*m^2]
    vscmgConfig.IW3 = 0.5 * vscmgConfig.IW1  # [kg*m^2]
    vscmgConfig.IG1 = 0.1  # [kg*m^2]
    vscmgConfig.IG2 = 0.2  # [kg*m^2]
    vscmgConfig.IG3 = 0.3  # [kg*m^2]
    vscmgConfig.U_s = 4.8e-06 * 1.0e4  # [kg*m]
    vscmgConfig.U_d = 1.54e-06 * 1.0e4  # [kg*m]
    vscmgConfig.l = 0.01  # [m]
    vscmgConfig.L = 0.1  # [m]
    vscmgConfig.rGcG_G = [[0.0001], [-0.02], [0.1]]  # [m]
    vscmgConfig.massW = 6.0  # [kg]
    vscmgConfig.massG = 6.0  # [kg]
    vscmgConfig.VSCMGModel = 2
    return vscmgConfig


def _setup_vscmg(context):
    """Attach a VSCMG state effector with repeated VSCMG devices."""

    vscmgEffector = vscmgStateEffector.VSCMGStateEffector()
    vscmgEffector.ModelTag = "vscmgBenchmark"
    axes = (
        ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)),
        ((0.0, 1.0, 0.0), (0.7071067812, 0.0, -0.7071067812), (0.7071067812, 0.0, 0.7071067812)),
        ((0.0, -1.0, 0.0), (-0.7071067812, 0.0, -0.7071067812), (-0.7071067812, 0.0, 0.7071067812)),
    )

    for vscmgIndex in range(context.config.numberOfComponents):
        vscmgConfig = _default_vscmg_payload()
        gsHat, gtHat, ggHat = axes[vscmgIndex % len(axes)]
        vscmgConfig.gsHat0_B = _column(gsHat)
        vscmgConfig.gtHat0_B = _column(gtHat)
        vscmgConfig.ggHat_B = _column(ggHat)
        vscmgConfig.Omega = (2000.0 - 150.0 * vscmgIndex) * macros.RPM  # [rad/s]
        vscmgConfig.gamma = 0.01 * vscmgIndex  # [rad]
        vscmgConfig.gammaDot = 0.005 * ((-1.0) ** vscmgIndex)  # [rad/s]
        vscmgConfig.rGB_B = [[0.05 * vscmgIndex], [0.02 * (vscmgIndex % 2)], [-0.02]]  # [m]
        vscmgEffector.AddVSCMG(vscmgConfig)

    context.scObject.addStateEffector(vscmgEffector)

    cmdArray = messaging.VSCMGArrayTorqueMsgPayload()
    cmdArray.wheelTorque = [0.001 * ((-1.0) ** index) for index in range(context.config.numberOfComponents)]  # [N*m]
    cmdArray.gimbalTorque = [
        0.002 * ((-1.0) ** (index + 1)) for index in range(context.config.numberOfComponents)
    ]  # [N*m]
    cmdMsg = messaging.VSCMGArrayTorqueMsg().write(cmdArray)
    vscmgEffector.cmdsInMsg.subscribeTo(cmdMsg)
    context.keepAlive.append(cmdMsg)
    _add_task_model(context, vscmgEffector)


CASE_DEFINITIONS = {
    "spacecraft": CaseDefinition("spacecraft", _setup_spacecraft_only),
    "dual-hinged-rigid-body": CaseDefinition("dual-hinged-rigid-body", _setup_dual_hinged_rigid_body),
    "gravity-gradient": CaseDefinition("gravity-gradient", _setup_gravity_gradient),
    "hinged-rigid-body": CaseDefinition("hinged-rigid-body", _setup_hinged_rigid_body),
    "linear-spring-mass-damper": CaseDefinition("linear-spring-mass-damper", _setup_linear_spring_mass_damper),
    "linear-translation-ndof": CaseDefinition("linear-translation-ndof", _setup_linear_translation_ndof),
    "linear-translation-one-dof": CaseDefinition("linear-translation-one-dof", _setup_linear_translation_one_dof),
    "n-hinged-rigid-body": CaseDefinition("n-hinged-rigid-body", _setup_n_hinged_rigid_body),
    "prescribed-motion": CaseDefinition("prescribed-motion", _setup_prescribed_motion),
    "reaction-wheel": CaseDefinition("reaction-wheel", _setup_reaction_wheel),
    "spherical-pendulum": CaseDefinition("spherical-pendulum", _setup_spherical_pendulum),
    "spinning-body-ndof": CaseDefinition("spinning-body-ndof", _setup_spinning_body_ndof),
    "spinning-body-one-dof": CaseDefinition("spinning-body-one-dof", _setup_spinning_body_one_dof),
    "spinning-body-two-dof": CaseDefinition("spinning-body-two-dof", _setup_spinning_body_two_dof),
    "vscmg": CaseDefinition("vscmg", _setup_vscmg),
}


CASE_ALIASES = {
    "dualhingedrigidbody": "dual-hinged-rigid-body",
    "dualhingedrigidbodystateeffector": "dual-hinged-rigid-body",
    "gravitygradienteffector": "gravity-gradient",
    "hingedrigidbodyeffector": "hinged-rigid-body",
    "hingedrigidbodystateeffector": "hinged-rigid-body",
    "linearspringmassdampereffector": "linear-spring-mass-damper",
    "linearspringmassdamper": "linear-spring-mass-damper",
    "lineartranslationndofstateeffector": "linear-translation-ndof",
    "lineartranslationonedofstateeffector": "linear-translation-one-dof",
    "nhingedrigidbodystateeffector": "n-hinged-rigid-body",
    "prescribedmotionstateeffector": "prescribed-motion",
    "reactionwheelstateeffector": "reaction-wheel",
    "sphericalpendulum": "spherical-pendulum",
    "spinningbodyndofstateeffector": "spinning-body-ndof",
    "spinningbodyonedofstateeffector": "spinning-body-one-dof",
    "spinningbodytwodofstateeffector": "spinning-body-two-dof",
    "vscmgstateeffector": "vscmg",
}


def _case_key(caseName):
    """Return a punctuation-insensitive lookup key for ``caseName``."""

    return "".join(character for character in caseName.lower() if character.isalnum())


def _canonical_case_name(caseName):
    """Map a user-supplied case name to a benchmark case key."""

    if caseName.lower() == "all":
        return "all"

    normalized = _case_key(caseName)
    for canonicalName in CASE_DEFINITIONS:
        if normalized == _case_key(canonicalName):
            return canonicalName

    if normalized in CASE_ALIASES:
        return CASE_ALIASES[normalized]

    validCases = ", ".join(["all"] + sorted(CASE_DEFINITIONS))
    raise argparse.ArgumentTypeError(f"unknown case '{caseName}'. Valid cases: {validCases}")


def _parse_args():
    """Parse command-line options for the benchmark."""

    parser = argparse.ArgumentParser(
        description="Benchmark selected Basilisk spacecraft dynamics effectors."
    )
    parser.add_argument(
        "--case",
        type=_canonical_case_name,
        nargs="+",
        default=("all",),
        help="case name, exact effector class alias, or 'all'",
    )
    parser.add_argument(
        "--list-cases",
        action="store_true",
        help="list available canonical case names and exit",
    )
    parser.add_argument(
        "--steps",
        type=_positive_int,
        default=BenchmarkConfig(("all",)).numberOfSteps,
        help="integration steps per measured trial",
    )
    parser.add_argument(
        "--trials",
        type=_positive_int,
        default=BenchmarkConfig(("all",)).numberOfTrials,
        help="number of measured trials",
    )
    parser.add_argument(
        "--warmup-steps",
        type=_nonnegative_int,
        default=BenchmarkConfig(("all",)).numberOfWarmupSteps,
        help="integration steps in the unreported warmup run",
    )
    parser.add_argument(
        "--dt",
        type=_positive_float,
        default=BenchmarkConfig(("all",)).timeStepSec,
        help="simulation time step in seconds",
    )
    parser.add_argument(
        "--components",
        type=_positive_int,
        default=BenchmarkConfig(("all",)).numberOfComponents,
        help="number of repeated devices or effectors in each benchmark case",
    )
    parser.add_argument(
        "--segments",
        type=_positive_int,
        default=BenchmarkConfig(("all",)).numberOfSegments,
        help="number of per-effector bodies in NDOF benchmark cases",
    )

    args = parser.parse_args()
    if args.list_cases:
        print("Available benchmark cases:")
        for caseName in sorted(CASE_DEFINITIONS):
            print(f"  {caseName}")
        raise SystemExit(0)

    selectedCases = tuple(args.case)
    if "all" in selectedCases:
        selectedCases = tuple(CASE_DEFINITIONS)

    return BenchmarkConfig(
        selectedCases=selectedCases,
        numberOfSteps=args.steps,
        numberOfTrials=args.trials,
        numberOfWarmupSteps=args.warmup_steps,
        timeStepSec=args.dt,
        numberOfComponents=args.components,
        numberOfSegments=args.segments,
    )


def _create_simulation(caseName, config):
    """Build and initialize one benchmark simulation."""

    processName = "dynamicsProcess"
    taskName = "dynamicsTask"
    processRateNs = macros.sec2nano(config.timeStepSec)  # [ns]

    scSim = SimulationBaseClass.SimBaseClass()
    dynProcess = scSim.CreateNewProcess(processName)
    dynProcess.addTask(scSim.CreateNewTask(taskName, processRateNs))

    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "dynamicsEffectorBenchmarkSpacecraft"
    _set_hub_properties(scObject)
    scSim.AddModelToTask(taskName, scObject)

    context = BenchmarkContext(
        scSim=scSim,
        scObject=scObject,
        taskName=taskName,
        config=config,
        keepAlive=[],
    )
    CASE_DEFINITIONS[caseName].setupFunction(context)
    scSim.benchmarkKeepAlive = context.keepAlive
    return scSim


def _run_trial(caseName, config, numberOfSteps):
    """Run one initialized benchmark simulation and return wall time."""

    stopTimeSec = numberOfSteps * config.timeStepSec  # [s]
    stopTimeNs = macros.sec2nano(stopTimeSec)  # [ns]

    scSim = _create_simulation(caseName, config)
    scSim.InitializeSimulation()
    scSim.ConfigureStopTime(stopTimeNs)

    startTimeSec = time.perf_counter()  # [s]
    scSim.ExecuteSimulation()
    elapsedTimeSec = time.perf_counter() - startTimeSec  # [s]

    return elapsedTimeSec


def _trial_metrics(elapsedTimeSec, numberOfSteps):
    """Compute display metrics from one elapsed wall time."""

    microsecondsPerStep = elapsedTimeSec * 1.0e6 / numberOfSteps  # [us/step]
    return microsecondsPerStep


def _print_header(config):
    """Print the benchmark setup."""

    stopTimeSec = config.numberOfSteps * config.timeStepSec  # [s]
    requestedCases = "all" if len(config.selectedCases) == len(CASE_DEFINITIONS) else ", ".join(config.selectedCases)
    print("Dynamics effector benchmark")
    print(f"Cases: {requestedCases}")
    print(f"Steps per trial: {config.numberOfSteps}")
    print(f"Time step: {config.timeStepSec:.6f} s")
    print(f"Simulated time per trial: {stopTimeSec:.3f} s")
    print(f"Trials: {config.numberOfTrials}")
    print(f"Warmup steps: {config.numberOfWarmupSteps}")
    print(f"Components per case: {config.numberOfComponents}")
    print(f"Segments per NDOF case: {config.numberOfSegments}")
    print("Gravity: disabled except gravity-gradient case")
    print("Recorders: disabled")
    print("Spacecraft integrator: RK4 default")


def _print_trial_row(label, elapsedTimeSec, numberOfSteps):
    """Print one single-case trial row."""

    microsecondsPerStep = _trial_metrics(elapsedTimeSec, numberOfSteps)
    print(f"{label:<9}{elapsedTimeSec:12.6f}{microsecondsPerStep:17.3f}")


def _print_summary_row(caseName, elapsedTimesSec, numberOfSteps):
    """Print one all-case summary row."""

    medianElapsedTimeSec = statistics.median(elapsedTimesSec)  # [s]
    minElapsedTimeSec = min(elapsedTimesSec)  # [s]
    medianMicrosecondsPerStep = _trial_metrics(medianElapsedTimeSec, numberOfSteps)  # [us/step]
    minMicrosecondsPerStep = _trial_metrics(minElapsedTimeSec, numberOfSteps)  # [us/step]
    print(
        f"{caseName:<40}{medianElapsedTimeSec:12.6f}"
        f"{medianMicrosecondsPerStep:17.3f}"
        f"{minElapsedTimeSec:12.6f}"
        f"{minMicrosecondsPerStep:15.3f}"
    )


def _run_case(caseName, config, showTrials):
    """Run warmup and measured trials for ``caseName``."""

    if config.numberOfWarmupSteps > 0:
        _run_trial(caseName, config, config.numberOfWarmupSteps)

    if showTrials:
        print()
        print(CASE_DEFINITIONS[caseName].displayName)
        print("Trial          Wall [s]     us/step")

    elapsedTimesSec = []  # [s]
    for trialIndex in range(config.numberOfTrials):
        elapsedTimeSec = _run_trial(caseName, config, config.numberOfSteps)  # [s]
        elapsedTimesSec.append(elapsedTimeSec)
        if showTrials:
            _print_trial_row(str(trialIndex + 1), elapsedTimeSec, config.numberOfSteps)

    if showTrials:
        print()
        _print_trial_row("median", statistics.median(elapsedTimesSec), config.numberOfSteps)
        _print_trial_row("min", min(elapsedTimesSec), config.numberOfSteps)

    return elapsedTimesSec


def main():
    """Run the dynamics effector benchmark."""

    config = _parse_args()
    showTrials = len(config.selectedCases) == 1
    _print_header(config)

    if not showTrials:
        print()
        print("Case                                      median [s]   median us/step      min [s]    min us/step")

    for caseName in config.selectedCases:
        elapsedTimesSec = _run_case(caseName, config, showTrials)
        if not showTrials:
            _print_summary_row(CASE_DEFINITIONS[caseName].displayName, elapsedTimesSec, config.numberOfSteps)


if __name__ == "__main__":
    main()
