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

r"""
Variable-mass scenario in the dynamics-engine comparison series (see
:ref:`scenarioCompareOrbit` for the introduction).

Where the earlier scenarios hold the spacecraft mass properties fixed, this one lets them
change. A spacecraft in a circular orbit performs a prograde orbit-raising burn: a main engine
draws propellant from a spherical tank, so the total mass, the tank inertia, and the system
center of mass all vary continuously through the burn, while the sloshing propellant reacts to
the thrust acceleration.

Modeling choices are deliberately those a GNC analyst would make rather than the ones that
maximize the effect:

- **Engine.** A monopropellant thruster, the MOOG Monarc-445, taken from the
  :ref:`simIncludeThruster` catalog. It is tied to the tank with ``addThrusterSet`` so the burn
  both applies thrust and consumes propellant, at roughly 0.19 kg/s.
- **Burn.** Fifteen minutes of continuous orbit-raising thrust, which expends roughly fifteen
  percent of the propellant.
- **Tank.** A spherical tank sized for hydrazine: 1500 kg in a 0.75 m sphere, sitting at about
  85% fill. It uses the centered ``FuelTankModelConstantVolume`` model, so its center of mass
  stays put within the tank and the inertia simply scales with the remaining mass. The tank is
  mounted aft of the dry-structure center of mass, so as it empties the system center of mass
  migrates forward along the thrust axis.
- **Attitude.** The spacecraft is velocity-aligned and pitches at the orbital rate, so the
  body-fixed engine stays pointed along the velocity vector as the orbit carries it around. It
  is not spinning or tumbling: a delta-v maneuver is flown attitude-stabilized.
- **Balance.** Every slosh element's equilibrium sits at the tank center, so the thrust line
  passes through the center of mass and the burn is torque-free.

Slosh model
-----------

The sloshing propellant is represented by the classical equivalent mechanical model, with
parameters taken from Dodge, *The New "Dynamic Behavior of Liquids in Moving Containers"* (SwRI,
2000), the successor to Abramson's NASA SP-106. For a spherical tank at this fill the first
lateral mode carries about a third of the propellant, and the rest is a non-sloshing mass that the
tank carries rigidly (``FuelTank.propMassInit``).

The primary element is therefore a single two-degree-of-freedom :ref:`sphericalPendulum`, hinged
at the tank center and hanging aft along the settling axis. Three additional orthogonal
:ref:`LinearSpringMassDamper` particles span the three body axes for translational slosh.

.. note::

    Damping is set for a bare smooth wall (damping ratio 0.0026) so the slosh rings through
    most of the burn maximizing the effect. A real 1500 kg hydrazine
    tank would almost certainly carry baffles or a diaphragm, which raise the damping by one to two
    orders of magnitude.

Every slosh mass depletes along with the tank, so the result is a fully coupled variable-mass rigid
body with internal slosh degrees of freedom on top of the six rigid-body ones. Because the slosh
dissipates energy, the vehicle drifts toward major-axis rotation, so the hub inertia is chosen to
put the pitch axis on the major axis, otherwise the slosh actively drives the vehicle off its
burn attitude.

The back-substitution (BSM) side is the reference. A ``useThruster`` switch replaces the engine
with an equivalent prescribed leak rate (``setFuelLeakRate``) at the same nominal mass flow,
isolating pure mass depletion with no thrust force (with the caveat, above, that the pendulum has
no restoring force without thrust).

.. note::

    The MuJoCo side of this comparison is a scaffold. Until that is finished, ``run`` executes
    the BSM reference only, plots it, and writes a ground-truth trajectory to ``results/`` for
    the mock-up to be validated against.

The script is found in the folder ``basilisk/examples/dynamicsComparison`` and executed by
using::

    python3 scenarioCompareVariableMass.py

Illustration of Simulation Results
----------------------------------

The burn raises the orbit: the semi-major axis climbs steadily while the propellant is expended.

.. image:: /_images/Scenarios/scenarioCompareVariableMass_orbit.svg
   :align: center

The tank propellant is drawn down through the burn. The plotted total system mass is the sum of
the dry hub, the bulk propellant, and the depleting slosh masses.

.. image:: /_images/Scenarios/scenarioCompareVariableMass_fuelMass.svg
   :align: center

The body rate holds the orbital pitch rate that keeps the engine velocity-aligned, with the
slosh coupling riding on top of it.

.. image:: /_images/Scenarios/scenarioCompareVariableMass_rate.svg
   :align: center

The slosh degrees of freedom (the three translational displacements and the pendulum
``phi``/``theta`` angles) are excited by the thrust step and by residual slosh carried into the
burn.

.. image:: /_images/Scenarios/scenarioCompareVariableMass_slosh.svg
   :align: center

"""

import os

import numpy as np
import matplotlib.pyplot as plt

from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros
from Basilisk.utilities import unitTestSupport
from Basilisk.utilities import simIncludeGravBody
from Basilisk.utilities import simIncludeThruster
from Basilisk.utilities import pythonVariableLogger
from Basilisk.utilities import orbitalMotion
from Basilisk.utilities import RigidBodyKinematics as rbk
from Basilisk.architecture import messaging
from Basilisk.simulation import spacecraft
from Basilisk.simulation import fuelTank
from Basilisk.simulation import linearSpringMassDamper
from Basilisk.simulation import sphericalPendulum
from Basilisk.simulation import thrusterDynamicEffector
from Basilisk.simulation import svIntegrators

try:
    from Basilisk.simulation import mujoco
    from Basilisk.simulation import NBodyGravity
    from Basilisk.simulation import pointMassGravityModel
    couldImportMujoco = True
except Exception:
    couldImportMujoco = False

# Flip to True once the MuJoCo variable-mass mock-up in buildMujoco() is finished. Until then
# run() executes and plots the BSM reference only (and still tests green).
MUJOCO_SIDE_IMPLEMENTED = False

# Consistent palette drawn from the standard Basilisk plotting colors.
COLOR_BSM = unitTestSupport.getLineColor(0, 3)
COLOR_AUX = unitTestSupport.getLineColor(2, 3)

fileName = os.path.basename(os.path.splitext(__file__)[0])

# Folder this scenario writes its JSON summary and reference trajectory into.
resultsPath = os.path.join(os.path.dirname(__file__), "results")

G0 = 9.80665  # [m/s^2] standard gravity used for the Isp-to-mass-flow conversion

# --- Orbit ---------------------------------------------------------------------------------
# A circular parking orbit that the burn raises. The spacecraft flies velocity-aligned and
# pitches at the orbital rate so the body-fixed engine tracks the velocity vector.
ORBIT_A = 7000.0e3  # [m] semi-major axis (circular)
ORBIT_I = 33.3*macros.D2R  # [rad] inclination
ORBIT_RAAN = 48.2*macros.D2R  # [rad] right ascension of the ascending node
ORBIT_ARGLAT = 85.3*macros.D2R  # [rad] argument of latitude at epoch

# --- Hub (dry structure) --------------------------------------------------------------------
HUB_MASS = 1500.0  # [kg] dry hub mass
# Solar arrays along the body x-axis make Ixx the smallest principal inertia and leave the pitch
# axis (body y, the orbit normal) the MAJOR axis of the loaded vehicle. That ordering matters:
# slosh dissipates energy, and an energy-dissipating body migrates toward major-axis rotation, so
# pitching about the intermediate axis instead lets the slosh actively drive the vehicle off its
# burn attitude (measured here: 49% of the pitch rate survives, against 85% about the major axis).
HUB_INERTIA = (1200.0, 2000.0, 1900.0)  # [kg*m^2] principal dry-hub inertia about Bc
HUB_R_BcB_B = (0.0, 0.0, 0.0)  # [m] dry-structure center of mass, at the body origin

# --- Tank and propellant ----------------------------------------------------------------------
# Spherical tank (the dominant shape for pressurized systems) holding monopropellant hydrazine,
# which is what the Monarc-445 burns. The radius is set by the propellant load: 1500 kg of
# hydrazine needs at least 1.494 m^3, so a 0.75 m sphere (1.767 m^3) sits at ~85% fill. It does
# NOT fit in a 0.70 m sphere.
#
# The FuelTankModelConstantVolume model keeps the load centered in the tank and scales the
# inertia with the remaining mass, which is the right idealization on orbit: a propellant
# management device holds the propellant in place, so it does not drain to one side (that is a
# launch/settled regime). Mounting the tank AFT of the dry-structure center of mass is what makes
# the SYSTEM center of mass migrate forward along the thrust axis (and so stay torque-free) as the
# propellant is spent, the physically honest source of a moving center of mass.
TANK_RADIUS = 0.75  # [m] spherical tank radius
TANK_R_TB_B = (0.0, 0.0, -0.8)  # [m] tank center, mounted aft of the dry CoM along -z
PROPELLANT_MASS = 1500.0  # [kg] total propellant (~50% of wet mass)
PROPELLANT_DENSITY = 1004.0  # [kg/m^3] hydrazine

# --- Slosh: equivalent mechanical model --------------------------------------------------------
# First-mode lateral slosh parameters for a SPHERICAL tank, from Dodge, "The New Dynamic Behavior
# of Liquids in Moving Containers", SwRI 2000 (the successor to Abramson, NASA SP-106), Figs. 1.11
# and 3.4. Two facts make the pendulum the right element here:
#
#   * The pendulum arm length is PURELY GEOMETRIC: L1/R is a function of fill fraction alone, so
#     the slosh frequency omega = sqrt(a/L1) tracks the thrust acceleration automatically as the
#     vehicle mass drops. A spring-mass-damper's sqrt(k/m) does not, so k must be retuned for every
#     acceleration level.
#   * For a sphere the pendulum hinge AND the non-sloshing mass both sit at the tank center at
#     every fill level (rotating a sphere about its center does not move an inviscid liquid), and
#     I0 = 0. This is exactly what Basilisk's sphericalPendulum + FuelTank express.
#
# Basilisk's FuelTank depletes every slosh mass PROPORTIONALLY, so the slosh-mass fraction is held
# constant and cannot track the true fill dependence (which runs 0.281 -> 0.339 over this burn).
# The parameters are therefore evaluated at the MID-BURN fill (~80%) to split the difference.
SLOSH_MASS_FRACTION = 0.335  # [-] m1/m_liq at ~80% fill (Dodge Fig. 3.4)
PEND_LENGTH_RATIO = 0.452  # [-] L1/R at ~80% fill (Dodge Fig. 3.4)
SLOSH_DAMPING_RATIO = 0.0026  # [-] zeta, bare smooth wall (Dodge Eq. 2.9b), see docstring

# Benchmarking fixture, NOT physics: the three orthogonal spring-mass-dampers are retained purely
# so the comparison exercises both Basilisk slosh effectors (a slide joint and a ball joint on the
# MuJoCo side). The classical lateral-slosh EMM has no axial degree of freedom, and one 2-DOF
# pendulum already spans both lateral axes. Their mass comes out of the non-sloshing mass so the
# propellant budget still closes. See the docstring.
SMD_MASS = 25.0  # [kg] each of the three particles
SMD_LATERAL_RHO0 = 0.02  # [m] residual lateral slosh carried into the burn

# P0 frame: rest axis pHat_01 along -z, so the bob hangs aft, the direction the propellant is
# pushed by a +z burn. pHat_03 = pHat_01 x pHat_02 keeps the triad right-handed. A left-handed
# triad silently produces an inverted, exponentially diverging pendulum.
PEND_PHAT_01 = (0.0, 0.0, -1.0)
PEND_PHAT_02 = (1.0, 0.0, 0.0)
PEND_PHAT_03 = (0.0, -1.0, 0.0)
PEND_RATE0 = 0.02  # [rad/s] initial phi and theta rates (residual slosh)

# --- Main-engine burn -------------------------------------------------------------------------
# MOOG Monarc-445: a real monopropellant thruster (445 N, Isp 234 s) from the simIncludeThruster
# catalog. Its catalog values are used as-is, giving mDot ~ 0.19 kg/s.
THRUSTER_TYPE = "MOOG_Monarc_445"
THRUST_DIR_B = (0.0, 0.0, 1.0)  # body +z, which starts aligned with the velocity vector
THRUST_POS_B = (0.0, 0.0, -1.5)  # [m] on the z-axis, aft: the burn is torque-free

# --- Simulation --------------------------------------------------------------------------------
SIM_DURATION = 900.0  # [s] a 15-minute orbit-raising burn
STEPS_PER_PERIOD = 150.0  # target RK4 steps per stiffest slosh oscillation
MAX_TIME_STEP = 0.05  # [s] cap on the integration step


def nominalMassFlow(maxThrust, steadyIsp):
    """Nominal propellant mass-flow rate of the engine [kg/s].

    Args:
        maxThrust (float): engine thrust [N]
        steadyIsp (float): specific impulse [s]

    Returns:
        float: mass flow rate [kg/s].
    """
    return maxThrust/(G0*steadyIsp)


def thrusterSpec():
    """Return the catalog ``(MaxThrust [N], steadyIsp [s])`` of the modeled engine."""
    factory = simIncludeThruster.thrusterFactory()
    device = factory.create(THRUSTER_TYPE, list(THRUST_POS_B), list(THRUST_DIR_B))
    return device.MaxThrust, device.steadyIsp


def earthMu():
    """Earth gravitational parameter [m^3/s^2], from the same body the sim uses."""
    return simIncludeGravBody.gravBodyFactory().createEarth().mu


def sloshParameters():
    """Derive the slosh model inputs from the literature ratios and the vehicle sizing.

    The pendulum is the primary element: its arm length is purely geometric, so its frequency
    ``omega1 = sqrt(a/L1)`` emerges from the thrust acceleration rather than being an input. The
    spring-mass-dampers are then tuned to that same physical slosh frequency, so the benchmarking
    fixture at least oscillates at the right rate.

    Returns:
        dict: slosh masses [kg], pendulum arm [m], slosh frequency [rad/s], spring constant
        [N/m], damping coefficients, the axial particle's settled offset [m], and the tank fill
        fraction [-].
    """
    maxThrust, _ = thrusterSpec()
    wetMass = HUB_MASS + PROPELLANT_MASS  # [kg]
    accel = maxThrust/wetMass  # [m/s^2] axial acceleration that restores the slosh

    pendMass = SLOSH_MASS_FRACTION*PROPELLANT_MASS  # [kg] first-mode slosh mass m1
    pendLength = PEND_LENGTH_RATIO*TANK_RADIUS  # [m] equivalent pendulum length L1
    omega1 = np.sqrt(accel/pendLength)  # [rad/s] first-mode slosh frequency (emergent)

    # The benchmarking-fixture particles come out of the non-sloshing mass so the budget closes.
    bulkMass = PROPELLANT_MASS - pendMass - 3.0*SMD_MASS  # [kg] non-sloshing mass m0

    tankVolume = 4.0/3.0*np.pi*TANK_RADIUS**3  # [m^3]
    fillFraction = PROPELLANT_MASS/(PROPELLANT_DENSITY*tankVolume)  # [-]

    return {
        "accel": accel,
        "pendMass": pendMass,
        "pendLength": pendLength,
        "omega1": omega1,
        "bulkMass": bulkMass,
        "fillFraction": fillFraction,
        "smdK": SMD_MASS*omega1**2,  # [N/m] matched to the physical slosh frequency
        "smdC": 2.0*SLOSH_DAMPING_RATIO*SMD_MASS*omega1,  # [N*s/m]
        "pendD": 2.0*SLOSH_DAMPING_RATIO*pendMass*pendLength*omega1,  # pendulum damping
        # Under thrust the axial particle settles at rho = -a/omega1^2 (which equals -L1). It is
        # started there, as a settling burn would leave it, rather than ringing down from zero.
        "axialRho0": -accel/omega1**2,
    }


def timeStep():
    """Integrator step that resolves the slosh oscillation [s]."""
    naturalPeriod = 2.0*np.pi/sloshParameters()["omega1"]  # [s]
    return min(MAX_TIME_STEP, naturalPeriod/STEPS_PER_PERIOD)


def initialOrbitState(mu):
    """Initial inertial position and velocity on the circular parking orbit.

    Args:
        mu (float): gravitational parameter [m^3/s^2]

    Returns:
        tuple: ``(rN [m], vN [m/s])`` as numpy arrays.
    """
    oe = orbitalMotion.ClassicElements()
    oe.a = ORBIT_A  # [m]
    oe.e = 0.0
    oe.i = ORBIT_I  # [rad]
    oe.Omega = ORBIT_RAAN  # [rad]
    oe.omega = 0.0  # [rad]
    oe.f = ORBIT_ARGLAT  # [rad]
    rN, vN = orbitalMotion.elem2rv(mu, oe)
    return np.array(rN), np.array(vN)


def velocityAlignedAttitude(rN, vN, mu):
    """Body attitude and rate that keep the engine pointed along the velocity vector.

    The body frame is built with z along the velocity (the thrust axis), y along the orbit
    normal, and x completing the triad. Holding that alignment as the orbit carries the
    spacecraft around requires pitching about the orbit normal at the orbital rate, which in body
    components is a rate purely about the body y-axis.

    Args:
        rN (numpy.ndarray): inertial position [m]
        vN (numpy.ndarray): inertial velocity [m/s]
        mu (float): gravitational parameter [m^3/s^2]

    Returns:
        tuple: ``(sigma_BN, omega_BN_B [rad/s], meanMotion [rad/s])``.
    """
    vHat = vN/np.linalg.norm(vN)
    hHat = np.cross(rN, vN)
    hHat = hHat/np.linalg.norm(hHat)
    xHat = np.cross(hHat, vHat)  # completes the right-handed triad (nadir for a circular orbit)
    dcm_BN = np.array([xHat, hHat, vHat])  # rows are the body axes in inertial components
    meanMotion = np.sqrt(mu/ORBIT_A**3)  # [rad/s]
    return rbk.C2MRP(dcm_BN), np.array([0.0, meanMotion, 0.0]), meanMotion


def buildBSM(dt, record, useThruster=True):
    """Build (and initialize) the back-substitution variable-mass reference simulation.

    Args:
        dt (float): integrator time step [s]
        record (bool): if True, attach the hub-state, fuel-tank and slosh recorders
        useThruster (bool, optional): if True, deplete the tank with the firing main engine that
            also applies thrust. If False, deplete it with an equivalent prescribed leak rate
            and no thrust force. Defaults to True.

    Returns:
        tuple: ``(scSim, recorders, handles)`` where ``recorders`` is a dict of the attached
        recorders (empty when ``record`` is False) and ``handles`` keeps the created modules and
        stand-alone messages alive.
    """
    scSim = SimulationBaseClass.SimBaseClass()
    process = scSim.CreateNewProcess("dyn")
    process.addTask(scSim.CreateNewTask("dynTask", macros.sec2nano(dt)))

    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "hub"
    scObject.hub.mHub = HUB_MASS  # [kg]
    scObject.hub.r_BcB_B = [[c] for c in HUB_R_BcB_B]  # [m]
    scObject.hub.IHubPntBc_B = np.diag(HUB_INERTIA).tolist()  # [kg*m^2]
    scSim.AddModelToTask("dynTask", scObject)

    integrator = svIntegrators.svIntegratorRK4(scObject)
    scObject.setIntegrator(integrator)

    gravFactory = simIncludeGravBody.gravBodyFactory()
    earth = gravFactory.createEarth()
    earth.isCentralBody = True
    gravFactory.addBodiesTo(scObject)
    mu = earth.mu  # [m^3/s^2]

    rN, vN = initialOrbitState(mu)
    sigma_BN, omega_BN_B, _ = velocityAlignedAttitude(rN, vN, mu)
    scObject.hub.r_CN_NInit = rN.tolist()  # [m]
    scObject.hub.v_CN_NInit = vN.tolist()  # [m/s]
    scObject.hub.sigma_BNInit = [[c] for c in sigma_BN]
    scObject.hub.omega_BN_BInit = [[c] for c in omega_BN_B]  # [rad/s] orbital pitch rate

    slosh = sloshParameters()

    # Three orthogonal spring-mass-damper particles (a benchmarking fixture, not the physical
    # slosh model, see the docstring). Every equilibrium sits at the tank center, because a
    # slosh mass's equilibrium is the propellant center of mass. That keeps the thrust line
    # through the center of mass and the burn torque-free. Off-centering them instead produces a
    # standing thrust moment that steadily spins the vehicle up. The lateral pair carries residual
    # slosh from earlier maneuvers. The axial one starts at the offset the thrust settles it to,
    # as a settling burn would leave it.
    smdInit = (
        ((1.0, 0.0, 0.0), SMD_LATERAL_RHO0),
        ((0.0, 1.0, 0.0), -SMD_LATERAL_RHO0),
        ((0.0, 0.0, 1.0), slosh["axialRho0"]),
    )
    particles = []
    for (pHat, rho0) in smdInit:
        particle = linearSpringMassDamper.LinearSpringMassDamper()
        particle.k = slosh["smdK"]  # [N/m] tuned to the physical first-mode slosh frequency
        particle.c = slosh["smdC"]  # [N*s/m]
        particle.r_PB_B = [[c] for c in TANK_R_TB_B]  # [m] equilibrium at the tank center
        particle.pHat_B = [[c] for c in pHat]
        particle.rhoInit = rho0  # [m]
        particle.rhoDotInit = 0.0  # [m/s]
        particle.massInit = SMD_MASS  # [kg]
        particles.append(particle)

    # First-mode lateral slosh: a 2-DOF spherical pendulum pivoted at the tank center and hanging
    # aft along the settling axis. This is the physically standard element (Dodge, SwRI 2000).
    pendulum = sphericalPendulum.SphericalPendulum()
    pendulum.pendulumRadius = slosh["pendLength"]  # [m] L1, purely geometric
    pendulum.d = [[c] for c in TANK_R_TB_B]  # [m] pivot at the tank center (correct for a sphere)
    pendulum.D = (slosh["pendD"]*np.eye(3)).tolist()
    # phiInit/thetaInit are not exposed and default to zero, so the bob starts hanging aft.
    pendulum.phiDotInit = PEND_RATE0  # [rad/s] residual slosh
    pendulum.thetaDotInit = PEND_RATE0  # [rad/s]
    pendulum.massInit = slosh["pendMass"]  # [kg] first-mode slosh mass m1
    pendulum.pHat_01 = [[c] for c in PEND_PHAT_01]
    pendulum.pHat_02 = [[c] for c in PEND_PHAT_02]
    pendulum.pHat_03 = [[c] for c in PEND_PHAT_03]

    # Spherical tank carrying the non-sloshing mass m0: centered load, I0 = 0, inertia scaling
    # with the remaining mass.
    tank = fuelTank.FuelTank()
    tank.ModelTag = "propTank"
    tankModel = fuelTank.FuelTankModelConstantVolume()
    tankModel.propMassInit = slosh["bulkMass"]  # [kg] non-sloshing mass m0
    tankModel.maxFuelMass = slosh["bulkMass"]  # [kg]
    tankModel.radiusTankInit = TANK_RADIUS  # [m]
    tankModel.r_TcT_TInit = [[0.0], [0.0], [0.0]]  # [m] load centered in the tank
    tank.setTankModel(tankModel)
    tank.setR_TB_B([[c] for c in TANK_R_TB_B])  # [m]

    sloshEffectors = particles + [pendulum]
    for effector in sloshEffectors:
        tank.pushFuelSloshParticle(effector)
    tank.setUpdateOnly(True)

    scObject.addStateEffector(tank)
    for effector in sloshEffectors:
        scObject.addStateEffector(effector)
    scSim.AddModelToTask("dynTask", tank)

    handles = [scObject, integrator, gravFactory, tank, tankModel] + sloshEffectors
    maxThrust, steadyIsp = thrusterSpec()
    if useThruster:
        thFactory = simIncludeThruster.thrusterFactory()
        thFactory.create(THRUSTER_TYPE, list(THRUST_POS_B), list(THRUST_DIR_B))
        thruster = thrusterDynamicEffector.ThrusterDynamicEffector()
        thFactory.addToSpacecraft("mainEngine", thruster, scObject)
        tank.addThrusterSet(thruster)

        onTime = messaging.THRArrayOnTimeCmdMsgPayload()
        onTime.OnTimeRequest = [2.0*SIM_DURATION]  # [s] keep the engine lit for the whole burn
        thrCmdMsg = messaging.THRArrayOnTimeCmdMsg().write(onTime)
        thruster.cmdsInMsg.subscribeTo(thrCmdMsg)
        scSim.AddModelToTask("dynTask", thruster)
        handles += [thFactory, thruster, thrCmdMsg]
    else:
        # Same nominal mass flow as the engine, but with no thrust force.
        tank.setFuelLeakRate(nominalMassFlow(maxThrust, steadyIsp))  # [kg/s]

    recorders = {}
    if record:
        recorders["state"] = scObject.scStateOutMsg.recorder(macros.sec2nano(dt))
        recorders["tank"] = tank.fuelTankOutMsg.recorder(macros.sec2nano(dt))
        scSim.AddModelToTask("dynTask", recorders["state"])
        scSim.AddModelToTask("dynTask", recorders["tank"])

        # Slosh internal states are not messages. Read them from the dynamics state manager. The
        # state objects only exist after InitializeSimulation, so each lookup is deferred into a
        # callable evaluated at logging time.
        def rhoGetter(particle):
            return lambda _: scObject.dynManager.getStateObject(
                particle.nameOfRhoState).getState()[0][0]

        def pendGetter(name):
            return lambda _: scObject.dynManager.getStateObject(name).getState()[0][0]

        loggerSpec = {f"rho{i+1}": rhoGetter(p) for i, p in enumerate(particles)}
        loggerSpec["phi"] = pendGetter(pendulum.nameOfPhiState)
        loggerSpec["theta"] = pendGetter(pendulum.nameOfThetaState)
        recorders["slosh"] = pythonVariableLogger.PythonVariableLogger(
            loggerSpec, macros.sec2nano(dt))
        scSim.AddModelToTask("dynTask", recorders["slosh"])

    scSim.InitializeSimulation()
    return scSim, recorders, handles


def mujocoModel():
    """Return a fixed-mass MJCF skeleton for the MuJoCo mock-up (starting point for the port).

    This is intentionally a fixed-mass approximation: MuJoCo assigns each body's mass and inertia
    at model-compile time, so the depleting tank and shrinking slosh masses are NOT represented
    here. The mock-up must add the variable-mass behavior on top (for example by rewriting the
    body masses each step on the known depletion schedule), which is the crux of what this
    comparison exercises.

    Returns:
        str: MJCF XML string.
    """
    ix, iy, iz = HUB_INERTIA
    tx, ty, tz = TANK_R_TB_B
    slosh = sloshParameters()
    m0 = slosh["bulkMass"]
    tankInertia = 0.4*m0*TANK_RADIUS**2  # [kg*m^2] solid-sphere inertia
    k = slosh["smdK"]
    c = slosh["smdC"]
    return f"""
<mujoco>
  <option gravity="0 0 0"/>
  <worldbody>
    <body name="hub">
      <freejoint/>
      <inertial pos="0 0 0" mass="{HUB_MASS}" fullinertia="{ix} {iy} {iz} 0 0 0"/>
      <!-- Non-sloshing propellant mass m0, mounted aft (TODO: deplete this mass and inertia). -->
      <body name="tank" pos="{tx} {ty} {tz}">
        <inertial pos="0 0 0" mass="{m0}"
                  diaginertia="{tankInertia} {tankInertia} {tankInertia}"/>
      </body>
      <!-- Three orthogonal slosh particles as slide joints with springs, at the tank center. -->
      <body name="sloshX" pos="{tx} {ty} {tz}">
        <joint name="sloshX" type="slide" axis="1 0 0" stiffness="{k}" damping="{c}"/>
        <inertial pos="0 0 0" mass="{SMD_MASS}" diaginertia="1e-6 1e-6 1e-6"/>
      </body>
      <body name="sloshY" pos="{tx} {ty} {tz}">
        <joint name="sloshY" type="slide" axis="0 1 0" stiffness="{k}" damping="{c}"/>
        <inertial pos="0 0 0" mass="{SMD_MASS}" diaginertia="1e-6 1e-6 1e-6"/>
      </body>
      <body name="sloshZ" pos="{tx} {ty} {tz}">
        <joint name="sloshZ" type="slide" axis="0 0 1" stiffness="{k}" damping="{c}"/>
        <inertial pos="0 0 0" mass="{SMD_MASS}" diaginertia="1e-6 1e-6 1e-6"/>
      </body>
      <!-- First-mode slosh: a ball joint at the tank center with the bob hanging aft. Note the
           pendulum's restoring force comes from the THRUST, not from gravity, so the mock-up must
           reproduce the thrust for this to behave at all. -->
      <body name="pendulum" pos="{tx} {ty} {tz}">
        <joint name="pendulum" type="ball" damping="{slosh['pendD']}"/>
        <inertial pos="0 0 {-slosh['pendLength']}" mass="{slosh['pendMass']}"
                  diaginertia="1e-4 1e-4 1e-4"/>
      </body>
    </body>
  </worldbody>
  <!-- TODO: main-engine thrust along +z applied at {THRUST_POS_B}. -->
</mujoco>
"""


def buildMujoco(dt, record):
    """MuJoCo scaffold for the variable-mass problem, with the depletion wiring unfinished.

    This builds the fixed-mass MJCF scene and supplies the same point-mass gravity field the BSM
    side uses (following :ref:`scenarioCompareOrbit`), then stops: it does NOT yet reproduce the
    propellant depletion (shrinking tank and slosh masses, migrating center of mass) or the
    main-engine thrust. Those are the pieces left to build. Once they are in place, set
    ``MUJOCO_SIDE_IMPLEMENTED = True`` above.

    Args:
        dt (float): integrator time step [s]
        record (bool): if True, attach a hub-state recorder

    Returns:
        tuple: ``(scSim, recorders, handles)`` matching :func:`buildBSM`.
    """
    if not couldImportMujoco:
        raise ImportError("Build Basilisk with --mujoco to run the MuJoCo comparison side.")

    scSim = SimulationBaseClass.SimBaseClass()
    process = scSim.CreateNewProcess("dyn")
    process.addTask(scSim.CreateNewTask("dynTask", macros.sec2nano(dt)))

    scene = mujoco.MJScene(mujocoModel())
    scene.ModelTag = "hubMj"
    scene.extraEoMCall = True
    scene.highOrderAttitudeIntegration = True
    scSim.AddModelToTask("dynTask", scene, 1)

    integrator = svIntegrators.svIntegratorRK4(scene)
    scene.setIntegrator(integrator)

    # MuJoCo's built-in gravity is disabled inside MJScene, so supply the same point-mass field
    # the BSM gravity effector uses.
    gravFactory = simIncludeGravBody.gravBodyFactory()
    earth = gravFactory.createEarth()
    mu = earth.mu  # [m^3/s^2]
    gravity = NBodyGravity.NBodyGravity()
    gravity.ModelTag = "gravity"
    scene.AddModelToDynamicsTask(gravity)
    gravityModel = pointMassGravityModel.PointMassGravityModel()
    gravityModel.muBody = mu  # [m^3/s^2]

    hub = scene.getBody("hub")

    recorders = {}
    if record:
        recorders["state"] = hub.getOrigin().stateOutMsg.recorder(macros.sec2nano(dt))
        scSim.AddModelToTask("dynTask", recorders["state"], 0)

    scSim.InitializeSimulation()
    rN, vN = initialOrbitState(mu)
    _, omega_BN_B, _ = velocityAlignedAttitude(rN, vN, mu)
    hub.setAttitudeRate(list(omega_BN_B))

    # Replace this raise with the depletion + thrust wiring, then
    #   return scSim, recorders, [scene, integrator, gravity, gravityModel, hub]
    raise NotImplementedError(
        "The MuJoCo scaffold builds the fixed-mass scene and its gravity field, but does not yet "
        "reproduce propellant depletion (shrinking tank/slosh masses and a migrating center of "
        "mass) or the main-engine thrust. Layer those on, then set MUJOCO_SIDE_IMPLEMENTED = True."
    )


def relativePrincipalAngle(sigmaA, sigmaB):
    """Per-sample principal rotation angle between two MRP attitude histories.

    Args:
        sigmaA (numpy.ndarray): first MRP history, shape ``(N, 3)``
        sigmaB (numpy.ndarray): second MRP history, shape ``(N, 3)``

    Returns:
        numpy.ndarray: principal angle per sample [rad].
    """
    nSamples = min(len(sigmaA), len(sigmaB))
    angle = np.empty(nSamples)
    for i in range(nSamples):
        dcmRel = rbk.MRP2C(sigmaA[i]).dot(rbk.MRP2C(sigmaB[i]).T)
        # 4*atan(|sigma_rel|) is well conditioned down to machine precision, unlike
        # arccos((trace-1)/2) which collapses to zero below ~1e-8 rad.
        angle[i] = 4.0*np.arctan(np.linalg.norm(rbk.C2MRP(dcmRel)))
    return angle


def pullBSM(recorders, mu):
    """Collect the BSM reference histories into a plain dictionary of numpy arrays.

    Args:
        recorders (dict): the recorder dict returned by :func:`buildBSM`
        mu (float): gravitational parameter [m^3/s^2]

    Returns:
        dict: time, hub state, orbit, tank mass and slosh-state histories.
    """
    stateRec = recorders["state"]
    tankRec = recorders["tank"]
    sloshRec = recorders["slosh"]

    rBN = np.array(stateRec.r_BN_N)  # [m]
    vBN = np.array(stateRec.v_BN_N)  # [m/s]
    rMag = np.linalg.norm(rBN, axis=1)  # [m]
    vMag = np.linalg.norm(vBN, axis=1)  # [m/s]
    semiMajorAxis = 1.0/(2.0/rMag - vMag**2/mu)  # [m] vis-viva

    fuelMass = np.array(tankRec.fuelMass)  # [kg] non-sloshing mass m0 history
    # The tank depletes m0 and every slosh mass proportionally to their current mass, so all mass
    # ratios stay fixed and the total propellant tracks m0 exactly. Total system mass = dry hub +
    # non-sloshing mass + the (proportionally depleting) slosh masses.
    bulkMass = sloshParameters()["bulkMass"]  # [kg] initial m0
    totalMass = HUB_MASS + fuelMass*(PROPELLANT_MASS/bulkMass)  # [kg]

    return {
        "t": np.array(stateRec.times())*macros.NANO2SEC,
        "sigma_BN": np.array(stateRec.sigma_BN),
        "omega_BN_B": np.array(stateRec.omega_BN_B),
        "r_BN_N": rBN,
        "v_BN_N": vBN,
        "semiMajorAxis": semiMajorAxis,
        "fuelMass": fuelMass,
        "totalMass": totalMass,
        "rho": np.column_stack([sloshRec.rho1, sloshRec.rho2, sloshRec.rho3]),
        "phi": np.array(sloshRec.phi),
        "theta": np.array(sloshRec.theta),
    }


def run(showPlots=False, saveJson=False, simDuration=SIM_DURATION, useThruster=True,
        saveReference=False):
    """Main function, see scenario description.

    Args:
        showPlots (bool, optional): if True, plot and show the simulation results.
            Defaults to False.
        saveJson (bool, optional): if True, write scalar comparison metrics to
            ``results/scenarioCompareVariableMass.json``. Defaults to False.
        simDuration (float, optional): burn/comparison window [s]. Defaults to ``SIM_DURATION``.
        useThruster (bool, optional): if True (default) deplete the tank with the firing main
            engine. If False, use an equivalent prescribed leak rate with no thrust force.
        saveReference (bool, optional): if True, write the BSM ground-truth trajectory to
            ``results/scenarioCompareVariableMass_reference.npz`` for the MuJoCo mock-up to be
            validated against. Defaults to False.

    Returns:
        dict: mapping from figure name to matplotlib figure.
    """
    dt = timeStep()  # [s]
    maxThrust, steadyIsp = thrusterSpec()

    bsmSim, bsmRec, _ = buildBSM(dt, True, useThruster)
    mu = earthMu()  # [m^3/s^2]
    bsmSim.ConfigureStopTime(macros.sec2nano(simDuration))
    bsmSim.ExecuteSimulation()
    bsm = pullBSM(bsmRec, mu)

    metrics = {
        "scenario": fileName,
        "timeStep": dt,
        "simDuration": simDuration,
        "useThruster": useThruster,
        "thruster": THRUSTER_TYPE,
        "maxThrust": maxThrust,
        "steadyIsp": steadyIsp,
        "nominalMassFlow": nominalMassFlow(maxThrust, steadyIsp),
        "propellantDepletedFraction": float(1.0 - bsm["fuelMass"][-1]/bsm["fuelMass"][0]),
        "wetMassChangeFraction": float(1.0 - bsm["totalMass"][-1]/bsm["totalMass"][0]),
        "semiMajorAxisRise": float(bsm["semiMajorAxis"][-1] - bsm["semiMajorAxis"][0]),
        "deltaV": float(np.linalg.norm(bsm["v_BN_N"][-1]) - np.linalg.norm(bsm["v_BN_N"][0])),
    }

    mj = None
    if couldImportMujoco and MUJOCO_SIDE_IMPLEMENTED:
        mjSim, mjRec, _ = buildMujoco(dt, True)
        mjSim.ConfigureStopTime(macros.sec2nano(simDuration))
        mjSim.ExecuteSimulation()
        mj = {
            "t": np.array(mjRec["state"].times())*macros.NANO2SEC,
            "sigma_BN": np.array(mjRec["state"].sigma_BN),
            "omega_BN_B": np.array(mjRec["state"].omega_BN_B),
        }
        metrics["attitudeErrorMax"] = float(
            np.max(relativePrincipalAngle(bsm["sigma_BN"], mj["sigma_BN"])))

    if saveReference:
        os.makedirs(resultsPath, exist_ok=True)
        np.savez(os.path.join(resultsPath, fileName+"_reference.npz"), **bsm)

    if saveJson:
        import json
        os.makedirs(resultsPath, exist_ok=True)
        with open(os.path.join(resultsPath, fileName+".json"), "w") as f:
            json.dump(metrics, f, indent=2)

    figureList = plotResults(bsm, mj)

    if showPlots:
        plt.show()
    plt.close("all")

    return figureList


def plotResults(bsm, mj=None):
    """Build the scenario figures.

    Args:
        bsm (dict): BSM reference histories from :func:`pullBSM`
        mj (dict, optional): MuJoCo histories, or None when the mock-up is not implemented

    Returns:
        dict: mapping from figure name to matplotlib figure.
    """
    t = bsm["t"]
    axisLabels = ("x", "y", "z")
    figureList = {}

    figureList[fileName+"_orbit"], ax = plt.subplots()
    ax.plot(t, (bsm["semiMajorAxis"] - bsm["semiMajorAxis"][0])*1e-3, color=COLOR_BSM)
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Semi-major axis rise [km]")

    figureList[fileName+"_fuelMass"], ax = plt.subplots()
    ax.plot(t, bsm["fuelMass"], color=COLOR_BSM, label="Tank bulk propellant")
    ax.plot(t, bsm["totalMass"], color=COLOR_AUX, label="Total system mass")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Mass [kg]")
    ax.legend(loc="best")

    figureList[fileName+"_rate"], ax = plt.subplots()
    for i in range(3):
        ax.plot(t, bsm["omega_BN_B"][:, i]*1e3, color=unitTestSupport.getLineColor(i, 3),
                label=r"$\omega_{"+axisLabels[i]+"}$ (BSM)")
    if mj is not None:
        for i in range(3):
            ax.plot(mj["t"], mj["omega_BN_B"][:, i]*1e3, "--",
                    color=unitTestSupport.getLineColor(i, 3))
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Hub body rate [mrad/s]")
    ax.legend(loc="best")

    figureList[fileName+"_slosh"], (axTop, axBot) = plt.subplots(2, 1, sharex=True)
    for i in range(3):
        axTop.plot(t, bsm["rho"][:, i]*1e3, color=unitTestSupport.getLineColor(i, 3),
                   label=r"$\rho_{"+axisLabels[i]+"}$")
    axTop.set_ylabel("Particle displacement [mm]")
    axTop.legend(loc="best")
    axBot.plot(t, bsm["phi"]*1e3, color=COLOR_BSM, label=r"$\phi$")
    axBot.plot(t, bsm["theta"]*1e3, color=COLOR_AUX, label=r"$\theta$")
    axBot.set_xlabel("Time [s]")
    axBot.set_ylabel("Pendulum angle [mrad]")
    axBot.legend(loc="best")

    return figureList


if __name__ == "__main__":
    run(showPlots=True, saveReference=True)
