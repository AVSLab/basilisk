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
- **Burn.** Fifteen minutes of continuous orbit-raising thrust, which expends roughly twelve
  percent of the propellant.
- **Tank.** A spherical tank sized for hydrazine: 1500 kg in a 0.75 m sphere, sitting at about
  85% fill. It uses the centered ``FuelTankModelConstantVolume`` model, so its center of mass
  stays put within the tank and the inertia simply scales with the remaining mass. The tank is
  mounted aft of the dry-structure center of mass, so as it empties the system center of mass
  migrates forward along the thrust axis.
- **Attitude.** The spacecraft starts velocity-aligned with the corresponding orbital pitch rate,
  so the body-fixed engine initially points along the velocity vector. No attitude controller or
  prescribed attitude is modeled; the subsequent attitude follows the passive coupled dynamics.
- **Balance.** Every slosh element's equilibrium sits at the tank center and the nominal thrust
  line follows the tank axis. The deliberately seeded lateral slosh displacements offset the
  instantaneous system center of mass, however, so the initial burn carries a small physical
  moment. The same offset and thrust geometry are applied in both engines.

Slosh model
-----------

The physical first lateral mode is a classical equivalent-mechanical model parameterized from
Dodge, *The New "Dynamic Behavior of Liquids in Moving Containers"* (SwRI, 2000). For a spherical
tank at this fill the mode carries about a third of the propellant. It is represented by a
two-degree-of-freedom :ref:`sphericalPendulum` hinged at the tank center and hanging aft.

Three orthogonal :ref:`LinearSpringMassDamper` particles are additional benchmarking fixtures, not
independent physical slosh modes. They ensure that the comparison exercises both Basilisk slosh
effectors and their MuJoCo slide- and hinge-joint counterparts. Their mass is removed from the
non-sloshing portion so the propellant budget closes. Every particle depletes proportionally with
the tank.

.. note::

    The physical pendulum mode uses the bare-wall damping ratio 0.0026, and the benchmark
    spring-mass-dampers use the same ratio. Both engines represent the spherical pendulum with two
    angular coordinates. Basilisk applies a viscous force at the bob through the rod moment arm,
    :math:`{\bf l}\times(-d\,{\bf l}')`; the MuJoCo model applies the corresponding generalized
    torques to its two hinges. A real hydrazine tank would carry baffles or a diaphragm and damp
    one to two orders of magnitude harder.

Running it
----------

::

    python3 scenarioCompareVariableMass.py

The scenario builds the same vehicle two ways and overlays them: the Backsubstitution
:ref:`spacecraft` (BSM, used as the plotting baseline) and the MuJoCo
:ref:`MJScene<MJScene>`. Two switches on :func:`run` control what is compared:

- ``useThruster`` (default True) fires the engine; set False to deplete via an equivalent
  prescribed leak rate with no thrust force, isolating pure mass loss.
- ``inOrbit`` (default True) flies the burn under Earth gravity; set False for a deep-space burn
  from rest, which removes gravity as a variable and isolates the variable-mass dynamics.

On the MuJoCo side, depletion is imposed by feeding each body's ``derivativeMassPropertiesInMsg`` a
constant mass-rate (their sum is the engine mass flow), matched to the BSM tank's proportional
depletion. Gravity (:ref:`NBodyGravity`) is applied to *every* massive body so the tree is in
free-fall. The BSM's system-CoM point gravity produces no internal gravity-gradient load, whereas
MuJoCo's per-body point gravity does; thrust and the explicit spring or pendulum laws provide the
remaining internal restoring forces.

The BSM tank uses ``setUpdateOnly(True)``. Under that convention both engines apply the same
nozzle thrust, integrate the retained component masses, and update their instantaneous centers of
mass and inertias. They omit the additional depletion-rate forces and torques caused by the
changing mass distribution and by momentum carried away through an offset nozzle as the
spacecraft rotates. They also do not resolve transport from a specified tank pickup through a
feed path. This is an instantaneous-property model rather than a momentum-closed open-system
model. No empirical cross-engine correction torque is applied.

What the comparison shows
-------------------------

The two engines agree on the depleting masses to grams and track the same rigid-body and slosh
states under the shared instantaneous-property convention. The controlled variants separate two
effects:

#. **In orbit, differential point gravity.** BSM applies gravity once at the system center of mass;
   MuJoCo applies one point force at each body center. The separated forces exert a net torque about
   the system center of mass. Neither treatment resolves gravity variation inside a rigid body.

#. **In deep space, the common variable-mass convention.** Removing gravity tests only depletion,
   thrust, and internal slosh. The full case measures the mapping between BSM's translational and
   spherical-pendulum effectors and MuJoCo's slide and hinge joints, without a fitted external load.
   The ``test_variable_mass_deep_space_numerical_agreement`` regression test also collapses the
   slosh masses and initial displacement to a rigid limit, where the engines agree to numerical
   precision.

The pendulum damping is an internal generalized force in both engines. Motors on the two MuJoCo
hinges apply the torques corresponding to the BSM bob damping force, so the child and parent
receive equal and opposite torques. Applying the same torque only at a body site would remove
angular momentum from the vehicle and would not match BSM.

Illustration of Simulation Results
----------------------------------

Each output file contains one plotting axis. Overlay and difference plots are stored separately.
The burn raises the orbit and draws the tank down. Both engines predict the same qualitative rise
and the same total system mass (dry hub plus depleting propellant), with the mass difference staying
at the gram level. Their final semi-major axes differ by about 825 m after 900 seconds, or 0.34
percent of the roughly 243 km BSM orbit rise. The deep-space control and forcing-scale estimate
indicate that the different gravity application points dominate this residual: BSM applies point
gravity at the system center of mass, while MuJoCo applies it to each body.

.. image:: /_images/Scenarios/scenarioCompareVariableMass_orbit.svg
   :align: center

.. image:: /_images/Scenarios/scenarioCompareVariableMass_orbit_difference.svg
   :align: center

.. image:: /_images/Scenarios/scenarioCompareVariableMass_fuelMass.svg
   :align: center

.. image:: /_images/Scenarios/scenarioCompareVariableMass_fuelMass_difference.svg
   :align: center

The hub body-rate and translational-slosh components are shown separately. The initial rate equals
the orbital pitch rate; it then evolves passively with the slosh motion. Both engines overlay and
the differences stay small.

.. image:: /_images/Scenarios/scenarioCompareVariableMass_rate_x.svg
   :align: center

.. image:: /_images/Scenarios/scenarioCompareVariableMass_rate_y.svg
   :align: center

.. image:: /_images/Scenarios/scenarioCompareVariableMass_rate_z.svg
   :align: center

.. image:: /_images/Scenarios/scenarioCompareVariableMass_slosh_x.svg
   :align: center

.. image:: /_images/Scenarios/scenarioCompareVariableMass_slosh_y.svg
   :align: center

.. image:: /_images/Scenarios/scenarioCompareVariableMass_slosh_z.svg
   :align: center

The attitude and center-of-mass figures show the remaining cross-engine difference over the burn.

.. image:: /_images/Scenarios/scenarioCompareVariableMass_attError.svg
   :align: center

.. image:: /_images/Scenarios/scenarioCompareVariableMass_comError.svg
   :align: center

Runtime cost
------------

Wall-clock propagation cost is reported as the median of five interleaved trials after one
discarded warm-up; model construction and recorder setup are excluded.

Generate the optional local CSV with
``make -C docs comparison-runtime-tables``; see
:ref:`scenarioCompareOrbit` for the benchmark requirements and interpretation.

Next comparison: :ref:`scenarioCompareParetoRwPanels` evaluates accuracy against
runtime across fixed-step and adaptive integrators.

"""

import os

import numpy as np
import matplotlib.pyplot as plt

from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros
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

import _comparePlots
import _comparisonValidation
import _runtimeTable

from Basilisk import hasBuildFeature

couldImportMujoco = hasBuildFeature("mujoco")
if couldImportMujoco:
    from Basilisk.simulation import mujoco
    from Basilisk.simulation import MJSystemCoM
    from Basilisk.simulation import twoHingeDamper

# Paul Tol high-contrast palette shared by the comparison figures.
COLOR_BSM = _comparePlots.COLOR_BSM
COLOR_MUJOCO = _comparePlots.COLOR_MUJOCO

fileName = os.path.basename(os.path.splitext(__file__)[0])

# Folder this scenario writes its JSON summary and reference trajectory into.
resultsPath = os.path.join(os.path.dirname(__file__), "results")

G0 = 9.80665  # [m/s^2] standard gravity used for the Isp-to-mass-flow conversion

# --- Orbit ---------------------------------------------------------------------------------
# A circular parking orbit that the burn raises. The spacecraft starts velocity-aligned with the
# corresponding instantaneous orbital pitch rate, then follows passive coupled dynamics.
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
# the SYSTEM center of mass migrate forward along the thrust axis as the propellant is spent, the
# physically honest source of a moving center of mass.
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
# so the comparison exercises both Basilisk slosh effectors (slide joints and a two-hinge
# pendulum on the MuJoCo side). The classical lateral-slosh EMM has no axial degree of freedom,
# and one 2-DOF pendulum already spans both lateral axes. Their mass comes out of the non-sloshing
# mass so the propellant budget still closes. See the docstring.
SMD_MASS = 25.0  # [kg] each of the three particles
SMD_LATERAL_RHO0 = 0.02  # [m] residual lateral slosh carried into the burn

# P0 frame: rest axis pHat_01 along -z, so the bob hangs aft, the direction the propellant is
# pushed by a +z burn. pHat_03 = pHat_01 x pHat_02 keeps the triad right-handed. A left-handed
# triad silently produces an inverted, exponentially diverging pendulum.
PEND_PHAT_01 = (0.0, 0.0, -1.0)
PEND_PHAT_02 = (1.0, 0.0, 0.0)
PEND_PHAT_03 = (0.0, -1.0, 0.0)
PEND_RATE0 = 0.02  # [rad/s] initial phi and theta rates (residual slosh)
PEND_BOB_INERTIA = 1.0e-4  # [kg*m^2] initial MuJoCo centroidal inertia on all bob axes

# --- Main-engine burn -------------------------------------------------------------------------
# MOOG Monarc-445: a real monopropellant thruster (445 N, Isp 234 s) from the simIncludeThruster
# catalog. Its catalog values are used as-is, giving mDot ~ 0.19 kg/s.
THRUSTER_TYPE = "MOOG_Monarc_445"
THRUST_DIR_B = (0.0, 0.0, 1.0)  # body +z, which starts aligned with the velocity vector
THRUST_POS_B = (0.0, 0.0, -1.5)  # [m] on the nominal tank axis, aft of the dry hub

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


def initialThrustTorqueAboutCoM(maxThrust=None, nearRigid=False):
    """Return the initial body-frame thrust moment about the wet-system center of mass."""
    if maxThrust is None:
        maxThrust, _ = thrusterSpec()
    slosh = sloshParameters(nearRigid)
    wetMass = HUB_MASS + PROPELLANT_MASS  # [kg]
    lateralCoMOffset = np.array([
        slosh["smdMass"]*slosh["lateralRho0"]/wetMass,
        -slosh["smdMass"]*slosh["lateralRho0"]/wetMass,
        0.0,
    ])  # [m]
    leverArm = np.asarray(THRUST_POS_B) - lateralCoMOffset  # [m]
    thrust = maxThrust*np.asarray(THRUST_DIR_B)  # [N]
    return np.cross(leverArm, thrust)  # [N*m]


def earthMu():
    """Earth gravitational parameter [m^3/s^2], from the same body the sim uses."""
    return simIncludeGravBody.gravBodyFactory().createEarth().mu


def sloshParameters(nearRigid=False):
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

    if nearRigid:
        pendMass = 1.0e-6  # [kg] negligible first-mode mass
        smdMass = 1.0e-9  # [kg] negligible fixture-particle mass
        lateralRho0 = 0.0  # [m] no residual lateral displacement
        pendRate0 = 0.0  # [rad/s] no residual pendulum motion
    else:
        pendMass = SLOSH_MASS_FRACTION*PROPELLANT_MASS  # [kg] first-mode slosh mass m1
        smdMass = SMD_MASS  # [kg]
        lateralRho0 = SMD_LATERAL_RHO0  # [m]
        pendRate0 = PEND_RATE0  # [rad/s]
    pendLength = PEND_LENGTH_RATIO*TANK_RADIUS  # [m] equivalent pendulum length L1
    omega1 = np.sqrt(accel/pendLength)  # [rad/s] first-mode slosh frequency (emergent)

    # The benchmarking-fixture particles come out of the non-sloshing mass so the budget closes.
    bulkMass = PROPELLANT_MASS - pendMass - 3.0*smdMass  # [kg] non-sloshing mass m0

    tankVolume = 4.0/3.0*np.pi*TANK_RADIUS**3  # [m^3]
    fillFraction = PROPELLANT_MASS/(PROPELLANT_DENSITY*tankVolume)  # [-]

    return {
        "accel": accel,
        "pendMass": pendMass,
        "pendLength": pendLength,
        "omega1": omega1,
        "bulkMass": bulkMass,
        "fillFraction": fillFraction,
        "smdMass": smdMass,
        "lateralRho0": lateralRho0,
        "pendRate0": pendRate0,
        "nearRigid": bool(nearRigid),
        "smdK": smdMass*omega1**2,  # [N/m] matched to the physical slosh frequency
        "smdC": 2.0*SLOSH_DAMPING_RATIO*smdMass*omega1,  # [N*s/m]
        # The first-mode pendulum carries its physical bare-wall damping. Basilisk's
        # sphericalPendulum applies this as a viscous force at the bob acting through the rod moment
        # arm, l x (-D l'); the MuJoCo velocity servos supply the same transverse joint torque.
        "pendD": 2.0*SLOSH_DAMPING_RATIO*pendMass*omega1,  # pendulum force damping [N*s/m]
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
    """Body attitude and instantaneous rate aligned with the velocity vector at the epoch.

    The body frame is built with z along the velocity (the thrust axis), y along the orbit
    normal, and x completing the triad. The derivative of this alignment at the epoch is a pitch
    about the orbit normal at the orbital rate, which in body components is purely about the body
    y-axis. No controller maintains the alignment during propagation.

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


def buildBSM(dt, record, useThruster=True, inOrbit=True,
             simDuration=SIM_DURATION, nearRigid=False):
    """Build (and initialize) the Backsubstitution variable-mass reference simulation.

    Args:
        dt (float): integrator time step [s]
        record (bool): if True, attach the hub-state, fuel-tank and slosh recorders
        useThruster (bool, optional): if True, deplete the tank with the firing main engine that
            also applies thrust. If False, deplete it with an equivalent prescribed leak rate
            and no thrust force. Defaults to True.
        inOrbit (bool, optional): if True (default) fly the burn on the circular parking orbit
            under point-mass Earth gravity. If False, run the identical vehicle as a deep-space
            burn with no gravity field and starting from rest, which removes the gravity-model
            difference so the two engines agree within the stated acceptance tolerances
            (see :func:`run`).
        simDuration (float, optional): requested propagation horizon [s]. The thruster command
            remains active for at least this duration. Defaults to :data:`SIM_DURATION`.
        nearRigid (bool, optional): if True, retain the same model topology
            with negligible slosh masses and zero residual slosh. Defaults to
            False.

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
    mu = earth.mu  # [m^3/s^2]
    if inOrbit:
        earth.isCentralBody = True
        gravFactory.addBodiesTo(scObject)

    # The orbit geometry always defines the initial attitude and body rate. In deep-space mode the
    # translational state is zeroed: the MuJoCo gyroscopic-bias round-off scales with the hub
    # speed, so starting from rest (rather than at ~7.5 km/s orbital speed) keeps the cross-engine
    # difference at the round-off floor once gravity is off.
    rN, vN = initialOrbitState(mu)
    sigma_BN, omega_BN_B, _ = velocityAlignedAttitude(rN, vN, mu)
    if not inOrbit:
        rN = np.zeros(3)
        vN = np.zeros(3)
    scObject.hub.r_CN_NInit = rN.tolist()  # [m]
    scObject.hub.v_CN_NInit = vN.tolist()  # [m/s]
    scObject.hub.sigma_BNInit = [[c] for c in sigma_BN]
    scObject.hub.omega_BN_BInit = [[c] for c in omega_BN_B]  # [rad/s] orbital pitch rate

    slosh = sloshParameters(nearRigid)

    # Three orthogonal spring-mass-damper particles (a benchmarking fixture, not the physical
    # slosh model, see the docstring). Every equilibrium sits at the tank center because a slosh
    # mass's equilibrium is the propellant center of mass. The lateral pair carries residual slosh
    # from earlier maneuvers, which gives the initial thrust a small physical moment about the
    # instantaneous system CoM. The axial one starts at the offset the thrust settles it to, as a
    # settling burn would leave it.
    smdInit = (
        ((1.0, 0.0, 0.0), slosh["lateralRho0"]),
        ((0.0, 1.0, 0.0), -slosh["lateralRho0"]),
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
        particle.massInit = slosh["smdMass"]  # [kg]
        particles.append(particle)

    # First-mode lateral slosh: a 2-DOF spherical pendulum pivoted at the tank center and hanging
    # aft along the settling axis. This is the physically standard element (Dodge, SwRI 2000).
    pendulum = sphericalPendulum.SphericalPendulum()
    pendulum.pendulumRadius = slosh["pendLength"]  # [m] L1, purely geometric
    pendulum.d = [[c] for c in TANK_R_TB_B]  # [m] pivot at the tank center (correct for a sphere)
    pendulum.D = (slosh["pendD"]*np.eye(3)).tolist()
    # phiInit/thetaInit are not exposed and default to zero, so the bob starts hanging aft.
    pendulum.phiDotInit = slosh["pendRate0"]  # [rad/s] residual slosh
    pendulum.thetaDotInit = slosh["pendRate0"]  # [rad/s]
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
        onTime.OnTimeRequest = [2.0*simDuration]  # [s] keep the engine lit for the whole burn
        thrCmdMsg = messaging.THRArrayOnTimeCmdMsg().write(onTime)
        thruster.cmdsInMsg.subscribeTo(thrCmdMsg)
        # Load the command before the spacecraft evaluates its first integration stage.
        scSim.AddModelToTask("dynTask", thruster, 10)
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
        loggerSpec["bulkMass"] = pendGetter(tank.getNameOfMassState())
        propellantMassStates = [
            tank.getNameOfMassState(),
            *(particle.nameOfMassState for particle in sloshEffectors),
        ]
        loggerSpec["propellantMass"] = lambda _: sum(
            scObject.dynManager.getStateObject(name).getState()[0][0]
            for name in propellantMassStates
        )
        recorders["slosh"] = pythonVariableLogger.PythonVariableLogger(
            loggerSpec, macros.sec2nano(dt))
        scSim.AddModelToTask("dynTask", recorders["slosh"])

    scSim.InitializeSimulation()
    return scSim, recorders, handles


def mujocoModel(pendulumBobInertia=PEND_BOB_INERTIA, nearRigid=False):
    """Return the MJCF model of the variable-mass vehicle for the :ref:`MJScene<MJScene>`.

    The tree mirrors the BSM effector stack: a free-floating ``hub`` carries a rigidly welded
    ``tank`` body (the non-sloshing propellant mass m0), three ``slide``-jointed spring-mass-damper
    particles, and a two-hinge pendulum bob hanging aft along the settling axis. A ``site``
    on the hub marks the main-engine application point, and a ``motor`` acting on it applies the
    thrust along the body +z axis.

    The masses written here are the INITIAL propellant masses. MuJoCo fixes body mass and inertia
    at model-compile time, so the depletion is layered on in :func:`buildMujoco` by feeding each
    propellant body's mass-rate into its ``derivativeMassPropertiesInMsg``: :ref:`MJScene<MJScene>`
    integrates the per-body mass as a state and rescales the body inertia (linearly, with the
    center of mass fixed) every step. That matches the ``FuelTankModelConstantVolume`` law on the
    BSM side, where the tank inertia is ``(2/5) m r^2`` about a fixed centroid.

    Args:
        pendulumBobInertia (float, optional): initial MuJoCo bob inertia about each centroidal
            axis [kg*m^2]. MuJoCo requires this value to be positive; the BSM pendulum treats the
            bob as a point mass. Defaults to :data:`PEND_BOB_INERTIA`.
        nearRigid (bool, optional): if True, use the negligible-mass slosh
            control configuration. Defaults to False.

    Returns:
        str: MJCF XML string.
    """
    if pendulumBobInertia <= 0.0:
        raise ValueError("The MuJoCo pendulum bob inertia must be positive.")

    ix, iy, iz = HUB_INERTIA
    tx, ty, tz = TANK_R_TB_B
    px, py, pz = THRUST_POS_B
    slosh = sloshParameters(nearRigid)
    m0 = slosh["bulkMass"]  # [kg] non-sloshing mass carried rigidly by the tank
    tankInertia = 0.4*m0*TANK_RADIUS**2  # [kg*m^2] (2/5) m0 r^2 solid-sphere inertia
    k = slosh["smdK"]  # [N/m] tuned to the physical first-mode slosh frequency
    c = slosh["smdC"]  # [N*s/m]
    length = slosh["pendLength"]  # [m] pendulum arm L1
    pendMass = slosh["pendMass"]  # [kg] first-mode slosh mass m1
    smdMass = slosh["smdMass"]  # [kg]
    # Point-mass slosh particles and the pendulum bob carry a small positive centroidal inertia
    # because MuJoCo bodies require one. The transverse inertia is dominated by the m*L^2
    # parallel-axis term MuJoCo forms from the offset bob, matching the BSM point-mass model.
    return f"""
<mujoco>
  <!-- No geoms, equalities, limits, or frictionloss in this model: disable the
       collision and constraint pipelines so each derivative evaluation skips
       broadphase and constraint-assembly bookkeeping (bit-identical results). -->
  <option gravity="0 0 0">
    <flag contact="disable" constraint="disable"/>
  </option>
  <worldbody>
    <body name="hub">
      <freejoint/>
      <inertial pos="0 0 0" mass="{HUB_MASS}" fullinertia="{ix} {iy} {iz} 0 0 0"/>
      <site name="thrustPoint" pos="{px} {py} {pz}"/>
      <!-- Non-sloshing propellant mass m0, welded aft of the hub center of mass. -->
      <body name="tank" pos="{tx} {ty} {tz}">
        <inertial pos="0 0 0" mass="{m0}"
                  diaginertia="{tankInertia} {tankInertia} {tankInertia}"/>
      </body>
      <!-- Three orthogonal spring-mass-damper particles as slide joints (springref=0 puts the
           equilibrium at the tank center); genuine dashpots, matching linearSpringMassDamper. -->
      <body name="sloshX" pos="{tx} {ty} {tz}">
        <joint name="sloshX" type="slide" axis="1 0 0" stiffness="{k}" damping="{c}" springref="0"/>
        <inertial pos="0 0 0" mass="{smdMass}" diaginertia="1e-6 1e-6 1e-6"/>
      </body>
      <body name="sloshY" pos="{tx} {ty} {tz}">
        <joint name="sloshY" type="slide" axis="0 1 0" stiffness="{k}" damping="{c}" springref="0"/>
        <inertial pos="0 0 0" mass="{smdMass}" diaginertia="1e-6 1e-6 1e-6"/>
      </body>
      <body name="sloshZ" pos="{tx} {ty} {tz}">
        <joint name="sloshZ" type="slide" axis="0 0 1" stiffness="{k}" damping="{c}" springref="0"/>
        <inertial pos="0 0 0" mass="{smdMass}" diaginertia="1e-6 1e-6 1e-6"/>
      </body>
      <!-- First-mode lateral slosh: two colocated hinges at the tank center with the bob hanging
           aft along -z. Their axes and order reproduce the BSM phi/theta rod direction. The
           restoring force comes from the thrust-induced acceleration, not gravity. -->
      <body name="pendulum" pos="{tx} {ty} {tz}">
        <joint name="pendulumPhi" type="hinge" axis="0 -1 0"/>
        <joint name="pendulumTheta" type="hinge" axis="1 0 0"/>
        <inertial pos="0 0 {-length}" mass="{pendMass}"
                  diaginertia="{pendulumBobInertia} {pendulumBobInertia} {pendulumBobInertia}"/>
      </body>
    </body>
  </worldbody>
  <actuator>
    <motor name="mainEngine" site="thrustPoint" gear="0 0 1 0 0 0"/>
  </actuator>
</mujoco>
"""


def bodyMassFlowRates(nearRigid=False):
    """Constant per-body propellant mass-flow rates for the MuJoCo depletion wiring [kg/s].

    On the BSM side the fuel tank depletes its own bulk mass and every attached slosh particle
    *proportionally* to their current mass, so the mass fractions are exact invariants and each
    body drains at a constant rate equal to its initial mass fraction times the total mass flow
    (see :func:`pullBSM`). MuJoCo has no equivalent coupling, so the same schedule is reproduced
    directly: each body is given a constant, negative mass-rate feeding its
    ``derivativeMassPropertiesInMsg``, and the rates sum to the engine's total mass flow.

    Returns:
        dict: body name -> mass-rate [kg/s] (negative), for ``tank`` and the four slosh bodies.
    """
    maxThrust, steadyIsp = thrusterSpec()
    totalFlow = nominalMassFlow(maxThrust, steadyIsp)  # [kg/s] engine propellant mass flow
    slosh = sloshParameters(nearRigid)
    # Initial mass fraction of each body within the PROPELLANT (the tank bulk mass plus every
    # slosh mass equals PROPELLANT_MASS; the dry hub does not deplete).
    rates = {"tank": -totalFlow*slosh["bulkMass"]/PROPELLANT_MASS}
    for name in ("sloshX", "sloshY", "sloshZ"):
        rates[name] = -totalFlow*slosh["smdMass"]/PROPELLANT_MASS
    rates["pendulum"] = -totalFlow*slosh["pendMass"]/PROPELLANT_MASS
    return rates


def buildMujoco(dt, record, initialState, useThruster=True, inOrbit=True,
                 pendulumBobInertia=PEND_BOB_INERTIA, nearRigid=False):
    """Build (and initialize) the MuJoCo variable-mass simulation.

    The MJCF tree from :func:`mujocoModel` is fixed-mass; this function layers on the depletion
    (a constant mass-rate per body, from :func:`bodyMassFlowRates`), the point-mass gravity on every
    body, and the thrust (a constant ``SingleActuatorMsg`` on the ``mainEngine`` motor, omitted when
    ``useThruster`` is False) -- see the module docstring for why each is done this way. Two build
    Initial conditions are all set *after* ``InitializeSimulation`` and taken from
    ``initialState`` so both engines start identically.

    Args:
        dt (float): integrator time step [s]
        record (bool): if True, attach the hub-state, system-center-of-mass and slosh recorders
        initialState (dict): initial ``r_BN_N`` [m], ``v_BN_N`` [m/s], ``sigma_BN`` and
            ``omega_BN_B`` [rad/s] of the hub-origin frame, taken from the BSM reference so both
            engines start from the identical state.
        useThruster (bool, optional): if True (default) fire the main engine; if False, deplete
            with no thrust force, matching the ``useThruster=False`` BSM leak-rate reference.
        inOrbit (bool, optional): if True (default) add the point-mass gravity field; if False,
            omit it for the deep-space burn, matching :func:`buildBSM`.
        pendulumBobInertia (float, optional): initial MuJoCo bob inertia about each centroidal
            axis [kg*m^2]. Defaults to :data:`PEND_BOB_INERTIA`.
        nearRigid (bool, optional): if True, use the negligible-mass slosh
            control configuration. Defaults to False.
    Returns:
        tuple: ``(scSim, recorders, handles)`` matching :func:`buildBSM`.
    """
    if not couldImportMujoco:
        raise ImportError("Build Basilisk with --mujoco to run the MuJoCo comparison side.")

    scSim = SimulationBaseClass.SimBaseClass()
    process = scSim.CreateNewProcess("dyn")
    process.addTask(scSim.CreateNewTask("dynTask", macros.sec2nano(dt)))

    slosh = sloshParameters(nearRigid)
    scene = mujoco.MJScene(mujocoModel(pendulumBobInertia, nearRigid))
    scene.ModelTag = "hubMj"
    scene.extraEoMCall = True
    # Advance the hub free-joint quaternion at the integrator's full order: the slosh coupling
    # continually turns the hub rate vector, so MuJoCo's default second-order attitude step would
    # dominate the cross-engine attitude difference.
    scene.highOrderAttitudeIntegration = True
    scSim.AddModelToTask("dynTask", scene, 2)

    integrator = svIntegrators.svIntegratorRK4(scene)
    scene.setIntegrator(integrator)

    hub = scene.getBody("hub")
    pendulumBody = scene.getBody("pendulum")
    phiJoint = pendulumBody.getScalarJoint("pendulumPhi")
    thetaJoint = pendulumBody.getScalarJoint("pendulumTheta")

    handles = [scene, integrator]
    if inOrbit:
        # The factory uses the same Earth descriptor as the BSM model, creates
        # NBodyGravity, and registers every scene body as a gravity target.
        gravFactory = simIncludeGravBody.gravBodyFactory()
        earth = gravFactory.createEarth()
        earth.isCentralBody = True
        gravity = gravFactory.addBodiesTo(scene)
        handles += [gravFactory, gravity]

    # Deplete each propellant body by driving a constant negative mass-rate into its mass-state
    # derivative. The dry hub is not registered, so it does not deplete.
    massRateMsgs = []
    for name, rate in bodyMassFlowRates(nearRigid).items():
        payload = messaging.SCMassPropsMsgPayload()
        payload.massSC = rate  # [kg/s]
        rateMsg = messaging.SCMassPropsMsg().write(payload)
        scene.getBody(name).derivativeMassPropertiesInMsg.subscribeTo(rateMsg)
        massRateMsgs.append(rateMsg)

    handles += massRateMsgs
    if useThruster:
        maxThrust, _ = thrusterSpec()
        thrustMsg = messaging.SingleActuatorMsg().write(
            messaging.SingleActuatorMsgPayload(input=maxThrust))  # [N] constant main-engine thrust
        scene.getSingleActuator("mainEngine").actuatorInMsg.subscribeTo(thrustMsg)
        handles.append(thrustMsg)

    # Match the Cartesian bob damping used by sphericalPendulum. In these coordinates,
    # Q_phi=-d*L^2*cos(theta)^2*phiDot and Q_theta=-d*L^2*thetaDot.
    pendulumDamper = twoHingeDamper.TwoHingeDamper()
    pendulumDamper.ModelTag = "pendulumDamper"
    pendulumDamper.dampingCoeff = slosh["pendD"]*slosh["pendLength"]**2
    pendulumDamper.applyTo(phiJoint, thetaJoint)
    scene.AddModelToDynamicsTask(pendulumDamper)
    handles.append(pendulumDamper)

    # The system center of mass, extracted directly from the MuJoCo tree, is the analogue of the
    # BSM hub state's r_CN_N/v_CN_N so the orbit can be compared across engines.
    systemCoM = MJSystemCoM.MJSystemCoM()
    systemCoM.ModelTag = "systemCoM"
    systemCoM.scene = scene
    scSim.AddModelToTask("dynTask", systemCoM, 1)
    handles.append(systemCoM)

    recorders = {}
    if record:
        recorders["state"] = hub.getOrigin().stateOutMsg.recorder(macros.sec2nano(dt))
        recorders["com"] = systemCoM.comStatesOutMsg.recorder(macros.sec2nano(dt))
        recorders["mass"] = {name: scene.getBody(name).massPropertiesOutMsg.recorder(
            macros.sec2nano(dt)) for name in ("tank", "sloshX", "sloshY", "sloshZ", "pendulum")}
        recorders["slosh"] = {name: scene.getBody(name).getScalarJoint(name).stateOutMsg.recorder(
            macros.sec2nano(dt)) for name in ("sloshX", "sloshY", "sloshZ")}
        recorders["pendulum"] = {
            "phi": phiJoint.stateOutMsg.recorder(macros.sec2nano(dt)),
            "theta": thetaJoint.stateOutMsg.recorder(macros.sec2nano(dt)),
        }
        scSim.AddModelToTask("dynTask", recorders["state"], 0)
        scSim.AddModelToTask("dynTask", recorders["com"], 0)
        for rec in recorders["pendulum"].values():
            scSim.AddModelToTask("dynTask", rec, 0)
        for rec in recorders["mass"].values():
            scSim.AddModelToTask("dynTask", rec, 0)
        for rec in recorders["slosh"].values():
            scSim.AddModelToTask("dynTask", rec, 0)

    scSim.InitializeSimulation()

    # Free-body initial conditions are set after initialization, matching the BSM start exactly.
    hub.setPosition(list(initialState["r_BN_N"]))  # [m] hub-origin inertial position
    hub.setVelocity(list(initialState["v_BN_N"]))  # [m/s]
    hub.setAttitude(list(initialState["sigma_BN"]))
    hub.setAttitudeRate(list(initialState["omega_BN_B"]))

    # Seed the residual slosh. The spring-mass-damper particles take their initial displacement
    # through the slide-joint position setters; the lateral pair carries residual slosh and the
    # axial one starts at the settled offset -a/omega1^2, exactly as on the BSM side.
    smdInit = {"sloshX": slosh["lateralRho0"],
               "sloshY": -slosh["lateralRho0"],
               "sloshZ": slosh["axialRho0"]}
    for name, rho0 in smdInit.items():
        scene.getBody(name).getScalarJoint(name).setPosition(rho0)  # [m]

    # Seed the same residual slosh rates as the BSM pendulum.
    phiJoint.setVelocity(slosh["pendRate0"])  # [rad/s]
    thetaJoint.setVelocity(slosh["pendRate0"])  # [rad/s]

    return scSim, recorders, handles


def relativePrincipalAngle(sigmaA, sigmaB):
    """Per-sample principal rotation angle between two MRP attitude histories.

    Args:
        sigmaA (numpy.ndarray): first MRP history, shape ``(N, 3)``
        sigmaB (numpy.ndarray): second MRP history, shape ``(N, 3)``

    Returns:
        numpy.ndarray: principal angle per sample [rad].
    """
    nSamples = _comparisonValidation.requireEqualLength(
        "variable-mass attitude", sigmaA, sigmaB)
    angle = np.empty(nSamples)
    for i in range(nSamples):
        dcmRel = rbk.MRP2C(sigmaA[i]).dot(rbk.MRP2C(sigmaB[i]).T)
        # 4*atan(|sigma_rel|) is well conditioned down to machine precision, unlike
        # arccos((trace-1)/2) which collapses to zero below ~1e-8 rad.
        angle[i] = 4.0*np.arctan(np.linalg.norm(rbk.C2MRP(dcmRel)))
    return angle


def gravityGradientRateEstimate(mu):
    """Standard gravity-gradient angular-acceleration scale ``3 (mu/r^3) dI/I`` [rad/s^2].

    This is the order-of-magnitude rate at which the MuJoCo hub attitude (per-body gravity, which
    carries a gravity-gradient torque) departs from the BSM hub attitude (single-point gravity at
    the system center of mass, which does not). The inertia spread is dominated by the tank and
    propellant mounted an arm ``TANK_R_TB_B`` off the hub center of mass.

    Args:
        mu (float): gravitational parameter [m^3/s^2]

    Returns:
        float: angular-acceleration scale [rad/s^2].
    """
    arm = np.linalg.norm(TANK_R_TB_B)  # [m] tank offset from the hub center of mass
    # [kg*m^2] parallel-axis spread from the offset propellant
    deltaInertia = PROPELLANT_MASS*arm**2
    meanInertia = np.mean(HUB_INERTIA)  # [kg*m^2]
    return 3.0*(mu/ORBIT_A**3)*deltaInertia/meanInertia  # [rad/s^2]


def pullBSM(recorders, mu):
    """Collect the BSM reference histories into a plain dictionary of numpy arrays.

    Args:
        recorders (dict): the recorder dict returned by :func:`buildBSM`
        mu (float): gravitational parameter [m^3/s^2]

    Returns:
        dict: time, hub state, orbit, tank mass and slosh-state histories.
    """
    stateRec = recorders["state"]
    sloshRec = recorders["slosh"]

    rBN = np.array(stateRec.r_BN_N)  # [m]
    vBN = np.array(stateRec.v_BN_N)  # [m/s]
    rCN = np.array(stateRec.r_CN_N)  # [m] system center of mass
    vCN = np.array(stateRec.v_CN_N)  # [m/s]
    rMag = np.linalg.norm(rCN, axis=1)  # [m]
    vMag = np.linalg.norm(vCN, axis=1)  # [m/s]
    with np.errstate(divide="ignore", invalid="ignore"):
        semiMajorAxis = 1.0/(2.0/rMag - vMag**2/mu)  # [m] vis-viva

    # Read mass from the integrated states rather than the tank output message, which is written
    # during the dynamics evaluation and can contain an intermediate Runge--Kutta stage value.
    fuelMass = np.array(sloshRec.bulkMass)  # [kg] non-sloshing mass m0 history
    totalMass = HUB_MASS + np.array(sloshRec.propellantMass)  # [kg]

    return {
        "t": np.array(stateRec.times())*macros.NANO2SEC,
        "sigma_BN": np.array(stateRec.sigma_BN),
        "omega_BN_B": np.array(stateRec.omega_BN_B),
        "r_BN_N": rBN,
        "v_BN_N": vBN,
        "r_CN_N": rCN,
        "v_CN_N": vCN,
        "semiMajorAxis": semiMajorAxis,
        "fuelMass": fuelMass,
        "totalMass": totalMass,
        "rho": np.column_stack([sloshRec.rho1, sloshRec.rho2, sloshRec.rho3]),
        "phi": np.array(sloshRec.phi),
        "theta": np.array(sloshRec.theta),
    }


def pullMujoco(recorders, mu):
    """Collect the MuJoCo histories into a dictionary matching :func:`pullBSM`.

    The hub-origin frame is read from the free-joint site, the system center of mass from
    :ref:`MJSystemCoM`, and the propellant masses and slosh displacements from the per-body mass
    and slide-joint recorders. The total system mass is summed from the dry hub and the recorded
    depleting propellant-body masses, the direct analogue of the BSM ``totalMass``.

    Args:
        recorders (dict): the recorder dict returned by :func:`buildMujoco`
        mu (float): gravitational parameter [m^3/s^2]

    Returns:
        dict: time, hub state, system-center-of-mass orbit, mass and slosh histories.
    """
    stateRec = recorders["state"]
    comRec = recorders["com"]

    rCN = np.array(comRec.r_CN_N)  # [m] system center of mass, MuJoCo per-body gravity
    vCN = np.array(comRec.v_CN_N)  # [m/s]
    rMag = np.linalg.norm(rCN, axis=1)  # [m]
    vMag = np.linalg.norm(vCN, axis=1)  # [m/s]
    with np.errstate(divide="ignore", invalid="ignore"):
        semiMajorAxis = 1.0/(2.0/rMag - vMag**2/mu)  # [m] vis-viva

    tankMass = np.array(recorders["mass"]["tank"].massSC)  # [kg] non-sloshing mass m0
    totalMass = HUB_MASS + tankMass.copy()
    for name in ("sloshX", "sloshY", "sloshZ", "pendulum"):
        totalMass = totalMass + np.array(recorders["mass"][name].massSC)

    return {
        "t": np.array(stateRec.times())*macros.NANO2SEC,
        "sigma_BN": np.array(stateRec.sigma_BN),
        "omega_BN_B": np.array(stateRec.omega_BN_B),
        "r_BN_N": np.array(stateRec.r_BN_N),
        "v_BN_N": np.array(stateRec.v_BN_N),
        "r_CN_N": rCN,
        "v_CN_N": vCN,
        "semiMajorAxis": semiMajorAxis,
        "fuelMass": tankMass,
        "totalMass": totalMass,
        "rho": np.column_stack([np.array(recorders["slosh"][name].state)
                                for name in ("sloshX", "sloshY", "sloshZ")]),
        "phi": np.asarray(recorders["pendulum"]["phi"].state),
        "theta": np.asarray(recorders["pendulum"]["theta"].state),
    }


def run(showPlots=False, saveJson=False, simDuration=SIM_DURATION, useThruster=True,
        inOrbit=True, saveReference=False, saveTiming=False, nearRigid=False,
        resultsDir=None):
    """Main function, see scenario description.

    Args:
        showPlots (bool, optional): if True, plot and show the simulation results.
            Defaults to False.
        saveJson (bool, optional): if True, write scalar comparison metrics to
            ``results/scenarioCompareVariableMass.json`` in orbit or
            ``results/scenarioCompareVariableMass_deepSpace.json`` in deep space.
            Defaults to False.
        simDuration (float, optional): burn/comparison window [s]. Defaults to ``SIM_DURATION``.
        useThruster (bool, optional): if True (default) deplete the tank with the firing main
            engine. If False, use an equivalent prescribed leak rate with no thrust force.
        inOrbit (bool, optional): if True (default) fly under Earth gravity (engines separate by the
            gravity gradient); if False, run a deep-space burn from rest that isolates the shared
            instantaneous-property depletion and internal slosh model.
        saveReference (bool, optional): if True, write the BSM ground-truth trajectory to
            ``results/scenarioCompareVariableMass_reference.npz``. Defaults to False.
        saveTiming (bool, optional): if True, measure the BSM-vs-MJScene wall-clock cost with the
            selected MuJoCo model and write
            ``results/scenarioCompareVariableMass_runtime.csv``. Defaults to False.
        nearRigid (bool, optional): if True, retain the same topology with
            negligible slosh masses and zero residual slosh. This control is
            available only in deep space. Defaults to False.
        resultsDir (str, optional): explicit artifact directory. Defaults to
            the scenario ``results`` folder.

    Returns:
        dict: mapping from figure name to matplotlib figure.
    """
    if nearRigid and inOrbit:
        raise ValueError("The near-rigid control is defined only in deep space.")

    dt = timeStep()  # [s]
    maxThrust, steadyIsp = thrusterSpec()
    slosh = sloshParameters(nearRigid)
    targetResults = resultsPath if resultsDir is None else resultsDir

    bsmSim, bsmRec, _ = buildBSM(
        dt, True, useThruster, inOrbit, simDuration, nearRigid)
    mu = earthMu()  # [m^3/s^2]
    bsmSim.ConfigureStopTime(macros.sec2nano(simDuration))
    bsmSim.ExecuteSimulation()
    bsm = pullBSM(bsmRec, mu)
    _comparisonValidation.validateHistory(
        "variable-mass BSM", bsm["t"], simDuration, dt,
        requireFinalSample=True,
        attitude=bsm["sigma_BN"], rate=bsm["omega_BN_B"],
        position=bsm["r_CN_N"], mass=bsm["totalMass"], slosh=bsm["rho"],
        pendulumPhi=bsm["phi"], pendulumTheta=bsm["theta"])

    resultStem = fileName if inOrbit else fileName+"_deepSpace"
    if nearRigid:
        resultStem += "_rigid"
    initialThrustTorque = (
        initialThrustTorqueAboutCoM(maxThrust, nearRigid)
        if useThruster
        else np.zeros(3)
    )  # [N*m]
    metrics = {
        "scenario": resultStem,
        "timeStep": dt,
        "simDuration": simDuration,
        "useThruster": useThruster,
        "inOrbit": inOrbit,
        "nearRigid": nearRigid,
        "thruster": THRUSTER_TYPE,
        "maxThrust": maxThrust,
        "steadyIsp": steadyIsp,
        "nominalMassFlow": nominalMassFlow(maxThrust, steadyIsp),
        "initialThrustTorqueAboutCoM": initialThrustTorque.tolist(),
        "initialThrustTorqueMagnitude": float(np.linalg.norm(initialThrustTorque)),
        "initialPendulumBobInertia": PEND_BOB_INERTIA,
        "initialPendulumMass": slosh["pendMass"],
        "initialSmdMass": slosh["smdMass"],
        "propellantDepletedFraction": float(1.0 - bsm["fuelMass"][-1]/bsm["fuelMass"][0]),
        "wetMassChangeFraction": float(1.0 - bsm["totalMass"][-1]/bsm["totalMass"][0]),
        "deltaV": float(np.linalg.norm(bsm["v_CN_N"][-1]) - np.linalg.norm(bsm["v_CN_N"][0])),
    }
    if inOrbit:
        metrics["semiMajorAxisRise"] = float(
            bsm["semiMajorAxis"][-1] - bsm["semiMajorAxis"][0])

    # Start the MuJoCo run from the BSM hub-origin state at t=0 so both engines begin from the
    # identical state (feeding r_CN_NInit to the hub origin would offset them by the tank arm).
    initialState = {"r_BN_N": bsm["r_BN_N"][0], "v_BN_N": bsm["v_BN_N"][0],
                    "sigma_BN": bsm["sigma_BN"][0], "omega_BN_B": bsm["omega_BN_B"][0]}
    mj = None
    if couldImportMujoco:
        mjSim, mjRec, _ = buildMujoco(
            dt, True, initialState, useThruster, inOrbit,
            nearRigid=nearRigid)
        mjSim.ConfigureStopTime(macros.sec2nano(simDuration))
        mjSim.ExecuteSimulation()
        mj = pullMujoco(mjRec, mu)
        _comparisonValidation.validateMatchingHistories(
            "variable-mass BSM/MuJoCo",
            bsm["t"], mj["t"], simDuration, dt, requireFinalSample=True)
        _comparisonValidation.validateHistory(
            "variable-mass MuJoCo", mj["t"], simDuration, dt,
            requireFinalSample=True,
            attitude=mj["sigma_BN"], rate=mj["omega_BN_B"],
            position=mj["r_CN_N"], mass=mj["totalMass"], slosh=mj["rho"],
            pendulumPhi=mj["phi"], pendulumTheta=mj["theta"])
        attError = relativePrincipalAngle(bsm["sigma_BN"], mj["sigma_BN"])  # [rad]
        comError = np.linalg.norm(bsm["r_CN_N"] - mj["r_CN_N"], axis=1)  # [m]
        rateError = np.linalg.norm(
            bsm["omega_BN_B"] - mj["omega_BN_B"], axis=1)  # [rad/s]
        metrics["attitudeErrorMax"] = float(np.max(attError))
        metrics["comPositionErrorMax"] = float(np.max(comError))
        metrics["rateErrorMax"] = float(np.max(rateError))
        metrics["totalMassErrorMax"] = float(
            np.max(np.abs(bsm["totalMass"] - mj["totalMass"])))
        metrics["sloshDisplacementErrorMax"] = float(
            np.max(np.abs(bsm["rho"] - mj["rho"])))
        pendulumError = np.column_stack((
            np.arctan2(
                np.sin(bsm["phi"] - mj["phi"]),
                np.cos(bsm["phi"] - mj["phi"]),
            ),
            np.arctan2(
                np.sin(bsm["theta"] - mj["theta"]),
                np.cos(bsm["theta"] - mj["theta"]),
            ),
        ))  # [rad]
        metrics["pendulumAngleErrorMax"] = float(np.max(np.abs(pendulumError)))
        if inOrbit:
            # Compare the residual with the gravity-gradient torque scale introduced by MuJoCo's
            # per-body gravity and omitted by BSM's single-point gravity. This scale supports a
            # dominant-mechanism interpretation but does not isolate every residual contribution.
            metrics["gravityGradientRateEstimate"] = float(gravityGradientRateEstimate(mu))
            metrics["semiMajorAxisRiseMujoco"] = float(
                mj["semiMajorAxis"][-1] - mj["semiMajorAxis"][0])
            metrics["semiMajorAxisDifferenceFinal"] = float(abs(
                bsm["semiMajorAxis"][-1] - mj["semiMajorAxis"][-1]))
            metrics["semiMajorAxisDifferenceAsFractionOfRise"] = float(
                metrics["semiMajorAxisDifferenceFinal"]
                / abs(metrics["semiMajorAxisRise"]))

    if saveTiming:
        def buildTimedBSM():
            simulation, recorders, handles = buildBSM(
                dt, False, useThruster, inOrbit, simDuration, nearRigid)
            simulation.ConfigureStopTime(macros.sec2nano(simDuration))
            return simulation, recorders, handles

        def buildTimedMujoco():
            simulation, recorders, handles = buildMujoco(
                dt, False, initialState, useThruster, inOrbit,
                nearRigid=nearRigid)
            simulation.ConfigureStopTime(macros.sec2nano(simDuration))
            return simulation, recorders, handles

        regime = "orbit" if inOrbit else "deep space"
        casePrefix = (
            f"Variable-mass hub + slosh ({simDuration:g} s, RK4 dt={dt:g} s), "
            f"{regime}"
        )
        if couldImportMujoco:
            bsmSeconds, mujocoSeconds = _runtimeTable.pairedPropagationTimes(
                buildTimedBSM, buildTimedMujoco)
            timingRows = [(casePrefix, bsmSeconds, mujocoSeconds)]
        else:
            bsmSeconds = _runtimeTable.medianPropagationTime(buildTimedBSM)
            timingRows = [(casePrefix, bsmSeconds, None)]
        _runtimeTable.saveRuntimeTable(
            fileName, os.path.dirname(__file__), timingRows,
            resultsDir=targetResults)

    if saveReference:
        os.makedirs(targetResults, exist_ok=True)
        np.savez(os.path.join(
            targetResults, resultStem+"_reference.npz"), **bsm)

    if saveJson:
        import json
        os.makedirs(targetResults, exist_ok=True)
        with open(os.path.join(targetResults, resultStem+".json"), "w") as f:
            json.dump(metrics, f, indent=2)

    figureList = plotResults(bsm, mj, inOrbit, figureStem=resultStem)

    _comparePlots.finalizeFigures(figureList)
    if showPlots:
        plt.show()
    plt.close("all")

    return figureList


def plotResults(bsm, mj=None, inOrbit=True, figureStem=fileName):
    """Build the scenario figures.

    The BSM curves are drawn as thick translucent underlays and the MuJoCo curves as thin lines on
    top, so the two engines stay distinguishable where they overlap. A dedicated figure shows the
    cross-engine attitude difference: in orbit it is dominated by the gravity-gradient torque the
    two gravity models treat differently; in deep space it measures the residual under the shared
    instantaneous-property and internal-damping conventions.

    Args:
        bsm (dict): BSM reference histories from :func:`pullBSM`
        mj (dict, optional): MuJoCo histories from :func:`pullMujoco`, or None when MuJoCo is
            unavailable
        inOrbit (bool, optional): whether the run was on orbit. Selects the first figure between
            the orbit-raise plot (in orbit) and the burn-speed plot (deep space).
        figureStem (str, optional): output-name prefix, used to distinguish orbit and deep-space
            runs.

    Returns:
        dict: mapping from figure name to matplotlib figure.
    """
    t = bsm["t"]
    figureList = {}
    if mj is not None:
        _comparisonValidation.validateMatchingHistories(
            "variable-mass plot", t, mj["t"])

    # Orbit-raise (in orbit) or accumulated burn speed (deep space): both engines with the
    # difference below.
    if inOrbit:
        yBSM = (bsm["semiMajorAxis"] - bsm["semiMajorAxis"][0])*1e-3  # [km]
        yMj = None if mj is None else (mj["semiMajorAxis"] - mj["semiMajorAxis"][0])*1e-3
        figureList.update(_comparePlots.scalarComparison(
            figureStem+"_orbit", t, yBSM, yMj, "Semi-major axis rise [km]"))
    else:
        yBSM = np.linalg.norm(bsm["v_CN_N"], axis=1) - np.linalg.norm(bsm["v_CN_N"][0])  # [m/s]
        yMj = (None if mj is None
               else np.linalg.norm(mj["v_CN_N"], axis=1) - np.linalg.norm(mj["v_CN_N"][0]))
        figureList.update(_comparePlots.scalarComparison(
            figureStem+"_burnSpeed", t, yBSM, yMj, "Burn speed [m/s]"))

    figureList.update(_comparePlots.scalarComparison(
        figureStem+"_fuelMass", t, bsm["totalMass"], None if mj is None else mj["totalMass"],
        "Total mass [kg]", diffUnit="g", diffScale=1e3))

    figureList.update(_comparePlots.componentComparison(
        figureStem+"_rate", t, bsm["omega_BN_B"], None if mj is None else mj["omega_BN_B"],
        r"$\omega_{BN}$", "mrad/s", scale=1e3))

    figureList.update(_comparePlots.componentComparison(
        figureStem+"_slosh", t, bsm["rho"], None if mj is None else mj["rho"],
        r"$\rho$", "mm", scale=1e3))

    if mj is not None:
        attError = relativePrincipalAngle(bsm["sigma_BN"], mj["sigma_BN"])  # [rad]
        comError = np.linalg.norm(bsm["r_CN_N"] - mj["r_CN_N"], axis=1)  # [m]

        figAtt, axAtt = plt.subplots(figsize=(4.8, 2.4), layout="constrained")
        axAtt.semilogy(t, np.maximum(np.degrees(attError), np.degrees(1e-12)),
                       color=COLOR_MUJOCO)
        axAtt.set_xlabel("Time [s]")
        axAtt.set_ylabel("Attitude difference [deg]")
        figureList[figureStem+"_attError"] = figAtt

        figCoM, axCoM = plt.subplots(figsize=(4.8, 2.4), layout="constrained")
        axCoM.plot(t, comError, color=COLOR_MUJOCO)
        axCoM.set_xlabel("Time [s]")
        axCoM.set_ylabel("CoM difference [m]")
        figureList[figureStem+"_comError"] = figCoM

    return figureList


if __name__ == "__main__":
    run(showPlots=True, saveReference=True)
