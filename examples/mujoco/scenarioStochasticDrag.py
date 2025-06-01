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
"""
This scenario uses :ref:`MJScene<MJScene>` as its ``DynamicObject``, although it is
not its main focus. For context on dynamics using :ref:`MJScene<MJScene>`, check out:

#. ``examples/mujoco/scenarioReactionWheel.py``

This scripts shows how to simulate a point-mass spacecraft using :ref:`MJScene<MJScene>`
(mujoco-based dynamics) with cannonball drag force and a stochastic atmospheric
density model. The script illustrates how one defines and handles dynamic states
that are driven by stochastic terms.

The spacecraft definition is given in ``CANNONBALL_SCENE_XML``; it contains a single
body for which we set its mass directly. We use the :ref:`NBodyGravity` model to compute
the gravity acting on this body due to a point-mass Earth.

The drag force on acting on the body is computed in the ``CannonballDrag`` model
defined in this scenario. This takes as input the state of the spacecraft (so that
its velocity and orientation) and the atmospheric density. It outputs a force vector
on the body-fixed reference frame (more accurately, the body's center of mass frame/site).

The atmospheric density used on the drag model is given by::

    densityStochastic = densityExponential * (1 + densityCorrection)

where ``densityExponential`` is computed by the :ref:`exponentialAtmosphere` model
while ``densityCorrection`` is a stochastic process centered at zero. This process
is modeled as an Ornstein-Uhlenbeck (OU) process, whose Stochastic Differential
Equation (SDE) is given by::

    d(densityCorrection) = -theta*densityCorrection*dt + sigma*dW

where ``theta`` and ``sigma`` are the terms that characterize the OU process.
In the SDE above, ``-theta*densityCorrection`` represents the 'drift' term (the
classical derivative). ``sigma``, on the other hand, represents the 'diffusion'
term, which maps the influence of the noise to the state.

The ``densityStochastic`` and ``densityCorrection`` are computed in
``StochasticAtmosphere``. ``densityCorrection`` must be modeled as a stochastic
dynamic state (since its evolution is given by its derivative and noise diffusion).

Illustration of Simulation Results
----------------------------------
The following images illustrate a possible simulation result.

The orbit is plotted in the orbital plane:

.. image:: /_images/Scenarios/scenarioStochasticDrag_orbit.svg
   :align: center

The altitude as a function of time is plotted.

.. image:: /_images/Scenarios/scenarioStochasticDrag_altitude.svg
   :align: center

The atmospheric density as a function of altitude is plotted in lin-log space.
This shows two lines: the deterministic, exponential density (should appear
linear); and the stochastic density.

.. image:: /_images/Scenarios/scenarioStochasticDrag_density.svg
   :align: center

The atmospheric density correction, which should have a standard deviation
of 0.15.

.. image:: /_images/Scenarios/scenarioStochasticDrag_densityDiff.svg
   :align: center

The magnitude of drag force over time is plotted in lin-log space.

.. image:: /_images/Scenarios/scenarioStochasticDrag_drag.svg
   :align: center

"""

from Basilisk.architecture import sysModel
from Basilisk.architecture import messaging
from Basilisk.simulation import mujoco
from Basilisk.simulation import StatefulSysModel
from Basilisk.simulation import dynParamManager
from Basilisk.simulation import svIntegrators
from Basilisk.simulation import pointMassGravityModel
from Basilisk.simulation import NBodyGravity
from Basilisk.simulation import exponentialAtmosphere
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros
from Basilisk.utilities import orbitalMotion
from Basilisk.utilities import simIncludeGravBody
from Basilisk.utilities import simSetPlanetEnvironment
from Basilisk.utilities import RigidBodyKinematics as rbk

import numpy as np
import matplotlib.pyplot as plt

"""An cannon-ball spacecraft.

We set its inertial properties directly, instead of defining them
through their geometry. The spacecraft has no sub-bodies or any
moving parts.
"""
CANNONBALL_SCENE_XML = r"""
<mujoco>
	<worldbody>
		<body name="ball">
			<freejoint/>
			<inertial pos="0 0 0" mass="1" diaginertia="1 1 1"/>
		</body>
	</worldbody>
</mujoco>
"""

def run(showPlots: bool = False):
    """Main function, see scenario description.

    Args:
        showPlots (bool, optional): If True, simulation results are plotted and show.
            Defaults to False.
    """
    initialAlt = 250 # km
    planet = simIncludeGravBody.BODY_DATA["earth"]

    # Set up a circular orbit using classical orbit elements
    oe = orbitalMotion.ClassicElements()
    oe.a = planet.radEquator + initialAlt * 1000  # meters
    oe.e = 0
    oe.i = 33.3 * macros.D2R
    oe.Omega = 48.2 * macros.D2R
    oe.omega = 347.8 * macros.D2R
    oe.f = 85.3 * macros.D2R
    rN, vN = orbitalMotion.elem2rv(planet.mu, oe)
    oe = orbitalMotion.rv2elem(planet.mu, rN, vN)
    orbitPeriod = 2. * np.pi / np.sqrt(planet.mu / oe.a**3)

    dt = 10 # s
    tf = 7.1 * orbitPeriod # s

    # Create a simulation, process, and task as usual
    scSim = SimulationBaseClass.SimBaseClass()
    process = scSim.CreateNewProcess("test")
    process.addTask(scSim.CreateNewTask("test", macros.sec2nano(dt)))

    # Create the Mujoco scene (our MuJoCo DynamicObject)
    # Load the XML file that defines the system from a file
    scene = mujoco.MJScene(CANNONBALL_SCENE_XML)
    scSim.AddModelToTask("test", scene)

    # Set the integrator of the DynamicObject to W2Ito2
    # This is a stochastic integrator, necessary since we have
    # states (related to density) that are driven by stochastic
    # dynamics
    integ = svIntegrators.svStochIntegratorW2Ito2(scene)
    scene.setIntegrator(integ)

    ### Get mujoco body and site (point, frame) of interest
    body: mujoco.MJBody = scene.getBody("ball")
    dragPoint: mujoco.MJSite = body.getCenterOfMass()

    ### Create the NBodyGravity model
    # add model to the dynamics task of the scene
    gravity = NBodyGravity.NBodyGravity()
    scene.AddModelToDynamicsTask(gravity)

    # Create a point-mass gravity source
    gravityModel = pointMassGravityModel.PointMassGravityModel()
    gravityModel.muBody = planet.mu
    gravity.addGravitySource("earth", gravityModel, True)

    # Create a gravity target from the mujoco body
    gravity.addGravityTarget("ball", body)

    ### Create the density model
    atmo = exponentialAtmosphere.ExponentialAtmosphere()
    atmo.ModelTag = "ExpAtmo"
    simSetPlanetEnvironment.exponentialAtmosphere(atmo, "earth")
    atmo.addSpacecraftToModel(dragPoint.stateOutMsg)

    # Will be updated with the task period
    scSim.AddModelToTask("test", atmo)

    stochasticAtmo = StochasticAtmosphere(
        densityCorrectionStandardDeviation=0.15,
        densityCorrectionTheta=.01
    )
    stochasticAtmo.ModelTag = "StochasticExpAtmo"

    stochasticAtmo.atmoDensInMsg.subscribeTo( atmo.envOutMsgs[0] )

    # The StochasticAtmosphere is a StatefulSysModel with a state
    # with both drift (regular derivative) and diffusion (stochastic)
    # so we need to add to both DynamicsTask and DiffusionDynamicsTask
    scene.AddModelToDynamicsTask(stochasticAtmo)
    scene.AddModelToDiffusionDynamicsTask(stochasticAtmo)

    ### Create drag force model
    drag = CannonballDrag(
        projectedArea=10,
        dragCoeff=2.2
    )
    drag.ModelTag = "DragEff"

    drag.frameInMsg.subscribeTo( dragPoint.stateOutMsg )
    drag.atmoDensInMsg.subscribeTo( stochasticAtmo.atmoDensOutMsg )

    # The CannonballDrag model will produce a force vector message.
    # We must connect it to an actuator so that the force is applied
    dragActuator: mujoco.MJForceActuator = scene.addForceActuator( "dragForce", dragPoint )
    dragActuator.forceInMsg.subscribeTo( drag.forceOutMsg )

    # Must be added to the dynamics task because it impacts the
    # dynamics of the scene (sets a force on a body)
    scene.AddModelToDynamicsTask(drag)

    ### Add recorders
    # Record the state of the 'ball' body through
    # the ``stateOutMsg`` of its 'origin' site (i.e. frame).
    bodyStateRecorder = body.getOrigin().stateOutMsg.recorder()
    scSim.AddModelToTask("test", bodyStateRecorder)

    deterministicDensityRecorder = stochasticAtmo.atmoDensInMsg.recorder()
    scSim.AddModelToTask("test", deterministicDensityRecorder)

    densityRecorder = stochasticAtmo.atmoDensOutMsg.recorder()
    scSim.AddModelToTask("test", densityRecorder)

    dragRecorder = drag.forceOutMsg.recorder()
    scSim.AddModelToTask("test", dragRecorder)

    # Initialize the simulation
    scSim.InitializeSimulation()

    # Set initial position and velocity
    body.setPosition(rN)
    body.setVelocity(vN)

    # Run the simulation
    scSim.ConfigureStopTime(macros.sec2nano(tf))
    scSim.ExecuteSimulation()

    figures = plotOrbits(
        timeAxis=bodyStateRecorder.times(),
        posData=bodyStateRecorder.r_BN_N,
        velData=bodyStateRecorder.v_BN_N,
        dragForce=dragRecorder.force_S,
        deterministicDenseData=deterministicDensityRecorder.neutralDensity,
        denseData=densityRecorder.neutralDensity,
        oe=oe,
        mu=planet.mu,
        planetRadius=planet.radEquator
    )

    if showPlots:
        plt.show()

    return figures

def plotOrbits(timeAxis, posData, velData, dragForce, deterministicDenseData, denseData, oe, mu, planetRadius):
    """
    Plot the results of the stochastic drag simulation, including orbit, altitude,
    density, density difference, and drag force over time.

    Args:
        timeAxis: Array of time values.
        posData: Position data array.
        velData: Velocity data array.
        dragForce: Drag force data array.
        deterministicDenseData: Deterministic atmospheric density data.
        denseData: Stochastic atmospheric density data.
        oe: Classical orbital elements object.
        mu: Gravitational parameter.
        planetRadius: Radius of the planet.
    Returns:
        Dictionary of matplotlib figure objects.
    """
    # draw the inertial position vector components
    figureList = {}

    figureList["orbit"], ax = plt.subplots()
    ax.axis('equal')
    planetColor = '#008800'
    ax.add_artist(plt.Circle((0, 0), planetRadius / 1000, color=planetColor))
    # draw the actual orbit
    rData = []
    fData = []
    for idx in range(0, len(posData)):
        oeData = orbitalMotion.rv2elem(mu, posData[idx], velData[idx])
        rData.append(oeData.rmag)
        fData.append(oeData.f + oeData.omega - oe.omega)
    ax.plot(rData * np.cos(fData) / 1000, rData * np.sin(fData) / 1000, color='#aa0000', linewidth=1.0
             )
    ax.set_xlabel('$i_e$ Cord. [km]')
    ax.set_ylabel('$i_p$ Cord. [km]')

    # draw altitude as a function of time
    figureList["altitude"], ax = plt.subplots()
    ax.ticklabel_format(useOffset=False, style='plain')
    alt = (np.array(rData) - planetRadius) / 1000
    ax.plot(timeAxis * macros.NANO2HOUR, alt)
    ax.set_xlabel('$t$ [h]')
    ax.set_ylabel('Altitude [km]')

    # draw density as a function of altitude
    figureList["density"], ax = plt.subplots()
    ax.semilogy(alt, denseData, label="Stochastic")
    ax.semilogy(alt, deterministicDenseData, label="Exponential")
    ax.legend(loc="upper right")
    ax.set_xlabel('Altitude [km]')
    ax.set_ylabel('$\\rho$ [kg/m$^3$]')

    # draw density as a function of altitude
    figureList["densityDiff"], ax = plt.subplots()
    ax.plot(timeAxis * macros.NANO2HOUR, (denseData / deterministicDenseData) - 1)
    ax.set_xlabel('Time [hr]')
    ax.set_ylabel(r'$(\rho_{stoch} / \rho_{exp}) - 1$ [-]')

    # draw drag as a function of time
    figureList["drag"], ax = plt.subplots()
    ax.semilogy(timeAxis * macros.NANO2HOUR, np.linalg.norm(dragForce, 2, 1))
    ax.set_xlabel('$t$ [hr]')
    ax.set_ylabel('$|F_drag|$ [N]')

    return figureList

class StochasticAtmosphere(StatefulSysModel.StatefulSysModel):
    """This model takes in an ``AtmoPropsMsg`` and outputs another
    ``AtmoPropsMsg`` with the density perturbed by some stochastic correction
    term.

    The output density is given by:

        densityOut = densityIn * (1 + densityCorrection)

    where ``densityCorrection`` is thus the relative density correction. This is
    modeled as an Ornstein-Uhlenbeck (OU) process:

        d(densityCorrection) = -theta*densityCorrection*dt + sigma*dW

    where ``theta`` and ``sigma`` are the terms that characterize the OU
    process. In the constructor of this class, ``theta`` is set through
    ``densityCorrectionTheta``, while ``sigma`` is computed from
    the constructor input ``densityCorrectionStandardDeviation``.
    """

    def __init__(self, densityCorrectionStandardDeviation: float, densityCorrectionTheta: float):
        """
        Initialize the StochasticAtmosphere model.

        Args:
            densityCorrectionStandardDeviation (float): Stationary standard deviation
                of the Ornstein-Uhlenbeck process that drives the relative atmospheric density
                correction.
            densityCorrectionTheta (float): ``theta`` term of the
                Ornstein-Uhlenbeck process that drives the relative atmospheric density correction.
        """
        super().__init__()
        self.densityCorrectionStandardDeviation = densityCorrectionStandardDeviation
        self.densityCorrectionTheta = densityCorrectionTheta

        self.atmoDensInMsg = messaging.AtmoPropsMsgReader()
        self.atmoDensOutMsg = messaging.AtmoPropsMsg()

    @property
    def densityCorrectionSigma(self):
        """
        Compute the ``sigma`` term of the Ornstein-Uhlenbeck process that
        drives the atmospheric density correction.

        Returns:
            float: The sigma value for the OU process.
        """
        return self.densityCorrectionStandardDeviation * np.sqrt(2*self.densityCorrectionTheta)

    def registerStates(self, registerer: StatefulSysModel.DynParamRegisterer):
        """
        Register the stochastic state with the simulation's state manager.

        Args:
            registerer: The DynParamRegisterer used to register states.
        """
        self.densityCorrectionState: dynParamManager.StateData = registerer.registerState(1, 1, "densityCorrection")
        self.densityCorrectionState.setNumNoiseSources(1)

    def UpdateState(self, CurrentSimNanos: int):
        """
        Update the state at each integrator step, computing both
        the drift and diffusion terms of the density correction.
        Also reads the input atmospheric density and writes the
        perturbed output density.

        Args:
            CurrentSimNanos (int): Current simulation time in nanoseconds.
        """
        densityCorrection = self.densityCorrectionState.getState()[0][0]

        # See class docstring for the dynamic equations driving
        # this correction term
        self.densityCorrectionState.setDerivative([[ -self.densityCorrectionTheta*densityCorrection ]])
        self.densityCorrectionState.setDiffusion([[ self.densityCorrectionSigma ]], index=0)

        atmoDensIn: messaging.AtmoPropsMsgPayload = self.atmoDensInMsg()

        atmoDensOut = messaging.AtmoPropsMsgPayload()
        atmoDensOut.neutralDensity = atmoDensIn.neutralDensity * (1 + densityCorrection)
        atmoDensOut.localTemp = atmoDensIn.localTemp
        self.atmoDensOutMsg.write(atmoDensOut, CurrentSimNanos, self.moduleID)


class CannonballDrag(sysModel.SysModel):
    """
    Implements a cannonball drag force model for a spacecraft.
    Computes the drag force based on atmospheric density and spacecraft velocity.
    """

    def __init__(self, projectedArea: float, dragCoeff: float):
        """
        Initialize the CannonballDrag model.

        Args:
            projectedArea (float): Projected area of the spacecraft [m^2].
            dragCoeff (float): Drag coefficient (dimensionless).
        """
        super().__init__()
        self.projectedArea = projectedArea
        self.dragCoeff = dragCoeff

        self.atmoDensInMsg = messaging.AtmoPropsMsgReader()
        self.frameInMsg = messaging.SCStatesMsgReader()

        self.forceOutMsg = messaging.ForceAtSiteMsg()

    def UpdateState(self, CurrentSimNanos: int):
        """
        Update the drag force at each integrator step.

        Args:
            CurrentSimNanos (int): Current simulation time in nanoseconds.
        """
        density = self.atmoDensInMsg().neutralDensity

        # N frame: inertial frame
        # B frame: body-fixed frame
        frame: messaging.SCStatesMsgPayload = self.frameInMsg()
        dcm_BN = rbk.MRP2C(frame.sigma_BN)
        v_BN_B = dcm_BN @ frame.v_BN_N
        vNorm = np.linalg.norm(v_BN_B)
        v_hat_BN_B = v_BN_B / vNorm

        dragForce_B = 0.5 * self.dragCoeff * vNorm**2 * self.projectedArea * density * (-v_hat_BN_B)

        payload = messaging.ForceAtSiteMsgPayload()
        payload.force_S = dragForce_B
        self.forceOutMsg.write(payload, CurrentSimNanos, self.moduleID)


if __name__ == "__main__":
    run(True)
