Executive Summary
-----------------
This module computes the required Delta-V to change the spacecraft velocity to the one obtained by the
:ref:`lambertSolver` module and checks if the resulting trajectory comes too close to the central body or too far away
from the desired target location.
The current spacecraft state from the :ref:`NavTransMsgPayload` navigation input message is propagated to the maneuver
time :math:`t_{maneuver}` using a 4th order Runge-Kutta (RK4) to obtain the expected state just before the Delta-V
maneuver.
The computed Delta-V and the specified state uncertainty are added to generate a set of initial states right after the
maneuver.
These initial states are propagated to the final time :math:`t_{final}` where the spacecraft is supposed to arrive at
the targeted position :math:`{}^N\mathbf{r}_{T/N}`.
If the final distance from the target position is less than the specified maximum distance :math:`r_{TB,max}` for all
trajectories, and none of the trajectories comes closer than :math:`r_{min}` to the central body, the computed Delta-V
is written to the :ref:`DvBurnCmdMsgPayload` output message.
If any of these constraints are violated, the message content remains zero.
The message is also zeroed if the :ref:`LambertSolutionMsgPayload` or :ref:`LambertPerformanceMsgPayload` messages
indicate that the Lambert solution is not valid or has not converged yet.


Message Connection Descriptions
-------------------------------
The following table lists all the module input and output messages.
The module msg connection is set by the user from python.
The msg type contains a link to the message structure definition, while the description
provides information on what this message is used for.

.. list-table:: Module I/O Messages
    :widths: 25 25 50
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - navTransInMsg
      - :ref:`NavTransMsgPayload`
      - translational navigation input message
    * - lambertProblemInMsg
      - :ref:`lambertProblemMsgPayload`
      - lambert problem setup input message
    * - lambertSolutionInMsg
      - :ref:`LambertSolutionMsgPayload`
      - lambert problem solution input message
    * - lambertPerformanceInMsg
      - :ref:`LambertPerformanceMsgPayload`
      - lambert problem performance message (additional information about the solution process)
    * - dvBurnCmdOutMsg
      - :ref:`DvBurnCmdMsgPayload`
      - Delta-V command output message


Module Assumptions and Limitations
----------------------------------
The equations of motion used inside the module to propagate the state assume simple two body point mass gravity, and the
motion is propagated using a 4th order Runge-Kutta (RK4). Additionally, this module assumes that
:math:`t_{final} > t_{maneuver} > t`, with final time :math:`t_{final}`, maneuver time :math:`t_{maneuver}` and
current time :math:`t`.


Algorithm
---------
Equations of motion (two body point mass gravity) with gravitational parameter :math:`\mu` and spacecraft position
vector :math:`\mathbf{r}`:

.. math::
    :label: eq:validatorEOM

    \mathbf{\ddot{r}} = - \frac{\mu}{r^3} \mathbf{r}

The 27 perturbed initial states that are propagated to check for any constraint violations are obtained in the
following way:
The uncertainty of each state, specified by the 6x6 matrix "uncertaintyStates" in Hill frame components, is added to
each state in the plus and minus direction.
For :math:`N=6` states, this gives :math:`2 \times N = 12` initial states.
The uncertainty of the Delta-V magnitude is applied to the Delta-V vector for each of those initial states, both in the
Delta-V direction and against the Delta-V direction (corresponding to maximum and minimum expected DV magnitude).
Applied to the 12 initial states, this gives 24 states.
The last 3 initial states are obtained by applying the maximum and minimum expected DV to the unperturbed spacecraft
state (+2), and by leaving the state and Delta-V vector entirely unperturbed (+1).


User Guide
----------
The module is first initialized as follows:

.. code-block:: python

    module = lambertValidator.LambertValidator()
    module.ModelTag = "lambertValidator"
    module.setFinalTime(2000.)
    module.setManeuverTime(1000.)
    module.setMaxDistanceTarget(3000.)
    module.setMinOrbitRadius(6378 * 1000.)
    module.setUncertaintyStates(np.diag([5., 5., 5., 0.01, 0.01, 0.001]))  # in Hill frame
    module.setUncertaintyDV(0.1)   # [m/s]
    module.setDvConvergenceTolerance(0.01)    # [m/s]
    unitTestSim.AddModelToTask(unitTaskName, module)

The input messages are then connected:

.. code-block:: python

    module.navTransInMsg.subscribeTo(navTransInMsg)
    module.lambertProblemInMsg.subscribeTo(lambertProblemInMsg)
    module.lambertPerformanceInMsg.subscribeTo(lambertPerformanceInMsg)
    module.lambertSolutionInMsg.subscribeTo(lambertSolutionInMsg)
