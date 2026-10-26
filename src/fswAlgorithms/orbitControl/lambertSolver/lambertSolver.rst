Executive Summary
-----------------
This module solves Lambert's problem using either the `algorithm by R.H. Gooding <https://doi.org/10.1007/BF00049511>`__
(longer, extensive report available `here <https://apps.dtic.mil/sti/citations/ADA200383>`__ ) or the
`algorithm by D. Izzo <https://doi.org/10.1007/s10569-014-9587-y>`__. Given two position vectors :math:`\mathbf{r}_{1}`
and :math:`\mathbf{r}_{2}` as well as a requested time-of-flight :math:`t`, the solution to Lambert's problem provides
the corresponding velocity vectors v1_N and v2_N of the transfer orbit. The information about the type of algorithm
(Gooding or Izzo), the position vectors, the time-of-flight as well as the number of complete revolutions around the
central body, is provided by the :ref:`lambertProblemMsgPayload` input message. The velocity vectors are computed in the
same frame as the input position vectors.

The input message's ``transferDirection`` selects ``PROGRADE`` or ``RETROGRADE``
relative to the positive z-axis of that frame. The zero/default value is
``PROGRADE``, preserving the behavior of existing callers. The position endpoints
and flight time alone do not specify a preferred direction. Both algorithms honor
this selection for zero- and multi-revolution transfers.

The algorithms by Gooding and Izzo provide solutions for elliptic, parabolic and hyperbolic transfer orbits.
In the zero-revolution case, exactly one solution exists. In the multi-revolution case (meaning that the transfer orbit
completes at least one complete revolution about the central body), either two or zero solutions exist, depending on
whether the requested transfer time is greater or less than the minimum time-of-flight for the given number of
revolutions. At the minimum time, within floating-point roundoff, the two solutions
coalesce. Both output slots then contain the same orbit with both validity flags
set to 1, and the root-finder iteration counts and step errors are zero. Above the
minimum time, solution 1 has :math:`x < x_M` and solution 2 has :math:`x > x_M`, where
:math:`x_M` is the location of the minimum of the time-of-flight curve.
The :ref:`LambertSolutionMsgPayload` output message includes two solutions. If a solution does not exist,
the corresponding velocity vectors in the message are equal to the zero vector, and the "validity" flag is equal to
zero. A validity flag is set to 1 only after the root converges and both endpoint
velocities are finite. If the minimum-time search cannot converge to a finite
minimum, the module logs a warning and returns zero velocities with both validity
flags set to zero.

Root Convergence
----------------
The Gooding Halley steps and Izzo Householder steps are safeguarded by a bracket
for each monotonic branch of the flight-time curve. A nonfinite step, a step
outside that bracket, or a high-order step that does not shrink sufficiently
relative to the preceding high-order step is replaced by bisection. Bisection
clears this step-size history, allowing an admissible high-order step to resume.
The first high-order step also starts with no step-size history. Multi-revolution
roots remain on their respective sides of the minimum-time point.
For a zero-revolution hyperbolic transfer, the upper bracket
is expanded at most 64 times. A finite starter at that upper endpoint is retained.
Each root then receives at most 80 refinement steps.

Convergence requires a finite root in its admissible domain, a final step or
computed correction no larger than :math:`10^{-8}`, and a relative
non-dimensional flight-time residual no larger than :math:`10^{-12}`. A small
correction with an acceptable residual is recognized before the bracket
safeguard, even when the correction rounds to zero. A small correction alone
does not establish convergence. An exact finite flight-time match has a zero
correction.

The analytic parabolic solution uses :math:`x=1` with zero iterations and zero
step error when it satisfies the same relative flight-time tolerance. Nearby
requests use the smooth Battin flight-time formula rather than a constant
parabolic approximation, so the residual check can distinguish them.

If numerical limits prevent convergence or velocity reconstruction produces a
nonfinite value, the module logs a warning. The affected solution slot has a zero
validity flag, zero velocities, and zero performance fields. The other branch
can still be valid. Every update starts with fresh root results, so a later
numerically resolvable request can recover after a root-search failure.

The performance message reports the number of root refinements, including
bisection steps, and the final step or converged correction magnitude. A valid
solution can require more iterations than older solver versions allowed. Downstream acceptance policies
still apply: :ref:`lambertValidator` currently requires fewer than six root
iterations when its constraint checks are enabled.

Message Connection Descriptions
-------------------------------
The following diagram and table list all the module input and output messages.  The module message connection is
set by the user from Python.  The message type contains a link to the message structure definition, while the
description provides information on what this message is used for.

.. bsk-module-io:: lambertSolver
    :caption: Module I/O Messages

    input lambertProblemInMsg lambertProblemMsgPayload
        lambert problem setup input message
    output lambertSolutionOutMsg LambertSolutionMsgPayload
        lambert problem solution output message
    output lambertPerformanceOutMsg LambertPerformanceMsgPayload
        lambert problem performance message (additional information about the solution process)

Module Assumptions and Limitations
----------------------------------
The algorithms require a positive time of flight. The transfer angle is measured
along the requested direction of motion. ``PROGRADE`` selects an orbit with
positive z-component of angular momentum, while ``RETROGRADE`` selects a negative
z-component. This convention is relative to the input frame, so the direction
must be selected consistently when changing frames. Neither direction implies a
short transfer angle: either can require an angle greater than 180 degrees.

For polar geometry, the z-axis cannot distinguish the two directions. The deterministic convention is
to use :math:`\mathbf{r}_{1}\times\mathbf{r}_{2}` as the orbit normal for ``PROGRADE``
(the shorter transfer angle), and its negative for ``RETROGRADE`` (the longer
transfer angle). This tie includes roundoff from normalizing the position vectors:
with radial unit vectors :math:`\mathbf{u}_1` and :math:`\mathbf{u}_2`, the solver
treats the geometry as polar when

.. math::

    |(\mathbf{u}_1\times\mathbf{u}_2)_z|
    \le 8\epsilon\left(|u_{1x}u_{2y}| + |u_{1y}u_{2x}|\right),

where :math:`\epsilon` is double-precision machine epsilon. Outside this bound,
the sign selects the direction normally, including for near-polar transfers.
The message stores ``transferDirection`` as an integer so unknown values raise a
Basilisk error before conversion to the internal direction enum.

An edge case exists for a transfer angle of 0 or 180 degrees, as the two position vectors do not define a plane, so an
infinite number of solutions exist. The module checks if the angle between the two position vectors is smaller than the
threshold "alignmentThreshold". In this case, the computed velocity vectors are set equal to the zero vector and the
validity flag of the solution is set to zero.

The convergence tolerances apply to the non-dimensional root and flight-time
equation; they are not absolute bounds on position or velocity error. Geometry
and floating-point resolution can limit accuracy. If the flight-time evaluator
cannot resolve a request to the convergence tolerances, the result is invalid.

User Guide
----------
The module is first initialized as follows:

.. code-block:: python

    lambertModule = lambertSolver.LambertSolver()
    lambertModule.ModelTag = "lambertSolver"
    lambertModule.setAlignmentThreshold(1.0) # module defaults this value to 1.0 degrees if not specified
    unitTestSim.AddModelToTask(unitTaskName, lambertModule)

The lambert problem input message is either created as a standalone message in python

.. code-block:: python

    lambertProblemInMsgData = messaging.LambertProblemMsgPayload()
    lambertProblemInMsgData.solverMethod = messaging.IZZO
    lambertProblemInMsgData.r1_N = np.array([10000. * 1000, 0. ,0.])
    lambertProblemInMsgData.r2_N = np.array([0., 8000. * 1000,0.])
    lambertProblemInMsgData.transferTime = 10000.
    lambertProblemInMsgData.mu = 3.986004418e14
    lambertProblemInMsgData.numRevolutions = 0
    lambertProblemInMsg = messaging.LambertProblemMsg().write(lambertProblemInMsgData)

or obtained from another FSW module. The lambert problem input message is then connected.

.. code-block:: python

    lambertModule.lambertProblemInMsg.subscribeTo(lambertProblemInMsg)

To request a retrograde transfer, set the direction and write the updated payload
to the connected problem message:

.. code-block:: python

    lambertProblemInMsgData.transferDirection = messaging.RETROGRADE
    lambertProblemInMsg.write(lambertProblemInMsgData)

Use ``messaging.PROGRADE`` to select the default explicitly. A zero-initialized
``LambertProblemMsgPayload`` already selects prograde motion. C and C++ producers
must initialize the new ``transferDirection`` field along with the other inputs.
