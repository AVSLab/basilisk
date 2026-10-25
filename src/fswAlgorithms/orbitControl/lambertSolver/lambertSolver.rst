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
zero. Otherwise, the "validity" flag is set to 1. If the minimum-time search cannot
converge to a finite minimum, the module logs a warning and returns zero velocities
with both validity flags set to zero.

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

While the module also works for parabolic transfer orbits, the solutions for orbits that are very close to - but not
exactly - parabolic, may not be as accurate.

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
