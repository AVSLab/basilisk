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

The algorithms by Gooding and Izzo provide solutions for elliptic, parabolic and hyperbolic transfer orbits.
In the zero-revolution case, exactly one solution exists. In the multi-revolution case (meaning that the transfer orbit
completes at least one complete revolution about the central body), either two or zero solutions exist, depending on
whether the requested transfer time is greater or less than the minimum time-of-flight for the given number of
revolutions. The :ref:`LambertSolutionMsgPayload` output message includes two solutions. If a solution does not exist,
the corresponding velocity vectors in the message are equal to the zero vector, and the "validity" flag is equal to
zero. Otherwise, the "validity" flag is set to 1.

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
    * - lambertProblemInMsg
      - :ref:`lambertProblemMsgPayload`
      - lambert problem setup input message
    * - lambertSolutionOutMsg
      - :ref:`LambertSolutionMsgPayload`
      - lambert problem solution output message
    * - lambertPerformanceOutMsg
      - :ref:`LambertPerformanceMsgPayload`
      - lambert problem performance message (additional information about the solution process)

Module Assumptions and Limitations
----------------------------------
The algorithms only compute solutions for a positive time-of-flight, and for positive transfer angles (meaning that the
true anomaly of :math:`\mathbf{r}_{2}` is greater than the true anomaly of :math:`\mathbf{r}_{1}`.

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
