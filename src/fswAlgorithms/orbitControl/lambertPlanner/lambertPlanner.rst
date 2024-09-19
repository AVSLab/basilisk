Executive Summary
-----------------
Lambert's problem aims at finding an orbit given two position vectors :math:`\mathbf{r}_{1}` and :math:`\mathbf{r}_{2}`
and the time if flight :math:`t_{flight}` between these two locations. That is, given the two position vectors
:math:`\mathbf{r}_{1}` and :math:`\mathbf{r}_{2}` and the time of flight :math:`t_{flight}`, one finds the two velocity
vectors :math:`\mathbf{v}_{1}` and :math:`\mathbf{v}_{2}` at the corresponding locations such that a spacecraft travels
from :math:`\mathbf{r}_{1}` to :math:`\mathbf{r}_{2}` at the given time of flight.

This module assumes that :math:`\mathbf{r}_{2}` is equal to a targeted location :math:`{}^N\mathbf{r}_{T/N}`, and that
the spacecraft should arrive at the targeted location at specified time :math:`t_{final}`. To get to the targeted
location, a maneuver should be performed at specified time :math:`t_{maneuver}`, where the maneuver time must be in the
future. The location of the maneuver is assumed to not be known, so a 4th order Runge-Kutta (RK4) is used to propagate
the current state (obtained from the :ref:`NavTransMsgPayload` navigation input message) to obtain the estimated
position at maneuver time, corresponding to :math:`\mathbf{r}_{1}`.
In other words, this module takes a targeted location :math:`{}^N\mathbf{r}_{T/N}`, final time :math:`t_{final}`,
maneuver time :math:`t_{maneuver}` and the spacecraft navigation message :ref:`NavTransMsgPayload` to compute the input
parameters :math:`\mathbf{r}_{1}`, :math:`\mathbf{r}_{2}` and time if flight
:math:`t_{flight} = t_{final} - t_{maneuver}` that are required to solve Lambert's problem. This module writes these
parameters to the :ref:`lambertProblemMsgPayload` message that is used as an input to the :ref:`lambertSolver` module,
which subsequently solves Lambert's problem.

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
    * - lambertProblemOutMsg
      - :ref:`lambertProblemMsgPayload`
      - lambert problem setup output message

Module Assumptions and Limitations
----------------------------------
The equations of motion used inside the module to propagate the state assume simple two body point mass gravity, and the
motion is propagated using a 4th order Runge-Kutta (RK4). Additionally, this module assumes that
:math:`t_{final} > t_{maneuver} \ge t`, with final time :math:`t_{final}`, maneuver time :math:`t_{maneuver}` and
current time :math:`t`.

Algorithm
---------
Equations of motion (two body point mass gravity) with gravitational parameter :math:`\mu` and spacecraft position
vector :math:`\mathbf{r}`:

.. math::
    :label: eq:plannerEOM

    \mathbf{\ddot{r}} = - \frac{\mu}{r^3} \mathbf{r}

User Guide
----------
The module is first initialized as follows:

.. code-block:: python

    module = lambertPlanner.LambertPlanner()
    module.ModelTag = "lambertPlanner"
    module.setR_TN_N(np.array([0., 8000. * 1000,0.]))
    module.setFinalTime(2000.)
    module.setManeuverTime(1000.)
    module.setMu(3.986004418e14)
    module.setNumRevolutions(0) # module defaults this value to 0 if not specified
    module.useSolverIzzoMethod() # module uses Izzo by default if not specified
    unitTestSim.AddModelToTask(unitTaskName, module)

The navigation translation message is then connected:

.. code-block:: python

    module.navTransInMsg.subscribeTo(navTransInMsg)
