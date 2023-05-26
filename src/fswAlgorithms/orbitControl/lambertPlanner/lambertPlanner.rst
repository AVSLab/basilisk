Executive Summary
-----------------
This module creates the :ref:`lambertProblemMsgPayload` message that is used as an input to the
:ref:`lambertSolver` module. The message includes the required Lambert problem setup information, such as the two
position vectors :math:`\mathbf{r}_{1}` and :math:`\mathbf{r}_{2}` as well as a requested time-of-flight. Given a
targeted position vector :math:`{}^N\mathbf{r}_{T/N}` with respect to the celestial body at final time :math:`t_{final}`
expressed in the inertial frame :math:`N`, the final time :math:`t_{final}`, and the maneuver time :math:`t_{maneuver}`,
the module uses a 4th order Runge-Kutta (RK4) to propagate the current state
(obtained from the :ref:`NavTransMsgPayload` navigation input message) to obtain the estimated position at maneuver
time. This estimated position corresponds to the first position vector :math:`\mathbf{r}_{1}` of Lambert's problem,
while :math:`{}^N\mathbf{r}_{T/N}` corresponds to the second position vector :math:`\mathbf{r}_{2}`.

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
    :label: eq:EOM

    \mathbf{\ddot{r}} = - \frac{\mu}{r^3} \mathbf{r}

User Guide
----------
The module is first initialized as follows:

.. code-block:: python

    module = lambertPlanner.LambertPlanner()
    module.ModelTag = "lambertPlanner"
    module.r_TN_N = np.array([0., 8000. * 1000,0.])
    module.finalTime = 2000.
    module.maneuverTime = 1000.
    module.mu = 3.986004418e14
    module.numRevolutions = 0 # module defaults this value to 0 if not specified
    module.useSolverIzzoMethod() # module uses Izzo by default if not specified
    unitTestSim.AddModelToTask(unitTaskName, module)

The navigation translation message is then connected:

.. code-block:: python

    module.navTransInMsg.subscribeTo(navTransInMsg)
