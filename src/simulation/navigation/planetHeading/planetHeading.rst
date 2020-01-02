Executive Summary
-----------------
Takes in a spacecraft position message and spice planet message and provides the heading to the planet in the s/c body frame.

Module Assumptions and Limitations
----------------------------------
- This model needs to be provided spacecraft body frame and planet positions in the inertial frame.
- This model captures only a static heading vector, not the rate of said heading vector.
- This model is limited to finding the body heading to a planet (not other spacecraft, etc).

Message Connection Descriptions
-------------------------------
The following table lists all the module input and output messages.  The module msg variable name is set by the user from python.  The msg type contains a link to the message structure definition, while the description provides information on what this message is used for.

.. table:: Module I/O Messages
        :widths: 25 25 100

        +-------------------------+---------------------------------+---------------------------------------------------+
        | Msg Variable Name       | Msg Type                        | Description                                       |
        +=========================+=================================+===================================================+
        | planetPositionInMsgName | :ref:`SpicePlanetStateSimMsg`   | This message is used to get the planet's position |
        +-------------------------+---------------------------------+---------------------------------------------------+
        | spacecraftStateInMsgName| :ref:`SCPlusStatesSimMsg`       | This message is used to get the spacecraft's      |
        |                         |                                 | position and attitude MRP                         |
        +-------------------------+---------------------------------+---------------------------------------------------+
        | planetHeadingOutMsgName | :ref:`BodyHeadingSimMsg`        | This message is used to output the                |
        |                         |                                 | unit heading vector to the planet in the          |
        |                         |                                 | spacecraft's body                                 |
        +-------------------------+---------------------------------+---------------------------------------------------+


Detailed Module Description
---------------------------
Many physical effects and spacecraft controls depend on the heading vector to a planet. Generally, it is useful for this vector to be given in spacecraft body frame components. Examples of such models in Basilisk that can use this data are course sun sensors (sun heading), and various control setups to point to a planet (planet heading). Because this information is so widely useful, it ought to be implemented in a single location to reduce computation and ensure consistency.

Design Philosophy
^^^^^^^^^^^^^^^^^
Overall, this module was created because many other modules need this information.

Equations
^^^^^^^^^
The heading is calculated by using the spacecraft and planet positions and spacecraft body frame attitude with respect to the inertial frame.

First the inertial vector to the planet is calculated:

.. math::

    {}^N\boldsymbol{r}_{\mathrm{PS}} = {}^N\boldsymbol{r}_{\mathrm{PN}} = {}^N\boldsymbol{r}_{\mathrm{SN}}

Then, it is converted to the body frame and normalized

.. math::

    {}^B\boldsymbol{r}_{\mathrm{PS}} = \mathrm{DCM(\boldsymbol{\sigma}_{BN})} {}^N \boldsymbol{r}_{\mathrm{PS}}

    {}^B \hat{\boldsymbol{r}}_{\mathrm{PS}} = \frac{{}^B\boldsymbol{r}_{\mathrm{PS}}}{||^B\bar{r}_{\mathrm{PS}}||}



User Guide
^^^^^^^^^^
The user can only instantiate this module, change the i/o names, and add it to a task.
The names below are only special in that they are useful defaults.  Only the `spacecraftStateInMsgName` is actually a default.

.. code-block:: python

    from Basilisk.simulation import planetHeading
    from Basilisk.utilities import SimulationBaseClass()

    sim = SimulationBaseClass.SimBaseClass()
    proc = sim.CreateNewProcess("proc")
    task = sim.CreateNewTask("task", int(1e9))
    proc.addTask(task)

    ph = planetHeading.PlanetHeading()
    ph.planetPositionInMsgName = "earth_planet_data"
    ph.spacecraftStateInMsgName = "inertial_state_output"
    ph.planetHeadingOutMsgName = "planet_heading"
    sim.AddModelToTask(task.Name, ph)

