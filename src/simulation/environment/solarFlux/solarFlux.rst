Executive Summary
-----------------
Takes in a spacecraft position message and sun position message and calculates the solar :math:`\text{W/m}^2` at the spacecraft location.  It also allows for an optional eclipse message to be considered in the solar flux calculation.

Module Assumptions and Limitations
----------------------------------

- This model uses the solar flux value, ``SOLAR_FLUX_EARTH``, and astronomical unit, AU, values from ``astroConstants.h``
- This model assumes constant solar flux and a perfect inverse square fall-off of flux as one moves further from the sun.
- This model can take in one spacecraft position message and one sun position message. The eclipse message must correspond to the right spacecraft/planet pair.

Message Connection Descriptions
-------------------------------
The following table lists all the module input and output messages.  The module msg variable name is set by the user from python.  The msg type contains a link to the message structure definition, while the description provides information on what this message is used for.

.. list-table:: Module I/O Messages
    :widths: 25 25 50
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - sunPositionInMsg
      - :ref:`SpicePlanetStateMsgPayload`
      - sun state input message
    * - spacecraftStateInMsg
      - :ref:`SCStatesMsgPayload`
      - spacecraft state input message
    * - eclipseInMsg
      - :ref:`EclipseMsgPayload`
      - (optional) eclipse input message
    * - solarFluxOutMsg
      - :ref:`SolarFluxMsgPayload`
      - solar flux output message


Detailed Module Description
---------------------------
Many physical effects from sun sensor outputs to spacecraft heating depend on the solar flux incident on a spacecraft. This module provides that flux given the sun and spacecraft positions.

From the user perspective, this module should be instantiated when there is a desire to provide the solar flux at a spacecraft location.

From the developer perspective, the use of this module in conjunction with the developer's other modules should be preferred over individual modules calculating the solar flux internally and applying it. Especially if eclipsing effects on the solar flux are desired, this module should be used rather than other modules implementing this calculation and applying eclipse themselves. This does a few things. First, it prevents code repetition. Second, it ensures consistency how solar flux values are calculated, avoiding multiple sources and hard-coding numbers. Third, it prevents other modules from needing to apply eclipse individually.

Design Philosophy
^^^^^^^^^^^^^^^^^
Overall, this module was created because many other modules need this information. The eclipse message is included as an optional message. It was decided to do this rather than have a separate module to apply the eclipse in order to reduce module count and maintain similar design philosophy throughout the framework.

Equations
^^^^^^^^^
The flux is calculated by scaling the flux at 1 AU and applying the :ref:`EclipseMsgPayload` ``shadowFactor``:

.. math::

    F_{\mathrm{out}} = F_{\mathrm{Earth}} * \frac{\mathrm{AU}^2}{r_{\mathrm{Sun}}^2} * f_s

where :math:`r_{\mathrm{Sun}}` is the distance between the spacecraft and the sun and :math:`f_s` is the shadow factor.


User Guide
----------
The user can only instantiate this module, change the i/o names, and add it to a task.
The names below are only special in that they are useful defaults and are actually the defaults, with the exception of the eclipse message which is defaulted to an empty string

.. code-block:: python

    from Basilisk.simulation import solarFlux
    from Basilisk.utilities import SimulationBaseClass()

    sim = SimulationBaseClass.SimBaseClass()
    proc = sim.CreateNewProcess("proc")
    task = sim.CreateNewTask("task", int(1e9))
    proc.addTask(task)

    sf = solarFlux.SolarFlux()
    sf.sunPositionInMsg.subscribeTo(sunMsg)
    sf.spacecraftStateInMsg.subscribeTo(scMsg)
    sf.eclipseInMsg.subscribeTo(eclMsg)
    sim.AddModelToTask(task.Name, sf)

    dataLog = sf.solarFluxOutMsg.recorder()

