
Executive Summary
-----------------

Drag dynamics class used to compute drag effects on spacecraft bodies

This class is used to implement drag dynamic effects on spacecraft using a variety of simple or complex models, which will include
cannonball (attitude-independent) drag, single flat-plate drag, faceted drag models, and an interface to full-CAD GPU-accellerated
drag models.
For more information see the
:download:`PDF Description </../../src/simulation/dynamics/facetDragEffector/_Documentation/Basilisk-facet_drag-20190515.pdf>`.

Wind Velocity Input
--------------------
When ``windVelInMsg`` is linked to a :ref:`windBase`-derived module (e.g. :ref:`zeroWindModel`),
the drag model subtracts the air velocity ``v_air_N`` from the spacecraft inertial velocity before
computing drag, yielding the atmosphere-relative velocity:

.. math::

    \mathbf{v}_{rel} = \mathbf{v}_{sc} - \mathbf{v}_{air}

If ``windVelInMsg`` is not linked, the inertial spacecraft velocity is used directly.

Example setup:

.. code-block:: python

    drag = facetDragDynamicEffector.FacetDragDynamicEffector()
    drag.addFacet(10.0, 2.2, [1, 0, 0], [0, 0, 0])
    drag.atmoDensInMsg.subscribeTo(atmo.envOutMsgs[0])

    # Optional: link a wind model for atmosphere-relative velocity
    drag.windVelInMsg.subscribeTo(windModel.envOutMsgs[0])

    scObject.addDynamicEffector(drag)

Input Message Timing
---------------------
Both ``atmoDensInMsg`` and ``windVelInMsg`` are refreshed only during ``UpdateState()``.
Because ``Spacecraft::computeForceTorque()`` calls dynamic effectors before the atmosphere
and wind models receive their next ``UpdateState()``, the drag computation always uses
the values from the previous time step (zero-initialized on the first step).
Both inputs are therefore treated as **piecewise-constant** over each integration step.

Message Connection Descriptions
-------------------------------
The following table lists all the module input and output messages.  The module msg variable name is set by the
user from python.  The msg type contains a link to the message structure definition, while the description
provides information on what this message is used for.

.. list-table:: Module I/O Messages
    :widths: 25 25 50
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - atmoDensInMsg
      - :ref:`AtmoPropsMsgPayload`
      - input message for atmospheric density information
    * - windVelInMsg
      - :ref:`WindMsgPayload`
      - (optional) wind velocity input message; when linked, ``v_air_N`` is subtracted from the
        spacecraft inertial velocity to obtain the atmosphere-relative velocity
