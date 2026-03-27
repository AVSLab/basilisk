Executive Summary
-----------------

Drag dynamics class used to compute drag effects on spacecraft bodies. This class is used to implement drag dynamic effects on spacecraft using a variety of simple or complex models, which will include cannonball (attitude-independent) drag, single flat-plate drag, faceted drag models, and an interface to full-CAD GPU-accelerated drag models.

Density Correction State
------------------------
The module supports an optional density correction state that scales the incoming atmospheric density.
If ``densityCorrectionStateName`` is set, the drag model uses

.. math::

    \rho = \rho_{\text{in}} \left(1 + \delta_\rho\right)

where :math:`\rho_{\text{in}}` is read from ``atmoDensInMsg`` and :math:`\delta_\rho` is the scalar
state referenced by ``densityCorrectionStateName``.

If ``densityCorrectionStateName`` is left empty (default), no correction is applied and
``atmoDensInMsg.neutralDensity`` is used directly.

Example setup:

.. code-block:: python

    # Drag model
    drag = dragDynamicEffector.DragDynamicEffector()
    drag.coreParams.dragCoeff = 2.2
    drag.coreParams.projectedArea = 10.0
    drag.atmoDensInMsg.subscribeTo(atmo.envOutMsgs[0])

    # Optional: apply multiplicative density correction from a scalar state
    stochasticAtmo = meanRevertingNoiseStateEffector.MeanRevertingNoiseStateEffector()
    # ... Configure stochasticAtmo here
    scObject.addStateEffector(stochasticAtmo)
    drag.densityCorrectionStateName = stochasticAtmo.getStateName()

    scObject.addDynamicEffector(drag)

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

    drag = dragDynamicEffector.DragDynamicEffector()
    drag.coreParams.dragCoeff = 2.2
    drag.coreParams.projectedArea = 10.0
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
The following table lists all the module input and output messages.  The module msg connection is set by the
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
      - atmospheric density input message
    * - windVelInMsg
      - :ref:`WindMsgPayload`
      - (optional) wind velocity input message; when linked, ``v_air_N`` is subtracted from the
        spacecraft inertial velocity to obtain the atmosphere-relative velocity
