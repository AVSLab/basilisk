
Executive Summary
-----------------

Drag dynamics class used to compute drag effects on spacecraft bodies

This class is used to implement drag dynamic effects on spacecraft using a variety of simple or complex models, which will include
cannonball (attitude-independent) drag, single flat-plate drag, faceted drag models, and an interface to full-CAD GPU-accellerated
drag models.
For more information see the
:download:`PDF Description </../../src/simulation/dynamics/facetDragEffector/_Documentation/Basilisk-facet_drag-20190515.pdf>`.

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

    drag = facetDragDynamicEffector.FacetDragDynamicEffector()
    drag.ModelTag = "FacetDrag"
    drag.atmoDensInMsg.subscribeTo(atmo.envOutMsgs[0])

    # Optional: apply multiplicative density correction from a scalar state
    stochasticAtmo = meanRevertingNoiseStateEffector.MeanRevertingNoiseStateEffector()
    # ... Configure stochasticAtmo here
    scObject.addStateEffector(stochasticAtmo)
    drag.setDensityCorrectionStateName(stochasticAtmo.getStateName())

    for i in range(0,len(scAreas)):
      drag.addFacet(scAreas[i], scCoeff[i], B_normals[i], B_locations[i])

    scObject.addDynamicEffector(drag)

Atmosphere Relative Velocity
-----------------------------
The drag computation can optionally use the spacecraft velocity relative to a rotating atmosphere instead of the inertial spacecraft velocity. This is useful for modeling atmospheric drag in Low Earth Orbit, where the atmosphere approximately co-rotates with the Earth.

If ``useAtmosphereRelativeVelocity`` is enabled, the drag model computes

.. math::

    \mathbf{v}_{rel} = \mathbf{v}_{sc} - (\boldsymbol{\omega}_{planet} \times \mathbf{r}_{sc})

where

- :math:`\mathbf{v}_{sc}` is the spacecraft inertial velocity
- :math:`\mathbf{r}_{sc}` is the spacecraft inertial position
- :math:`\boldsymbol{\omega}_{planet}` is the planetary rotation vector

For Earth simulations, the planetary rotation rate is automatically taken from ``OMEGA_EARTH`` in ``astroConstants.h``.
For other bodies it can be configured through ``planetOmega_N``.

If ``useAtmosphereRelativeVelocity`` is left disabled (default), the drag model uses the spacecraft inertial velocity directly.

Example setup:

.. code-block:: python

    drag = facetDragDynamicEffector.FacetDragDynamicEffector()
    drag.ModelTag = "FacetDrag"
    drag.atmoDensInMsg.subscribeTo(atmo.envOutMsgs[0])

    # ... Define arrays scAreas, scCoeff, B_normals, B_locations here

    for i in range(0,len(scAreas)):
      drag.addFacet(scAreas[i], scCoeff[i], B_normals[i], B_locations[i])

    # Enable atmosphere-relative velocity
    drag.setUseAtmosphereRelativeVelocity(True)
    drag.setPlanetOmega_N([0.0, 0.0, 7.2921159e-5])

    scObject.addDynamicEffector(drag)

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
