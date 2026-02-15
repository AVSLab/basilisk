Executive Summary
-----------------
The ``StochasticDragCoeff`` module applies scalar mean-reverting stochastic noise to aerodynamic drag coefficient by
specializing :ref:`MeanRevertingNoise <meanRevertingNoise>`. It perturbs ``dragCoeff`` in a
``DragGeometryMsgPayload`` and republishes the modified geometry message.

Module Description
------------------
The inherited Ornstein-Uhlenbeck state :math:`x` evolves as

.. math::
   dx = -\frac{1}{\tau}x\,dt + \sqrt{\frac{2}{\tau}}\sigma_{st}\,dW

The output drag coefficient is

.. math::
   C_{D,out} = C_{D,in}(1 + x)

The remaining geometry fields are passed through unchanged.

Message Interfaces
------------------
.. list-table:: Module I/O Messages
    :widths: 25 25 50
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - dragGeomInMsg
      - :ref:`DragGeometryMsgPayload`
      - Input drag geometry message containing ``dragCoeff``, projected area, and center-of-pressure data.
    * - dragGeomOutMsg
      - :ref:`DragGeometryMsgPayload`
      - Output drag geometry message with stochastic correction applied to ``dragCoeff``.

Detailed Behavior
-----------------
At each update step, the module performs the following operations:

1. Reads the current OU state :math:`x` from the base class.
2. Reads ``dragGeomInMsg``.
3. Multiplies ``dragCoeff`` by :math:`(1+x)`.
4. Writes the modified payload to ``dragGeomOutMsg``.

The OU process parameters are configured through base-class setters:

- ``setStationaryStd(sigma_st)``
- ``setTimeConstant(tau)``

Module Assumptions and Limitations
----------------------------------
- The model perturbs only ``dragCoeff`` and leaves projected area and center-of-pressure unchanged.
- If :math:`x < -1`, the corrected drag coefficient can become negative unless constrained externally.

Verification and Testing
------------------------
This module shares the same OU stochastic base implementation as ``StochasticAtmDensity`` and uses the same
parameterization and update flow. Current automated OU-statistics testing is provided in
``src/simulation/mujocoDynamics/meanRevertingNoise/_UnitTest/test_meanRevertingNoise.py`` through the
``StochasticAtmDensity`` subclass path.
