Executive Summary
-----------------

Sensor model to simulate a Star Tracker.

The principal-rotation-vector error state evolves according to

.. math::

    \boldsymbol e_{k+1} = A\boldsymbol e_k + P\boldsymbol z_k,
    \qquad \boldsymbol z_k \sim \mathcal{N}(\boldsymbol 0, I),

where ``PMatrix`` is a matrix square root of the process-noise covariance and
``setAMatrix()`` configures the propagation matrix. The propagation matrix
defaults to zero, so configuring only ``PMatrix`` produces independent white
Gaussian attitude errors. ``RNGSeed`` controls the repeatable random sequence.

A correlated process or random walk must be configured explicitly. For
example, identity propagation with positive walk bounds creates a bounded
random walk::

    tracker.setAMatrix([[1.0, 0.0, 0.0],
                        [0.0, 1.0, 0.0],
                        [0.0, 0.0, 1.0]])
    tracker.setWalkBounds([0.01, 0.01, 0.01])  # [rad]

Each positive walk-bound entry is an exact hard bound on the corresponding
error state. A non-positive entry disables clipping for that state.

The module
:download:`PDF Description </../../src/simulation/sensors/starTracker/_Documentation/Basilisk-star_tracker-20161101.pdf>`
contains further information on this module's function,
how to run it, as well as testing.
The corruption types are outlined in this
:download:`PDF Description </../../src/simulation/sensors/imuSensor/_Documentation/BasiliskCorruptions.pdf>`.



Message Connection Descriptions
-------------------------------
The following table lists all the module input and output messages.  The module msg connection is set by the
user from python.  The msg type contains a link to the message structure definition, while the description
provides information on what this message is used for.

.. bsk-module-io:: starTracker

   input scStateInMsg SCStatesMsgPayload
      sc input state message

   output sensorOutMsg STSensorMsgPayload
      sensor output state message
