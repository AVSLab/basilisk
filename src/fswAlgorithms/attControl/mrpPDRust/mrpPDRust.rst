
.. note::

   This module is a native Rust port of the C :ref:`mrpPD` module. The two
   implementations intentionally expose equivalent configuration parameters,
   message ports, inertia caching, and control-law results.

Executive Summary
-----------------

``mrpPDRust`` provides modified Rodrigues parameter (MRP)
proportional-derivative attitude control. It is a native Rust counterpart to
``mrpPD`` and is similar to :ref:`mrpFeedback`, but it does not include
reaction-wheel or integral-feedback terms. When the modeled dynamics are
known and the requested external torque is realized, the controller
asymptotically tracks a general reference attitude trajectory.

Message Connection Descriptions
-------------------------------

The following diagram and table list the module input and output messages.
Both input messages are required.

.. bsk-module-io:: mrpPDRust
   :caption: Module I/O Messages
   :module-type: Rust

   output cmdTorqueOutMsg CmdTorqueBodyMsgPayload
      Commanded external control torque in body-frame components.

   input vehConfigInMsg VehicleConfigMsgPayload
      Spacecraft inertia and mass properties. The inertia is cached during
      reset.

   input guidInMsg AttGuidMsgPayload
      Attitude error, angular-rate tracking error, reference-frame rate, and
      reference-frame acceleration.

Detailed Module Description
---------------------------

The module implements the MRP feedback law discussed in section 8.4.1 of
`Analytical Mechanics of Space Systems
<https://doi.org/10.2514/4.105210>`_. Define
:math:`\pmb{\omega}_{BN}`
as the body angular rate, :math:`\pmb{\omega}_{RN}` as the reference-frame
angular rate, and
:math:`\delta\pmb{\omega}=\pmb{\omega}_{BR}` as their tracking error. The
requested external control torque is

.. math::
   :label: eq-mrppdrust-control

   {\bf L}_{r} =
   -K\pmb{\sigma}_{BR}
   -P\delta\pmb{\omega}
   +\pmb{\omega}_{RN}\times[I]\pmb{\omega}_{BN}
   +[I]\left(
       \dot{\pmb{\omega}}_{RN}
       -\pmb{\omega}_{BN}\times\pmb{\omega}_{RN}
   \right)
   -{\bf L}.

Here, :math:`K` is the MRP proportional gain, :math:`P` is the angular-rate
error gain, :math:`[I]` is the spacecraft inertia about point B, and
:math:`{\bf L}` is a known external body torque. The generated Rust lifecycle
wrapper reads the input payloads and publishes the returned
``CmdTorqueBodyMsgPayload`` with the module ID and current simulation time.

During reset, the module validates both required input connections and caches
``vehConfigInMsg.ISCPntB_B`` as a fixed-size ``nalgebra`` matrix in Rust-owned
private state. Because inertia arrives through a message rather than a
configuration setter, reset verifies that it is finite, symmetric, and
positive definite before caching it. Each update converts the C-compatible
guidance arrays to ``nalgebra`` vectors, evaluates the control law, and
returns the commanded torque as a message array. The inertia remains fixed
until the next reset, matching the C implementation.

Module Assumptions and Limitations
----------------------------------

The controller assumes a rigid spacecraft whose inertia tensor does not
change between resets. The inertia must be finite, symmetric, and positive
definite. The controller assumes the reference attitude, reference rates, and
reference accelerations are dynamically consistent. The requested external
control torque must be realized by an actuator system such as a thruster
cluster; this module does not perform actuator allocation.

User Guide
----------

Create the controller, configure its gains, connect both required inputs, and
add it to a task like any other compiled Basilisk module:

.. code-block:: python

   from Basilisk.fswAlgorithms import mrpPDRust

   controller = mrpPDRust.mrpPDRust()
   controller.ModelTag = "mrpPDRust"
   controller.K = 0.15             # [N*m]
   controller.P = 150.0            # [N*m*s]
   controller.knownTorquePntB_B = [0.0, 0.0, 0.0]  # [N*m]

   controller.guidInMsg.subscribeTo(guidance_message)
   controller.vehConfigInMsg.subscribeTo(vehicle_config_message)
   simulation.AddModelToTask("controlTask", controller)

The Python-visible configuration is equivalent to ``mrpPD``:

``K``
   MRP proportional feedback gain [N*m].

``P``
   Angular-rate tracking-error feedback gain [N*m*s].

``knownTorquePntB_B``
   Optional known external torque expressed in body-frame components [N*m].
   Its typed Rust default is a zero vector.

``guidInMsg``
   Required attitude-guidance input.

``vehConfigInMsg``
   Required vehicle-configuration input. Update the message and reset the
   module when the modeled inertia changes.

``cmdTorqueOutMsg``
   Commanded external body torque output.

Leaving either input disconnected causes initialization to raise
``BasiliskError`` with the missing port name. The module's gains default to
zero; set ``K`` and ``P`` explicitly before initializing the simulation.
Generated setters require both gains to be finite and nonnegative and every
known-torque component to be finite. Invalid assignments raise
``BasiliskError`` immediately without changing the previous value. Explicit
methods such as ``setK(value)``, ``getK()``, and
``setKnownTorquePntB_B(value)`` are equivalent to property access.

Inertia is not a Python configuration property of this module. It arrives
through ``vehConfigInMsg`` and is therefore validated during reset. An
invalid inertia raises ``BasiliskError`` before the controller caches or uses
the matrix.
