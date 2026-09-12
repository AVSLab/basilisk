Executive Summary
-----------------

This state effector models reaction wheels attached to a spacecraft hub, including wheel-speed states,
motor torques, friction, and the resulting spacecraft coupling. It supports balanced wheels, simple
jitter, and fully coupled jitter. The jitter models also maintain wheel-angle states.

The module
:download:`PDF Description </../../src/simulation/dynamics/reactionWheels/_Documentation/Basilisk-REACTIONWHEELSTATEEFFECTOR-20170816.pdf>`
contains further information on this module's function,
how to run it, as well as testing.


Module Assumptions and Limitations
----------------------------------

The wheel model and ``includeWheelMassProperties`` option determine how mass properties are accounted
for. The option defaults to ``False`` to preserve existing simulations: balanced and simple-jitter
wheels then require their constant mass properties to be included in the hub.

.. list-table:: Default wheel mass-property accounting (``includeWheelMassProperties = False``)
    :header-rows: 1
    :widths: 20 40 40

    * - ``RWModel``
      - Effector contribution
      - Required hub configuration
    * - ``BalancedWheels``
      - No mass, first mass moment, or constant inertia contribution.
      - Include the wheel's constant mass properties in the hub.
    * - ``JitterSimple``
      - No mass, first mass moment, or constant inertia contribution. Imbalance is approximated
        through applied forces and torques.
      - Include the wheel's constant mass properties in the hub, as for balanced wheels.
    * - ``JitterFullyCoupled``
      - Adds the configured wheel mass, center-of-mass contribution, inertia, and their applicable
        time derivatives, including imbalance effects.
      - Exclude the mass properties already represented by this wheel effector from the hub.

With the default option, setting a nonzero ``mass`` on a ``BalancedWheels`` or ``JitterSimple`` wheel does not increase the
spacecraft mass. Likewise, setting ``rWB_B``, ``Jt``, or ``Jg`` does not add the wheel's constant
center-of-mass or inertia contribution in these two models. The spin-axis inertia ``Js`` is still
used in wheel acceleration, spacecraft rotational coupling, angular momentum, and energy; it must
be configured even when the wheel's constant inertia is included in the hub.

With ``includeWheelMassProperties = True``, balanced and simple-jitter wheels instead contribute
their nominal mass, center-of-mass offset, and axisymmetric inertia through the effector. The option
does not change the accounting for fully coupled wheels, which always contribute their own properties.

See :ref:`reactionWheelMassAccounting` for the hub inputs and model-switching guidance.


Message Connection Descriptions
-------------------------------
The following table lists all the module input and output messages.  The module msg connection is set by the
user from python.  The msg type contains a link to the message structure definition, while the description
provides information on what this message is used for.

.. bsk-module-io:: reactionWheelStateEffector
    :caption: Module I/O Messages

    input rwMotorCmdInMsg ArrayMotorTorqueMsgPayload
        (optional) RW motor torque array cmd input message.  If not connected the motor torques are set to zero.
    output rwSpeedOutMsg RWSpeedMsgPayload
        RW speed array output message.
    output rwOutMsgs RWConfigLogMsgPayload
        vector of RW log output messages.

User Guide
-----------

The reaction wheel state effector module provides functionality for simulating reaction wheels in a spacecraft.
It includes safety mechanisms to prevent numerical instability that can occur with excessive wheel acceleration
or when using unlimited torque with small spacecraft inertia.

.. _reactionWheelMassAccounting:

Configuring Hub and Wheel Mass Properties
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

By default, for ``BalancedWheels`` and ``JitterSimple``, configure the hub to include the wheels as if they were
locked relative to the body. The effector separately models their relative spin. Update all three
hub inputs consistently:

* ``hub.mHub``: combined mass of the rigid hub and the wheels accounted for in the hub, in kg.
* ``hub.r_BcB_B``: location of that combined center of mass relative to body-frame origin
  :math:`B`, expressed in body-frame components, in m.
* ``hub.IHubPntBc_B``: combined inertia about that combined center of mass :math:`B_c`, expressed
  in body-frame components, in kg m\ :sup:`2`. Include each wheel's full constant inertia,
  including its spin-axis inertia, and the parallel-axis contributions from its location.

For example, let :math:`m_0`, :math:`\mathbf r_0`, and :math:`I_0` describe the rigid hub before adding
the wheels, with :math:`I_0` taken about its own center of mass. For each wheel to be included in
the hub, let :math:`m_i`, :math:`\mathbf r_i`, and :math:`I_i` be its mass, nominal center-of-mass
location, and constant inertia about its own center of mass. All position vectors are measured from
:math:`B`, and all vectors and inertia tensors are expressed in body-frame components. Then set:

.. math::

    m_h = m_0 + \sum_i m_i, \qquad
    \mathbf r_h = \frac{m_0\mathbf r_0 + \sum_i m_i\mathbf r_i}{m_h}

.. math::

    \begin{aligned}
    I_h &= I_0 + m_0 P(\mathbf r_0 - \mathbf r_h)
        + \sum_i \left[I_i + m_i P(\mathbf r_i - \mathbf r_h)\right], \\
    P(\mathbf a) &= (\mathbf a^T\mathbf a)\mathbf 1_3 - \mathbf a\mathbf a^T
    \end{aligned}

Here :math:`\mathbf 1_3` is the identity matrix. Assign :math:`m_h`, :math:`\mathbf r_h`, and
:math:`I_h` to ``mHub``, ``r_BcB_B``, and ``IHubPntBc_B``, respectively. Rotate wheel inertia tensors
into the body frame before combining them. For an axisymmetric balanced wheel, the constant tensor
about its center of mass is :math:`I_i = J_t\mathbf 1_3 + (J_s-J_t)\hat{\mathbf g}_s\hat{\mathbf g}_s^T`,
where :math:`J_g=J_t` and :math:`\hat{\mathbf g}_s` is ``gsHat_B``. Other state effectors continue to
contribute their own mass properties separately.

For ``JitterFullyCoupled``, the effector performs the wheel mass-property accounting. Its wheel
center of mass is ``rWB_B + d * w2Hat_B``, with ``d = U_s / mass``. It rotates the wheel inertia
defined by ``Js``, ``Jt``, ``Jg``, and ``J13 = U_d`` into the body frame and includes the parallel-axis
contribution about :math:`B`. Include only the remaining rigid structure in the hub; any stationary
wheel housing not represented by the configured wheel mass properties still belongs in the hub.
Check the selected factory's mass and inertia definitions when partitioning the hardware.

.. warning::

    Count each wheel's mass properties exactly once. With the default option, if the hub inputs already describe the assembled
    spacecraft with balanced or simple-jitter wheels, do not add those wheels again. When switching
    to ``JitterFullyCoupled``, remove the mass properties represented by those wheels from the hub
    and recompute its center of mass and inertia about that center. When switching back, include them
    again. Changing ``RWModel`` does not adjust the hub inputs automatically. For mixed wheel models,
    apply this convention to each wheel individually.

Automatic Inclusion of Wheel Mass Properties
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

To let the effector account for the constant properties of its balanced and simple-jitter wheels,
set the option before ``InitializeSimulation()``:

.. code-block:: python

    rwStateEffector = reactionWheelStateEffector.ReactionWheelStateEffector()
    rwStateEffector.includeWheelMassProperties = True
    rwFactory.addToSpacecraft("ReactionWheels", rwStateEffector, scObject)

Configure ``scObject.hub`` with only the remaining rigid structure, excluding the mass properties
represented by these wheels. The option applies to all balanced and simple-jitter wheels in this
effector. For mixed wheel models, fully coupled wheels still contribute exactly once. Leave the
option unchanged during a simulation run.

For each balanced or simple-jitter wheel, the effector uses ``mass``, ``rWB_B``, ``gsHat_B``, ``Js``,
and ``Jt`` to compute the nominal mass properties. Its constant inertia is the axisymmetric tensor
given above, shifted from ``rWB_B`` to :math:`B` using the parallel-axis theorem. The effector also
includes the corresponding rigid-body kinetic energy and angular momentum, in addition to the
existing relative-spin terms. The derivatives of these constant mass properties are zero.

This option requires finite, positive ``mass``, ``Js``, ``Jt``, and ``Jg``, with ``Jt = Jg`` and
``Js <= 2*Jt`` for a physically valid axisymmetric inertia. The inertia comparisons allow a relative
tolerance of :math:`10^{-12}`. ``rWB_B`` must be finite, and ``gsHat_B`` must be a finite unit vector
(squared norm within :math:`10^{-12}` of unity). Invalid configurations raise ``BSK_ERROR`` during
state registration or ``Reset()``. The axisymmetry restriction applies to balanced and simple-jitter
wheels when the option is enabled.

The option does not turn a simplified wheel model into a fully coupled imbalance model. Balanced
wheels continue to ignore imbalance parameters; simple-jitter wheels retain their existing applied
imbalance forces and torques, while their mass properties use nominal centered geometry. Use
``JitterFullyCoupled`` when the moving center of mass and coupled imbalance dynamics are needed.

When enabling the option on an existing simulation, remove the wheels' constant mass properties from
the hub and recompute its center of mass and inertia about that center. When disabling it, add the
balanced and simple-jitter wheel properties back into the hub. With the option enabled, switching
between balanced wheels and fully coupled wheels with zero imbalance and axisymmetric inertia does
not require repartitioning the hub mass properties.

Configuration Validation
~~~~~~~~~~~~~~~~~~~~~~~~

Every wheel model requires finite, positive ``Js``, since it is used in the spin dynamics regardless
of the mass-accounting option. Balanced and simple-jitter wheels with the default option do not
require ``mass``, ``Jt``, or ``Jg`` to be populated: those constant properties are accounted for in
the hub instead.

``JitterFullyCoupled`` always requires finite, positive ``mass``, ``Jt``, and ``Jg``, as well as the
finite position and unit spin axis described above. Its ``U_s`` and ``U_d`` must be finite, the derived
offset ``U_s / mass`` must remain finite, and the full rotor inertia tensor with ``J13 = U_d`` must
be positive definite. With positive diagonal moments, this last check requires ``U_d**2 < Js*Jg``.
These checks apply even when ``includeWheelMassProperties`` is ``False``, and occur before the
derived wheel configuration is used. They run during state registration and ``Reset()``.

Validation
~~~~~~~~~~

``test_reactionWheelMassProperties.py`` compares the enabled option with zero-imbalance,
axisymmetric fully coupled wheels and with the default model using manually combined hub properties.
It checks spacecraft mass, center of mass, inertia, translational and attitude histories, wheel
speeds, energy, and angular momentum for balanced, simple-jitter, and mixed configurations. The
tests use offset wheels and a nonzero hub center of mass. Free-motion cases verify energy and
momentum conservation; motor-driven cases verify momentum conservation and matching energy histories.
Additional cases check unchanged default accounting, invalid simplified and fully coupled
configuration handling, and spin-inertia validation across all models.

Initialization and Reset
~~~~~~~~~~~~~~~~~~~~~~~~

When the attached spacecraft registers the effector states, the module initializes the wheel-speed and applicable
wheel-angle states. For each fully coupled jitter wheel, it also derives the center-of-mass offset ``d`` from
``U_s / mass`` and sets the off-diagonal inertia ``J13`` to ``U_d``. This mandatory attachment path initializes the
dynamics configuration even when the reaction wheel state effector is not added to a simulation task.

The ``Reset()`` method refreshes these derived configuration values, initializes every wheel's command entry to zero,
reports a warning when a Stribeck coefficient is zero, and clears the wheel-speed output buffer.

The wheel count cannot exceed `MAX_EFF_CNT
<https://github.com/AVSLab/basilisk/blob/develop/src/architecture/utilities/macroDefinitions.h>`__, because the effector
publishes a fixed-size wheel-speed array even when its motor-command input is unlinked. State registration, ``Reset()``, command
reading, and wheel-speed publication reject excessive counts with ``BasiliskError`` before accessing the arrays.
Command storage is also sized during state registration and input reading, so it does not depend on a scheduled
``Reset()``. Direct calls to ``ConfigureRWRequests()`` may supply partial ``NewRWCmds`` vectors, but the vector
cannot exceed the wheel count.

Configure all wheels and their models before ``InitializeSimulation()`` registers their states. After registration,
``addReactionWheel()`` raises ``BasiliskError`` without adding a device. The wheel count and the allocation of a
jitter-angle state to each wheel must remain unchanged. Changes through ``ReactionWheelData`` are rejected before
reset, command processing, output publication, or dynamics access, even when the effector is absent from the task.
The derived ``numRW`` and ``numRWJitter`` counts must not be edited. ``Reset()`` preserves the registered states;
use a new effector and simulation when changing the state layout. Calling ``Reset()`` before state registration
does not freeze the layout.

Threshold Parameters
~~~~~~~~~~~~~~~~~~~~

The module includes two configurable threshold parameters:

* ``maxWheelAcceleration``: Maximum allowed wheel acceleration to prevent numerical instability. Default value is 1.0e6 rad/s^2.
* ``largeTorqueThreshold``: Threshold for warning about large torque with unlimited torque setting. Default value is 10.0 Nm.

These parameters can be accessed and modified using the following getter and setter methods:

.. code-block:: python

    # Get the current maximum wheel acceleration threshold
    current_max_accel = reactionWheelStateEffector.getMaxWheelAcceleration()

    # Set a new maximum wheel acceleration threshold
    reactionWheelStateEffector.setMaxWheelAcceleration(2.0e6)  # rad/s^2

    # Get the current large torque threshold
    current_torque_threshold = reactionWheelStateEffector.getLargeTorqueThreshold()

    # Set a new large torque threshold
    reactionWheelStateEffector.setLargeTorqueThreshold(15.0)  # Nm
