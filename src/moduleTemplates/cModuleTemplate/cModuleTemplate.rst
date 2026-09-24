Executive Summary
-----------------
The C module template demonstrates message handling, a simple vector calculation, reset
behavior, and variable logging. On every update, it copies an optional input vector and
adds an increasing counter to the first component. It is a working starting point for
new C modules; :ref:`cppModuleTemplate` implements the same example in C++.

This page also serves as an example of module documentation. See :ref:`makingModules-3`
for the RST authoring tutorial, including equations, figures, tables, and code blocks.

Module Assumptions and Limitations
----------------------------------
The calculation is an instructional example with dimensionless vectors and a dimensionless
counter. It does not model a physical system. The counter advances once per scheduled
update, independently of the task period. No configuration is required to run the module.

The input is optional. When connected, the module uses the latest message payload; it
does not check its age or whether the publisher has written a new value since the last
update. When disconnected, the input vector is zero.

Message Connection Descriptions
-------------------------------
Connect the input from Python using ``subscribeTo()``. Both messages use the three-element
``dataVector`` field of :ref:`CModuleTemplateMsgPayload`.

.. bsk-module-io:: cModuleTemplate
    :caption: Module I/O Messages

    input dataInMsg CModuleTemplateMsgPayload
        Optional dimensionless input vector. Uses a zero vector when disconnected.

    output dataOutMsg CModuleTemplateMsgPayload
        Dimensionless input vector with the update counter added to its first component.
        Reset publishes a zero vector; each update publishes the calculated vector.

Detailed Module Description
---------------------------
Let :math:`\mathbf{x}_k` be the input vector at update :math:`k`, :math:`c_k` the counter,
and :math:`\mathbf{y}_k` the output. After reset, :math:`c_0 = 0`; the first update is
:math:`k = 1`. Each update computes

.. math::
    :label: eq-cModuleTemplate-update

    c_k = c_{k-1} + 1, \qquad
    \mathbf{y}_k = \mathbf{x}_k + \begin{bmatrix} c_k & 0 & 0 \end{bmatrix}^{T}.

For a disconnected input, :math:`\mathbf{x}_k = \mathbf{0}` and the output is
:math:`[c_k, 0, 0]^T`. The implementation demonstrates ``v3SetZero()`` and ``v3Copy()``
from :ref:`linearAlgebra`, and zeroes the output payload before populating it.

``SelfInit()`` initializes the C output message. ``Reset()`` clears the counter and writes
a zero output payload at the reset time. It preserves the sample configuration vector.
The next update starts the counter at one and evaluates Eq. :eq:`eq-cModuleTemplate-update`.

.. _moduleTemplateVariableRoles:

Configuration and Runtime State
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Configuration holds values supplied by the user, while runtime state holds values the module updates
as the simulation runs. The C configuration structure contains both kinds of data; all its members
are public. The sample variables in the C and C++ templates have these roles:

- ``updateCounter`` is a dimensionless runtime counter. ``Reset()`` clears it, including any value assigned
  before ``InitializeSimulation()``. Each update increments it and adds it to the first component
  of the input vector to form the output. The C++ setter demonstrates scalar validation and access.
- ``sampleConfigVector`` is a dimensionless sample configuration vector used to demonstrate assignment and
  variable logging. Resets and updates preserve it, and it does not affect the output calculation.
  Python-created modules default this vector to zero; the C++ setter requires positive components
  when used.
- ``inputVector`` is scratch storage for the current input. Each update overwrites it with the
  connected input vector or zeros if the optional input is disconnected. C stores it in the
  module structure; C++ uses a local array.

See :ref:`bskPrinciples-6` for an example of recording the counter and the sample vector.

User Guide
----------
The following complete script runs three updates, at 0, 0.5, and 1 second, and checks the
output vectors. Add the recorder after the module so it samples the newly written output
at each task time. ``InitializeSimulation()`` calls the module's initialization and reset
methods before the scheduled updates begin.

.. code-block:: python

    import numpy as np

    from Basilisk.architecture import messaging
    from Basilisk.moduleTemplates import cModuleTemplate
    from Basilisk.utilities import SimulationBaseClass, macros

    simulation = SimulationBaseClass.SimBaseClass()
    time_step = macros.sec2nano(0.5)  # [ns]
    process = simulation.CreateNewProcess("exampleProcess")
    process.addTask(simulation.CreateNewTask("exampleTask", time_step))

    module = cModuleTemplate.cModuleTemplate()
    module.ModelTag = "cModuleExample"
    module.sampleConfigVector = [1.0, 2.0, 3.0]  # [-] Preserved by reset.
    simulation.AddModelToTask("exampleTask", module)

    input_payload = messaging.CModuleTemplateMsgPayload()
    input_payload.dataVector = [1.0, 2.0, 3.0]  # [-]
    input_message = messaging.CModuleTemplateMsg().write(input_payload)
    module.dataInMsg.subscribeTo(input_message)

    recorder = module.dataOutMsg.recorder()
    simulation.AddModelToTask("exampleTask", recorder)
    simulation.InitializeSimulation()
    simulation.ConfigureStopTime(2 * time_step)
    simulation.ExecuteSimulation()

    expected = [[2.0, 2.0, 3.0], [3.0, 2.0, 3.0], [4.0, 2.0, 3.0]]  # [-]
    np.testing.assert_array_equal(recorder.dataVector, expected)
    np.testing.assert_array_equal(recorder.times(), [0, time_step, 2 * time_step])
    assert module.updateCounter == 3

To try the disconnected-input case, omit the ``subscribeTo()`` call and change ``expected``
to ``[[1.0, 0.0, 0.0], [2.0, 0.0, 0.0], [3.0, 0.0, 0.0]]``. The initial zero payload
written by reset is replaced by the first update before the recorder samples at time zero.
