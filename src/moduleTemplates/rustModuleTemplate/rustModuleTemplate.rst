.. Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
..
.. SPDX-License-Identifier: ISC

Executive Summary
-----------------

This basic Rust Basilisk module can be copied as a starting point for a new
Rust module. It demonstrates both an individual message connection and a
fixed-size array of two message connections. The module reads optional input
messages, increments each first data-vector element, and writes the results to
the corresponding outputs. Its implementation is in ``rustModuleTemplate.rs``
beside this documentation file, following the normal Basilisk module naming
convention.

Message Connection Descriptions
-------------------------------

The following diagram and table list the module input and output messages.

.. bsk-module-io:: rustModuleTemplate
   :caption: Module I/O Messages
   :module-type: Rust

   input dataInMsg CModuleTemplateMsgPayload
      (optional) Input data vector. A zero vector is used when this message is
      not connected.

   input dataInMsgs CModuleTemplateMsgPayload
      (optional, two-element array) Input data vectors. A zero vector is used
      for each element that is not connected.

   output dataOutMsg CModuleTemplateMsgPayload
      Input data vector with its first element incremented by the module state.

   output dataOutMsgs CModuleTemplateMsgPayload
      (two-element array) Input data vectors with each first element
      incremented by the module state.

Module Assumptions and Limitations
----------------------------------

This module is a template only and does not model a physical system. It
demonstrates the Rust module lifecycle, explicit message-port annotations,
named input and output values, optional message inputs, output messages, and
fixed-size arrays of message ports. It also demonstrates a Python-configurable
``increment`` parameter that is initialized in Rust and validated during reset
with ``BskResult``. The ``panicOnUpdate`` field is a test-only fault-injection
hook used to verify that the generated ABI contains an unexpected Rust panic
before it crosses into C++.

User Guide
----------

Enable Rust module support when configuring Basilisk as described in
:ref:`rustModules`. Import and add the module to a task like any other
compiled Basilisk module:

.. code-block:: python

   from Basilisk.architecture import messaging
   from Basilisk.moduleTemplates import rustModuleTemplate

   module = rustModuleTemplate.rustModuleTemplate()
   simulation.AddModelToTask("taskName", module)

``module.increment`` defaults to 1 and must be finite and strictly positive.
An invalid value returns ``BskError`` from Rust and causes initialization to
raise ``BasiliskError`` without using cross-language unwinding.

Leave ``module.panicOnUpdate`` set to its default value of ``False`` during
normal use. Setting it to ``True`` deliberately panics during update so the
unit tests can verify panic containment and the rejection of later lifecycle
calls on the poisoned instance. Rust still destroys the instance normally.
The panic is reported once as ``BasiliskError`` without an additional default
Rust panic-hook report.
Operational modules should return ``BskError`` for expected failures rather
than adding a similar test hook.

The template's Rust implementation shows how one individual output and a
fixed-size output array are published together:

.. code-block:: rust

   Ok(RustModuleTemplateOutputs {
       dataOutMsg: data_out_msg,
       dataOutMsgs: data_out_msgs,
   })

The generated lifecycle code writes ``data_out_msg`` to ``dataOutMsg``. It
also writes ``data_out_msgs[0]`` and ``data_out_msgs[1]`` to the corresponding
elements of ``dataOutMsgs``. The returned values contain only
``CModuleTemplateMsg`` payload data. Basilisk automatically stamps every
published message with this module's ``moduleID`` and the current simulation
time, and sets its ``isWritten`` flag. Both reset and update return a complete
``RustModuleTemplateOutputs`` value, so all three output messages are
published after each successful call.

Connect ``module.dataInMsg`` when input data is available. When it is unconnected,
the module starts from a zero vector.

The ``dataInMsgs`` and ``dataOutMsgs`` fields demonstrate fixed-size arrays of
message ports. Python exposes each as a two-element list of normal Basilisk
message interfaces:

.. code-block:: python

   input_messages = [
       messaging.CModuleTemplateMsg().write(first_payload),
       messaging.CModuleTemplateMsg().write(second_payload),
   ]
   for input_port, input_message in zip(module.dataInMsgs, input_messages):
       input_port.subscribeTo(input_message)

   output_recorders = [
       output_port.recorder() for output_port in module.dataOutMsgs
   ]
   for recorder in output_recorders:
       simulation.AddModelToTask("taskName", recorder)

Each element is a live port. The array length is fixed by the Rust declaration,
and the property cannot be assigned. Changing the entries or length of the
returned Python list does not change the module's fixed set of ports. See
:ref:`rustModules` for required arrays, optional arrays, lifecycle value types,
and the current fixed-size-only limitation.
