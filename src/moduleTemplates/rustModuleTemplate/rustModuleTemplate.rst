.. Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
..
.. SPDX-License-Identifier: ISC

Executive Summary
-----------------

This basic Rust Basilisk module can be copied as a starting point for a new
Rust module. It reads an optional input message, increments the first element,
and writes the result to its output message. Its implementation is in
``rustModuleTemplate.rs`` beside this documentation file, following the normal
Basilisk module naming convention.

Message Connection Descriptions
-------------------------------

The following diagram and table list the module input and output messages.

.. bsk-module-io:: rustModuleTemplate
   :caption: Module I/O Messages
   :module-type: Rust

   input dataInMsg CModuleTemplateMsgPayload
      (optional) Input data vector. A zero vector is used when this message is
      not connected.

   output dataOutMsg CModuleTemplateMsgPayload
      Input data vector with its first element incremented by the module state.

Module Assumptions and Limitations
----------------------------------

This module is a template only and does not model a physical system. It
demonstrates the Rust module lifecycle, explicit message-port annotations,
named input and output values, optional message inputs, output messages, and
logging. It also demonstrates a Python-configurable ``increment`` parameter
that is initialized in Rust and validated during reset with ``BskResult``.
The ``panicOnUpdate`` field is a test-only fault-injection hook used to verify
that the generated ABI contains an unexpected Rust panic before it crosses
into C++.

User Guide
----------

Enable Rust module support when configuring Basilisk as described in
:ref:`rustModules`. Import and add the module to a task like any other
compiled Basilisk module:

.. code-block:: python

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

Connect ``module.dataInMsg`` when input data is available. When it is unconnected,
the module starts from a zero vector.
