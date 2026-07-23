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
logging.

User Guide
----------

Enable Rust module support when configuring Basilisk as described in
:ref:`rustModules`. Import and add the module to a task like any other
compiled Basilisk module:

.. code-block:: python

   from Basilisk.moduleTemplates import rustModuleTemplate

   module = rustModuleTemplate.rustModuleTemplate()
   simulation.AddModelToTask("taskName", module)

Connect ``module.dataInMsg`` when input data is available. When it is unconnected,
the module starts from a zero vector.
