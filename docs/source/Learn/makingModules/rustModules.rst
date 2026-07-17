.. Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
..
.. SPDX-License-Identifier: ISC

.. _rustModules:

Making Rust Modules
===================

.. sidebar:: Rust Module Support

   Rust module support is experimental. The interface may change between
   Basilisk releases.

Rust modules are compiled Basilisk modules written in Rust. They use the same
module lifecycle and Python interface as C and C++ modules, but define the
module configuration and behavior with a Rust ``#[repr(C)]`` struct and the
``BskModule`` trait.

This page describes modules contributed to the Basilisk source tree. For a
separately packaged Rust extension, see :ref:`writingRustPlugins`.

The :ref:`rustModuleTemplate` module provides a minimal in-tree Rust module
that can be copied as a starting point.

Building with Rust Support
--------------------------

Rust module discovery is disabled by default, so that Rust and Cargo remain
optional for users who do not build Rust modules. Install Rust and Cargo, then
configure Basilisk with:

.. code-block:: bash

    python3 conanfile.py --rustModules True

This enables Rust module discovery and builds every in-tree module crate. Use
the normal Basilisk build and test workflow after configuration.

Module Layout
-------------

Place a Rust module in a normal Basilisk module directory. The Cargo manifest
identifies the directory as a Rust module; no hand-written C header or SWIG
interface file is needed:

.. code-block:: text

    src/fswAlgorithms/<category>/myModule/
    |-- Cargo.toml
    |-- build.rs
    |-- src/
    |   `-- lib.rs
    `-- _UnitTest/
        `-- test_myModule.py

The module target name is the directory containing ``Cargo.toml``. Basilisk
generates the C header and SWIG interface while building the crate.

Configure the Crate
-------------------

Set the crate type to ``staticlib`` and depend on the in-tree Rust support
crates. From the layout above, ``Cargo.toml`` contains:

.. code-block:: toml

    [package]
    name = "myModule"
    version = "0.1.0"
    edition = "2021"

    [lib]
    crate-type = ["staticlib"]

    [dependencies]
    bsk-messages = { path = "../../../../rust/bsk_messages" }
    bsk-build = { path = "../../../../rust/bsk_build" }

    [build-dependencies]
    bsk-build = { path = "../../../../rust/bsk_build", features = ["codegen"] }

The path to each crate is relative to the module directory. Adjust it if the
module is placed elsewhere in the source tree.

The ``build.rs`` file generates the C and SWIG bindings:

.. code-block:: rust

    fn main() {
        bsk_build::generate();
    }

Write the Module
----------------

Import the support types from ``bsk-messages``, define a ``#[repr(C)]``
configuration struct, and implement ``BskModule``:

.. code-block:: rust

    use bsk_messages::*;

    #[repr(C)]
    pub struct myModuleConfig {
        /// [-] Basilisk runtime information
        pub runtime: BskModuleRuntime,
        /// [Nm] proportional gain
        pub K: f64,
        /// [-] attitude guidance input
        pub attGuidInMsg: MsgReader<AttGuidMsg>,
        /// [Nm] commanded torque output
        pub cmdTorqueOutMsg: MsgWriter<CmdTorqueBodyMsg>,
        /// [-] Basilisk logger
        pub bskLogger: *mut BSKLogger,
    }

    impl BskModule for myModuleConfig {
        type Inputs = (AttGuidMsg,);
        type Outputs = (CmdTorqueBodyMsg,);

        fn update(&mut self, inputs: Self::Inputs, _current_sim_nanos: u64) -> Self::Outputs {
            let (att_guid_in_msg,) = inputs;
            let _ = att_guid_in_msg;
            (CmdTorqueBodyMsg::default(),)
        }
    }

    #[cfg(not(test))]
    bsk_build::bsk_module!();

The configuration struct must contain a field named ``runtime`` with type
``BskModuleRuntime``. Other fields are optional and can be scalars, fixed-size
arrays (``[T; N]`` or multi-dimensional ``[[T; N]; M]`` with literal lengths),
nested ``#[repr(C)]`` structs, ``Option<Box<T>>`` state,
``MsgReader<T>`` and ``MsgWriter<T>`` ports, and a ``*mut BSKLogger`` field.

The ``Inputs`` and ``Outputs`` tuples match message ports in declaration order.
A bare input type is required; wrap an input type in ``Option<T>`` to receive
``None`` when its message is unlinked.

``update`` is required and receives message values rather than message ports.
Both ``reset`` and ``update`` must return ``Self::Outputs``; the framework
writes the returned values to the output ports. ``reset`` has a default
implementation that returns ``Self::Outputs::default()``. Override it when the
module needs non-zero initial output values, parameter validation, or state
reset.

Override ``init()`` to set non-zero parameter defaults and initial state.
It runs before Python configures the module; the default implementation is a
no-op (all fields start at zero).

Use the Generated Wrapper
-------------------------

The generated wrapper, named after the module target, provides the Basilisk
``SysModel`` interface. Construct and schedule it as for a C or C++ module:

.. code-block:: python

    module = myModule.myModule()
    simulation.AddModelToTask("taskName", module)

The wrapper owns a separate Rust configuration object and exposes its
parameters and message ports directly. Configure fields and connect messages
through ``module``.

Built-in and Custom Messages
----------------------------

``bsk-messages`` provides Rust bindings for built-in Basilisk message types.
After adding the dependency shown above, import types with
``use bsk_messages::*;`` or use fully qualified names such as
``bsk_messages::AttGuidMsg``.

Custom message types require their normal Basilisk ``*_C`` C-interface header.

Testing
-------

Gate ``bsk_module!()`` with ``#[cfg(not(test))]`` as shown above. This lets
pure Rust tests run without linking Basilisk:

.. code-block:: bash

    cd src/fswAlgorithms/<category>/myModule
    cargo test

``init()``, ``reset()``, and ``update()`` are all testable this way.
Logger calls (``bskLogger.warning(...)`` etc.) work in tests when the ``test_logger`` dev-dependency
feature is enabled:

.. code-block:: toml

    # Cargo.toml
    [dev-dependencies]
    bsk-build = { path = "../../../../rust/bsk_build", features = ["test_logger"] }

Logger calls in test builds print to ``stderr`` instead of calling
Basilisk's C symbols. ``bsk_module!()`` must still be gated with
``#[cfg(not(test))]`` because the generated shim references message-port
C symbols.

Add the normal Basilisk Python unit test in the module's ``_UnitTest``
directory. Run it after building Basilisk with Rust support:

.. code-block:: bash

    python3 -m pytest src/fswAlgorithms/<category>/myModule/_UnitTest -v

Before contributing a module, follow the :ref:`bskModuleCheckoutList`.

Further Reading
---------------

The :ref:`writingRustPlugins` guide covers advanced Rust configuration types,
stateful modules, logging, and testing. It also explains how to package the
same kind of module as an out-of-tree extension.
