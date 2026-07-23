.. toctree::
   :maxdepth: 1
   :hidden:

.. _writingRustPlugins:

Quick Start: Writing a Rust Basilisk Extension
===============================================

.. sidebar:: Rust Extension Example

    The `bsk-sdk repository <https://github.com/AVSLab/bsk_sdk>`_ contains a
    complete `Rust MRP extension
    <https://github.com/AVSLab/bsk_sdk/tree/master/examples/rust-mrp-plugin>`_.

.. warning::

   Rust extension support is experimental. Its interface may change between
   ``bsk-sdk`` releases.

This guide explains how to package and build a Basilisk extension whose module
lifecycle methods are implemented in Rust. It uses the same extension packaging
described in :ref:`writingExtensions`
(``pyproject.toml``, a Python package directory, ``bsk-sdk`` as the build
dependency), plus ``bsk_add_rust_module`` to compile the Cargo crate and add it
to the Python extension.

Define a ``#[repr(C)]`` configuration struct and implement ``BskModule``. The
Rust build tooling generates the C header and wrapper code required by
Basilisk.

.. note::

   To contribute a Rust module directly to the Basilisk source tree, see
   :ref:`rustModules`.

Prerequisites
-------------

You need Rust and Cargo, CMake 3.26 or newer, and a C++17 compiler. Create a
clean Python environment and install matching Basilisk and SDK versions:

.. code-block:: bash

    python3 -m venv .venv
    source .venv/bin/activate
    python -m pip install --upgrade pip
    python -m pip install "bsk[all]==2.X.Y" "bsk-sdk==2.X.Y" \
      build scikit-build-core cmake ninja

Replace ``2.X.Y`` with the targeted Basilisk release. The ``bsk`` and
``bsk-sdk`` versions must match exactly.

Build the Working Example First
-------------------------------

Before creating an extension, review and build the Rust MRP extension linked
above. It provides a minimal, working package layout and verifies that the
Python environment, Rust toolchain, CMake, SDK, and Basilisk installation work
together.

Create the Project Layout
-------------------------

A Rust extension uses a Python package and a Cargo crate. Build artifacts are
created in the CMake build tree, not in the crate source directory:

.. code-block:: text

    my-rust-plugin/
    |-- pyproject.toml
    |-- CMakeLists.txt
    |-- my_rust_plugin/             # Python package
    |   `-- __init__.py
    `-- myModule/                   # Rust crate
        |-- Cargo.toml
        |-- build.rs
        |-- myModule.rs             # Configuration struct and module implementation
        |-- myModule.rst            # Basilisk module documentation
        `-- _UnitTest/
            `-- test_myModule.py

.. code-block:: rust

    // build.rs
    fn main() {
        bsk_build::generate_from("myModule.rs");
    }

The Rust source and reStructuredText documentation names match the module
directory, following the standard Basilisk module layout. The source path in
``build.rs`` must match the library path in ``Cargo.toml``.

Add ``bsk-build`` as both a normal and build dependency. Enable its
``codegen`` feature for the build dependency:

``bsk-build``, ``bsk-messages``, and ``bsk-utilities`` are not yet published
on crates.io. Depend on the Basilisk repository through Cargo's ``git``
support:

.. code-block:: toml

    [lib]
    path = "myModule.rs"
    crate-type = ["staticlib"]

    [dependencies]
    bsk-messages = { git = "https://github.com/AVSLab/basilisk", tag = "v2.X.Y" }
    bsk-build    = { git = "https://github.com/AVSLab/basilisk", tag = "v2.X.Y" }

    [build-dependencies]
    bsk-build    = { git = "https://github.com/AVSLab/basilisk", tag = "v2.X.Y", features = ["codegen"] }

Replace ``v2.X.Y`` with the targeted Basilisk release. To develop against a
local Basilisk checkout, add this override to a gitignored
``.cargo/config.toml``:

.. code-block:: toml

    [patch."https://github.com/AVSLab/basilisk"]
    bsk-messages = { path = "/path/to/basilisk/src/architecture/rust/bsk_messages" }
    bsk-build    = { path = "/path/to/basilisk/src/architecture/rust/bsk_build" }

An out-of-tree extension is not a member of Basilisk's in-tree Cargo
workspace. After configuring its dependencies, generate and commit the
extension's own lockfile so CMake can build with deterministic dependencies:

.. code-block:: bash

    cargo generate-lockfile --manifest-path myModule/Cargo.toml

The CMake integration passes ``--locked`` to Cargo and reports an error when
the lockfile is missing or inconsistent with the manifest.

Write the Rust Module
---------------------

Define the configuration struct and implement ``BskModule``. The build tooling
generates the C header, SWIG wrapper, and message I/O code from this definition:

.. code-block:: rust

    use bsk_messages::*;

    #[repr(C)]
    pub struct myModuleConfig {
        /// [-] SysModel runtime mirror — see below
        pub runtime: BskModuleRuntime,
        /// [Nm] proportional gain
        pub K: f64,
        /// [-] attitude guidance input
        pub attGuidInMsg: MsgReader<AttGuidMsg>,
        /// [Nm] commanded torque output
        pub cmdTorqueOutMsg: MsgWriter<CmdTorqueBodyMsg>,
        /// [-] BSK logging handle
        pub bskLogger: *mut BSKLogger,
    }

    impl BskModule for myModuleConfig {
        type Inputs = (AttGuidMsg,);
        type Outputs = (CmdTorqueBodyMsg,);

        fn update(&mut self, inputs: Self::Inputs, _current_sim_nanos: u64) -> Self::Outputs {
            let (att_guid_in_msg,) = inputs;
            // ... pure Rust control law ...
            (CmdTorqueBodyMsg { torqueRequestBody: [0.0, 0.0, 0.0] },)
        }
    }

    #[cfg(not(test))]
    bsk_build::bsk_module!();

``bsk-build`` reads the struct during ``build.rs`` and generates the C header
and wrapper code. ``bsk_module!()`` includes the generated code in the crate.

Field doc comments written with ``///`` become Doxygen comments in the
generated header. Follow the Basilisk unit-first convention used by the other
module languages, for example ``/// [m/s] Velocity``. Use ``[-]`` for
dimensionless fields when it improves clarity.

Module Lifecycle
~~~~~~~~~~~~~~~~

Three ``BskModule`` trait methods map to the Basilisk module lifecycle:

``init()``
   Called before Python configures the module. Override to set non-zero
   parameter defaults and initial state — the equivalent of a C++ module
   constructor. The default implementation is a no-op (all fields stay zero).

``reset(current_sim_nanos)`` → ``Self::Outputs``
   Called at simulation start and on every ``Reset()``. Returns initial
   values for every output port; the framework writes them automatically.
   The default implementation returns ``Self::Outputs::default()``. Override
   when the module has non-zero initial outputs, parameter validation, or
   state to reset.

``update(inputs, current_sim_nanos)`` → ``Self::Outputs``
   Called every tick. Receives message values (not ports) and returns output
   values; the framework handles all I/O. No default — must always be
   implemented.

``Inputs`` and ``Outputs`` are tuples matching the ``MsgReader`` and
``MsgWriter`` fields in declaration order.

Gate ``bsk_module!()`` with ``#[cfg(not(test))]`` so ``cargo test`` (see
`Testing`_ below) does not require Basilisk to be linked.

Use the Generated Wrapper
~~~~~~~~~~~~~~~~~~~~~~~~~

The generated Python module class, named after the CMake target, provides the
``SysModel`` interface. Construct and schedule it as for a C or C++ module:

.. code-block:: python

    from my_rust_plugin import myModule

    module = myModule.myModule()
    simulation.AddModelToTask("taskName", module)

The wrapper owns a separate Rust configuration object and exposes its
parameters and message ports directly. Configure fields and connect messages
through ``module``.

Supported Configuration Fields
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Every module configuration struct must include a field named ``runtime`` of
type ``BskModuleRuntime``. All other fields are optional. They can include:

* scalar, fixed-size array (``[T; N]``, multi-dimensional ``[[T; N]; M]``, etc.), and nested ``#[repr(C)]`` parameter fields;
* ``Option<Box<T>>`` for state owned by a stateful module;
* ``MsgReader<T>`` input ports and ``MsgWriter<T>`` output ports; and
* ``*mut BSKLogger`` for Basilisk logging.

Nested structs, owned state, and message ports have additional requirements
described below. ``bsk-build`` rejects unsupported field types, including raw
pointers other than ``*mut BSKLogger`` and Rust enums.

Runtime Context
~~~~~~~~~~~~~~~

``runtime: BskModuleRuntime`` provides module runtime information, such as the
module ID and name. ``bsk-build`` rejects a configuration struct without this
field. The generated wrapper updates it before each lifecycle call:

.. code-block:: rust

    fn update(&mut self, inputs: Self::Inputs, current_sim_nanos: u64) -> Self::Outputs {
        let id = self.runtime.module_id();
        // ...
    }

Values borrowed from ``runtime`` are valid only during the current lifecycle
call.

``current_sim_nanos`` is passed separately to ``reset`` and ``update``.

Logging
-------

A ``bskLogger: *mut BSKLogger`` field gets standard Basilisk logging through
the ``BskLoggerExt`` trait, which adds
``.debug()``/``.info()``/``.warning()``/``.bsk_error()`` methods directly on
the field:

.. code-block:: rust

    fn reset(&mut self, _current_sim_nanos: u64) -> Self::Outputs {
        if self.K <= 0.0 {
            self.bskLogger.warning("K should be positive");
        }
        Self::Outputs::default()   // shim writes these to the output ports
    }

``.bsk_error(msg)`` raises the standard fatal ``BasiliskError`` and never
returns.

In test builds (enabled by adding ``bsk-build`` with the ``test_logger``
feature to ``[dev-dependencies]``), every logger call prints to ``stderr``
and ``.bsk_error()`` panics — no C symbols required. This means
``bskLogger.warning(...)`` calls in ``reset()`` and ``update()`` work
for unit tests:

.. code-block:: toml

    # Cargo.toml
    [dev-dependencies]
    bsk-build = { git = "https://github.com/AVSLab/basilisk", tag = "v2.X.Y",
                  features = ["test_logger"] }

Messaging
---------

The ``bsk-messages`` crate
~~~~~~~~~~~~~~~~~~~~~~~~~~~

Add ``bsk-messages`` to the Cargo dependencies shown above. It provides the
standard Basilisk message types and re-exports ``BskModuleRuntime``,
``MsgReader``, ``MsgWriter``, ``BskModule``, and ``BskLoggerExt``. Import the
types with:

.. code-block:: rust

    use bsk_messages::*;

Built-in messages
~~~~~~~~~~~~~~~~~

Built-in message types are available through ``bsk-messages``. After adding
the crate dependency, import a type as above or use its fully qualified name,
such as ``bsk_messages::AttGuidMsg``.

Message ports: ``MsgReader``/``MsgWriter``
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

A config struct field of type ``MsgReader<Foo>`` is an input port that reads
``Foo`` values; ``MsgWriter<Foo>`` is an output port that writes them (``Foo``
being, for example, ``AttGuidMsg`` or ``CmdTorqueBodyMsg``). The generated
wrapper reads inputs, passes their values to ``update()``, and writes the
returned output values to their ports.

Multiple inputs and outputs
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Any number of ``MsgReader``/``MsgWriter`` fields are supported. They map to
the ``BskModule::Inputs``/``Outputs`` tuple types, **in field declaration
order**:

.. code-block:: rust

    #[repr(C)]
    pub struct myModuleConfig {
        pub runtime: BskModuleRuntime,
        pub navAttInMsg: MsgReader<NavAttMsg>,
        pub attRefInMsg: MsgReader<AttRefMsg>,
        pub cmdTorqueOutMsg: MsgWriter<CmdTorqueBodyMsg>,
        pub cmdRateOutMsg: MsgWriter<RateCmdMsg>,
        pub bskLogger: *mut BSKLogger,
    }

    impl BskModule for myModuleConfig {
        type Inputs = (NavAttMsg, AttRefMsg);
        type Outputs = (CmdTorqueBodyMsg, RateCmdMsg);

        fn update(&mut self, inputs: Self::Inputs, _t: u64) -> Self::Outputs {
            let (nav_att, att_ref) = inputs;
            // ...
            (CmdTorqueBodyMsg { ..Default::default() },
             RateCmdMsg { ..Default::default() })
        }
    }

A module with zero inputs uses ``type Inputs = ();`` and
``update(&mut self, _: (), t: u64)``; zero outputs uses
``type Outputs = ();`` and returns ``()``.

Required vs. optional inputs
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Whether an input is required or optional is determined from the ``Inputs``
tuple type in the ``impl BskModule`` block. There is nothing extra to
annotate: the type already required for ``update`` to type-check is the
single source of truth.

- A bare message type (``AttGuidMsg``) means **required**: connectivity is
  checked in ``Reset`` and before every ``Update`` read, raising the
  standard ``BasiliskError`` if unconnected.
- Wrapping the type in ``Option<Msg>`` means **optional**: ``update()``
  receives ``Option<Msg>`` (``None`` when unlinked) instead of an error.

.. code-block:: rust

    pub struct myModuleConfig {
        pub runtime: BskModuleRuntime,
        pub attGuidInMsg: MsgReader<AttGuidMsg>,
        /// supplemental disturbance-torque estimate
        pub disturbanceInMsg: MsgReader<CmdTorqueBodyMsg>,
        // ...
    }

    impl BskModule for myModuleConfig {
        type Inputs = (AttGuidMsg, Option<CmdTorqueBodyMsg>);
        //             ^^^^^^^^^^  ^^^^^^^^^^^^^^^^^^^^^^^^^
        //             required    optional (attGuidInMsg / disturbanceInMsg,
        //                         matched by field declaration order)
        // ...
    }

``bsk-build`` matches ``Inputs`` tuple elements to ``MsgReader`` fields by
declaration order and fails the build immediately if the counts or message
types don't line up, rather than silently misrouting inputs.

Outputs
~~~~~~~

Output ports are initialized in both ``reset()`` and ``update()``. Both
methods must return a value for every output port (``Self::Outputs``); the
generated shim writes the returned values to the ports with the module ID
and current simulation timestamp. This means every output is guaranteed to
hold a valid, module-authored value before the first ``UpdateState`` tick.

``reset()`` has a default implementation that returns
``Self::Outputs::default()``, so a module only needs to override it when the
initial outputs should be non-zero, when parameters need validating, or when
internal state needs resetting. ``update()`` has no default and must always
be implemented.

Custom message types
~~~~~~~~~~~~~~~~~~~~

A **custom** message type needs its ``*_C`` C-interface header defined the
same way any Basilisk extension defines a custom message: write (or generate,
via Basilisk's message codegen pipeline) a C struct payload and a
``*_C.h``/``*_C.cpp`` interface pair, and add its directory to
``bsk_add_rust_module(... INCLUDE_DIRS ...)``. Once that header exists, the
field is treated identically to any built-in message port.

``bsk-messages`` is generated from Basilisk's ``cMsgCInterface`` headers and
committed to this repository. Using it does not require ``bindgen`` or
``libclang``; those tools are only needed when regenerating the bindings after
a message payload changes.


Stateful modules — owned heap state
--------------------------------------

Modules needing heap state that persists across steps (filters, integrators)
add an ``Option<Box<T>>`` field — ordinary, safe Rust ownership, no manual
pointer casts:

.. code-block:: rust

    #[repr(C)]
    pub struct myModuleConfig {
        pub runtime: BskModuleRuntime,
        pub state: Option<Box<MyState>>,
        // ...
    }

    impl BskModule for myModuleConfig {
        fn init(&mut self) {
            self.state = Some(Box::new(MyState::default()));
        }

        fn update(&mut self, inputs: Self::Inputs, _t: u64) -> Self::Outputs {
            let state = self.state.as_mut().expect("set in init");
            // ...
        }
    }

``bsk-build`` maps ``Option<Box<T>>`` to a nullable ``void *`` in the
generated header and arranges for it to be freed automatically whenever the
owning object is destroyed.

Python has no legitimate reason to touch this field directly, so
``bsk_add_rust_module`` marks it ``%immutable``: the setter is absent from
the Python API, so a script can't null it out (leaking the boxed value) or
alias it to an unrelated pointer (causing a crash on cleanup). The getter
still works and returns an opaque pointer handle.

Grouping parameters — nested structs
--------------------------------------

A field may be another ``#[repr(C)]`` struct defined in the same crate, by
value, to group related parameters:

.. code-block:: rust

    #[repr(C)]
    pub struct Vec2 {
        pub x: f64,
        pub y: f64,
    }

    #[repr(C)]
    pub struct myModuleConfig {
        pub runtime: BskModuleRuntime,
        pub target: Vec2,
        // ...
    }

``bsk-build`` generates ``Vec2``'s own C struct ahead of ``myModuleConfig``'s,
and Python reads and writes it field-by-field like any other struct member
(``ctrl.target.x = 1.0``). Nested structs cannot be self-referential. Message ports and
``Option<Box<T>>`` owned state are only meaningful on the top-level config
struct and are rejected on a nested struct.

A field may **not** be a raw pointer to one of these structs (e.g. ``*mut
Vec2``), or to any other type — ``BSKLogger`` is the only pointee
``bsk-build`` allows. Python setters for a pointer field transfer ownership
away from the Python object, and nothing on the Rust side would ever free
it; embedding by value (above) has no such issue, so ``bsk-build`` rejects
every other pointer form outright, including a raw pointer to a primitive
(e.g. ``*mut u8``). There is currently no supported field type for a
persistent string or byte-buffer parameter.

A field also may **not** be a Rust ``enum``, even a fieldless ``#[repr(u8)]``
(or similar) one — SWIG's setter for an enum-typed field accepts any integer,
not just ones matching a declared variant, so a match on the field's value
could hit undefined behavior once Python is free to write, say, ``5`` into a
field that's only supposed to be ``0`` or ``1``. Use the underlying integer
type as the field (e.g. ``pub mode: u8``) and convert it to your enum with a
checked conversion inside ``update``/``reset`` instead.

Fixed-size arrays
------------------

A field of type ``[T; N]`` — ``T`` a Rust primitive or a nested
``#[repr(C)]`` struct, ``N`` a plain integer literal — is supported and maps
to the standard C array declaration ``T name[N];``. This is the same form
used throughout Basilisk's C message payloads (e.g.
``double torqueRequestBody[3]``). Multi-dimensional arrays are also
supported: ``[[T; N]; M]`` maps to ``T name[M][N];``.

.. code-block:: rust

    #[repr(C)]
    pub struct myModuleConfig {
        pub runtime: BskModuleRuntime,
        /// [Nm] max torque per axis
        pub maxRwTorques: [f64; 3],
        /// [-] DCM from body to reference frame
        pub dcm_BR: [[f64; 3]; 3],
        // ...
    }

From Python, SWIG exposes a 1-D array as a flat list and a 2-D array as a
list of lists — the same convention as message payload fields:

.. code-block:: python

    module.maxRwTorques = [0.001, 0.001, 0.001]
    module.dcm_BR = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]

Array lengths must be plain integer literals. Expressions that reference a
``const`` item, use arithmetic (``N * 2``), or involve generic parameters are
a build error: ``bsk-build`` runs before the crate's own constants are
evaluated.

.. note::

   SWIG returns a **fresh copy** of the array as a Python list on every read;
   mutating the returned list does not change the struct. You must re-assign
   to write back::

       v = module.maxRwTorques   # a copy — changes to v go nowhere
       v[0] = 0.005
       module.maxRwTorques = v   # this is the actual write

   For 1-D arrays, a shorter-than-expected input is silently zero-padded
   (``[0.1, 0.2]`` assigned to a ``[f64; 3]`` field becomes
   ``[0.1, 0.2, 0.0]``); a longer input raises ``ValueError``. This matches
   the behavior of all Basilisk message payload array fields.

Configure CMake
---------------

.. code-block:: cmake

    bsk_add_rust_module(
      TARGET      myModule
      MANIFEST    "${CMAKE_CURRENT_SOURCE_DIR}/myModule/Cargo.toml"
      OUTPUT_DIR  "${PKG_DIR}"
    )

This one call builds the crate, generates the C header and a matching SWIG
interface (auto-detecting message port and ``Option<Box<T>>`` fields — see
`Stateful modules — owned heap state`_ above), and links the result into
the SWIG extension. No hand-written C/C++ ``SOURCES`` are needed.

See the comment block at the top of ``cmake/bsk_add_rust_module.cmake`` in
the ``bsk-sdk`` repository for the full argument reference (``INTERFACE``,
``CRATE_NAME``, ``INCLUDE_DIRS``, ``LINK_LIBS``, ``DEPENDS``, ...).

Configure Python Packaging
--------------------------

.. code-block:: toml

    [build-system]
    requires = ["scikit-build-core>=0.9.3", "bsk-sdk==2.X.Y", "bsk==2.X.Y", "swig==4.4.1"]
    build-backend = "scikit_build_core.build"

    [project]
    dependencies = ["bsk==2.X.Y"]

    [tool.scikit-build]
    wheel.packages = []   # CMake installs the package directly; nothing to collect from source

.. note::

   ``bsk-sdk``, ``bsk``, and the runtime ``dependencies`` entry must all be
   **pinned to the same version**, exactly as for a C++/C extension (see
   :ref:`writingExtensions`). Replace ``2.X.Y`` with the Basilisk version being
   targeted.

Build, Install, and Test
------------------------

Testing
~~~~~~~

.. list-table::
   :header-rows: 1
   :widths: 25 45 20

   * - Layer
     - Command
     - Needs Basilisk?
   * - Pure-Rust unit tests
     - ``cargo test --locked`` (from the module's crate dir)
     - No
   * - Raw CMake build + CTest
     - ``cmake -S . -B _build && cmake --build _build && ctest --test-dir _build``
     - Yes (build only)
   * - Editable install + pytest
     - ``pip install -e . --no-build-isolation && pytest <module>/_UnitTest``
     - Yes
   * - Installed wheel
     - ``python -m build --wheel --no-isolation`` then ``pip install dist/*.whl``
     - Yes

**``cargo test`` never touches Basilisk.** Gate ``bsk_module!()`` behind
``#[cfg(not(test))]`` (see `Write the Rust Module`_
above) so ``init()``, ``reset()``, and ``update()`` can be exercised as
plain Rust functions with hand-built message values — no linking, no Python.

Add the ``test_logger`` dev-dependency (see `Logging`_ above) so that
``bskLogger.warning(...)`` and similar calls work in unit tests. Logger calls in test
builds print to ``stderr`` rather than calling Basilisk's C symbols.

Common Build Problems
---------------------

``Basilisk version mismatch``
   Use the same exact Basilisk version for ``bsk``, ``bsk-sdk``, and the Cargo
   dependencies, then rebuild the extension.

``Cargo cannot obtain Basilisk crates``
   Confirm that the Cargo ``git`` dependency points to a released Basilisk tag
   or configure the local ``[patch]`` override shown above.

``Custom message type is unavailable``
   Define its ``*_C`` C-interface header and include its directory in
   ``bsk_add_rust_module(... INCLUDE_DIRS ...)``.

``cargo test`` attempts to link Basilisk
   Ensure that ``bsk_module!()`` is gated with ``#[cfg(not(test))]``.

Known limitations
--------------------

- Rust module support is experimental; the ABI (``BskModuleRuntime``,
  the generated SWIG interface, the ``BskModule`` trait) may change between
  ``bsk-sdk`` releases.
- ``bsk-build``, ``bsk-messages``, and ``bsk-utilities`` are not yet published
  on crates.io. Extensions depend on this repository through a Cargo ``git``
  dependency (see `Create the Project Layout`_ above).
- ``bsk-messages``' and ``bsk-utilities``' bindings
  (``src/architecture/rust/gen_rust_messages.py`` and
  ``src/architecture/rust/gen_rust_utilities.py``) are generated and committed
  here, so they only need regenerating (and re-committing) when the underlying
  C message or utility headers change — not on every checkout.
- Custom message types require a hand-written ``*_C`` C-interface header —
  there is no Rust-side message-definition DSL.
- ``bsk-utilities`` covers C ABI utilities/constants only; C++/Eigen
  utilities (``Eigen::Vector3d``, ``Eigen::Matrix3d``, ...) are out of scope
  until a dedicated C shim is designed.

Reference
---------

The ABI reference is the header comment in
:download:`bsk_rust_module.h <../../../src/architecture/_GeneralModuleFiles/bsk_rust_module.h>`,
and the module documentation in ``src/architecture/rust/bsk_build/src/lib.rs``.
