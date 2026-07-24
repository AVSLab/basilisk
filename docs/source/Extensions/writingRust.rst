.. toctree::
   :maxdepth: 1
   :hidden:

.. _writingRustExtensions:

Quick Start: Writing a Rust Basilisk Extension
===============================================

.. sidebar:: Rust Extension Example

    The `bsk-sdk repository <https://github.com/AVSLab/bsk_sdk>`_ contains a
    complete Rust MRP extension example.

.. warning::

   Rust extension support is experimental. Its interface may change between
   ``bsk-sdk`` releases.

This guide explains how to package and build a Basilisk extension whose module
lifecycle methods are implemented in Rust. It uses the same extension packaging
described in :ref:`writingExtensions`
(``pyproject.toml``, a Python package directory, ``bsk-sdk`` as the build
dependency), plus ``bsk_add_rust_module`` to compile the Cargo crate and add it
to the Python extension.

Mark a ``#[repr(C)]`` configuration struct with ``#[bsk_build::module]`` and
implement ``BskModule``. The Rust build tooling generates the C header and
wrapper code required by Basilisk.

.. note::

   To contribute a Rust module directly to the Basilisk source tree, see
   :ref:`rustModules`.

Prerequisites
-------------

You need Rust 1.85 or newer and Cargo, CMake 3.26 or newer, and a C++17
compiler. Create a clean Python environment and install matching Basilisk and
SDK versions:

.. code-block:: bash

    python3 -m venv .venv
    source .venv/bin/activate
    python -m pip install --upgrade pip
    python -m pip install "bsk[all]==2.X.Y" "bsk-sdk==2.X.Y" \
      build scikit-build-core cmake ninja

Replace ``2.X.Y`` with the targeted Basilisk release. The ``bsk`` and
``bsk-sdk`` versions must match exactly. Rust 1.85 is the minimum supported
Rust version; a newer compatible stable toolchain may be used.

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

    my-rust-extension/
    |-- pyproject.toml
    |-- CMakeLists.txt
    |-- my_rust_extension/          # Python package
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
        bsk_build::generate_bindings("myModuleConfig");
    }

The Rust source and reStructuredText documentation names match the module
directory, following the standard Basilisk module layout. Pass the exact name
of the struct marked with ``#[bsk_build::module]`` to
``generate_bindings()``. Cargo obtains the source path from ``[lib] path``.

Add ``bsk-build`` as both a normal and build dependency. Enable its
``codegen`` feature for the build dependency:

``bsk-build``, ``bsk-messages``, and ``bsk-utilities`` are not yet published
on crates.io. Depend on the Basilisk repository through Cargo's ``git``
support:

.. code-block:: toml

    [package]
    name = "myModule"
    version = "0.1.0"
    edition = "2021"
    rust-version = "1.85"

    [lib]
    path = "myModule.rs"
    crate-type = ["staticlib"]

    [dependencies]
    bsk-messages = { git = "https://github.com/AVSLab/basilisk", tag = "v2.X.Y" }
    bsk-build    = { git = "https://github.com/AVSLab/basilisk", tag = "v2.X.Y" }

    [build-dependencies]
    bsk-build    = { git = "https://github.com/AVSLab/basilisk", tag = "v2.X.Y",
                     default-features = false, features = ["codegen"] }

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

    #[bsk_build::module]
    #[repr(C)]
    pub struct myModuleConfig {
        /// [Nm] proportional gain
        pub K: f64,
        /// [-] attitude guidance input
        #[bsk(input)]
        pub attGuidInMsg: MsgReader<AttGuidMsg>,
        /// [Nm] commanded torque output
        #[bsk(output)]
        pub cmdTorqueOutMsg: MsgWriter<CmdTorqueBodyMsg>,
    }

    impl BskModule for myModuleConfig {
        type State = ();
        type Inputs = myModuleInputs;
        type Outputs = myModuleOutputs;

        fn update(
            &mut self,
            _state: &mut Self::State,
            _context: &BskContext<'_>,
            inputs: Self::Inputs,
            _current_sim_nanos: u64,
        ) -> BskResult<Self::Outputs> {
            let _att_guid_in_msg = inputs.attGuidInMsg;
            // ... pure Rust control law ...
            Ok(myModuleOutputs {
                cmdTorqueOutMsg: CmdTorqueBodyMsg {
                    torqueRequestBody: [0.0, 0.0, 0.0],  // [Nm]
                },
            })
        }
    }

``#[bsk_build::module]`` explicitly identifies the top-level module
configuration, validates its basic C ABI requirements, and generates the Rust
lifecycle entry points. ``bsk-build`` reads the marked struct during
``build.rs`` and generates the C header and wrapper code.

Field doc comments written with ``///`` become Doxygen comments in the
generated header. Follow the Basilisk unit-first convention used by the other
module languages, for example ``/// [m/s] Velocity``. Use ``[-]`` for
dimensionless fields when it improves clarity.

Module Lifecycle
~~~~~~~~~~~~~~~~

Three ``BskModule`` trait methods map to the Basilisk module lifecycle:

``init(state)`` → ``BskResult<()>``
   Called before Python configures the module. Override to set non-zero
   parameter defaults and initial state — the equivalent of a C++ module
   constructor. Before this call, Rust initializes every configuration field
   and ``State`` through ``Default``. The default ``init(state)``
   implementation returns ``Ok(())``.

``reset(state, context, current_sim_nanos)`` → ``BskResult<Self::Outputs>``
   Called at simulation start and on every ``Reset()``. Returns initial
   values for every output port; the framework writes them automatically.
   The default implementation returns ``Ok(Self::Outputs::default())``.
   Override when the module has non-zero initial outputs, parameter
   validation, or state to reset.

``update(state, context, inputs, current_sim_nanos)`` → ``BskResult<Self::Outputs>``
   Called every tick. Receives message values (not ports) and returns output
   values; the framework handles all I/O. ``state`` is private Rust-owned
   storage, while ``context`` supplies borrowed framework metadata and
   logging. No default — must always be implemented.

Return ``Err(BskError::new("..."))`` for an expected configuration, input, or
runtime failure. The generated Rust boundary carries the error across the C
ABI as data, and the C++ wrapper raises ``BasiliskError`` only after Rust has
returned normally. No output messages are written for a failed lifecycle call.

The attribute generates named ``Inputs`` and ``Outputs`` structs from the
annotated message ports. Module code accesses those values by field name, not
by declaration order.

The attribute automatically omits its generated FFI entry points from test
builds, so ``cargo test`` (see `Testing`_ below) does not require Basilisk to
be linked.

Use the Generated Wrapper
~~~~~~~~~~~~~~~~~~~~~~~~~

The generated Python module class, named after the CMake target, provides the
``SysModel`` interface. Construct and schedule it as for a C or C++ module:

.. code-block:: python

    from my_rust_extension import myModule

    module = myModule.myModule()
    simulation.AddModelToTask("taskName", module)

The wrapper owns an opaque Rust module instance and exposes its borrowed
configuration view directly. Configure parameters and connect messages through
``module``. The Python proxy for ``myModuleConfig`` is an implementation
detail and cannot be constructed independently.

Supported Configuration Fields
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Module configuration fields can include:

* scalar, fixed-size array (``[T; N]``, multi-dimensional ``[[T; N]; M]``, etc.), and nested ``#[repr(C)]`` parameter fields;
* ``MsgReader<T>`` input ports and ``MsgWriter<T>`` output ports.

Every field type must implement ``Default``. The built-in scalar, array,
message-port, and ``Option`` types already do. Add ``#[derive(Default)]`` or a
manual ``Default`` implementation to module-defined nested structs. The
generated constructor allocates the configuration in Rust, applies these
defaults, and then calls ``init(state)``.

Nested structs, internal state, and message ports have additional requirements
described below. ``bsk-build`` rejects unsupported field types, including raw
pointers and Rust enums.

Runtime Context
~~~~~~~~~~~~~~~

``BskContext`` provides module runtime information, such as the module ID and
name. The generated lifecycle adapter borrows it for one call:

.. code-block:: rust

    fn update(
        &mut self,
        state: &mut Self::State,
        context: &BskContext<'_>,
        inputs: Self::Inputs,
        current_sim_nanos: u64,
    ) -> BskResult<Self::Outputs> {
        let id = context.module_id();
        // ...
        Ok(Self::Outputs::default())
    }

The context also provides ``model_tag()``, ``call_counts()``, ``rng_seed()``,
and ``logger()``. Values borrowed from it are valid only during the current
lifecycle call.

The C++ wrapper owns an opaque Rust instance handle and passes this context to
each lifecycle call. Runtime services do not appear in the Python-visible
configuration.

``current_sim_nanos`` is passed separately to ``reset`` and ``update``.

Logging
-------

``context.logger()`` returns a borrowed ``BskLoggerRef`` with the standard
nonfatal Basilisk logging methods ``debug()``, ``info()``, and ``warning()``:

.. code-block:: rust

    fn reset(
        &mut self,
        state: &mut Self::State,
        context: &BskContext<'_>,
        _current_sim_nanos: u64,
    ) -> BskResult<Self::Outputs> {
        if self.K <= 0.0 {
            return Err(BskError::new("K must be positive"));
        }
        context.logger().info("Module parameters are valid.");
        Ok(Self::Outputs::default())  // lifecycle code writes these to the output ports
    }

Use ``BskResult`` for expected failures, as shown above. Rust logging is
deliberately nonfatal: it reports diagnostics but does not replace the
lifecycle method's error return. The C++ logging adapter catches any logger
exception before returning to Rust, so a C++ exception never unwinds through
Rust frames.

The generated ABI also catches unexpected Rust panics and returns their
diagnostics as error data. The C++ wrapper raises ``BasiliskError`` only after
Rust has returned. Panics are defect containment, not a normal module error
mechanism; return ``BskError`` for validation and runtime failures that module
code can anticipate.

A caught lifecycle panic poisons that module instance because its internal
state may be only partially updated. Later lifecycle calls fail without
re-entering Rust module code, while destruction remains safe. An expected
``BskError`` does not poison the instance, allowing callers to correct module
configuration and retry.

The returned ``BasiliskError`` is the single user-facing panic diagnostic.
The Rust runtime suppresses its default ``thread ... panicked`` hook output
only while the current thread is inside a generated lifecycle boundary.
Panics on other threads or outside Basilisk lifecycle calls continue through
the application's previously installed panic hook.

In test builds (enabled by adding ``bsk-build`` with the ``test_logger``
feature to ``[dev-dependencies]``), logger calls print to ``stderr`` with no C
symbols required. This means context logger calls in ``reset()`` and
``update()`` work for unit tests.

The C++ wrapper owns the framework logger reference and borrows it into
``BskContext`` for each lifecycle call. It is not part of the Rust
configuration struct.

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
standard Basilisk message types and re-exports ``BskContext``,
``BskModuleRuntime``, ``MsgReader``, ``MsgWriter``, ``BskModule``, and
``BskError``/``BskResult``. It also re-exports ``BskLoggerRef``. Import the
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

Any number of ``MsgReader``/``MsgWriter`` fields are supported. Annotate each
port explicitly; the generated input and output structs use the same field
names:

.. code-block:: rust

    #[bsk_build::module]
    #[repr(C)]
    pub struct myModuleConfig {
        #[bsk(input)]
        pub navAttInMsg: MsgReader<NavAttMsg>,
        #[bsk(input)]
        pub attRefInMsg: MsgReader<AttRefMsg>,
        #[bsk(output)]
        pub cmdTorqueOutMsg: MsgWriter<CmdTorqueBodyMsg>,
        #[bsk(output)]
        pub cmdRateOutMsg: MsgWriter<RateCmdMsg>,
    }

    impl BskModule for myModuleConfig {
        type State = ();
        type Inputs = myModuleInputs;
        type Outputs = myModuleOutputs;

        fn update(
            &mut self,
            _state: &mut Self::State,
            _context: &BskContext<'_>,
            inputs: Self::Inputs,
            _t: u64,
        ) -> BskResult<Self::Outputs> {
            let nav_att = inputs.navAttInMsg;
            let att_ref = inputs.attRefInMsg;
            // ...
            Ok(myModuleOutputs {
                cmdTorqueOutMsg: CmdTorqueBodyMsg::default(),
                cmdRateOutMsg: RateCmdMsg::default(),
            })
        }
    }

The generated structs are named by removing a final ``Config`` from the
configuration type and adding ``Inputs`` or ``Outputs``. Thus
``myModuleConfig`` produces ``myModuleInputs`` and ``myModuleOutputs``. Set the
``BskModule`` associated types to those generated types as shown above.

A module with zero inputs or outputs still uses its generated empty named
struct. No tuple or unit-type special case is needed.

Required vs. optional inputs
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Input optionality is declared next to the corresponding port:

- ``#[bsk(input)]`` means **required**: connectivity is
  checked in ``Reset`` and before every ``Update`` read. A missing connection
  becomes an expected ``BskError`` and the wrapper raises the standard
  ``BasiliskError`` after Rust returns.
- ``#[bsk(input, optional)]`` means **optional**: the generated input field
  has type ``Option<Msg>`` and is ``None`` when unlinked.

.. code-block:: rust

    pub struct myModuleConfig {
        #[bsk(input)]
        pub attGuidInMsg: MsgReader<AttGuidMsg>,
        /// supplemental disturbance-torque estimate
        #[bsk(input, optional)]
        pub disturbanceInMsg: MsgReader<CmdTorqueBodyMsg>,
        // ...
    }

    impl BskModule for myModuleConfig {
        type State = ();
        type Inputs = myModuleInputs;
        type Outputs = myModuleOutputs;

        fn update(
            &mut self,
            _state: &mut Self::State,
            _context: &BskContext<'_>,
            inputs: Self::Inputs,
            _t: u64,
        ) -> BskResult<Self::Outputs> {
            let _required_attitude = inputs.attGuidInMsg;
            let _optional_disturbance = inputs.disturbanceInMsg;
            // ...
            Ok(myModuleOutputs::default())
        }
    }

The annotation macro creates those named fields directly. Rust then checks
that ``BskModule::Inputs`` is the generated input type, eliminating the
field-order correlation that could otherwise misroute same-typed messages.

Outputs
~~~~~~~

Output ports are initialized during ``SelfInit``. Both ``reset()`` and
``update()`` return ``BskResult<Self::Outputs>``. On success, the named output
value contains every output port and the generated lifecycle code writes it
with the module ID and current simulation timestamp. On error, no output is
written for that call.

``reset()`` has a default implementation that returns
``Ok(Self::Outputs::default())``, so a module only needs to override it when
the initial outputs should be non-zero, when parameters need validating, or
when internal state needs resetting. ``update()`` has no default and must
always be implemented.

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


Stateful modules — Rust-owned internal state
---------------------------------------------

Set ``BskModule::State`` to a type that implements ``Default`` when a module
needs state that persists across lifecycle calls. The complete module instance
is allocated and destroyed by Rust. Only the separate configuration view
crosses the FFI boundary, so state may contain unrestricted safe Rust types:

.. code-block:: rust

    #[derive(Default)]
    pub struct MyState {
        history: Vec<f64>,
        status: String,
        mode: InternalMode,
    }

    #[derive(Default)]
    enum InternalMode {
        #[default]
        Idle,
        Running,
    }

    impl BskModule for myModuleConfig {
        type State = MyState;
        type Inputs = myModuleInputs;
        type Outputs = myModuleOutputs;

        fn update(
            &mut self,
            state: &mut Self::State,
            context: &BskContext<'_>,
            inputs: Self::Inputs,
            current_sim_nanos: u64,
        ) -> BskResult<Self::Outputs> {
            state.history.push(self.K);
            state.status = format!(
                "module {} updated at {current_sim_nanos} ns",
                context.module_id()
            );
            state.mode = InternalMode::Running;
            // ...
            Ok(Self::Outputs::default())
        }
    }

``MyState`` does not use ``#[repr(C)]`` and does not appear in the generated
header or Python module. Rust drop glue releases all of its contents when the
opaque module instance is destroyed. Use ``type State = ();`` for modules
without internal state.

Grouping parameters — nested structs
--------------------------------------

A field may be another ``#[repr(C)]`` struct defined in the same crate, by
value, to group related parameters:

.. code-block:: rust

    #[repr(C)]
    #[derive(Default)]
    pub struct Vec2 {
        pub x: f64,
        pub y: f64,
    }

    #[bsk_build::module]
    #[repr(C)]
    pub struct myModuleConfig {
        pub target: Vec2,
        // ...
    }

``bsk-build`` generates ``Vec2``'s own C struct ahead of ``myModuleConfig``'s,
and Python reads and writes it field-by-field like any other struct member
(``ctrl.target.x = 1.0``). Nested structs must implement ``Default`` because
Rust constructs the complete configuration before calling ``init(state)``.
They cannot be self-referential. Message ports belong on the top-level config
struct so the generated lifecycle adapter can process them. Internal state
belongs in ``BskModule::State`` and does not need a C representation.

A field may **not** be a raw pointer to one of these structs (e.g. ``*mut
Vec2``), or to any other type. Python setters for a pointer field transfer
ownership away from the Python object, and nothing on the Rust side would ever
free it; embedding by value (above) has no such issue, so ``bsk-build`` rejects
all pointer fields, including a raw pointer to a primitive (e.g. ``*mut u8``).
There is currently no supported field type for a persistent string or
byte-buffer parameter.

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
``#[repr(C)]`` struct and ``N`` a compile-time length — is supported and maps
to the standard C array declaration ``T name[N];``. Named length constants
are emitted into the C header as macros. This is the same form used
throughout Basilisk's C message payloads (e.g.
``double torqueRequestBody[3]``). Multi-dimensional arrays are also
supported: ``[[T; N]; M]`` maps to ``T name[M][N];``.

.. code-block:: rust

    #[bsk_build::module]
    #[repr(C)]
    pub struct myModuleConfig {
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

The module configuration itself cannot be generic. Array lengths can be
integer literals or crate constants that ``cbindgen`` can emit into the C
header.

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
interface (auto-detecting message port fields), and links the result into
the SWIG extension. The generated interface supplies only module-specific
metadata and delegates the common Python ownership and lifecycle behavior to
Basilisk's shared Rust wrapper template. No hand-written C/C++ ``SOURCES`` or
SWIG wrapper implementation is needed.

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

**``cargo test`` never touches Basilisk.** The ``#[bsk_build::module]``
attribute omits its C ABI lifecycle entry points in test builds, so
``init(state)``, ``reset(...)``, and ``update(...)`` can be exercised as plain
Rust functions with hand-built message values — no linking, no Python.

Add the ``test_logger`` dev-dependency (see `Logging`_ above) so that
``context.logger().warning(...)`` and similar calls work in unit tests.
Logger calls in test builds print to ``stderr`` rather than calling
Basilisk's C symbols.

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
   Ensure that the config uses ``#[bsk_build::module]``. The generated
   lifecycle functions are automatically omitted from test builds.

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
- Configuration values are exposed to Python as public struct fields.
  Exporting arbitrary inherent Rust methods through SWIG is not currently
  supported.
- ``bsk-utilities`` covers C ABI utilities/constants only; C++/Eigen
  utilities (``Eigen::Vector3d``, ``Eigen::Matrix3d``, ...) are out of scope
  until a dedicated C shim is designed.

Reference
---------

The ABI reference is the header comment in
:download:`bsk_rust_module.h <../../../src/architecture/_GeneralModuleFiles/bsk_rust_module.h>`,
and the module documentation in ``src/architecture/rust/bsk_build/src/lib.rs``.
