.. _rustModules:

:beta:`Making Rust Modules`
===========================

.. important::

   Rust module support requires Rust 1.85 or newer.

.. sidebar:: Rust Module Support

   Rust module support is experimental. Its interface may change between
   Basilisk releases.

Rust modules are compiled Basilisk modules whose lifecycle methods are written
in Rust instead of C or C++. They use the normal ``SysModel`` lifecycle,
message system, Python configuration, module documentation, and unit-test
structure. A Python user imports and schedules a Rust module in exactly the
same way as another compiled Basilisk module.

This guide is for developers who already understand C or C++ Basilisk modules.
It introduces the Rust-specific terms and files needed to create a module
inside the Basilisk source tree. The :ref:`rustModuleTemplate` is the complete
working example and should be used as the starting point for a new module.

Rust Terms Used in This Guide
-----------------------------

Rust uses different names for several familiar build concepts:

``rustc``
   The Rust compiler.

``Cargo``
   Rust's build tool and dependency manager. Cargo reads ``Cargo.toml``, runs
   the module's ``build.rs`` script, compiles the Rust source, and runs
   Rust-native tests. The standard Rust installer supplies both ``rustc`` and
   Cargo.

Cargo package
   A directory described by one ``Cargo.toml`` manifest. Each Basilisk Rust
   module directory is a Cargo package.

crate
   A Rust compilation unit. A Cargo package can contain one or more crates.
   Each Basilisk Rust module contains one library crate, so this guide
   occasionally uses *module crate* to mean the Rust library compiled from the
   module directory.

Cargo workspace
   A group of related Cargo packages that share dependency resolution and one
   lockfile. All Rust packages inside the Basilisk repository belong to the
   workspace defined by ``src/Cargo.toml``.

``Cargo.lock``
   The file recording the exact dependency versions selected for a workspace.
   Basilisk commits one shared lockfile at ``src/Cargo.lock`` so local and CI
   builds use the same versions.

attribute
   Metadata written above a Rust item with ``#[...]``. For example,
   ``#[bsk_build::module]`` identifies the configuration struct and asks
   Basilisk's Rust tooling to generate its lifecycle interface.

trait
   A Rust interface defining behavior that a type must provide. It is similar
   in purpose to a C++ abstract base-class interface. A module implements the
   ``BskModule`` trait to provide its lifecycle methods.

The official `Rust Book <https://doc.rust-lang.org/book/>`__ provides an
introduction to general Rust syntax. The
`Cargo Book <https://doc.rust-lang.org/cargo/>`__ covers Cargo terminology and
commands in more detail.

Comparison with C and C++ Modules
---------------------------------

The Rust tooling generates the C header and SWIG interface needed to join the
existing Basilisk build. The module author writes the Rust behavior and the
normal Basilisk documentation and tests.

.. list-table:: Mapping familiar Basilisk concepts to Rust
   :header-rows: 1
   :widths: 31 31 38

   * - Basilisk concept
     - C or C++
     - Rust
   * - Implementation
     - ``myModule.c/.cpp`` and usually ``myModule.h``
     - ``myModule.rs``
   * - Build description
     - CMake source lists
     - ``Cargo.toml`` plus workspace registration
   * - SWIG and C boundary
     - Hand-written ``myModule.i`` and public declarations
     - Generated from the marked Rust configuration
   * - Python-visible configuration
     - Config struct or public class fields
     - ``#[repr(C)]`` struct marked with ``#[bsk_build::module]``
   * - Private implementation state
     - Private C++ members or fields hidden from SWIG
     - ``BskModule::State``
   * - Initialization
     - Constructor and ``SelfInit()``
     - Rust ``Default``/``init()`` and generated ``SelfInit()``
   * - Reset and update
     - ``Reset()`` and ``UpdateState()``
     - ``reset()`` and ``update()`` methods of ``BskModule``
   * - Message I/O
     - Reader/writer fields and explicit read/write calls
     - Annotated ports and generated named input/output values
   * - Logging
     - ``bskLogger``
     - ``context.logger()``
   * - Expected failure
     - ``BasiliskError`` or ``BSK_ERROR``
     - Return ``Err(BskError::new(...))``

Install and Build with Rust Support
-----------------------------------

Install the stable toolchain with the official
`rustup installer <https://rust-lang.org/tools/install/>`__. The
:ref:`Linux <installLinux>`, :ref:`macOS <installMacOS>`, and
:ref:`Windows <installWindows>` setup pages contain platform-specific
instructions.

Open a new terminal after installing Rust and verify both tools:

.. code-block:: console

    rustc --version
    cargo --version

Rust module discovery is disabled in a normal Basilisk build, so users who do
not need Rust modules do not need these tools. Enable Rust modules with:

.. code-block:: console

    python3 conanfile.py --rustModules True --rustCorrosion True

On Windows, use ``python`` instead of ``python3`` if that is the command for
the active Python installation.

``rustModules`` enables discovery and compilation of in-tree Rust modules.
``rustCorrosion`` selects Corrosion, the CMake integration layer that imports
Cargo libraries as CMake targets. Corrosion is downloaded at its pinned
revision during configuration; it is not a separate compiler and does not
need to be installed manually.

After the build, verify the template through its normal Python unit test:

.. code-block:: console

    python3 -m pytest src/moduleTemplates/rustModuleTemplate/_UnitTest -v

Quick-Start from ``rustModuleTemplate``
---------------------------------------

Copy ``src/moduleTemplates/rustModuleTemplate`` into the desired
``src/fswAlgorithms`` or ``src/simulation`` category. Then:

#. Rename the directory to the new module name.
#. Rename ``rustModuleTemplate.rs`` and ``rustModuleTemplate.rst`` so each
   filename matches the new module directory.
#. Update the package name and ``[lib] path`` in ``Cargo.toml``.
#. Rename the marked configuration struct in the Rust source.
#. Pass that exact struct name to ``generate_bindings()`` in ``build.rs``.
#. Replace the template implementation, documentation, and Python unit test.
#. Add the new module to the workspace as described in
   `Register the Module in the Workspace`_.

The template intentionally demonstrates more than a minimal control law. It
contains an optional input, output message, Python-visible parameters,
Rust-owned private state, logging, expected errors, Rust-native tests, and a
test-only panic used to verify the language boundary.

Module Layout
-------------

Place the files in a normal Basilisk module directory:

.. code-block:: text

    src/fswAlgorithms/<category>/myModule/
    |-- Cargo.toml
    |-- build.rs
    |-- myModule.rs
    |-- myModule.rst
    `-- _UnitTest/
        `-- test_myModule.py

``Cargo.toml``
   Describes the module package, Rust source path, output library, and
   dependencies. It fills the role that Cargo expects from a package manifest.

``build.rs``
   A short Cargo build script that runs before compilation. It asks
   ``bsk-build`` to generate the module's C header and SWIG interface.

``myModule.rs``
   Contains the configuration struct, lifecycle implementation, private state,
   and optional Rust-native tests.

``myModule.rst``
   The normal Basilisk module documentation page.

``_UnitTest/test_myModule.py``
   Tests the public Python interface and message behavior in a Basilisk
   simulation.

No hand-written ``.h`` or ``.i`` file is required. Generated files are build
products placed under ``dist3`` and are removed by the normal clean build.

Configure the Module Package
----------------------------

The module's ``Cargo.toml`` should follow this pattern:

.. code-block:: toml

    [package]
    name = "myModule"
    version = "0.1.0"
    edition = "2021"
    rust-version.workspace = true

    [package.metadata.basilisk]
    module = true

    [lib]
    path = "myModule.rs"
    crate-type = ["staticlib"]

    [dependencies]
    bsk-messages = { path = "../../../architecture/rust/bsk_messages" }
    bsk-build = { path = "../../../architecture/rust/bsk_build" }

    [build-dependencies]
    bsk-build = { path = "../../../architecture/rust/bsk_build",
                  default-features = false, features = ["codegen"] }

The important entries are:

``rust-version.workspace = true``
   Uses Basilisk's minimum Rust version instead of declaring a different
   version for each module.

``[package.metadata.basilisk]``
   Marks this Cargo package as a Basilisk module. Support libraries under
   ``src/architecture/rust`` are Cargo packages too, but they do not have this
   marker and are not exposed as Python modules.

``[lib] path``
   Selects ``myModule.rs`` as the library source. Cargo would otherwise expect
   the conventional Rust path ``src/lib.rs``.

``crate-type = ["staticlib"]``
   Produces a native static library that Basilisk links into the generated
   Python module.

``[dependencies]``
   Lists Rust libraries used by the module:

   * ``bsk-messages`` supplies Basilisk message value types, message ports,
     lifecycle types, context, logging, and errors.
   * ``bsk-build`` supplies the ``#[bsk_build::module]`` attribute and runtime
     support.

``[build-dependencies]``
   Lists code used only by ``build.rs``. Disabling default features prevents
   the host build script from trying to link Basilisk runtime symbols; the
   ``codegen`` feature enables binding generation.

The dependency paths are relative to the module directory. Adjust them when a
module is at a different directory depth.

Generate the Language Boundary
------------------------------

The complete ``build.rs`` file is normally:

.. code-block:: rust

    fn main() {
        bsk_build::generate_bindings("myModuleConfig");
    }

Pass the exact name of the struct marked with ``#[bsk_build::module]``.
During the build:

#. Cargo runs ``build.rs``.
#. ``bsk-build`` finds the marked configuration and its message annotations.
#. ``cbindgen`` generates a C-compatible header from the Rust declarations.
#. ``bsk-build`` generates the module-specific SWIG interface.
#. CMake links the Rust static library and generated wrapper into the normal
   Basilisk Python package.

``cbindgen`` is a Rust-to-C header generator used internally by this process.
It is obtained as a Cargo dependency; module authors do not install or invoke
it separately. Do not edit the generated header or SWIG file.

Register the Module in the Workspace
------------------------------------

All in-tree Rust packages belong to the Cargo workspace rooted at
``src/Cargo.toml``. The workspace gives Basilisk one dependency resolution and
one committed ``src/Cargo.lock``.

Add the new module's path, relative to ``src``, to the explicit member list:

.. code-block:: toml

    [workspace]
    resolver = "2"
    members = [
        "architecture/rust/bsk_build",
        "architecture/rust/bsk_macros",
        "architecture/rust/bsk_messages",
        "architecture/rust/bsk_utilities",
        "fswAlgorithms/<category>/myModule",
        "moduleTemplates/rustModuleTemplate",
    ]

After adding the member or changing dependencies, let Cargo update the shared
lockfile deliberately:

.. code-block:: console

    cargo check --manifest-path src/fswAlgorithms/<category>/myModule/Cargo.toml
    git diff -- src/Cargo.lock

Review and commit both ``src/Cargo.toml`` and ``src/Cargo.lock`` with the new
module. Do not add a ``Cargo.lock`` inside the module directory. Cargo uses
the workspace lockfile, and module-local lockfiles are ignored.

Normal Basilisk and CI builds pass ``--locked`` to Cargo. A locked build fails
when ``Cargo.toml`` and ``src/Cargo.lock`` disagree instead of silently
selecting new dependency versions.

Write the Module
----------------

The following module has one Python-visible gain, one required input, one
output, and no private state:

.. code-block:: rust

    use bsk_messages::*;

    #[bsk_build::module]
    #[repr(C)]
    pub struct myModuleConfig {
        /// [Nm] Proportional gain
        pub K: f64,
        /// [-] Attitude guidance input
        #[bsk(input)]
        pub attGuidInMsg: MsgReader<AttGuidMsg>,
        /// [Nm] Commanded body torque output
        #[bsk(output)]
        pub cmdTorqueOutMsg: MsgWriter<CmdTorqueBodyMsg>,
    }

    impl BskModule for myModuleConfig {
        type State = ();
        type Inputs = myModuleInputs;
        type Outputs = myModuleOutputs;

        fn init(&mut self, _state: &mut Self::State) -> BskResult<()> {
            self.K = 1.0; // [Nm]
            Ok(())
        }

        fn reset(
            &mut self,
            _state: &mut Self::State,
            _context: &BskContext<'_>,
            _current_sim_nanos: u64,
        ) -> BskResult<Self::Outputs> {
            if self.K <= 0.0 {
                return Err(BskError::new("K must be positive"));
            }
            Ok(Self::Outputs::default())
        }

        fn update(
            &mut self,
            _state: &mut Self::State,
            _context: &BskContext<'_>,
            inputs: Self::Inputs,
            _current_sim_nanos: u64,
        ) -> BskResult<Self::Outputs> {
            let guidance = inputs.attGuidInMsg;
            Ok(myModuleOutputs {
                cmdTorqueOutMsg: CmdTorqueBodyMsg {
                    torqueRequestBody: [
                        -self.K * guidance.sigma_BR[0],
                        -self.K * guidance.sigma_BR[1],
                        -self.K * guidance.sigma_BR[2],
                    ],
                },
            })
        }
    }

The Rust-specific declarations mean:

``use bsk_messages::*;``
   Imports the Basilisk Rust message values and module support types used in
   the file.

``#[repr(C)]``
   Requests a C-compatible field layout for the Python-visible configuration.
   This is required because the generated C++/SWIG wrapper accesses these
   fields across the language boundary.

``#[bsk_build::module]``
   Identifies the one top-level module configuration, validates its supported
   field types, and generates the lifecycle entry points.

``pub``
   Makes a configuration field public to the generated wrapper. These fields
   become normal attributes on the Python module object.

``type State``, ``Inputs``, and ``Outputs``
   Select the private state type and the named message-value structs used by
   the lifecycle methods. The input and output types are generated from the
   annotated ports. For ``myModuleConfig`` they are named
   ``myModuleInputs`` and ``myModuleOutputs``.

``BskResult<T>``
   Represents either success, ``Ok(T)``, or an expected module failure,
   ``Err(BskError)``.

Module Lifecycle
----------------

The generated wrapper provides ``SelfInit()``, ``Reset()``, and
``UpdateState()`` to Basilisk. The Rust implementation supplies these
``BskModule`` methods:

``init(state) -> BskResult<()>``
   Runs once while Rust constructs the module, before Python configures it.
   Override it to set non-zero configuration defaults or initial private
   state. Rust first initializes every configuration and state field through
   its ``Default`` implementation. The default ``init`` returns ``Ok(())``.

``reset(state, context, current_sim_nanos) -> BskResult<Outputs>``
   Runs at simulation start and on every Basilisk ``Reset()``. Use it for
   parameter validation, private-state reset, and non-zero initial output
   values. The default returns zero/default output values.

``update(state, context, inputs, current_sim_nanos) -> BskResult<Outputs>``
   Runs on each task update. It receives message values through ``inputs`` and
   returns all output message values. The generated wrapper performs the
   actual message reads and writes. ``update`` has no default and must be
   implemented.

If ``reset`` or ``update`` returns ``Err(BskError::new("..."))``, the wrapper
raises the normal Python ``BasiliskError`` after Rust returns. No output
message is written for the failed lifecycle call.

Message Ports and Values
------------------------

``bsk-messages`` provides a Rust value type for each built-in Basilisk message.
For example, the Rust ``AttGuidMsg`` corresponds to the familiar
``AttGuidMsgPayload`` data. A port uses one of two generic types:

.. important::

   Rust ports use Basilisk's generated C message interfaces, such as
   ``AttGuidMsg_C``; Rust does not bind directly to the C++
   ``Message<Payload>`` classes. This does not prevent C++ interoperability.
   A Rust port can connect to a C or C++ module port with the same payload
   type, just as existing C and C++ Basilisk modules connect to each other.
   In Rust source, always declare ports with ``MsgReader<T>`` or
   ``MsgWriter<T>`` from ``bsk-messages``.

``MsgReader<AttGuidMsg>``
   An input port that reads an ``AttGuidMsg`` value.

``MsgWriter<CmdTorqueBodyMsg>``
   An output port that writes a ``CmdTorqueBodyMsg`` value.

Every port must be annotated. Required and optional inputs are explicit:

.. code-block:: rust

    /// [-] Required attitude input
    #[bsk(input)]
    pub navAttInMsg: MsgReader<NavAttMsg>,

    /// [Nm] Optional disturbance estimate
    #[bsk(input, optional)]
    pub disturbanceInMsg: MsgReader<CmdTorqueBodyMsg>,

    /// [Nm] Commanded body torque
    #[bsk(output)]
    pub cmdTorqueOutMsg: MsgWriter<CmdTorqueBodyMsg>,

A required input causes ``Reset`` or ``UpdateState`` to fail with
``BasiliskError`` when it is not connected. The generated input field for an
optional port has type ``Option<Msg>``: it is ``Some(value)`` when connected
and ``None`` when unconnected.

The generated input and output structs use the same field names as the ports.
Any number of inputs and outputs is supported, and declaration order does not
control message routing. Both ``reset`` and ``update`` must return a value for
every output port.

Adding or Changing a Basilisk Message
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

After adding or editing a Basilisk message payload, rerun the build with Rust
module support enabled:

.. code-block:: console

    python conanfile.py --rustModules True

This first regenerates the normal C message interfaces and then generates the
matching Rust bindings in Cargo's build directory. No separate Rust generator
must be run, and the generated bindings are not committed.

When Rust modules are disabled, the Rust binding generator does not run.

Python-Visible Configuration
----------------------------

The marked configuration struct is the equivalent of a C module config struct
or the public fields of a C++ module. It can contain:

* scalar integer, floating-point, and Boolean parameters;
* fixed-size arrays such as ``[f64; 3]`` and ``[[f64; 3]; 3]``;
* nested, by-value ``#[repr(C)]`` parameter structs; and
* annotated ``MsgReader<T>`` and ``MsgWriter<T>`` ports.

Every configuration field must implement Rust's ``Default`` behavior because
Rust constructs the complete module before calling ``init``. Built-in scalar,
array, and message-port types already provide defaults. Add
``#[derive(Default)]`` or a manual implementation to module-defined nested
structs.

Configuration fields cannot contain raw pointers, Rust enums, dynamically
sized strings, or ``Vec`` collections. These types do not have a safe,
general Python/C representation. Put them in private Rust state instead.

Nested parameters can be grouped by value:

.. code-block:: rust

    #[repr(C)]
    #[derive(Default)]
    pub struct ControllerGains {
        /// [Nm] Proportional gain
        pub K: f64,
        /// [Nm/(rad/s)] Rate gain
        pub P: f64,
    }

    #[bsk_build::module]
    #[repr(C)]
    pub struct myModuleConfig {
        /// [-] Controller parameters
        pub gains: ControllerGains,
        // Message ports follow.
    }

Fixed-size arrays map to normal C arrays and appear as Python lists:

.. code-block:: rust

    /// [Nm] Maximum torque on each body axis
    pub maxTorques: [f64; 3],
    /// [-] Direction cosine matrix
    pub dcm_BR: [[f64; 3]; 3],

.. code-block:: python

    module.maxTorques = [0.001, 0.001, 0.001]  # [Nm]
    module.dcm_BR = [[1.0, 0.0, 0.0],          # [-]
                     [0.0, 1.0, 0.0],
                     [0.0, 0.0, 1.0]]

Reading an array field from Python returns a copy. Reassign the list after
changing an element:

.. code-block:: python

    values = module.maxTorques
    values[0] = 0.005  # [Nm]
    module.maxTorques = values

Rust-Owned Private State
------------------------

``BskModule::State`` stores implementation details that must persist between
lifecycle calls but should not appear in Python. Unlike the configuration,
this state never crosses the C interface and can use normal safe Rust types
such as ``Vec``, ``String``, enums, and smart pointers:

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
            _context: &BskContext<'_>,
            _inputs: Self::Inputs,
            _current_sim_nanos: u64,
        ) -> BskResult<Self::Outputs> {
            state.history.push(self.K);
            state.status = String::from("updated");
            state.mode = InternalMode::Running;
            Ok(Self::Outputs::default())
        }
    }

Use ``type State = ();`` when the module is stateless. Rust allocates and
destroys the complete module instance, so ordinary Rust cleanup releases
private state automatically.

Runtime Context and Logging
---------------------------

``BskContext`` gives each lifecycle call a temporary view of Basilisk runtime
information and services:

* ``module_id()`` returns the module identifier;
* ``model_tag()`` returns the Python-visible model tag;
* ``call_counts()`` returns the update-call count;
* ``rng_seed()`` returns the module random-number seed; and
* ``logger()`` returns the Basilisk logger.

The context is borrowed only for the current call. Do not store references
obtained from it in the configuration or private state.

Use the logger for nonfatal diagnostics:

.. code-block:: rust

    context.logger().debug("Starting update.");
    context.logger().info("Module parameters are valid.");
    context.logger().warning("Using a fallback input.");

For an expected failure that must stop the lifecycle call, return an error:

.. code-block:: rust

    if self.K <= 0.0 {
        return Err(BskError::new("K must be positive"));
    }

Do not deliberately use a Rust panic for validation or normal error control.
The generated boundary catches unexpected panics so they do not unwind into
C++. A caught panic becomes ``BasiliskError`` and poisons that module instance
because its private state may be partially updated. Later lifecycle calls on
that instance fail, but Rust can still destroy it safely. An expected
``BskError`` does not poison the module.

Use the Module from Python
--------------------------

The generated wrapper is imported, configured, connected, and scheduled like
another compiled module:

.. code-block:: python

    from Basilisk.fswAlgorithms import myModule

    module = myModule.myModule()
    module.ModelTag = "myRustModule"
    module.K = 0.25  # [Nm]
    module.attGuidInMsg.subscribeTo(attitudeGuidanceMessage)
    simulation.AddModelToTask("taskName", module)

The Python object directly exposes the marked configuration fields and normal
``SysModel`` fields. The generated ``myModuleConfig`` proxy is an
implementation detail and should not be constructed separately.

Document the Module
-------------------

Keep the user-facing description in ``myModule.rst`` and follow the same
documentation structure used by C and C++ modules. Use the
``.. bsk-module-io::`` directive for the message diagram and table. The
:ref:`rustModuleTemplate` documentation provides a complete example.

Rust uses ``///`` for documentation attached to the following field, struct,
or function. It uses ``//!`` for documentation about the containing source
module. Follow Basilisk's unit-first convention:

.. code-block:: rust

    /// [m] Inertial position vector
    pub position: [f64; 3],
    /// [m/s] Inertial velocity vector
    pub velocity: [f64; 3],
    /// [-] Dimensionless gain
    pub gain: f64,

``cbindgen`` copies these field comments into the generated C header, and the
normal Doxygen/Breathe pipeline includes the generated module configuration
API in the Sphinx page.

Build Basilisk with Rust support before generating documentation:

.. code-block:: console

    python3 conanfile.py --rustModules True --rustCorrosion True
    cd docs
    make html

The generated HTML is stored under ``docs/build/html``. The Rust API section
is omitted when its generated header is unavailable.

Test the Module
---------------

Rust modules should have both Rust-native tests and the normal Basilisk Python
unit test.

Rust-Native Tests
~~~~~~~~~~~~~~~~~

Place ``#[cfg(test)]`` tests in ``myModule.rs`` or in the package's Rust test
layout. Run them from the module directory:

.. code-block:: console

    cargo test --locked

The module attribute omits generated C lifecycle entry points during
``cargo test``, so these tests can call ``init``, ``reset``, and ``update``
without linking Basilisk.

To test code that calls ``context.logger()``, add the test logger feature:

.. code-block:: toml

    [dev-dependencies]
    bsk-build = { path = "../../../architecture/rust/bsk_build",
                  features = ["test_logger"] }

Rust-native logger calls then print to ``stderr`` instead of calling Basilisk
C symbols. Use ``BskModuleRuntime::for_testing()`` and
``BskContext::for_testing()`` to create a context, as demonstrated by
``rustModuleTemplate.rs``.

Python Unit Test
~~~~~~~~~~~~~~~~

Add ``_UnitTest/test_myModule.py`` to validate the public Python attributes,
message connections, lifecycle behavior, and numerical results:

.. code-block:: console

    python3 -m pytest src/fswAlgorithms/<category>/myModule/_UnitTest -v

After a Rust-enabled Basilisk build, the normal ``python3 run_all_test.py``
workflow includes the module's Python test.

Contributor Checks
~~~~~~~~~~~~~~~~~~

Before submitting a Rust module, run the complete workspace checks from the
repository root:

.. code-block:: console

    cargo test --workspace --all-features --locked --manifest-path src/Cargo.toml
    cargo test -p bsk-build --no-default-features --locked --manifest-path src/Cargo.toml
    cargo clippy --workspace --all-targets --all-features --locked \
        --manifest-path src/Cargo.toml -- -D warnings

Pull-request CI also checks the minimum supported Rust version and verifies
that generated language boundaries retain panic containment. Follow the
:ref:`bskModuleCheckoutList` for the remaining module review requirements.

Current Limitations
-------------------

* Rust module support is experimental, and its generated interface may change
  between Basilisk releases.
* Python-visible configuration is limited to supported C-compatible types.
  Put dynamic Rust data in ``BskModule::State``.
* Arbitrary Rust methods are not exported to Python; the public interface is
  the standard Basilisk lifecycle, configuration fields, and messages.
* The Rust utility bindings cover C-compatible Basilisk utilities and
  constants. C++-only and Eigen-based utilities require a C-compatible wrapper
  before Rust can use them.
* Adding or changing a message payload requires regeneration of the committed
  ``bsk-messages`` bindings.

Reference
---------

The generated ABI is described in
:download:`bsk_rust_module.h <../../../../src/architecture/_GeneralModuleFiles/bsk_rust_module.h>`.
The Rust runtime and generation API are documented in
``src/architecture/rust/bsk_build/src/lib.rs``. For a working implementation,
configuration, Python test, and Rust-native tests, see
:ref:`rustModuleTemplate`.
