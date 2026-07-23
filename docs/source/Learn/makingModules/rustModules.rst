
.. _rustModules:

:beta:`Making Rust Modules`
===========================

.. sidebar:: Rust Module Support

   Rust module support is experimental. The interface may change between
   Basilisk releases.

Rust modules are compiled Basilisk modules written in Rust. They use the same
module lifecycle and Python interface as C and C++ modules, but define the
module configuration and behavior with a Rust ``#[repr(C)]`` struct marked by
the ``#[bsk_build::module]`` attribute and the ``BskModule`` trait.

This page describes modules contributed to the Basilisk source tree. For a
separately packaged Rust extension, see :ref:`writingRustPlugins`.

The :ref:`rustModuleTemplate` module provides a minimal in-tree Rust module
that can be copied as a starting point.

Building with Rust Support
--------------------------

Rust module discovery is disabled by default, so that Rust and Cargo remain
optional for users who do not build Rust modules. Install Rust 1.85 or newer
and Cargo, then configure Basilisk with:

.. code-block:: bash

    python3 conanfile.py --rustModules True

This enables Rust module discovery and builds every in-tree module crate. Use
the normal Basilisk build and test workflow after configuration. An explicit
Rust-enabled configuration fails if Cargo is unavailable rather than silently
omitting Rust modules.

An opt-in trial of Corrosion is available as the CMake/Cargo integration for
discovered Rust modules. Enable it explicitly with:

.. code-block:: bash

    python3 conanfile.py --rustModules True --rustCorrosion True

Corrosion is fetched only for this opt-in build, at an immutable revision.
Basilisk's existing Cargo driver remains available when ``rustCorrosion`` is
disabled while the trial is evaluated. Pull-request CI exercises the
Corrosion path on Linux, macOS, and Windows.

Rust 1.85 is Basilisk's minimum supported Rust version (MSRV). The repository
does not pin one exact toolchain for local development: Cargo accepts any
newer compatible stable toolchain, while pull-request CI separately tests the
MSRV and the runner's current stable toolchain.

Module Layout
-------------

Place a Rust module in a normal Basilisk module directory. The Cargo manifest's
``[package.metadata.basilisk]`` marker identifies the directory as a Rust
module; no hand-written C header or SWIG interface file is needed:

.. code-block:: text

    src/fswAlgorithms/<category>/myModule/
    |-- Cargo.toml
    |-- build.rs
    |-- myModule.rs
    |-- myModule.rst
    `-- _UnitTest/
        `-- test_myModule.py

The Rust source and reStructuredText documentation names match the module
directory, following the same convention as C, C++, and Python Basilisk
modules. The module target name is the directory containing ``Cargo.toml``. The
``[package.metadata.basilisk]`` marker shown below distinguishes modules from
Rust support crates. Basilisk generates the C header and SWIG interface while
building the crate.

Configure the Crate
-------------------

Set the crate type to ``staticlib`` and depend on the in-tree Rust support
crates. From the layout above, ``Cargo.toml`` contains:

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

The ``[lib] path`` selects the Basilisk-style module source instead of Cargo's
default ``src/lib.rs``. The path to each support crate is relative to the
module directory. Adjust those dependency paths if the module is placed
elsewhere in the source tree. Keep ``default-features = false`` on the build
dependency: ``build.rs`` needs code generation, not APIs that link the final
module to Basilisk's C/C++ runtime.

The ``build.rs`` file generates the C and SWIG bindings:

.. code-block:: rust

    fn main() {
        bsk_build::generate_bindings("myModuleConfig");
    }

Pass the exact name of the struct marked with ``#[bsk_build::module]``.
Cargo obtains the source file from ``[lib] path``; ``cbindgen`` then renders
the crate's C-compatible types for the generated header.

Register the Crate in the Workspace
-----------------------------------

All Rust code contributed to the Basilisk source tree belongs to the
`Cargo workspace <https://doc.rust-lang.org/cargo/reference/workspaces.html>`__
rooted at ``src/Cargo.toml``. The workspace provides one dependency resolution
and one committed lockfile, ``src/Cargo.lock``, for the support crates and
every in-tree Rust module. In-tree crates inherit the workspace MSRV with
``rust-version.workspace = true``.

Add the new module's path, relative to ``src``, to the workspace member list:

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

After adding the member or changing its dependencies, update the shared
lockfile deliberately from the repository root:

.. code-block:: bash

    cargo check --manifest-path src/fswAlgorithms/<category>/myModule/Cargo.toml
    git diff -- src/Cargo.lock

Review and commit both ``src/Cargo.toml`` and ``src/Cargo.lock`` with the new
module. Do not create or commit a ``Cargo.lock`` beside an in-tree module.
Member-local lockfiles are ignored because Cargo uses the workspace lock.

Normal CMake builds pass ``--locked`` to Cargo. Consequently, a build fails
instead of silently changing dependency versions when a contributor changes a
manifest without updating ``src/Cargo.lock``. Run an unlocked Cargo command
only when intentionally updating the shared lockfile; use ``--locked`` for
normal checks and tests.

Before submitting the module, run the complete Rust workspace test suite:

.. code-block:: bash

    cargo test --workspace --all-features --locked --manifest-path src/Cargo.toml
    cargo test -p bsk-build --no-default-features --locked --manifest-path src/Cargo.toml
    cargo clippy --workspace --all-targets --all-features --locked \
        --manifest-path src/Cargo.toml -- -D warnings

Pull-request CI runs these commands and
``.github/scripts/check_rust_unwind_policy.py``. The latter deliberately
compiles ``rustModuleTemplate`` with ``panic="abort"`` and requires the
procedural macro to reject the build. This prevents a profile change from
silently disabling panic containment at the generated C ABI. CI also runs the
workspace tests with Rust 1.85.0 so new code or dependency updates cannot
raise the documented MSRV accidentally.

Write the Module
----------------

Write the implementation in ``myModule.rs``. Import the support types from
``bsk-messages``, mark a ``#[repr(C)]`` configuration struct with
``#[bsk_build::module]``, and implement ``BskModule``:

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
            let _ = inputs.attGuidInMsg;
            Ok(myModuleOutputs {
                cmdTorqueOutMsg: CmdTorqueBodyMsg::default(),
            })
        }
    }

The ``#[bsk_build::module]`` attribute explicitly identifies the top-level
module configuration, validates its basic C ABI requirements, and generates
the lifecycle entry points. Configuration fields can be scalars, fixed-size
arrays (``[T; N]`` or multi-dimensional ``[[T; N]; M]``), nested
``#[repr(C)]`` structs, and ``MsgReader<T>`` or ``MsgWriter<T>`` ports.
Framework metadata and logging arrive through ``BskContext``. Put internal
implementation state in the ``BskModule::State`` associated type rather than
the configuration view.

Annotate every message port with ``#[bsk(input)]`` or ``#[bsk(output)]``.
Use ``#[bsk(input, optional)]`` when an unlinked input should produce ``None``
instead of an error. For ``myModuleConfig``, the attribute generates the named
``myModuleInputs`` and ``myModuleOutputs`` types used above. Their fields have
the same names as the corresponding config ports, so port declaration order
does not affect message routing.

``update`` is required and receives message values rather than message ports.
Both ``reset`` and ``update`` return ``BskResult<Self::Outputs>``; the
framework writes the returned values to the output ports only for ``Ok``.
Return ``Err(BskError::new("..."))`` for expected configuration or runtime
failures. The C++ wrapper translates that error into ``BasiliskError`` after
Rust returns normally. ``reset`` has a default implementation that returns
``Ok(Self::Outputs::default())``. Override it when the module needs non-zero
initial output values, parameter validation, or state reset.

``context.logger()`` provides nonfatal ``debug()``, ``info()``, and
``warning()`` methods. Use these for diagnostics and return ``BskError`` when
the lifecycle call must fail; Rust logging is not an error-control mechanism.

The generated ABI catches unexpected Rust panics before they can unwind into
C++. The C++ wrapper then translates the returned diagnostic into
``BasiliskError``. This containment protects the language boundary; use
``BskResult`` rather than panics for failures that module code can anticipate.
A caught lifecycle panic poisons that module instance, and later lifecycle
calls fail without invoking its potentially inconsistent internal state.
Expected ``BskError`` results do not poison the instance.
The translated ``BasiliskError`` is the single user-facing diagnostic; the
guarded call suppresses duplicate default Rust panic-hook output on its thread.
Panic-hook behavior outside generated lifecycle calls remains unchanged.

Override ``init(state)`` to set non-zero parameter defaults and initial state.
Rust first initializes every configuration field and ``State`` through their
``Default`` implementations. It then calls ``init(state)`` before Python
configures the module. ``init`` returns ``BskResult<()>`` and its default
implementation returns ``Ok(())``. Module-defined nested configuration structs
and state types must therefore derive or implement ``Default``. Use
``type State = ();`` for a stateless module.

Use the Generated Wrapper
-------------------------

The generated wrapper, named after the module target, provides the Basilisk
``SysModel`` interface. Construct and schedule it as for a C or C++ module:

.. code-block:: python

    module = myModule.myModule()
    simulation.AddModelToTask("taskName", module)

The wrapper owns an opaque Rust module instance and exposes its borrowed
configuration view directly. Configure parameters and connect messages
through ``module``. The Python proxy for ``myModuleConfig`` is an
implementation detail and cannot be constructed independently.

Built-in and Custom Messages
----------------------------

``bsk-messages`` provides Rust bindings for built-in Basilisk message types.
After adding the dependency shown above, import types with
``use bsk_messages::*;`` or use fully qualified names such as
``bsk_messages::AttGuidMsg``.

Custom message types require their normal Basilisk ``*_C`` C-interface header.

Documenting Rust Source
-----------------------

Keep the user-facing module description in ``myModule.rst``. This page is
included in the main Basilisk Sphinx documentation in the same way as the
documentation for C and C++ modules.

Use Rust documentation comments for the source API: ``//!`` documents the
crate or containing module, while ``///`` documents the following struct,
field, function, or method.

Use the usual Basilisk unit-first convention in Rust documentation comments:

.. code-block:: rust

    /// [m] Inertial position vector
    pub position: [f64; 3],
    /// [m/s] Inertial velocity vector
    pub velocity: [f64; 3],
    /// [-] Dimensionless gain
    pub gain: f64,

Place the unit immediately after ``///`` so the source matches C, C++, and
Python Basilisk modules. Use ``[-]`` for dimensionless fields when it improves
clarity. The generated C header preserves these unit comments for Doxygen.

After building Basilisk with Rust support, generate the complete Basilisk HTML
site from the repository root with:

.. code-block:: bash

    cd docs
    make html

The Sphinx module page includes the generated C header through the existing
Doxygen/Breathe pipeline. This provides API documentation for configuration
fields because ``cbindgen`` copies their ``///`` comments into the generated
header.

The generated API section is optional so documentation can still be built
without a Rust toolchain. Build Basilisk with ``--rustModules True`` before
building the documentation to include it. The default header location is
``dist3/rust_headers``; set ``BSK_RUST_HEADER_DIR`` when using a different
build directory.

Testing
-------

The ``#[bsk_build::module]`` attribute automatically omits its lifecycle entry
points from test builds. This lets pure Rust tests run without linking
Basilisk:

.. code-block:: bash

    cd src/fswAlgorithms/<category>/myModule
    cargo test --locked

``init(state)``, ``reset(...)``, and ``update(...)`` are all testable this way.
Logger calls through ``context.logger()`` work in tests when the
``test_logger`` dev-dependency feature is enabled:

.. code-block:: toml

    # Cargo.toml
    [dev-dependencies]
    bsk-build = { path = "../../../architecture/rust/bsk_build", features = ["test_logger"] }

Logger calls in test builds print to ``stderr`` instead of calling Basilisk's
C symbols.

Add the normal Basilisk Python unit test in the module's ``_UnitTest``
directory. Run it after building Basilisk with Rust support:

.. code-block:: bash

    python3 -m pytest src/fswAlgorithms/<category>/myModule/_UnitTest -v

Before contributing a module, follow the :ref:`bskModuleCheckoutList`.

Out-of-tree Rust extensions are not members of the Basilisk workspace. They
remain independent Cargo packages or workspaces and should commit their own
``Cargo.lock`` files.

Further Reading
---------------

The :ref:`writingRustPlugins` guide covers advanced Rust configuration types,
stateful modules, logging, and testing. It also explains how to package the
same kind of module as an out-of-tree extension.
