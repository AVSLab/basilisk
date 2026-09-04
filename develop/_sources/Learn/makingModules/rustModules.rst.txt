.. _rustModules:

:beta:`Making Rust Modules`
===========================

.. important::

   Rust module support requires Rust 1.89 or newer. The current stable Rust
   toolchain is recommended.

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

    python3 conanfile.py --rustModules True

On Windows, use ``python`` instead of ``python3`` if that is the command for
the active Python installation.

``rustModules`` enables discovery and compilation of in-tree Rust modules and
Rust modules supplied through ``pathToExternalModules``. Cargo builds the Rust
code, while Corrosion connects Cargo packages to Basilisk's CMake build as
ordinary CMake targets. Basilisk downloads a pinned Corrosion version
automatically during the first Rust-enabled configuration, so developers only
need to install Rust and Cargo.

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
contains individual and fixed-size arrays of input and output messages,
Python-visible parameters, Rust-owned private state, logging, expected errors,
Rust-native tests, and a test-only panic used to verify the language boundary.

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
   marker and are not exposed as Python modules. During CMake configuration,
   Basilisk asks Cargo for workspace metadata and reads this typed Boolean
   marker from Cargo's package model; Basilisk does not interpret
   ``Cargo.toml`` syntax itself.

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
        bsk_build::generate_bindings("MyModuleConfig");
    }

Pass the exact name of the struct marked with ``#[bsk_build::module]``.
Cargo compiles and runs ``build.rs`` as a separate host program before it
compiles the module crate, so the build script cannot refer to
``MyModuleConfig`` as a Rust type. ``generate_bindings()`` passes the selected
identifier into the subsequent Rust compilation, where the procedural
attribute verifies that both names match. A stale or misspelled name therefore
produces a compile error.

During the build:

#. Cargo runs ``build.rs``.
#. ``bsk-build`` uses ``cbindgen`` to generate a C-compatible header for the
   selected configuration.
#. ``bsk-build`` generates the module-specific SWIG interface.
#. Rust compiles the module, and ``#[bsk_build::module]`` verifies the selected
   configuration name while generating the lifecycle boundary.
#. CMake links the Rust static library and generated wrapper into the normal
   Basilisk Python package.

The generated header and SWIG interface are build products; do not edit them.

Register the Module in the Workspace
------------------------------------

All in-tree Rust packages belong to the Cargo workspace rooted at
``src/Cargo.toml``. The workspace gives Basilisk one dependency resolution and
one committed ``src/Cargo.lock``. Workspace registration is also required for
module discovery: CMake considers the packages returned by ``cargo metadata``
and selects those whose ``[package.metadata.basilisk]`` ``module`` value is
``true``.

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

Integrated External Rust Modules
--------------------------------

Rust modules can remain in a separate source repository while participating
in an integrated Basilisk build through ``pathToExternalModules``. Place each
module directly under the external project's ``ExternalModules`` directory:

.. code-block:: text

    External/
    |-- Cargo.toml
    |-- Cargo.lock
    `-- ExternalModules/
        `-- myRustModule/
            |-- Cargo.toml
            |-- build.rs
            |-- myRustModule.rs
            |-- myRustModule.rst
            `-- _UnitTest/
                `-- test_myRustModule.py

A Cargo workspace at ``External/Cargo.toml`` is recommended so all Rust
modules in that project share one dependency resolution and lockfile:

.. code-block:: toml

    [workspace]
    resolver = "2"
    members = ["ExternalModules/myRustModule"]

    [workspace.package]
    rust-version = "1.89"

    [profile.dev]
    panic = "unwind"

    [profile.release]
    panic = "unwind"

An external module uses the same package marker, static-library crate type,
source layout, and ``build.rs`` as an in-tree module. Its dependencies must
point to the support crates in the Basilisk checkout used for the integrated
build. For example:

.. code-block:: toml

    [dependencies]
    bsk-messages = { path = "/path/to/basilisk/src/architecture/rust/bsk_messages" }
    bsk-build = { path = "/path/to/basilisk/src/architecture/rust/bsk_build" }

    [build-dependencies]
    bsk-build = { path = "/path/to/basilisk/src/architecture/rust/bsk_build",
                  default-features = false, features = ["codegen"] }

Use paths relative to the external project when its location relative to the
Basilisk checkout is controlled; this keeps the project portable between
machines. Until the Basilisk Rust support crates are distributed separately,
the Cargo dependency paths couple the external module to that checkout.

Generate and commit the external workspace's lockfile, then enable both build
options:

.. code-block:: console

    cargo generate-lockfile --manifest-path /path/to/External/Cargo.toml
    python3 conanfile.py --clean --rustModules True \
        --pathToExternalModules "/path/to/External"

The resulting module is imported from ``Basilisk.ExternalModules``. External
C payload headers under ``External/msgPayloadDefC`` are included in the normal
C-message generation step and are therefore available through
``bsk-messages``. Rerun the same ``conanfile.py`` command after adding or
editing one of those payloads.

For a project containing only one Rust module, the module may instead be an
independent Cargo package with its own committed ``Cargo.lock`` and no
workspace manifest at the external root. Basilisk discovers marked
``Cargo.toml`` files directly under ``ExternalModules/<module>``. A shared
external workspace is preferred when the project contains multiple Rust
packages.

Write the Module
----------------

The following module has one Python-visible gain, one required input, one
output, and no private state:

.. code-block:: rust

    use bsk_messages::*;

    #[bsk_build::module]
    #[repr(C)]
    pub struct MyModuleConfig {
        /// [Nm] Proportional gain
        #[bsk(validate = validate_gain)]
        pub K: f64,
        /// [-] Attitude guidance input
        pub attGuidInMsg: MsgReader<AttGuidMsg>,
        /// [Nm] Commanded body torque output
        pub cmdTorqueOutMsg: MsgWriter<CmdTorqueBodyMsg>,
    }

    fn validate_gain(
        _config: &MyModuleConfig,
        proposed_gain: &f64,
    ) -> BskResult<()> {
        if !proposed_gain.is_finite() || *proposed_gain <= 0.0 {
            return Err(BskError::new("K must be finite and positive"));
        }
        Ok(())
    }

    impl BskModule for MyModuleConfig {
        type State = ();
        type Inputs = MyModuleInputs;
        type Outputs = MyModuleOutputs;

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
            // Retain reset-time validation for defaults, internal changes,
            // and relationships involving multiple fields.
            validate_gain(self, &self.K)?;
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
            Ok(MyModuleOutputs {
                cmdTorqueOutMsg: Some(CmdTorqueBodyMsg {
                    torqueRequestBody: [
                        -self.K * guidance.sigma_BR[0],
                        -self.K * guidance.sigma_BR[1],
                        -self.K * guidance.sigma_BR[2],
                    ],
                }),
            })
        }
    }

The Rust-specific declarations mean:

``use bsk_messages::*;``
   Imports the Basilisk Rust message values and module support types used in
   the file.

``#[repr(C)]``
   Requests a C-compatible field layout for the Python-visible configuration.
   The generated ABI copies complete typed values between the C++/SWIG wrapper
   and the Rust-owned module.

``#[bsk_build::module]``
   Identifies the one top-level module configuration, validates its supported
   field types, and generates the lifecycle entry points.

``pub``
   Makes a configuration field visible to ``cbindgen``. The generated wrapper
   hides direct field access and exposes a Rust-backed Python property plus
   ``getX`` and ``setX`` methods.

``type State``, ``Inputs``, and ``Outputs``
   Select the private state type and the named message-value structs used by
   the lifecycle methods. The input and output types are generated from the
   annotated ports. For ``MyModuleConfig`` they are named
   ``MyModuleInputs`` and ``MyModuleOutputs``.

``BskResult<T>``
   Represents either success, ``Ok(T)``, or an expected module failure,
   ``Err(BskError)``.

Configuration Getters, Setters, and Validation
----------------------------------------------

Every non-port configuration field receives generated getter and setter
methods. For a field named ``increment``, Python can use either the familiar
property or the explicit methods:

.. code-block:: python

   module.increment = 2.0
   value = module.increment

   module.setIncrement(2.0)
   value = module.getIncrement()

Both forms call the same guarded Rust accessor. Fixed-size arrays are copied
and validated as complete values. Assigning the wrong number of elements
raises ``BasiliskError`` without changing the field.

Use ``#[bsk(validate = function_name)]`` when an assignment requires immediate
validation. The named function receives the current configuration and a
borrowed proposed value:

.. code-block:: rust

   #[bsk(validate = validate_gain)]
   pub K: f64,

   fn validate_gain(
       _config: &MyModuleConfig,
       proposed_gain: &f64,
   ) -> BskResult<()> {
       if !proposed_gain.is_finite() || *proposed_gain <= 0.0 {
           return Err(BskError::new("K must be finite and positive"));
       }
       Ok(())
   }

The generated setter calls the validator before assignment. Returning
``Err(BskError)`` raises Python ``BasiliskError`` and preserves the previous
value. The first argument provides the current configuration when checks need
to involve other fields.

Module code can still update ``self.K`` directly inside Rust. Such internal
assignments do not call the external setter. Retain appropriate reset-time
validation for required values, relationships involving several fields, and
configuration that may be changed internally.

Data arriving through an input message also does not pass through a
configuration setter. Validate such data after reading it in ``reset`` or
``update``. For example, :ref:`mrpPDRust` validates its message-delivered
spacecraft inertia during reset.

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

   ``Default`` provides the familiar initial values for primitive
   configuration fields: numeric fields start at zero, booleans at ``false``,
   arrays are initialized element-by-element, and message ports start empty.
   Custom Rust-owned state uses its own ``Default`` implementation and may
   therefore begin with non-zero values, allocated collections, strings, or
   enum variants. Use ``init`` for any non-zero Python-visible configuration
   defaults that should be applied before the user configures the module.

``reset(state, context, current_sim_nanos) -> BskResult<Outputs>``
   Runs at simulation start and on every Basilisk ``Reset()``. Use it for
   parameter validation and private-state reset. The default returns an
   ``Outputs`` value containing ``None`` for every port and therefore does not
   publish during reset. Override it to return ``Some(payload)`` for any port
   that requires an explicit reset value.

``update(state, context, inputs, current_sim_nanos) -> BskResult<Outputs>``
   Runs on each task update. It receives message values through ``inputs`` and
   selects output messages through ``outputs``. The generated wrapper performs
   the actual message reads and writes. ``update`` has no default and must be
   implemented.

If ``reset`` or ``update`` returns ``Err(BskError::new("..."))``, the wrapper
raises the normal Python ``BasiliskError`` after Rust returns. No output
message is written for the failed lifecycle call.

Unused Lifecycle Parameters
---------------------------

The ``BskModule`` trait fixes the lifecycle method signatures.  Keep an unused
argument in the implementation and prefix its binding with an underscore to
tell the Rust compiler that it is intentionally unused:

.. code-block:: rust

   fn reset(
       &mut self,
       _state: &mut Self::State,
       _context: &BskContext<'_>,
       _current_sim_nanos: u64,
   ) -> BskResult<Self::Outputs> {
       Ok(Self::Outputs::default())
   }

Use a descriptive underscore-prefixed name instead of a bare ``_`` so the
argument's purpose remains clear.  Remove the underscore if the implementation
later uses the value.

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

The port type declares its direction: ``MsgReader<T>`` is an input and
``MsgWriter<T>`` is an output. Required readers and all writers need no port
annotation. Add ``#[bsk(optional)]`` only when an input may be unconnected:

.. code-block:: rust

    /// [-] Required attitude input
    pub navAttInMsg: MsgReader<NavAttMsg>,

    /// [Nm] Optional disturbance estimate
    #[bsk(optional)]
    pub disturbanceInMsg: MsgReader<CmdTorqueBodyMsg>,

    /// [Nm] Commanded body torque
    pub cmdTorqueOutMsg: MsgWriter<CmdTorqueBodyMsg>,

A required input causes ``Reset`` or ``UpdateState`` to fail with
``BasiliskError`` when it is not connected. The generated input field for an
optional port has type ``Option<Msg>``: it is ``Some(value)`` when connected
and ``None`` when unconnected.

The generated input and output structs use the same field names as the ports.
Any number of inputs and outputs is supported, and declaration order does not
control message routing. Each output field has type ``Option<Msg>``:
``Some(payload)`` publishes that port and ``None`` leaves it unchanged for the
current lifecycle call.

Writing Output Messages
~~~~~~~~~~~~~~~~~~~~~~~

Rust module logic returns message payload values in its generated ``Outputs``
struct. It does not call ``MsgWriter`` or fill in a Basilisk message header
directly. Each returned field is matched by name to the corresponding
``MsgWriter<T>`` configuration field. For example, the return value

.. code-block:: rust

    Ok(RustModuleTemplateOutputs {
        dataOutMsg: Some(data_out_msg),
        dataOutMsgs: data_out_msgs.map(Some),
    })

writes ``data_out_msg`` to ``dataOutMsg`` and each wrapped element of
``data_out_msgs`` to the same-index element of ``dataOutMsgs``. Return ``None``
for any individual field or array element that should not publish. Additional
output ports are handled by adding their named optional payloads to the same
generated output struct. ``Outputs::default()`` sets every field and array
element to ``None``, which is useful when a lifecycle call publishes nothing.

After ``reset`` or ``update`` returns ``Ok(...)``, the generated lifecycle
code publishes each selected ``Some(payload)`` output and completes its
message header automatically:

* ``moduleID`` is the unique ID assigned when the Python/C++ module wrapper is
  constructed. Every output published by that module receives this ID.
* ``timeWritten`` is the ``current_sim_nanos`` value for that ``Reset`` or
  ``UpdateState`` call [ns]. All outputs published by one lifecycle call
  receive the same timestamp.
* ``isWritten`` is set to true by the normal Basilisk C message write
  interface.

Thus, these header values do not appear in the Rust ``Outputs`` value and the
module developer should not set them. ``SelfInit`` initializes the output
ports but does not publish payloads. A lifecycle call that returns ``Err(...)``
does not write any output. A successful call may publish all outputs, only a
subset, or none. A skipped port retains its previous payload and message-header
state.

Fixed-Size Arrays of Message Ports
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Use a Rust array when a module has a fixed number of ports with the same
message type. The array length is part of the module interface:

.. code-block:: rust

    /// [-] Required navigation inputs
    pub navAttInMsgs: [MsgReader<NavAttMsg>; 2],

    /// [-] Optional navigation inputs
    #[bsk(optional)]
    pub optionalNavAttInMsgs: [MsgReader<NavAttMsg>; 2],

    /// [Nm] Commanded body torques
    pub cmdTorqueOutMsgs: [MsgWriter<CmdTorqueBodyMsg>; 2],

The generated lifecycle value fields preserve the same length. In this
example, ``inputs.navAttInMsgs`` has type ``[NavAttMsg; 2]``,
``inputs.optionalNavAttInMsgs`` has type ``[Option<NavAttMsg>; 2]``, and the
output field has type ``cmdTorqueOutMsgs: [Option<CmdTorqueBodyMsg>; 2]``.
Every element of a required input array must be connected. An unconnected
required element reports its array index in the resulting ``BasiliskError``.

Process and return the arrays with normal Rust array operations:

.. code-block:: rust

    let cmd_torques = inputs.navAttInMsgs.map(|navigation| {
        CmdTorqueBodyMsg {
            torqueRequestBody: [
                -self.K * navigation.sigma_BN[0],
                -self.K * navigation.sigma_BN[1],
                -self.K * navigation.sigma_BN[2],
            ],
        }
    });

    Ok(MyModuleOutputs {
        cmdTorqueOutMsgs: cmd_torques.map(Some),
    })

Construct the ``[Option<MessageType>; N]`` value with a literal array,
``core::array::from_fn``, or the array ``map`` method. Each element is
independent; for example, ``[Some(first_torque), None]`` publishes only array
element zero.

Python returns a list containing the fixed set of normal message interfaces:

.. code-block:: python

    for input_port, source_message in zip(module.navAttInMsgs, source_messages):
        input_port.subscribeTo(source_message)

    output_recorders = [
        output_port.recorder() for output_port in module.cmdTorqueOutMsgs
    ]

Each list element is a live port, so calling ``subscribeTo()`` or
``recorder()`` affects the module directly. The property cannot be assigned.
Adding, removing, or replacing entries in the returned Python list only
changes that temporary list, not the module's fixed set of ports.

An optional input array can represent a variable number of connected inputs
up to a fixed maximum, as a C module commonly does with a fixed-capacity
message array. Its lifecycle value is an array of ``Option<Msg>`` values, so
the Rust implementation can process the connected ``Some`` entries and skip
the unconnected ``None`` entries.

Only fixed-size message-port arrays are supported in the Python-visible
configuration. A Rust ``Vec<MsgReader<T>>`` or ``Vec<MsgWriter<T>>`` does not
have a stable C layout and is rejected. The dynamically sized C++
``std::vector`` message pattern does not yet have a Rust-module equivalent.

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
* nested, by-value ``#[repr(C)]`` parameter structs that derive
  ``bsk_build::BskConfigValue``; and
* ``MsgReader<T>`` and ``MsgWriter<T>`` ports, individually or in fixed-size
  arrays.

Every configuration field must implement Rust's ``Default`` behavior because
Rust constructs the complete module before calling ``init``. Every non-port
field must also implement ``BskConfigValue``. This safety contract restricts
the generated raw-copy boundary to types with matching Rust and C++ layouts,
valid bit patterns, and no Rust ownership or borrowed references.

Basilisk provides ``BskConfigValue`` for Boolean, fixed-width integer, and
floating-point scalars and recursively for fixed-size arrays. A type alias for
one of these types inherits the implementation. Module-defined nested structs
must use plain ``#[repr(C)]`` and derive ``Clone``, ``Copy``, ``Default``, and
``bsk_build::BskConfigValue``. The derive verifies that the struct and all of
its fields satisfy the same boundary contract.

Configuration fields cannot contain raw pointers, Rust enums, dynamically
sized strings, ``Vec`` collections, references, characters, or other owning
Rust types. These types do not have a safe, general Python/C representation.
Put them in private Rust state instead. Do not manually implement the unsafe
``BskConfigValue`` trait merely to bypass a compiler error; a manual
implementation assumes responsibility for every cross-language layout and
bit-validity guarantee enforced by the derive.

Nested parameters can be grouped by value:

.. code-block:: rust

    #[repr(C)]
    #[derive(Clone, Copy, Default, bsk_build::BskConfigValue)]
    pub struct ControllerGains {
        /// [Nm] Proportional gain
        pub K: f64,
        /// [Nm/(rad/s)] Rate gain
        pub P: f64,
    }

    #[bsk_build::module]
    #[repr(C)]
    pub struct MyModuleConfig {
        /// [-] Controller parameters
        pub gains: ControllerGains,
        // Message ports follow.
    }

The getter returns a copy of a nested value. Modify that copy and assign the
complete value through the setter:

.. code-block:: python

    gains = module.gains
    gains.K = 0.1  # [Nm]
    module.gains = gains

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

The :ref:`rustModuleTemplate` module exercises both a nested configuration
struct and a multidimensional fixed-size array through the complete generated
Rust, C++, SWIG, and Python interface.

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

    impl BskModule for MyModuleConfig {
        type State = MyState;
        type Inputs = MyModuleInputs;
        type Outputs = MyModuleOutputs;

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

Basilisk C Utilities
--------------------

The ``bsk-utilities`` package provides safe Rust wrappers around selected
Basilisk utilities that already have a C interface. An in-tree module enables
the shared workspace dependency in ``Cargo.toml``:

.. code-block:: toml

    [dependencies]
    bsk-utilities.workspace = true

    [dev-dependencies]
    bsk-utilities = { workspace = true, features = ["ffi-tests"] }

Import and call the wrappers with ordinary Rust values:

.. code-block:: rust

    use bsk_utilities::attitude;

    let sigma = attitude::dcm_to_mrp(dcm); // [-]
    let wrapped_angle = attitude::wrap_to_pi(angle); // [rad]

The safe wrappers convert fixed-size Rust arrays to the pointers expected by
the existing C functions. The Basilisk build links their implementation from
``ArchitectureUtilities``. The ``ffi-tests`` development feature compiles the
same C sources into Rust test executables, allowing ``cargo test`` to execute
utility-backed module code without a configured CMake build. Module code
should not call functions in the generated ``bsk_utilities::raw`` module
directly. See
:ref:`rustModuleTemplate` for an in-tree module that calls a safe utility
wrapper from its update method.

Linear Algebra with ``nalgebra``
--------------------------------

Basilisk's Rust workspace provides
`nalgebra <https://nalgebra.rs/>`__ for vector and matrix operations. In-tree
modules select the shared, reviewed version with
``nalgebra.workspace = true`` when they need linear algebra:

.. code-block:: toml

    [dependencies]
    nalgebra.workspace = true

This is the Rust equivalent of using Eigen in a C++ Basilisk module. Common
fixed-size types include ``Vector3<f64>`` and ``Matrix3<f64>``:

.. code-block:: rust

    use nalgebra::{Matrix3, Vector3};

    let inertia = Matrix3::from_row_slice(&vehicle_config.ISCPntB_B); // [kg*m^2]
    let omega = Vector3::from_column_slice(&guidance.omega_BR_B); // [rad/s]
    let angular_momentum = inertia * omega; // [N*m*s]

Keep ``nalgebra`` types inside lifecycle methods or Rust-owned private state.
Python-visible configuration and Basilisk C message payloads must retain their
C-compatible array types. Convert an array to a matrix with
``Matrix3::from_row_slice`` because Basilisk stores matrix payload fields in
row-major order. Convert a result back to an array when constructing an output
message:

.. code-block:: rust

    let torque: Vector3<f64> = calculate_torque(); // [N*m]
    let output = CmdTorqueBodyMsg {
        torqueRequestBody: [torque[0], torque[1], torque[2]],
    };

See :ref:`mrpPDRust` for a complete controller that uses ``nalgebra`` while
keeping its generated C/Python interface unchanged.

Simulation Time
---------------

Basilisk passes the current simulation time to ``reset()`` and ``update()`` as
the ``u64`` value ``current_sim_nanos`` in nanoseconds.  Convert an absolute
timestamp to seconds with ``NANO2SEC``:

.. code-block:: rust

    use bsk_messages::BskError;
    use bsk_utilities::constants::NANO2SEC;

    let absolute_time_sec = current_sim_nanos as f64 * NANO2SEC; // [s]

This conversion remains finite during simulations longer than
:math:`2^{53}` nanoseconds, approximately 104 days, although an ``f64`` can no
longer preserve every individual nanosecond at that scale.

For a relative time, subtract the integer timestamps before converting.  Use
``checked_sub()`` when the stored timestamp is expected to precede the current
time:

.. code-block:: rust

    let elapsed_nanos = current_sim_nanos
        .checked_sub(previous_sim_nanos)
        .ok_or_else(|| BskError::new("simulation time moved backwards"))?;
    let elapsed_time_sec = elapsed_nanos as f64 * NANO2SEC; // [s]

Performing the subtraction first retains the precision of a small elapsed
interval even when the absolute simulation time is large.  Store the previous
timestamp as a ``u64`` and update it only after the elapsed-time calculation.

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

The Python object exposes generated properties for the marked configuration
fields along with the normal ``SysModel`` fields. Each property uses its Rust
getter or setter. The generated ``MyModuleConfig`` proxy is an implementation
detail and should not be constructed separately.

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

    python3 conanfile.py --rustModules True
    cd docs
    make html

The generated HTML is stored under ``docs/build/html``. The Rust API section
is omitted when its generated header is unavailable.

Test the Module
---------------

Every in-tree Rust module must have the normal Basilisk Python unit test. Add
Rust-native tests when they provide useful direct coverage of private state,
validation, numerical helpers, or other internal Rust behavior.

Rust-Native Tests
~~~~~~~~~~~~~~~~~

Place ``#[cfg(test)]`` tests in ``myModule.rs`` or in the package's Rust test
layout. These tests do not link the compiled Basilisk library, so that library
does not need to be built with ``--rustModules True``. However,
``bsk-messages`` generates its Rust bindings from the normal C-message headers
under ``dist3/autoSource/cMsgCInterface``. Configure Basilisk at least once to
generate those headers; either a normal or Rust-enabled configuration works:

.. code-block:: console

    python3 conanfile.py

Then run the Rust-native tests from the module directory:

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

Guard the test before importing its generated wrapper. This skips the test in
a Rust-disabled Basilisk build, while still reporting a missing wrapper as a
failure when Rust support was requested:

.. code-block:: python

    import pytest

    from Basilisk import hasBuildFeature

    rustModulesEnabled = hasBuildFeature("rustModules")
    pytestmark = pytest.mark.skipif(
        not rustModulesEnabled,
        reason="Requires Basilisk built with --rustModules True",
    )
    if rustModulesEnabled:
        from Basilisk.fswAlgorithms import myModule

After a Rust-enabled Basilisk build, the normal ``python3 run_all_test.py``
workflow includes the module's Python test. The same command also runs all
Cargo workspace tests when ``cargo`` is installed and available on ``PATH``.
It reports that Rust tests were skipped, without failing, when Rust support is
disabled or ``cargo`` is not available. The Cargo tests exercise the Rust
packages directly without linking the generated Basilisk module library, but
they still require the C-message headers produced by the Rust-enabled
Basilisk configuration performed before ``run_all_test.py``.

Contributor Checks
~~~~~~~~~~~~~~~~~~

Before submitting a Rust module, run the complete workspace checks from the
repository root:

.. code-block:: console

    cargo fmt --all --check --manifest-path src/Cargo.toml
    cargo test --workspace --all-features --locked --manifest-path src/Cargo.toml
    cargo test -p bsk-build --no-default-features --locked --manifest-path src/Cargo.toml
    cargo clippy --workspace --all-targets --all-features --locked \
        --manifest-path src/Cargo.toml -- -D warnings

The repository's local pre-commit hook runs the same Rust formatting check
whenever Rust source is changed. The local hook reports a skip instead of
failing when Cargo or ``rustfmt`` is not installed, keeping Rust optional for
non-Rust development. Pull-request CI installs and always runs the formatter,
checks the minimum supported Rust version, and verifies that generated
language boundaries retain panic containment. Follow the
:ref:`bskModuleCheckoutList` for the remaining module review requirements.

Current Limitations
-------------------

* Rust module support is experimental, and its generated interface may change
  between Basilisk releases.
* Python-visible configuration is limited to supported C-compatible types.
  Put dynamic Rust data in ``BskModule::State``.
* Arbitrary Rust methods are not exported to Python. Configuration fields
  receive generated getters, setters, and properties; other public methods
  remain the standard Basilisk lifecycle and message interfaces.
* The Rust utility bindings cover C-compatible Basilisk utilities and
  constants. C++-only utilities require a C-compatible wrapper before Rust can
  use them; use ``nalgebra`` for native Rust vector and matrix operations.
* After adding or changing a message payload, rerun
  ``python3 conanfile.py --rustModules True`` to regenerate the matching Rust
  bindings in the build directory.

Reference
---------

The generated ABI is described in
:download:`bsk_rust_module.h <../../../../src/architecture/_GeneralModuleFiles/bsk_rust_module.h>`.
The Rust runtime and generation API are documented in
``src/architecture/rust/bsk_build/src/lib.rs``. For a working implementation,
configuration, Python test, and Rust-native tests, see
:ref:`rustModuleTemplate`.
