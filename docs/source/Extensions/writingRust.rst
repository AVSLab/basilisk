.. toctree::
   :maxdepth: 1
   :hidden:

.. _writingRustPlugins:

Writing a Rust Plugin
======================

.. sidebar:: Status: Experimental

    Rust module support is experimental. The interface and support may change between ``bsk-sdk`` releases.

A Rust module is a Basilisk C module whose ``SelfInit``/``Reset``/``Update``
lifecycle functions happen to be implemented in Rust instead of C.  It uses
the same plugin packaging described in :ref:`writingPlugins`
(``pyproject.toml``, a Python package directory, ``bsk-sdk`` as the build
dependency) plus one additional CMake macro, ``bsk_add_rust_module``, that
compiles a Cargo crate and wires it into the SWIG extension.

The Rust-specific pieces are all generated for you: define one
``#[repr(C)]`` config struct and a ``BskModule`` implementation, and
``build.rs`` (via the ``bsk-build`` crate) generates the C header and glue
code needed to link it into Basilisk. You never write raw FFI by hand.

A complete working example is in the
`bsk-sdk repository <https://github.com/AVSLab/bsk_sdk/tree/master/examples/rust-mrp-plugin>`_.
It is intentionally a minimal *pattern* demo — enough to exercise every
mechanism on this page in context — not a feature-complete port of any
particular Basilisk module.

Prerequisites
-------------

- Basilisk installed: ``pip install "bsk[all]"`` (or a local editable install)
- The SDK: ``pip install bsk-sdk``
- Rust and Cargo: ``curl https://sh.rustup.rs -sSf | sh``
- CMake ≥ 3.26 and a C++17 compiler (same requirement as Basilisk itself)
- ``scikit-build-core``, ``build``, ``cmake``, and ``ninja`` Python packages:
  ``pip install scikit-build-core build cmake ninja``

Plugin Layout
-------------

A Rust module's source directory only ever contains ``Cargo.toml``, ``src/``,
a 3-line ``build.rs``, and its own ``_UnitTest/``.  Nothing is generated back
into it — every build artifact (C header, SWIG wrapper, compiled extension)
lands in the CMake build tree instead:

.. code-block:: text

    my-rust-plugin/
    ├── pyproject.toml
    ├── CMakeLists.txt
    ├── my_rust_plugin/            # Python package (compiled extension installs here)
    │   └── __init__.py
    └── myModule/                  # Rust crate
        ├── Cargo.toml
        ├── build.rs                 # 3 lines — delegates to bsk-build
        ├── src/
        │   └── lib.rs                 # config struct + BskModule impl
        └── _UnitTest/
            └── test_myModule.py

.. code-block:: rust

    // build.rs — the entire file, for every Rust module
    fn main() {
        bsk_build::generate();
    }

``bsk-build`` appears twice in ``Cargo.toml``: once as a normal dependency
(for the ``bsk_module!()`` macro used in ``src/lib.rs``, see below) and once
as a build dependency (for ``generate()`` above). The normal-dependency
entry disables default features so it pulls in only the macro, not the
codegen that ``build.rs`` needs:

.. code-block:: toml

    [dependencies]
    bsk-messages = "..."
    bsk-build    = { version = "...", default-features = false }

    [build-dependencies]
    bsk-build    = "..."

The Config Struct and the ``BskModule`` Trait
-----------------------------------------------

Define the config struct once, in Rust, and everything else — the C header,
the SWIG wrapper, the message I/O glue — is generated from it:

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

``bsk-build`` (run from ``build.rs``) reads this struct and generates the C
header consumed by CMake/SWIG, plus the code that reads/writes messages and
calls into your ``BskModule`` implementation — ``bsk_module!()`` (from
``bsk-build``) pulls that generated code in.

Field ``///`` doc comments become the Doxygen ``/*!< … */`` comment in the
generated header, so document fields the same way you would in a
hand-written Basilisk C module. ``self_init`` and ``reset`` have empty
default bodies — implement them only if the module needs them. ``update`` is
mandatory. ``Inputs``/``Outputs`` are tuples of message types matching the
``MsgReader``/``MsgWriter`` fields, in declaration order (see `Messaging`_
below).

Gate ``bsk_module!()`` with ``#[cfg(not(test))]`` so ``cargo test`` (see
`Testing at every layer`_ below) doesn't require Basilisk to be linked in.

Field ordering convention
~~~~~~~~~~~~~~~~~~~~~~~~~~

Existing BSK C modules don't follow one strict field-ordering convention —
the one convention that does hold everywhere is ``bskLogger`` being the last
field. ``runtime`` isn't required to be first, but every other field reads it, so putting it first reads well:

.. code-block:: text

    1. runtime: BskModuleRuntime        — required; see below
    2. scalar / array parameters
    3. state: Option<Box<T>>            — heap state, stateful modules only
    4. MsgReader<..> input fields
    5. MsgWriter<..> output fields
    6. bskLogger: *mut BSKLogger 

``BskModuleRuntime`` — the ``SysModel`` mirror
------------------------------------------------

A Rust module has no C++ base class, so ``runtime: BskModuleRuntime``
mirrors the relevant ``SysModel`` fields (module ID, name, ...) and is kept
up to date before every ``SelfInit``/``Reset``/``Update`` call, so Rust code
reads it like any other config field — use autocomplete on ``self.runtime``
to see what's available:

.. code-block:: rust

    fn update(&mut self, inputs: Self::Inputs, current_sim_nanos: u64) -> Self::Outputs {
        let id = self.runtime.module_id();
        // ...
    }

Anything borrowed from runtime (e.g. the module's name, from ``model_tag()``) is
only valid for the call that received it. This isn't just documented
convention — ``BskModuleRuntime`` isn't ``Copy``/``Clone``, and
``model_tag()`` returns a ``&str`` borrowed from ``&self.runtime``, so the
compiler itself rejects any attempt to stash either one somewhere that
outlives the current ``update``/``reset``/``self_init`` call.

``currentSimNanos`` (nanoseconds) is passed as a separate argument to ``reset`` and ``update``.

Logging
-------

A ``bskLogger: *mut BSKLogger`` field gets standard Basilisk logging through
``bsk-messages``' ``BskLoggerExt`` trait, which adds
``.debug()``/``.info()``/``.warning()``/``.bsk_error()`` methods directly on
the field:

.. code-block:: rust

    fn reset(&mut self, _current_sim_nanos: u64) {
        if self.K <= 0.0 {
            self.bskLogger.warning("K should be positive");
        }
    }

``.bsk_error(msg)`` raises the standard fatal ``BasiliskError`` and never
returns. A null ``bskLogger`` (module under test, or Python never set one)
falls back to a default logger, so these are always safe to call.

Messaging
---------

Message ports: ``MsgReader``/``MsgWriter``
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

A config struct field of type ``MsgReader<Foo>`` is an input port that reads
``Foo`` values; ``MsgWriter<Foo>`` is an output port that writes them (``Foo``
being e.g. ``AttGuidMsg`` or ``CmdTorqueBodyMsg``). Reads and writes are
handled automatically for you, shouldn't need to worry about the reader and writer ports.
For example, ``update()`` is wrapped with code that reads the input message values and 
gives them to you in a tuple, and takes the output of your ``update()`` and writes to the output message ports.

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

Output ports are initialized automatically. ``update()`` just returns the
message value(s); they're written out with the module's ID and the current
simulation time stamped into the header, exactly like a hand-written C
module.

Custom (non-built-in) message types
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Built-in Basilisk messages (anything under ``cMsgCInterface/`` in
``bsk-sdk``) work out of the box.

A **custom** message type needs its ``*_C`` C-interface header defined the
same way any Basilisk plugin defines a custom message: write (or generate,
via Basilisk's message codegen pipeline) a C struct payload and a
``*_C.h``/``*_C.cpp`` interface pair, and add its directory to
``bsk_add_rust_module(... INCLUDE_DIRS ...)``. Once that header exists, the
field is treated identically to any built-in message port.

The ``bsk-messages`` crate
~~~~~~~~~~~~~~~~~~~~~~~~~~~

``bsk-messages`` is generated by ``tools/gen_rust_messages.py`` using ``bindgen`` 
against the vendored Basilisk headers, then regenerated whenever ``tools/sync_headers.py`` runs
(if ``bindgen`` is available). It provides:

- Message types for standard Basilisk messages (``AttGuidMsg``,
  ``CmdTorqueBodyMsg``, ...) — read/written via ``MsgReader``/``MsgWriter``.
- ``MsgHeader``, ``BSKLogger``, ``logLevel_t``.
- ``BskModuleRuntime``, the ``BskModule`` trait, and the ``BskLoggerExt``
  logging trait (see `Logging`_ above).


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
        fn self_init(&mut self) {
            self.state = Some(Box::new(MyState::default()));
        }

        fn update(&mut self, inputs: Self::Inputs, _t: u64) -> Self::Outputs {
            let state = self.state.as_mut().expect("set in self_init");
            // ...
        }
    }

``bsk-build`` maps ``Option<Box<T>>`` to a nullable ``void *`` in the
generated header and arranges for it to be freed automatically whenever the
owning object is destroyed — Python garbage collection, explicit ``del``, or
process exit — with no cleanup code to write by hand.

Python has no legitimate reason to touch this field directly, so
``bsk_add_rust_module`` marks it ``%immutable``: the setter is absent from
the Python API, so a script can't null it out (leaking the boxed value) or
alias it to an unrelated pointer (causing a crash on cleanup). The getter
still works, returning an opaque, useless pointer handle.

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
(``ctrl.target.x = 1.0``) — no special handling needed. Nesting may be
arbitrarily deep, but not self-referential. Message ports and
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

CMakeLists.txt: ``bsk_add_rust_module``
------------------------------------------

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

pyproject.toml conventions
---------------------------

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
   **pinned to the same version**, exactly as for a C++/C plugin (see
   :ref:`writingPlugins`). Replace ``2.X.Y`` with the Basilisk version being
   targeted.

Building, testing, and installing
------------------------------------

Testing at every layer
~~~~~~~~~~~~~~~~~~~~~~~~

.. list-table::
   :header-rows: 1
   :widths: 25 45 20

   * - Layer
     - Command
     - Needs Basilisk?
   * - Pure-Rust unit tests
     - ``cargo test`` (from the module's crate dir)
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
``#[cfg(not(test))]`` (see `The Config Struct and the BskModule Trait`_
above) so ``update()`` and friends can be exercised as plain Rust functions
with hand-built message values — no linking, no Python.

Known limitations
--------------------

- Rust module support is experimental; the ABI (``BskModuleRuntime``,
  the generated SWIG interface, the ``BskModule`` trait) may change between
  ``bsk-sdk`` releases.
- The ``bsk-messages`` and ``bsk-utilities`` crates must be regenerated
  after Basilisk header syncs; ``tools/sync_headers.py`` does this
  automatically when ``bindgen``/``bindgen-cli`` is available, otherwise it
  is a manual step.
- Custom message types require a hand-written ``*_C`` C-interface header —
  there is no Rust-side message-definition DSL.
- ``bsk-utilities`` covers C ABI utilities/constants only; C++/Eigen
  utilities (``Eigen::Vector3d``, ``Eigen::Matrix3d``, ...) are out of scope
  until a dedicated C shim is designed.

Source of truth
------------------

The canonical reference for this ABI is the header comment in
:download:`bsk_rust_module.h <../../../src/architecture/_GeneralModuleFiles/bsk_rust_module.h>`
(vendored into ``bsk-sdk`` and kept in sync by ``tools/sync_headers.py``),
plus the module-level doc comment at the top of ``rust/bsk_build/src/lib.rs``
in the ``bsk-sdk`` repository. This page summarizes both for quick
reference; when they disagree, the source headers win.
