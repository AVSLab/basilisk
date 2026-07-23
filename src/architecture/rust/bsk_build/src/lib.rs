//  ISC License
//
//  Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
//
//  Permission to use, copy, modify, and/or distribute this software for any
//  purpose with or without fee is hereby granted, provided that the above
//  copyright notice and this permission notice appear in all copies.

//! Build-script helpers for Basilisk Rust modules.
//!
//! A conventionally laid-out Cargo crate's ``build.rs`` can contain:
//!
//! ```rust,ignore
//! fn main() { bsk_build::generate(); }
//! ```
//!
//! An in-tree BSK module keeps its crate root beside ``Cargo.toml`` and calls
//! ``generate_from("myModule.rs")`` instead. Both functions scan the selected
//! source path for the ``#[bsk_build::module]`` struct with an
//! ``impl BskModule for <Type>`` block and emit two build artifacts. The
//! procedural attribute emits the Rust lifecycle entry points itself.
//!
//! * **``<ModuleName>.h``** in ``$OUT_DIR`` by default — C header for
//!   CMake/SWIG, generated from the Rust struct.  Set ``BSK_HEADER_PATH`` to
//!   place it at a CMake-managed build path (its file stem also becomes the
//!   module's symbol/header name — see below). Do not edit by hand.
//! * **``<ModuleName>_rust_wrap.i``** in ``$OUT_DIR`` by default — the SWIG
//!   interface file. Set ``BSK_INTERFACE_PATH`` to place it at a CMake-managed
//!   build path. CMake's job is then just "run cargo build, then run SWIG on
//!   the file it wrote" — it never has to work out this module's message
//!   ports/config type name itself. Do not edit by hand.
//!
//! See the Basilisk documentation's "Writing a Rust Plugin" page for the
//! full usage guide; this module-level doc covers only what a plugin author
//! needs to know to write the config struct correctly.
//!
//! # Module name
//!
//! The generated symbol suffix (``SelfInit_<name>`` / ``Reset_<name>`` /
//! ``Update_<name>``) and the header's filename/include guard come from
//! ``BSK_HEADER_PATH``'s file stem, which CMake sets to match the
//! ``bsk_add_rust_module`` ``TARGET``. Direct ``cargo build``/``cargo test``
//! (no CMake) falls back to the crate name.
//!
//! # The config struct
//!
//! Like a hand-written Basilisk C module's config struct, it holds a
//! mandatory ``runtime: BskModuleRuntime`` mirror field (see below),
//! parameters, persistent state, ``MsgReader``/``MsgWriter`` message ports,
//! and normally a ``bskLogger`` pointer. Macro-generated lifecycle code owns
//! raw port I/O; the ``update`` function receives and returns plain message
//! values.
//!
//! ``///`` doc comments on struct fields become the Doxygen ``/*!< … */``
//! inline comment in the generated C header.
//!
//! # Type mapping
//!
//! | Rust                  | C                   |
//! |-----------------------|---------------------|
//! | ``f64``               | ``double``          |
//! | ``f32``               | ``float``           |
//! | ``i64`` / ``u64``     | ``int64_t`` / ``uint64_t`` |
//! | ``i32`` / ``u32``     | ``int32_t`` / ``uint32_t`` |
//! | ``bool``              | ``bool``            |
//! | ``*mut T``            | ``T *``             |
//! | ``*const T``          | ``const T *``       |
//! | ``bsk_messages::Foo`` | ``Foo`` (last segment) |
//! | ``MsgReader<Foo>`` / ``MsgWriter<Foo>`` | ``Foo_C`` |
//! | ``BskModuleRuntime``  | ``BskRustModuleRuntime`` (see below) |
//! | ``[T; N]`` (``N`` a literal) | ``T name[N]`` (see "Fixed-size arrays") |
//!
//! # Runtime mirror field (required)
//!
//! The config struct **must** have a field named exactly ``runtime`` of type
//! [`BskModuleRuntime`]. It mirrors the ``SysModel`` fields
//! (``moduleID``, ``ModelTag``, ``CallCounts``, ``RNGSeed``); the lifecycle code
//! refreshes it before every lifecycle call, so the rest of the module reads
//! it like any other config field (``self.runtime``). ``build.rs`` panics if
//! the field is missing.
//!
//! # Messaging
//!
//! Annotate each message-port field so its role is explicit:
//!
//! * ``#[bsk(input)]`` on ``MsgReader<Foo>`` creates a required ``Foo`` field
//!   in the generated named input struct.
//! * ``#[bsk(input, optional)]`` creates an ``Option<Foo>`` field that is
//!   ``None`` when the port is unlinked.
//! * ``#[bsk(output)]`` on ``MsgWriter<Foo>`` creates a ``Foo`` field in the
//!   generated named output struct.
//!
//! Given ``MyModuleConfig``, the attribute names those structs
//! ``MyModuleInputs`` and ``MyModuleOutputs``. Set ``BskModule::Inputs`` and
//! ``BskModule::Outputs`` to those types. Inputs are accessed and outputs are
//! constructed by field name, so config field declaration order has no
//! behavioral meaning. Required inputs are checked with ``is_linked()`` in
//! ``Reset`` and before every ``Update`` read, raising the standard Basilisk
//! error on failure.
//!
//! For BSK built-in messages this works out-of-the-box. Custom message
//! types need their own ``*_C`` C-interface header on the module's include
//! path; the field is then treated identically to any built-in message port.
//!
//! # Owned heap state
//!
//! A field of type ``Option<Box<T>>`` (any `T`) holds heap state that
//! persists across calls (filters, integrators, ...) — normal, safe Rust
//! ownership, no manual `Box::into_raw`/`from_raw`. It maps to a nullable
//! `void *` in the generated C header. Cleanup is automatic: the header also
//! gets a C++ destructor that runs the struct's regular Rust drop glue, so
//! the state is freed whenever the owning C++ wrapper object is (Python
//! garbage collection, explicit `del`, or process exit) — no
//! `Cleanup_*`-style function or custom SWIG destructor to write by hand.
//! It's also the *only* supported form of raw pointer field: a bare
//! `*mut`/`*const c_void` field is a build error.
//!
//! # Nested structs
//!
//! A field whose type is another `#[repr(C)]` struct defined in the configured
//! source path (by value, not a pointer) is supported: `bsk-build` finds
//! it the same way it finds the config struct, generates its own C
//! `typedef struct` ahead of the config struct's, and lets Python read and
//! write it field-by-field through SWIG like any other struct member —
//! SWIG's generated getter returns a live reference into the parent, and
//! its setter is type-checked against the exact struct type, so this is no
//! less safe than a plain `f64` field. Nesting may be arbitrarily deep,
//! but not self-referential (a struct can't contain itself by value in C).
//! `MsgReader`/`MsgWriter` and `Option<Box<T>>` fields are only meaningful
//! on the top-level config struct and are rejected on nested structs.
//!
//! # Fixed-size arrays
//!
//! A field of type ``[T; N]`` — ``T`` a primitive or a nested ``#[repr(C)]``
//! struct, ``N`` a literal integer — is supported and maps to a plain C array
//! field (``T name[N];``). Multi-dimensional arrays (``[[T; N]; M]``, etc.)
//! are also supported and map to ``T name[M][N];`` in C order (outermost
//! dimension first). This matches the pattern hand-written Basilisk C modules
//! and message payloads already use (e.g. ``double torqueRequestBody[3]``,
//! ``double dcm[3][3]``). SWIG wraps a 1-D array as a Python list of scalars;
//! a 2-D array as a list of lists; and so on. Array lengths must be plain
//! integer literals — ``const`` expressions, generics, and ``N * 2`` are
//! build errors because `bsk-build` runs before the crate's own consts can
//! be evaluated.
//!
//! Every other field type is a hard build error: there is no "pass the
//! type name through unchanged" fallback, so a typo or unsupported type is
//! caught at build time instead of producing a broken header.
//!
//! # Identifying the module config
//!
//! Add ``#[bsk_build::module]`` to the top-level configuration struct. The
//! attribute explicitly distinguishes the module config from other
//! ``#[repr(C)]`` structs in the crate and asks rustc to validate the basic
//! cross-language ABI requirements and emits the lifecycle entry points.
//! ``build.rs`` renders only the C header and SWIG interface for a marked
//! module.
//!
//! # Add `bsk-build` as a plain dependency too
//!
//! Besides the generator itself ([`generate()`] or [`generate_from()`], called
//! from `build.rs` and gated behind the opt-in `codegen` feature), this crate
//! also has an always-available module-code surface:
//! ``#[module]``, [`BskModule`], [`BskModuleRuntime`],
//! [`MsgReader`]/[`MsgWriter`], and [`BskLoggerExt`]. Add a second,
//! feature-less ``bsk-build`` entry for this surface, alongside the existing
//! ``[build-dependencies]`` one (which needs `codegen`):
//!
//! ```toml
//! [dependencies]
//! bsk-build = { path = "..." }
//!
//! [build-dependencies]
//! bsk-build = { path = "...", features = ["codegen"] }
//! ```
//!
//! ``bsk-messages`` re-exports the runtime traits and types, so
//! ``use bsk_messages::*;`` brings those in with the message types. Refer to
//! the attribute through the direct dependency as ``bsk_build::module``.

/// Include the ``build.rs``-generated lifecycle shim for a legacy module.
///
/// New modules use ``#[bsk_build::module]``, which emits lifecycle code
/// directly and does not call this macro. This compatibility macro remains
/// available while unmarked out-of-tree modules migrate.
#[macro_export]
macro_rules! bsk_module {
    () => {
        include!(concat!(env!("OUT_DIR"), "/bsk_shim.rs"));
    };
}

/// Mark and validate a Basilisk module's top-level configuration struct.
///
/// The attribute validates that the type is a public, named ``#[repr(C)]``
/// struct with public fields and a ``runtime: BskModuleRuntime`` member. It
/// generates named input/output value structs and the C ABI lifecycle
/// functions. Message ports must use ``#[bsk(input)]``,
/// ``#[bsk(input, optional)]``, or ``#[bsk(output)]``. ``bsk-build`` also uses
/// the marker to select this struct when generating the C header and wrapper
/// artifacts.
pub use bsk_macros::module;

/// Rust-side mirror of the C ``BskRustModuleRuntime`` struct declared in
/// ``bsk_rust_module.h``. Refreshed from the config struct's own ``runtime``
/// field before every ``SelfInit``/``Reset``/``Update`` call, so a module
/// reads framework state (``moduleID``, ``ModelTag``, ``CallCounts``,
/// ``RNGSeed``) the same way it reads any other config field.
///
/// Deliberately not `Copy`/`Clone`: `model_tag()` borrows from `&self`, and
/// its pointee is only valid for the current lifecycle call, so the compiler
/// rejects any attempt to hold onto the string past that call. Duplicating
/// this struct by value (e.g. into a field of your own) would let that
/// pointer be read again later, once it's dangling — see `model_tag()`.
#[repr(C)]
#[derive(Debug)]
pub struct BskModuleRuntime {
    module_id: i64,
    model_tag: *const core::ffi::c_char,
    call_counts: u64,
    rng_seed: u32,
}

impl BskModuleRuntime {
    /// Framework-assigned message-writer identifier (``SysModel::moduleID``).
    pub const fn module_id(&self) -> i64 { self.module_id }
    /// Step counter (``SysModel::CallCounts``).
    pub const fn call_counts(&self) -> u64 { self.call_counts }
    /// Random seed (``SysModel::RNGSeed``).
    pub const fn rng_seed(&self) -> u32 { self.rng_seed }
    /// Python-assigned module name (``SysModel::ModelTag``).
    ///
    /// Borrowed from `&self`, which in turn is only ever handed to module
    /// code for the duration of the current lifecycle call — the borrow
    /// checker rejects any attempt to retain the returned `&str` past it.
    pub fn model_tag(&self) -> &str {
        if self.model_tag.is_null() {
            return "";
        }
        unsafe { core::ffi::CStr::from_ptr(self.model_tag) }
            .to_str()
            .unwrap_or("")
    }
}

impl BskModuleRuntime {
    /// An all-zero runtime (module ID 0, empty tag, no calls yet) for
    /// constructing a config struct in a `#[cfg(test)]` unit test, without a
    /// live simulation to supply a real one. Not named/spelled `default()`
    /// (no `Default` impl) so `mem::take`/`mem::replace` can't be used to
    /// pull a live runtime out of a config struct by value.
    pub const fn for_testing() -> Self {
        Self { module_id: 0, model_tag: core::ptr::null(), call_counts: 0, rng_seed: 0 }
    }
}

#[cfg(all(test, target_pointer_width = "64"))]
mod runtime_abi_tests {
    use super::BskModuleRuntime;
    use core::mem::{align_of, offset_of, size_of};

    #[test]
    fn runtime_layout_matches_cpp_mirror() {
        assert_eq!(size_of::<BskModuleRuntime>(), 32);
        assert_eq!(align_of::<BskModuleRuntime>(), 8);
        assert_eq!(offset_of!(BskModuleRuntime, module_id), 0);
        assert_eq!(offset_of!(BskModuleRuntime, model_tag), 8);
        assert_eq!(offset_of!(BskModuleRuntime, call_counts), 16);
        assert_eq!(offset_of!(BskModuleRuntime, rng_seed), 24);
    }
}

/// A Basilisk message value, e.g. ``AttGuidMsg`` or ``CmdTorqueBodyMsg``.
///
/// Implemented once per Basilisk message type by ``bsk-messages`` (generated
/// from the vendored headers). Module authors never call these methods
/// directly — read/write messages through [`MsgReader`]/[`MsgWriter`] instead.
pub trait Msg: Sized + Copy {
    /// The C-interface port type this message is read from / written to.
    type Port: Default;
    #[doc(hidden)]
    fn __is_linked(port: &mut Self::Port) -> bool;
    #[doc(hidden)]
    fn __read(port: &mut Self::Port) -> Self;
    #[doc(hidden)]
    fn __init(port: &mut Self::Port);
    #[doc(hidden)]
    fn __write(data: &Self, port: &mut Self::Port, module_id: i64, current_sim_nanos: u64);
}

/// An input message port — reads a [`Msg`] written by another module.
///
/// Use as an annotated config struct field, e.g.
/// ``#[bsk(input)] pub attGuidInMsg: MsgReader<AttGuidMsg>``. Has the same
/// memory layout as the message's C-interface port struct, so the generated C
/// header sees a plain ``AttGuidMsg_C`` field.
#[repr(transparent)]
pub struct MsgReader<T: Msg>(T::Port);

impl<T: Msg> Default for MsgReader<T> {
    fn default() -> Self { Self(T::Port::default()) }
}

impl<T: Msg> MsgReader<T> {
    /// Whether another module has subscribed this port to a source message.
    pub fn is_linked(&mut self) -> bool { T::__is_linked(&mut self.0) }
    /// Read the current message value.
    pub fn read(&mut self) -> T { T::__read(&mut self.0) }
}

/// An output message port — writes a [`Msg`] for other modules to read.
///
/// Use as an annotated config struct field, e.g.
/// ``#[bsk(output)] pub cmdTorqueOutMsg: MsgWriter<CmdTorqueBodyMsg>``.
#[repr(transparent)]
pub struct MsgWriter<T: Msg>(T::Port);

impl<T: Msg> Default for MsgWriter<T> {
    fn default() -> Self { Self(T::Port::default()) }
}

impl<T: Msg> MsgWriter<T> {
    /// Claim ownership of this output message. Called automatically from the
    /// generated ``SelfInit`` — module code does not need to call this.
    pub fn init(&mut self) { T::__init(&mut self.0) }
    /// Write a new message value.
    pub fn write(&mut self, data: &T, module_id: i64, current_sim_nanos: u64) {
        T::__write(data, &mut self.0, module_id, current_sim_nanos)
    }
}

#[doc(hidden)]
pub trait BskModuleInput<Message: Msg>: Sized {
    fn validate(
        port: &mut MsgReader<Message>,
        logger: *mut BSKLogger,
        missing_message: &str,
    );
    fn read(
        port: &mut MsgReader<Message>,
        logger: *mut BSKLogger,
        missing_message: &str,
    ) -> Self;
}

impl<Message: Msg> BskModuleInput<Message> for Message {
    fn validate(
        port: &mut MsgReader<Message>,
        logger: *mut BSKLogger,
        missing_message: &str,
    ) {
        if !port.is_linked() {
            BskLoggerExt::bsk_error(logger, missing_message);
        }
    }

    fn read(
        port: &mut MsgReader<Message>,
        logger: *mut BSKLogger,
        missing_message: &str,
    ) -> Self {
        Self::validate(port, logger, missing_message);
        port.read()
    }
}

impl<Message: Msg> BskModuleInput<Message> for Option<Message> {
    fn validate(
        _port: &mut MsgReader<Message>,
        _logger: *mut BSKLogger,
        _missing_message: &str,
    ) {
    }

    fn read(
        port: &mut MsgReader<Message>,
        _logger: *mut BSKLogger,
        _missing_message: &str,
    ) -> Self {
        if port.is_linked() { Some(port.read()) } else { None }
    }
}

#[cfg(test)]
mod module_input_tests {
    use super::*;

    #[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
    struct TestMessage(u64);

    #[derive(Default)]
    struct TestPort {
        linked: bool,
        value: TestMessage,
    }

    impl Msg for TestMessage {
        type Port = TestPort;

        fn __is_linked(port: &mut Self::Port) -> bool { port.linked }
        fn __read(port: &mut Self::Port) -> Self { port.value }
        fn __init(_port: &mut Self::Port) {}
        fn __write(
            _data: &Self,
            _port: &mut Self::Port,
            _module_id: i64,
            _current_sim_nanos: u64,
        ) {
        }
    }

    #[test]
    fn required_input_reads_linked_message() {
        let mut reader = MsgReader(TestPort { linked: true, value: TestMessage(42) });
        let value = <TestMessage as BskModuleInput<TestMessage>>::read(
            &mut reader,
            core::ptr::null_mut(),
            "missing required input",
        );
        assert_eq!(value, TestMessage(42));
    }

    #[test]
    fn optional_input_returns_none_when_unlinked() {
        let mut reader = MsgReader(TestPort::default());
        <Option<TestMessage> as BskModuleInput<TestMessage>>::validate(
            &mut reader,
            core::ptr::null_mut(),
            "optional input",
        );
        let value = <Option<TestMessage> as BskModuleInput<TestMessage>>::read(
            &mut reader,
            core::ptr::null_mut(),
            "optional input",
        );
        assert_eq!(value, None);
    }

    #[test]
    #[should_panic(expected = "missing required input")]
    fn required_input_rejects_unlinked_port() {
        let mut reader = MsgReader(TestPort::default());
        <TestMessage as BskModuleInput<TestMessage>>::validate(
            &mut reader,
            core::ptr::null_mut(),
            "missing required input",
        );
    }
}

/// Strongly typed Rust lifecycle interface exposed through the Basilisk C ABI.
///
/// A module's config struct implements this trait directly — no Rust code
/// depends on the C++ ``SysModel`` class. Runtime data (module ID,
/// ``ModelTag``, etc.) is available through the config's own
/// ``runtime: BskModuleRuntime`` field, read like any other config field.
/// ``current_sim_nanos`` is passed directly, since ``SysModel`` doesn't store it.
///
/// Three ``BskModule`` trait methods map to the Basilisk module lifecycle:
///
/// ```text
/// init()                           — before Python configures the module.
///                                    Override to set non-zero defaults.
///
/// reset(current_sim_nanos)         — at sim start and on every Reset().
///   └─ returns Self::Outputs         Framework writes them to output ports.
///
/// update(inputs, current_sim_nanos) — every tick.
///   └─ returns Self::Outputs         Framework reads inputs, writes outputs.
/// ```
pub trait BskModule {
    /// Named input-value struct generated by ``#[bsk_build::module]``.
    type Inputs;
    /// Named output-value struct generated by ``#[bsk_build::module]``.
    type Outputs;

    /// Called before Python has configured any fields. Override to set
    /// non-zero parameter defaults and initial state. The default
    /// implementation is a no-op (all fields remain zero-initialized).
    fn init(&mut self) {}

    /// Called during `Reset()`. Must return initialized values for every
    /// output message port; the generated lifecycle code writes them so
    /// they are valid before the first `UpdateState` tick.
    ///
    /// The default implementation returns `Self::Outputs::default()`, which
    /// works for modules whose output payload types all implement `Default`
    /// (all built-in Basilisk message payloads do). Override this method
    /// whenever reset needs to write non-zero initial outputs, validate
    /// parameters, or reset internal state.
    fn reset(&mut self, _current_sim_nanos: u64) -> Self::Outputs
    where
        Self::Outputs: Default,
    {
        Self::Outputs::default()
    }
    fn update(&mut self, inputs: Self::Inputs, current_sim_nanos: u64) -> Self::Outputs;
}

// ---------------------------------------------------------------------------
// Logging — the Rust-side equivalent of a hand-written C module calling
// `_bskLog(configData->bskLogger, BSK_WARNING, info)` / `_bskError(...)`.
//
// `BSKLogger` here is an opaque marker type, not a bindgen mirror of the real
// C++ class: Rust only ever holds a `*mut BSKLogger` obtained from C++ (via
// `_BSKLogger()` or a config struct field set from Python) and passes it
// straight back through `extern "C-unwind"` calls, never constructing or
// reading one directly — so its Rust-side layout is irrelevant. `bsk-build`
// having no bindgen step of its own is exactly why this stays hand-declared
// rather than mirrored: `_bskLog`/`_bskError`/`_BSKLogger` are the stable,
// non-variadic `extern "C"` wrappers Basilisk itself declares in
// `bskLogging.h` for exactly this purpose (bindgen would otherwise also
// emit the raw, Itanium-mangled, variadic `BSKLogger::bskLog`/`bskError` C++
// methods, which are fragile to call across the FFI boundary and not meant
// for direct use). They are declared `extern "C-unwind"`, not plain
// `extern "C"`: `_bskError` throws a C++ `BasiliskError` exception, and per
// Rust's stabilized C-unwind ABI
// (https://github.com/rust-lang/rfcs/blob/master/text/2945-c-unwind-abi.md),
// letting an exception cross a plain `extern "C"` boundary is undefined
// behavior — only `"C-unwind"` guarantees it propagates cleanly instead of
// aborting the process.

/// Opaque handle to a Basilisk ``BSKLogger``. Only ever used behind a
/// pointer (``*mut BSKLogger``) — see the module for why.
#[repr(C)]
pub struct BSKLogger {
    _opaque: [u8; 0],
}

#[allow(non_camel_case_types)]
pub type logLevel_t = core::ffi::c_uint;
pub const BSK_DEBUG: logLevel_t = 0;
pub const BSK_INFORMATION: logLevel_t = 1;
pub const BSK_WARNING: logLevel_t = 2;
pub const BSK_ERROR: logLevel_t = 3;
pub const BSK_SILENT: logLevel_t = 4;

// The C symbols are only available when linking against a real Basilisk build.
// Gate their declarations (and the impl that calls them) so that `cargo test`
// in module crates compiles without Basilisk on the link line. Module crates
// opt in to the test replacement by adding
//   bsk-build = { ..., features = ["test_logger"] }
// to their [dev-dependencies]; Cargo feature unification activates
// `test_logger` in the test binary while leaving production builds unaffected.
#[cfg(not(any(test, feature = "test_logger")))]
extern "C-unwind" {
    fn _BSKLogger() -> *mut BSKLogger;
    fn _bskLog(logger: *mut BSKLogger, level: logLevel_t, info: *const core::ffi::c_char);
    fn _bskError(logger: *mut BSKLogger, info: *const core::ffi::c_char) -> !;
}

/// Basilisk's standard logging levels and error-raising, for a config
/// struct's ``bskLogger: *mut BSKLogger`` field — the same
/// ``_bskLog``/``_bskError`` entry points a hand-written Basilisk C module
/// calls directly (``_bskLog(configData->bskLogger, BSK_WARNING, info)``).
///
/// In test builds (enabled via the ``test_logger`` Cargo feature — see
/// ``bsk_build``'s ``Cargo.toml``) the implementation prints to ``stderr``
/// and panics instead of calling into Basilisk's C symbols, so module code
/// can call these methods freely without ``#[cfg(not(test))]`` guards.
pub trait BskLoggerExt {
    /// Log `msg` at `level` (one of the four ``BSK_*`` constants).
    fn bsk_log(self, level: logLevel_t, msg: &str);
    /// [`BSK_DEBUG`] convenience wrapper.
    fn debug(self, msg: &str)
    where
        Self: Sized,
    {
        self.bsk_log(BSK_DEBUG, msg);
    }
    /// [`BSK_INFORMATION`] convenience wrapper.
    fn info(self, msg: &str)
    where
        Self: Sized,
    {
        self.bsk_log(BSK_INFORMATION, msg);
    }
    /// [`BSK_WARNING`] convenience wrapper.
    fn warning(self, msg: &str)
    where
        Self: Sized,
    {
        self.bsk_log(BSK_WARNING, msg);
    }
    /// Logs at [`BSK_ERROR`] and raises the standard fatal ``BasiliskError``
    /// (propagated to Python through the generated ``extern "C-unwind"``
    /// lifecycle entry points) — the Rust equivalent of a C module's
    /// ``_bskError(configData->bskLogger, info)``.
    ///
    /// In test builds this panics instead of raising a C++ exception.
    fn bsk_error(self, msg: &str) -> !;
}

/// Real Basilisk build: forward every call through `_bskLog`/`_bskError`.
#[cfg(not(any(test, feature = "test_logger")))]
impl BskLoggerExt for *mut BSKLogger {
    fn bsk_log(self, level: logLevel_t, msg: &str) {
        unsafe {
            let logger = if self.is_null() { _BSKLogger() } else { self };
            if let Ok(c_msg) = std::ffi::CString::new(msg) {
                _bskLog(logger, level, c_msg.as_ptr());
            }
        }
    }

    fn bsk_error(self, msg: &str) -> ! {
        unsafe {
            let logger = if self.is_null() { _BSKLogger() } else { self };
            if let Ok(c_msg) = std::ffi::CString::new(msg) {
                _bskError(logger, c_msg.as_ptr());
            }
            _bskError(logger, c"BSK Rust module error (message contained a NUL byte)".as_ptr());
        }
    }
}

/// Test / `test_logger` build: print to stderr and panic — no C link required.
#[cfg(any(test, feature = "test_logger"))]
impl BskLoggerExt for *mut BSKLogger {
    fn bsk_log(self, level: logLevel_t, msg: &str) {
        let tag = match level {
            BSK_DEBUG => "DEBUG",
            BSK_INFORMATION => "INFO",
            BSK_WARNING => "WARNING",
            BSK_ERROR => "ERROR",
            _ => "LOG",
        };
        eprintln!("[bsk-test] {tag}: {msg}");
    }

    fn bsk_error(self, msg: &str) -> ! {
        panic!("[bsk-test] bsk_error: {msg}");
    }
}

#[cfg(feature = "codegen")]
pub use codegen::{generate, generate_from};

#[cfg(feature = "codegen")]
mod codegen;
