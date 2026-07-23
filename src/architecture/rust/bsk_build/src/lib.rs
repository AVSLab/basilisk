//  ISC License
//
//  Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
//
//  Permission to use, copy, modify, and/or distribute this software for any
//  purpose with or without fee is hereby granted, provided that the above
//  copyright notice and this permission notice appear in all copies.

//! Build-script helpers for Basilisk Rust modules.
//!
//! A Rust module crate's ``build.rs`` names its marked configuration type:
//!
//! ```rust,ignore
//! fn main() {
//!     bsk_build::generate_bindings("MyModuleConfig");
//! }
//! ```
//!
//! ``cbindgen`` reads the crate's C-compatible types and emits two build
//! artifacts. The ``#[bsk_build::module]`` procedural attribute emits named
//! message I/O values and the Rust lifecycle entry points.
//!
//! * **``<ModuleName>.h``** in ``$OUT_DIR`` by default — C header for
//!   CMake/SWIG, generated from the Rust struct.  Set ``BSK_HEADER_PATH`` to
//!   place it at a CMake-managed build path (its file stem also becomes the
//!   module's symbol/header name — see below). Do not edit by hand.
//! * **``<ModuleName>_rust_wrap.i``** in ``$OUT_DIR`` by default — the SWIG
//!   interface file. Set ``BSK_INTERFACE_PATH`` to place it at a CMake-managed
//!   build path. This small file contains the module-specific message and
//!   field metadata, then invokes the shared ``%rust_wrap_2`` template.
//!   CMake's job is just "run cargo build, then run SWIG on the file it
//!   wrote" — it never has to work out this module's message ports/config
//!   type name itself. Do not edit by hand.
//!
//! See the Basilisk documentation's "Writing a Rust Plugin" page for the
//! full usage guide; this module-level doc covers only what a plugin author
//! needs to know to write the config struct correctly.
//!
//! # Module name
//!
//! The generated symbol suffix (``Create_<name>`` / ``Config_<name>`` /
//! ``Destroy_<name>`` and the lifecycle entry points) and the header's
//! filename/include guard come from ``BSK_HEADER_PATH``'s file stem, which
//! CMake sets to match the ``bsk_add_rust_module`` ``TARGET``. Direct
//! ``cargo build``/``cargo test`` without CMake falls back to the crate name.
//!
//! # The config struct
//!
//! Like a hand-written Basilisk C module's config struct, it holds a
//! module's Python-configurable parameters and ``MsgReader``/``MsgWriter``
//! message ports. Framework metadata and logging arrive through
//! [`BskContext`], while persistent implementation state belongs in
//! [`BskModule::State`]. Neither appears in this FFI view. Macro-generated
//! lifecycle code owns raw port I/O; the ``update`` function receives and
//! returns plain message values.
//!
//! ``///`` doc comments on structs and fields become Doxygen comments in the
//! generated C header.
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
//! | ``bsk_messages::Foo`` | ``Foo`` (last segment) |
//! | ``MsgReader<Foo>`` / ``MsgWriter<Foo>`` | ``Foo_C`` |
//! | ``[T; N]`` (compile-time ``N``) | ``T name[N]`` (see "Fixed-size arrays") |
//!
//! Rust allocates the complete module instance. It initializes every config
//! field and the associated [`BskModule::State`] through ``Default`` before
//! calling [`BskModule::init`]. Built-in supported field types already
//! implement `Default`; module-defined nested structs and state types must
//! derive or implement it.
//!
//! # Framework context
//!
//! Lifecycle methods receive [`BskContext`], which provides the ``SysModel``
//! module ID, model tag, call count, random seed, and Basilisk logger. The
//! context is valid only for the current call and is not part of the
//! Python-visible configuration struct. [`BskLoggerRef`] exposes only
//! nonfatal ``debug``, ``info``, and ``warning`` operations. Expected
//! failures return [`BskError`] rather than using logging as control flow.
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
//! ``Reset`` and before every ``Update`` read. A missing connection becomes
//! an expected [`BskError`] that the C++ wrapper translates into the standard
//! Basilisk error after Rust returns.
//!
//! For BSK built-in messages this works out-of-the-box. Custom message
//! types need their own ``*_C`` C-interface header on the module's include
//! path; the field is then treated identically to any built-in message port.
//!
//! # Rust-owned state
//!
//! Set [`BskModule::State`] to any ``Default`` Rust type that should persist
//! across lifecycle calls. It may contain ``Vec``, ``String``, enums, maps,
//! smart pointers, and other types with no C representation. Generated Rust
//! code stores it beside the config inside an opaque module instance.
//! ``Destroy_<name>`` runs ordinary Rust drop glue for both values, so no
//! ``Cleanup_*`` function or manual pointer conversion is needed. Use ``()``
//! for a stateless module.
//!
//! # Lifecycle results
//!
//! [`BskModule::init`], [`BskModule::reset`], and [`BskModule::update`] return
//! [`BskResult`]. Use ``Err(BskError::new("..."))`` for an expected invalid
//! configuration, unavailable input, or runtime failure. The generated ABI
//! boundary returns that failure as data, and the C++ wrapper raises
//! ``BasiliskError`` only after Rust has returned normally. Output messages
//! are written only after the lifecycle method returns ``Ok``.
//!
//! # Nested structs
//!
//! A field whose type is another `#[repr(C)]` struct (by value, not a
//! pointer) is supported. ``cbindgen`` emits referenced structs in dependency
//! order, and SWIG exposes the nested value field-by-field. Nesting may be
//! arbitrarily deep, but not self-referential because C cannot represent a
//! struct containing itself by value.
//! Keep `MsgReader`/`MsgWriter` fields on the top-level config struct, where
//! the module lifecycle adapter processes them. Keep internal state in
//! [`BskModule::State`].
//!
//! # Fixed-size arrays
//!
//! A field of type ``[T; N]`` — ``T`` a primitive or nested ``#[repr(C)]``
//! struct — maps to a C array field. Multi-dimensional arrays map in C order
//! (outermost dimension first). SWIG wraps a 1-D array as a Python list of
//! scalars, a 2-D array as a list of lists, and so on.
//!
//! # Identifying the module config
//!
//! Add ``#[bsk_build::module]`` to the top-level configuration struct. The
//! attribute explicitly distinguishes the module config from ordinary Rust
//! data and asks rustc to validate its basic ABI requirements. Pass the same
//! type name to [`generate_bindings`] in ``build.rs``.
//!
//! # Add `bsk-build` as a plain dependency too
//!
//! Besides [`generate_bindings`] (called from `build.rs` and gated behind the
//! opt-in `codegen` feature), this crate
//! also has an always-available module-code surface:
//! ``#[module]``, [`BskModule`], [`BskContext`], [`BskModuleRuntime`],
//! [`BskError`]/[`BskResult`], [`MsgReader`]/[`MsgWriter`], and
//! [`BskLoggerRef`]. Add a second, feature-less ``bsk-build`` entry for this
//! surface, alongside the existing ``[build-dependencies]`` one (which needs
//! `codegen`):
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

/// Mark and validate a Basilisk module's top-level configuration struct.
///
/// The attribute validates that the type is a public, named ``#[repr(C)]``
/// struct with public FFI-safe fields. It generates named input/output value
/// structs and the C ABI lifecycle functions. Message ports must use
/// ``#[bsk(input)]``,
/// ``#[bsk(input, optional)]``, or ``#[bsk(output)]``. The module's
/// ``build.rs`` passes this type's exact name to [`generate_bindings`] when
/// generating the C header and wrapper artifacts.
pub use bsk_macros::module;

/// An expected failure reported by a Rust Basilisk module.
///
/// Return this through [`BskResult`] for invalid configuration, unavailable
/// input data, or another condition that should stop the simulation without
/// panicking. The generated lifecycle boundary will translate it into a
/// ``BasiliskError`` after Rust returns normally.
#[must_use]
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct BskError {
    message: String,
}

impl BskError {
    /// Create an error with the message that should be reported to Basilisk.
    pub fn new(message: impl Into<String>) -> Self {
        Self {
            message: message.into(),
        }
    }

    /// Return the error message.
    pub fn message(&self) -> &str {
        &self.message
    }
}

impl core::fmt::Display for BskError {
    fn fmt(&self, formatter: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        formatter.write_str(&self.message)
    }
}

impl std::error::Error for BskError {}

impl From<String> for BskError {
    fn from(message: String) -> Self {
        Self::new(message)
    }
}

impl From<&str> for BskError {
    fn from(message: &str) -> Self {
        Self::new(message)
    }
}

/// Result returned by fallible Rust module lifecycle methods.
pub type BskResult<T> = Result<T, BskError>;

/// Classification attached to an error returned through the Rust module ABI.
///
/// C and C++ see the matching ``BSK_RUST_ERROR_*`` constants declared in
/// ``bsk_rust_module.h``.
#[repr(u32)]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BskRustErrorKind {
    /// An expected module failure returned as [`BskError`].
    Expected = 1,
    /// A Rust panic caught at the generated lifecycle boundary.
    Panic = 2,
    /// An invalid pointer or another violation of the lifecycle ABI contract.
    InvalidArgument = 3,
}

/// Opaque C handle for an error allocated and owned by Rust.
///
/// Foreign code may inspect this only with [`BskRustError_kind`] and
/// [`BskRustError_message`]. It must eventually return every non-null handle
/// to [`Destroy_BskRustError`].
#[repr(C)]
#[derive(Debug)]
pub struct BskRustError {
    _private: [u8; 0],
}

struct BskRustErrorInner {
    kind: BskRustErrorKind,
    message: std::ffi::CString,
}

impl BskRustErrorInner {
    fn new(kind: BskRustErrorKind, message: String) -> Self {
        // A C string cannot represent an embedded NUL. Preserve its location
        // visibly rather than truncating the diagnostic at that byte.
        let message = message.replace('\0', "\\0");
        let message = std::ffi::CString::new(message)
            .expect("replacing embedded NUL bytes must produce a valid C string");
        #[cfg(test)]
        LIVE_BSK_RUST_ERRORS.fetch_add(1, core::sync::atomic::Ordering::SeqCst);
        Self { kind, message }
    }
}

impl Drop for BskRustErrorInner {
    fn drop(&mut self) {
        #[cfg(test)]
        LIVE_BSK_RUST_ERRORS.fetch_sub(1, core::sync::atomic::Ordering::SeqCst);
    }
}

#[cfg(test)]
static LIVE_BSK_RUST_ERRORS: core::sync::atomic::AtomicUsize =
    core::sync::atomic::AtomicUsize::new(0);

impl BskRustError {
    fn into_raw(kind: BskRustErrorKind, message: String) -> *mut Self {
        std::boxed::Box::into_raw(std::boxed::Box::new(BskRustErrorInner::new(kind, message)))
            .cast::<Self>()
    }

    /// Convert an expected module error into an owning FFI handle.
    #[doc(hidden)]
    pub fn __from_error(error: BskError) -> *mut Self {
        Self::into_raw(BskRustErrorKind::Expected, error.message)
    }

    /// Convert a caught panic payload into an owning FFI handle.
    #[doc(hidden)]
    pub fn __from_panic(operation: &str, payload: Box<dyn core::any::Any + Send>) -> *mut Self {
        let message = panic_payload_message(payload);
        Self::into_raw(
            BskRustErrorKind::Panic,
            format!("Rust panic in {operation}: {message}"),
        )
    }

    /// Construct an ABI-contract error.
    #[doc(hidden)]
    pub fn __invalid_argument(message: impl Into<String>) -> *mut Self {
        Self::into_raw(BskRustErrorKind::InvalidArgument, message.into())
    }
}

fn panic_payload_message(payload: Box<dyn core::any::Any + Send>) -> String {
    let payload = match payload.downcast::<String>() {
        Ok(message) => return *message,
        Err(payload) => payload,
    };
    let payload = match payload.downcast::<&'static str>() {
        Ok(message) => return (*message).to_owned(),
        Err(payload) => payload,
    };

    // An arbitrary panic payload may itself panic when dropped. It is safer
    // to leak this exceptional object than to permit a second panic while
    // constructing the FFI error that contains the first one.
    core::mem::forget(payload);
    String::from("non-string panic payload")
}

/// Return the classification of an opaque Rust error.
///
/// A null pointer reports [`BskRustErrorKind::InvalidArgument`].
///
/// # Safety
///
/// A non-null `error` must be a live handle returned by this library and must
/// not have been passed to [`Destroy_BskRustError`].
#[no_mangle]
#[allow(non_snake_case)]
pub unsafe extern "C" fn BskRustError_kind(error: *const BskRustError) -> BskRustErrorKind {
    let Some(error) = (unsafe { error.cast::<BskRustErrorInner>().as_ref() }) else {
        return BskRustErrorKind::InvalidArgument;
    };
    error.kind
}

/// Borrow the NUL-terminated message stored in an opaque Rust error.
///
/// The returned pointer remains valid until `error` is passed to
/// [`Destroy_BskRustError`]. A null error pointer returns a static
/// invalid-argument diagnostic.
///
/// # Safety
///
/// A non-null `error` must be a live handle returned by this library and must
/// not have been passed to [`Destroy_BskRustError`].
#[no_mangle]
#[allow(non_snake_case)]
pub unsafe extern "C" fn BskRustError_message(
    error: *const BskRustError,
) -> *const core::ffi::c_char {
    let Some(error) = (unsafe { error.cast::<BskRustErrorInner>().as_ref() }) else {
        return c"bsk-build: Rust error pointer must not be null".as_ptr();
    };
    error.message.as_ptr()
}

/// Destroy an opaque Rust error and release its message.
///
/// Passing null is a no-op.
///
/// # Safety
///
/// A non-null `error` must be a live handle returned by this library. Each
/// handle may be passed to this function exactly once.
#[no_mangle]
#[allow(non_snake_case)]
pub unsafe extern "C" fn Destroy_BskRustError(error: *mut BskRustError) {
    if !error.is_null() {
        drop(unsafe { std::boxed::Box::from_raw(error.cast::<BskRustErrorInner>()) });
    }
}

/// Run one generated ABI operation without allowing a Rust panic to escape.
///
/// A null return is success. An expected [`BskError`] or caught unwinding
/// panic becomes an owning [`BskRustError`] handle for the C++ wrapper.
#[doc(hidden)]
pub fn __ffi_boundary(
    operation: &str,
    action: impl FnOnce() -> BskResult<()>,
) -> *mut BskRustError {
    match std::panic::catch_unwind(std::panic::AssertUnwindSafe(action)) {
        Ok(Ok(())) => core::ptr::null_mut(),
        Ok(Err(error)) => BskRustError::__from_error(error),
        Err(payload) => BskRustError::__from_panic(operation, payload),
    }
}

#[cfg(test)]
mod error_tests {
    use super::{
        BskError, BskResult, BskRustError, BskRustErrorKind, BskRustError_kind,
        BskRustError_message, Destroy_BskRustError, LIVE_BSK_RUST_ERRORS, __ffi_boundary,
    };
    use core::sync::atomic::Ordering;
    use std::ffi::CStr;
    use std::sync::Mutex;

    static ERROR_HANDLE_TEST_LOCK: Mutex<()> = Mutex::new(());

    fn handle_message(error: *const BskRustError) -> String {
        let message = unsafe { BskRustError_message(error) };
        unsafe { CStr::from_ptr(message) }
            .to_str()
            .expect("error messages must be valid UTF-8")
            .to_owned()
    }

    #[test]
    fn expected_error_supports_result_and_standard_error_apis() {
        fn validate(valid: bool) -> BskResult<()> {
            if valid {
                Ok(())
            } else {
                Err(BskError::new("gain must be positive"))
            }
        }

        let error = validate(false).expect_err("invalid input must return an error");
        assert_eq!(error.message(), "gain must be positive");
        assert_eq!(error.to_string(), "gain must be positive");
        assert_eq!(
            BskError::from(String::from("owned")),
            BskError::new("owned")
        );
        assert_eq!(BskError::from("borrowed"), BskError::new("borrowed"));
        assert!(validate(true).is_ok());
    }

    #[test]
    fn expected_error_handle_owns_and_releases_its_message() {
        let _lock = ERROR_HANDLE_TEST_LOCK.lock().expect("test lock");
        let initial_count = LIVE_BSK_RUST_ERRORS.load(Ordering::SeqCst);
        let error = BskRustError::__from_error(BskError::new("before\0after"));

        assert!(!error.is_null());
        assert_eq!(
            unsafe { BskRustError_kind(error) },
            BskRustErrorKind::Expected
        );
        assert_eq!(handle_message(error), "before\\0after");
        assert_eq!(
            LIVE_BSK_RUST_ERRORS.load(Ordering::SeqCst),
            initial_count + 1
        );

        unsafe { Destroy_BskRustError(error) };
        assert_eq!(LIVE_BSK_RUST_ERRORS.load(Ordering::SeqCst), initial_count);
    }

    #[test]
    fn panic_handles_preserve_string_payloads_and_classification() {
        let _lock = ERROR_HANDLE_TEST_LOCK.lock().expect("test lock");
        let owned =
            BskRustError::__from_panic("sample::update", Box::new(String::from("owned panic")));
        let borrowed = BskRustError::__from_panic("sample::reset", Box::new("borrowed panic"));

        for error in [owned, borrowed] {
            assert_eq!(unsafe { BskRustError_kind(error) }, BskRustErrorKind::Panic);
        }
        assert_eq!(
            handle_message(owned),
            "Rust panic in sample::update: owned panic"
        );
        assert_eq!(
            handle_message(borrowed),
            "Rust panic in sample::reset: borrowed panic"
        );

        unsafe {
            Destroy_BskRustError(owned);
            Destroy_BskRustError(borrowed);
        }
    }

    #[test]
    fn non_string_panic_payload_uses_safe_fallback() {
        let _lock = ERROR_HANDLE_TEST_LOCK.lock().expect("test lock");
        let error = BskRustError::__from_panic("sample::init", Box::new(7_u32));

        assert_eq!(
            handle_message(error),
            "Rust panic in sample::init: non-string panic payload"
        );
        unsafe { Destroy_BskRustError(error) };
    }

    #[test]
    fn invalid_argument_handle_preserves_contract_diagnostic() {
        let _lock = ERROR_HANDLE_TEST_LOCK.lock().expect("test lock");
        let error = BskRustError::__invalid_argument("module handle must not be null");

        assert_eq!(
            unsafe { BskRustError_kind(error) },
            BskRustErrorKind::InvalidArgument
        );
        assert_eq!(handle_message(error), "module handle must not be null");
        unsafe { Destroy_BskRustError(error) };
    }

    #[test]
    fn null_ffi_error_access_is_deterministic() {
        let _lock = ERROR_HANDLE_TEST_LOCK.lock().expect("test lock");
        assert_eq!(
            unsafe { BskRustError_kind(core::ptr::null()) },
            BskRustErrorKind::InvalidArgument
        );
        assert_eq!(
            handle_message(core::ptr::null()),
            "bsk-build: Rust error pointer must not be null"
        );
        unsafe { Destroy_BskRustError(core::ptr::null_mut()) };
    }

    #[test]
    fn ffi_error_kind_has_stable_u32_representation() {
        assert_eq!(core::mem::size_of::<BskRustErrorKind>(), 4);
        assert_eq!(BskRustErrorKind::Expected as u32, 1);
        assert_eq!(BskRustErrorKind::Panic as u32, 2);
        assert_eq!(BskRustErrorKind::InvalidArgument as u32, 3);
    }

    #[test]
    fn ffi_boundary_returns_null_on_success() {
        let _lock = ERROR_HANDLE_TEST_LOCK.lock().expect("test lock");
        let error = __ffi_boundary("sample::success", || Ok(()));

        assert!(error.is_null());
    }

    #[test]
    fn ffi_boundary_converts_expected_error() {
        let _lock = ERROR_HANDLE_TEST_LOCK.lock().expect("test lock");
        let error = __ffi_boundary("sample::reset", || Err(BskError::new("invalid gain")));

        assert_eq!(
            unsafe { BskRustError_kind(error) },
            BskRustErrorKind::Expected
        );
        assert_eq!(handle_message(error), "invalid gain");
        unsafe { Destroy_BskRustError(error) };
    }

    #[test]
    fn ffi_boundary_catches_rust_panic() {
        let _lock = ERROR_HANDLE_TEST_LOCK.lock().expect("test lock");
        let error = __ffi_boundary("sample::update", || panic!("index out of range"));

        assert_eq!(
            unsafe { BskRustError_kind(error) },
            BskRustErrorKind::Panic
        );
        assert_eq!(
            handle_message(error),
            "Rust panic in sample::update: index out of range"
        );
        unsafe { Destroy_BskRustError(error) };
    }
}

/// Rust-side mirror of the C ``BskRustModuleRuntime`` struct declared in
/// ``bsk_rust_module.h``.
///
/// Module logic receives a borrowed view of this snapshot exclusively through
/// [`BskContext`]. Use the context accessors for ``moduleID``, ``ModelTag``,
/// ``CallCounts``, and ``RNGSeed``.
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
    /// constructing a [`BskContext`] in a `#[cfg(test)]` unit test without a
    /// live simulation to supply a real one.
    pub const fn for_testing() -> Self {
        Self {
            module_id: 0,
            model_tag: core::ptr::null(),
            call_counts: 0,
            rng_seed: 0,
        }
    }
}

/// Raw C-compatible storage behind [`BskContext`].
///
/// Module code should use the borrowed [`BskContext`] interface instead of
/// reading this representation. The C++ wrapper owns the runtime strings and
/// logger referenced here; neither pointer may be retained after a lifecycle
/// call returns.
#[doc(hidden)]
#[repr(C)]
#[derive(Debug)]
pub struct BskModuleContext {
    runtime: BskModuleRuntime,
    bsk_logger: *mut BSKLogger,
}

/// Framework metadata and services borrowed for one module lifecycle call.
///
/// This type deliberately contains references rather than owned runtime
/// values. Consequently, safe module code cannot retain the context or its
/// model tag beyond the lifecycle call that supplied it.
#[derive(Debug)]
pub struct BskContext<'a> {
    runtime: &'a BskModuleRuntime,
    logger: BskLoggerRef<'a>,
}

impl<'a> BskContext<'a> {
    /// Framework-assigned message-writer identifier.
    pub const fn module_id(&self) -> i64 { self.runtime.module_id() }

    /// Python-assigned module name.
    pub fn model_tag(&self) -> &str { self.runtime.model_tag() }

    /// Number of times the module has been called by the scheduler.
    pub const fn call_counts(&self) -> u64 { self.runtime.call_counts() }

    /// Random seed inherited from the module's ``SysModel`` wrapper.
    pub const fn rng_seed(&self) -> u32 { self.runtime.rng_seed() }

    /// Borrow the module's Basilisk logger for this lifecycle call.
    pub const fn logger(&self) -> BskLoggerRef<'a> { self.logger }

    /// Construct an empty context for a pure Rust module unit test.
    ///
    /// The returned context borrows `runtime`, which is normally created with
    /// [`BskModuleRuntime::for_testing`]. Logging uses the `test_logger`
    /// implementation when that feature is enabled.
    pub const fn for_testing(runtime: &'a BskModuleRuntime) -> Self {
        Self {
            runtime,
            logger: BskLoggerRef::from_raw(core::ptr::null_mut()),
        }
    }

    /// Borrow a context supplied through the generated lifecycle ABI.
    ///
    /// # Safety
    ///
    /// `context` must be non-null, properly aligned, and point to a valid
    /// [`BskModuleContext`] whose runtime strings and logger remain alive for
    /// the returned value's full lifetime.
    #[doc(hidden)]
    pub unsafe fn __from_raw(context: *const BskModuleContext) -> Self {
        let context = unsafe { context.as_ref() }
            .expect("bsk-build: lifecycle context pointer must not be null");
        Self {
            runtime: &context.runtime,
            logger: BskLoggerRef::from_raw(context.bsk_logger),
        }
    }
}

#[cfg(all(test, target_pointer_width = "64"))]
mod runtime_abi_tests {
    use super::{
        BskContext, BskModuleContext, BskModuleRuntime, BSKLogger,
    };
    use core::mem::{align_of, offset_of, size_of};
    use std::ffi::CString;

    #[test]
    fn runtime_layout_matches_cpp_mirror() {
        assert_eq!(size_of::<BskModuleRuntime>(), 32);
        assert_eq!(align_of::<BskModuleRuntime>(), 8);
        assert_eq!(offset_of!(BskModuleRuntime, module_id), 0);
        assert_eq!(offset_of!(BskModuleRuntime, model_tag), 8);
        assert_eq!(offset_of!(BskModuleRuntime, call_counts), 16);
        assert_eq!(offset_of!(BskModuleRuntime, rng_seed), 24);
    }

    #[test]
    fn context_layout_matches_cpp_mirror() {
        assert_eq!(size_of::<BskModuleContext>(), 40);
        assert_eq!(align_of::<BskModuleContext>(), 8);
        assert_eq!(offset_of!(BskModuleContext, runtime), 0);
        assert_eq!(offset_of!(BskModuleContext, bsk_logger), 32);
    }

    #[test]
    fn borrowed_context_exposes_runtime_and_logger() {
        let model_tag = CString::new("rustContextTest").expect("valid model tag");
        let raw = BskModuleContext {
            runtime: BskModuleRuntime {
                module_id: 42,
                model_tag: model_tag.as_ptr(),
                call_counts: 7,
                rng_seed: 11,
            },
            bsk_logger: core::ptr::null_mut::<BSKLogger>(),
        };
        let context = unsafe { BskContext::__from_raw(&raw) };

        assert_eq!(context.module_id(), 42);
        assert_eq!(context.model_tag(), "rustContextTest");
        assert_eq!(context.call_counts(), 7);
        assert_eq!(context.rng_seed(), 11);
        context.logger().info("borrowed context logger");
    }

    #[test]
    fn testing_context_uses_empty_runtime() {
        let runtime = BskModuleRuntime::for_testing();
        let context = BskContext::for_testing(&runtime);

        assert_eq!(context.module_id(), 0);
        assert_eq!(context.model_tag(), "");
        assert_eq!(context.call_counts(), 0);
        assert_eq!(context.rng_seed(), 0);
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
        missing_message: &str,
    ) -> BskResult<()>;
    fn read(
        port: &mut MsgReader<Message>,
        missing_message: &str,
    ) -> BskResult<Self>;
}

impl<Message: Msg> BskModuleInput<Message> for Message {
    fn validate(
        port: &mut MsgReader<Message>,
        missing_message: &str,
    ) -> BskResult<()> {
        if port.is_linked() {
            Ok(())
        } else {
            Err(BskError::new(missing_message))
        }
    }

    fn read(
        port: &mut MsgReader<Message>,
        missing_message: &str,
    ) -> BskResult<Self> {
        Self::validate(port, missing_message)?;
        Ok(port.read())
    }
}

impl<Message: Msg> BskModuleInput<Message> for Option<Message> {
    fn validate(
        _port: &mut MsgReader<Message>,
        _missing_message: &str,
    ) -> BskResult<()> {
        Ok(())
    }

    fn read(
        port: &mut MsgReader<Message>,
        _missing_message: &str,
    ) -> BskResult<Self> {
        Ok(if port.is_linked() { Some(port.read()) } else { None })
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
            "missing required input",
        )
        .expect("linked input must be readable");
        assert_eq!(value, TestMessage(42));
    }

    #[test]
    fn optional_input_returns_none_when_unlinked() {
        let mut reader = MsgReader(TestPort::default());
        <Option<TestMessage> as BskModuleInput<TestMessage>>::validate(
            &mut reader,
            "optional input",
        )
        .expect("an optional input never requires a connection");
        let value = <Option<TestMessage> as BskModuleInput<TestMessage>>::read(
            &mut reader,
            "optional input",
        )
        .expect("an optional input must remain readable when unlinked");
        assert_eq!(value, None);
    }

    #[test]
    fn required_input_returns_error_when_unlinked() {
        let mut reader = MsgReader(TestPort::default());
        let error = <TestMessage as BskModuleInput<TestMessage>>::validate(
            &mut reader,
            "missing required input",
        )
        .expect_err("an unlinked required input must fail validation");
        assert_eq!(error, BskError::new("missing required input"));
    }
}

/// Strongly typed Rust lifecycle interface exposed through the Basilisk C ABI.
///
/// A module's config struct implements this trait directly — no Rust code
/// depends on the C++ ``SysModel`` class. [`BskContext`] supplies runtime
/// metadata and logging, while [`BskModule::State`] holds implementation
/// details that never cross the FFI boundary. ``current_sim_nanos`` is passed
/// directly, since ``SysModel`` doesn't store it.
///
/// Three ``BskModule`` trait methods map to the Basilisk module lifecycle:
///
/// ```text
/// init(state)                      — before Python configures the module.
///   └─ returns BskResult<()>         Override to set non-zero defaults.
///
/// reset(state, context, time)      — at sim start and on every Reset().
///   └─ returns BskResult<Outputs>    Framework writes them to output ports.
///
/// update(state, context, inputs, time) — every tick.
///   └─ returns BskResult<Outputs>    Framework reads inputs, writes outputs.
/// ```
pub trait BskModule {
    /// Arbitrary Rust-owned state retained between lifecycle calls.
    ///
    /// This state never crosses the FFI boundary and may contain normal Rust
    /// types such as ``Vec``, ``String``, enums, maps, and smart pointers.
    type State: Default;
    /// Named input-value struct generated by ``#[bsk_build::module]``.
    type Inputs;
    /// Named output-value struct generated by ``#[bsk_build::module]``.
    type Outputs;

    /// Called before Python has configured any fields. The configuration and
    /// state have already been initialized through their respective
    /// ``Default`` implementations. Override this method to set non-default
    /// parameters or state values. Returning an error prevents construction
    /// of the module's opaque instance.
    fn init(&mut self, _state: &mut Self::State) -> BskResult<()> {
        Ok(())
    }

    /// Called during `Reset()`. Must return initialized values for every
    /// output message port; the generated lifecycle code writes them so
    /// they are valid before the first `UpdateState` tick.
    ///
    /// The default implementation returns
    /// `Ok(Self::Outputs::default())`, which works for modules whose output
    /// payload types all implement `Default` (all built-in Basilisk message
    /// payloads do). Override this method whenever reset needs to write
    /// non-zero initial outputs, validate parameters, or reset internal
    /// state. No output is written when this method returns an error.
    fn reset(
        &mut self,
        _state: &mut Self::State,
        _context: &BskContext<'_>,
        _current_sim_nanos: u64,
    ) -> BskResult<Self::Outputs>
    where
        Self::Outputs: Default,
    {
        Ok(Self::Outputs::default())
    }

    /// Called every simulation tick. Receives named input message values and
    /// returns named output message values for the framework to write. No
    /// output is written when this method returns an error.
    fn update(
        &mut self,
        state: &mut Self::State,
        context: &BskContext<'_>,
        inputs: Self::Inputs,
        current_sim_nanos: u64,
    ) -> BskResult<Self::Outputs>;
}

// ---------------------------------------------------------------------------
// Logging — the Rust-side equivalent of a hand-written C module calling
// `_bskLog(configData->bskLogger, BSK_WARNING, info)`.
//
// `BSKLogger` here is an opaque marker type, not a bindgen mirror of the real
// C++ class: Rust only ever borrows a `*mut BSKLogger` through `BskContext`
// and passes it through the nonfatal `_bskLogNoThrow` C wrapper. That adapter
// catches every C++ exception before returning to Rust. Rust never constructs
// or reads the logger object, so its Rust-side layout is irrelevant. A null
// logger disables logging for the current context.

/// Opaque handle to a Basilisk ``BSKLogger``. Only ever used behind a
/// pointer (``*mut BSKLogger``) — see the module for why.
#[repr(C)]
pub struct BSKLogger {
    _opaque: [u8; 0],
}

/// Safe borrowed access to a Basilisk logger supplied in [`BskContext`].
///
/// The lifetime prevents module code from retaining the handle beyond the
/// framework context that supplied it.
#[derive(Clone, Copy, Debug)]
pub struct BskLoggerRef<'a> {
    raw: *mut BSKLogger,
    _lifetime: core::marker::PhantomData<&'a BSKLogger>,
}

impl<'a> BskLoggerRef<'a> {
    const fn from_raw(raw: *mut BSKLogger) -> Self {
        Self {
            raw,
            _lifetime: core::marker::PhantomData,
        }
    }

    /// Log a debug message through Basilisk.
    pub fn debug(self, msg: &str) {
        log_nonfatal(self.raw, BSK_DEBUG, msg);
    }

    /// Log an informational message through Basilisk.
    pub fn info(self, msg: &str) {
        log_nonfatal(self.raw, BSK_INFORMATION, msg);
    }

    /// Log a warning through Basilisk.
    pub fn warning(self, msg: &str) {
        log_nonfatal(self.raw, BSK_WARNING, msg);
    }
}

type LogLevel = core::ffi::c_uint;
const BSK_DEBUG: LogLevel = 0;
const BSK_INFORMATION: LogLevel = 1;
const BSK_WARNING: LogLevel = 2;

// The C symbols are only available when linking against a real Basilisk build.
// Gate their declarations (and the impl that calls them) so that `cargo test`
// in module crates compiles without Basilisk on the link line. Module crates
// opt in to the test replacement by adding
//   bsk-build = { ..., features = ["test_logger"] }
// to their [dev-dependencies]; Cargo feature unification activates
// `test_logger` in the test binary while leaving production builds unaffected.
#[cfg(not(any(test, feature = "test_logger")))]
extern "C" {
    fn _bskLogNoThrow(
        logger: *mut BSKLogger,
        level: LogLevel,
        info: *const core::ffi::c_char,
    ) -> core::ffi::c_int;
}

/// Real Basilisk build: forward nonfatal calls through a C++ no-throw adapter.
#[cfg(not(any(test, feature = "test_logger")))]
fn log_nonfatal(logger: *mut BSKLogger, level: LogLevel, msg: &str) {
    if logger.is_null() {
        return;
    }

    let message = msg.replace('\0', "\\0");
    let message = std::ffi::CString::new(message)
        .expect("replacing embedded NUL bytes must produce a valid C string");
    let status = unsafe { _bskLogNoThrow(logger, level, message.as_ptr()) };
    if status != 0 {
        panic!("Basilisk logger raised while handling a nonfatal Rust log message");
    }
}

/// Test / `test_logger` build: print to stderr with no C link required.
#[cfg(any(test, feature = "test_logger"))]
fn log_nonfatal(_logger: *mut BSKLogger, level: LogLevel, msg: &str) {
    let tag = match level {
        BSK_DEBUG => "DEBUG",
        BSK_INFORMATION => "INFO",
        BSK_WARNING => "WARNING",
        _ => unreachable!("Rust exposes only nonfatal Basilisk log levels"),
    };
    eprintln!("[bsk-test] {tag}: {msg}");
}

#[cfg(feature = "codegen")]
pub use codegen::generate_bindings;

#[cfg(feature = "codegen")]
mod codegen;
