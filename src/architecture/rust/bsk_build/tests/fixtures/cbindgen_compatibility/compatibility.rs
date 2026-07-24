// ISC License
//
// Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
//
// Permission to use, copy, modify, and/or distribute this software for any
// purpose with or without fee is hereby granted, provided that the above
// copyright notice and this permission notice appear in all copies.

#![allow(dead_code)]

use core::ffi::c_void;

/// Scalar alias used by a module configuration.
pub type Gain = f64;

/// Length of the example vector.
pub const VECTOR_LENGTH: usize = 3;

/// Nested configuration data passed across the Rust/C boundary.
#[repr(C)]
pub struct NestedConfig {
    /// [m] Position vector.
    pub position: [f64; 3],
}

/// Feature-dependent field type.
#[cfg(feature = "wide")]
pub type FeatureScalar = f64;

/// Feature-dependent field type.
#[cfg(not(feature = "wide"))]
pub type FeatureScalar = f32;

/// Concrete C message-port representation.
#[repr(C)]
pub struct ExampleMsg_C {
    /// [-] Opaque Basilisk message storage.
    pub message: *mut c_void,
}

/// Rust-owned state stored behind an optional box.
#[repr(C)]
pub struct OwnedState {
    /// [-] Persistent state value.
    pub value: f64,
}

/// C-compatible mode used to evaluate enum rendering.
#[repr(u8)]
pub enum Mode {
    /// [-] First mode.
    First,
    /// [-] Second mode.
    Second,
}

/// A configuration containing the C-representable cases in this spike.
#[repr(C)]
pub struct CompatibleConfig {
    /// [-] Scalar alias.
    pub gain: Gain,
    /// [-] Nested configuration.
    pub nested: NestedConfig,
    /// [-] Fixed-size vector.
    pub vector: [f64; VECTOR_LENGTH],
    /// [-] Fixed-size matrix.
    pub matrix: [[f64; 3]; 2],
    /// [-] Feature-dependent scalar.
    pub feature_scalar: FeatureScalar,
    /// [-] Concrete input message port.
    pub input_port: ExampleMsg_C,
    /// [-] Concrete output message port.
    pub output_port: ExampleMsg_C,
    /// [-] Rust-owned persistent state.
    pub state: Option<Box<OwnedState>>,
    /// [-] C-compatible enum value.
    pub mode: Mode,
}

/// Make the compatible configuration reachable from cbindgen's public C API.
#[no_mangle]
pub extern "C" fn use_compatible_config(_config: *mut CompatibleConfig) {}

/// Minimal model of the associated-port abstraction used by ``bsk-build``.
pub trait Msg {
    /// Concrete C-interface port associated with a Rust message value.
    type Port;
}

/// Minimal model of ``bsk_build::MsgReader<T>``.
#[repr(transparent)]
pub struct MsgReader<T: Msg>(pub T::Port);

/// Rust message value used to select a concrete C port.
#[repr(C)]
pub struct ExampleMsg {
    /// [-] Message payload value.
    pub value: f64,
}

impl Msg for ExampleMsg {
    type Port = ExampleMsg_C;
}

/// A configuration using the current Basilisk associated-port abstraction.
#[repr(C)]
pub struct AssociatedPortConfig {
    /// [-] Input message port.
    pub input_port: MsgReader<ExampleMsg>,
}

/// Make the associated-port configuration reachable from the public C API.
#[no_mangle]
pub extern "C" fn use_associated_port_config(_config: *mut AssociatedPortConfig) {}
