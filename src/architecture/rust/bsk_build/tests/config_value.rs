// ISC License
//
// Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
//
// Permission to use, copy, modify, and/or distribute this software for any
// purpose with or without fee is hereby granted, provided that the above
// copyright notice and this permission notice appear in all copies.

//! Compile-time and runtime checks for Python-visible Rust configuration values.

use bsk_build::BskConfigValue;

#[repr(C)]
#[derive(Clone, Copy, Default, BskConfigValue)]
pub struct ControllerGains {
    /// [N*m] Proportional gain.
    pub proportional: f64,
    /// [N*m*s] Derivative gain.
    pub derivative: f64,
}

fn require_config_value<T: BskConfigValue>() {}

#[test]
fn accepts_supported_scalars_arrays_and_nested_structs() {
    require_config_value::<bool>();
    require_config_value::<i32>();
    require_config_value::<u64>();
    require_config_value::<f64>();
    require_config_value::<[f64; 3]>();
    require_config_value::<[[f64; 3]; 3]>();
    require_config_value::<ControllerGains>();
    require_config_value::<[ControllerGains; 2]>();
}

/// Array initialization is independent of Rust's whole-array Default limit.
#[test]
fn initializes_arrays_on_both_sides_of_the_default_limit() {
    fn check_length<const LENGTH: usize>() {
        require_config_value::<[f64; LENGTH]>();
        assert_eq!(<[f64; LENGTH]>::default_value(), [0.0; LENGTH]);
        assert_eq!(<[i32; LENGTH]>::default_value(), [0; LENGTH]);
        assert_eq!(<[bool; LENGTH]>::default_value(), [false; LENGTH]);
    }
    check_length::<32>();
    check_length::<33>();
    check_length::<64>();
}

/// Resolve aliases through the trait and initialize every matrix dimension.
#[test]
fn initializes_large_aliased_and_multidimensional_arrays() {
    type Coefficient = f64;
    type Coefficients = [Coefficient; 64];
    type CoefficientAlias = Coefficients;
    assert_eq!(CoefficientAlias::default_value(), [0.0; 64]);
    assert_eq!(<[[f64; 33]; 2]>::default_value(), [[0.0; 33]; 2]);
    assert_eq!(<[[f64; 2]; 33]>::default_value(), [[0.0; 2]; 33]);
    assert_eq!(<[[f64; 33]; 33]>::default_value(), [[0.0; 33]; 33]);
}

#[repr(C)]
#[derive(Clone, Copy, BskConfigValue)]
pub struct CalibratedSample {
    /// [-] Sample with a deliberately nonzero user-defined default.
    pub value: f64,
}

impl Default for CalibratedSample {
    fn default() -> Self {
        Self { value: 7.25 } // [-]
    }
}

#[repr(C)]
#[derive(Clone, Copy, BskConfigValue)]
pub struct LargeParameters {
    /// [-] Nested array initialized through the elements' custom defaults.
    pub samples: [CalibratedSample; 64],
}

impl Default for LargeParameters {
    fn default() -> Self {
        Self {
            samples: core::array::from_fn(|_| CalibratedSample::default()),
        }
    }
}

/// Larger arrays and nested structs must preserve user-defined defaults.
#[test]
fn preserves_custom_defaults_in_large_nested_arrays() {
    let samples = <[CalibratedSample; 64]>::default_value();
    let nested = LargeParameters::default_value();
    for sample in samples.iter().chain(nested.samples.iter()) {
        assert_eq!(sample.value, 7.25); // [-]
    }
}
