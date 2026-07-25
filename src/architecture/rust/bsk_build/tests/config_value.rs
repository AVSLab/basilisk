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
