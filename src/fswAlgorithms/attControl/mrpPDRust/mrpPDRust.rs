// ISC License
//
// Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
//
// Permission to use, copy, modify, and/or distribute this software for any
// purpose with or without fee is hereby granted, provided that the above
// copyright notice and this permission notice appear in all copies.
//
// THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
// WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
// MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
// ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
// WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
// ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
// OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

//! Native Rust implementation of the Basilisk MRP proportional-derivative
//! attitude controller.

#![allow(non_snake_case)]

use bsk_messages::*;
use nalgebra::{Matrix3, Vector3};

/// Python-visible controller configuration and message ports.
#[bsk_build::module]
#[repr(C)]
pub struct MrpPDRustConfig {
    /// [N*m] Proportional gain applied to the MRP attitude error
    #[bsk(validate = "validate_proportional_gain")]
    pub K: f64,
    /// [N*m*s] Derivative gain applied to the angular-rate tracking error
    #[bsk(validate = "validate_derivative_gain")]
    pub P: f64,
    /// [N*m] Known external torque about point B, expressed in body components
    #[bsk(validate = "validate_known_torque")]
    pub knownTorquePntB_B: [f64; 3],
    /// [N*m] Commanded external control torque
    #[bsk(output)]
    pub cmdTorqueOutMsg: MsgWriter<CmdTorqueBodyMsg>,
    /// [-] Attitude and angular-rate tracking errors
    #[bsk(input)]
    pub guidInMsg: MsgReader<AttGuidMsg>,
    /// [-] Spacecraft mass properties
    #[bsk(input)]
    pub vehConfigInMsg: MsgReader<VehicleConfigMsg>,
}

/// Rust-owned state retained between reset and update calls.
#[derive(Default)]
pub struct MrpPDRustState {
    /// [kg*m^2] Spacecraft inertia about point B
    inertia: Matrix3<f64>,
}

/// Validate a proposed proportional gain before storing it.
///
/// :param config: Current controller configuration.
/// :param gain: Proposed proportional gain [N*m].
/// :returns: Success when the gain is finite and nonnegative.
fn validate_proportional_gain(_config: &MrpPDRustConfig, gain: &f64) -> BskResult<()> {
    validate_nonnegative_gain("mrpPDRust.K", *gain)
}

/// Validate a proposed derivative gain before storing it.
///
/// :param config: Current controller configuration.
/// :param gain: Proposed derivative gain [N*m*s].
/// :returns: Success when the gain is finite and nonnegative.
fn validate_derivative_gain(_config: &MrpPDRustConfig, gain: &f64) -> BskResult<()> {
    validate_nonnegative_gain("mrpPDRust.P", *gain)
}

fn validate_nonnegative_gain(name: &str, gain: f64) -> BskResult<()> {
    if !gain.is_finite() || gain < 0.0 {
        return Err(BskError::new(format!(
            "{name} must be finite and nonnegative"
        )));
    }
    Ok(())
}

/// Validate a proposed known external torque before storing it.
///
/// :param config: Current controller configuration.
/// :param torque: Proposed known external body torque [N*m].
/// :returns: Success when every component is finite.
fn validate_known_torque(_config: &MrpPDRustConfig, torque: &[f64; 3]) -> BskResult<()> {
    if torque.iter().any(|component| !component.is_finite()) {
        return Err(BskError::new(
            "mrpPDRust.knownTorquePntB_B components must be finite",
        ));
    }
    Ok(())
}

/// Validate spacecraft inertia received through the input message.
///
/// :param inertia: Proposed spacecraft inertia about point B [kg*m^2].
/// :returns: Success when the matrix is finite, symmetric, and positive definite.
fn validate_inertia(inertia: &Matrix3<f64>) -> BskResult<()> {
    if inertia.iter().any(|component| !component.is_finite()) {
        return Err(BskError::new(
            "mrpPDRust vehicle inertia components must be finite",
        ));
    }
    let scale = inertia.norm().max(1.0); // [kg*m^2]
    let symmetry_tolerance = 100.0 * f64::EPSILON * scale; // [kg*m^2]
    let asymmetry = (inertia - inertia.transpose()).norm(); // [kg*m^2]
    if asymmetry > symmetry_tolerance {
        return Err(BskError::new("mrpPDRust vehicle inertia must be symmetric"));
    }
    if inertia.cholesky().is_none() {
        return Err(BskError::new(
            "mrpPDRust vehicle inertia must be positive definite",
        ));
    }
    Ok(())
}

impl BskModule for MrpPDRustConfig {
    type State = MrpPDRustState;
    type Inputs = MrpPDRustInputs;
    type Outputs = MrpPDRustOutputs;

    fn reset(
        &mut self,
        state: &mut Self::State,
        _context: &BskContext<'_>,
        _current_sim_nanos: u64,
    ) -> BskResult<Self::Outputs> {
        // Required ports are validated by the generated lifecycle before this
        // method runs. Values arriving through messages do not pass through a
        // configuration setter, so validate the inertia here before caching it.
        let vehicle_config = self.vehConfigInMsg.read();
        let inertia = Matrix3::from_row_slice(&vehicle_config.ISCPntB_B);
        validate_inertia(&inertia)?;
        state.inertia = inertia;

        Ok(MrpPDRustOutputs::default())
    }

    fn update(
        &mut self,
        state: &mut Self::State,
        _context: &BskContext<'_>,
        inputs: Self::Inputs,
        _current_sim_nanos: u64,
    ) -> BskResult<Self::Outputs> {
        let requested_torque = calculate_control_torque(
            self.K,
            self.P,
            self.knownTorquePntB_B,
            &state.inertia,
            &inputs.guidInMsg,
        ); // [N*m]

        Ok(MrpPDRustOutputs {
            cmdTorqueOutMsg: CmdTorqueBodyMsg {
                torqueRequestBody: requested_torque,
            },
        })
    }
}

/// Evaluate the MRP proportional-derivative control law.
///
/// :param k: MRP attitude-error gain [N*m].
/// :param p: Angular-rate error gain [N*m*s].
/// :param known_torque: Known external body torque [N*m].
/// :param inertia: Spacecraft inertia about point B [kg*m^2].
/// :param guidance: Current attitude and rate tracking errors.
/// :returns: Requested external control torque in body components [N*m].
fn calculate_control_torque(
    k: f64,
    p: f64,
    known_torque: [f64; 3],
    inertia: &Matrix3<f64>,
    guidance: &AttGuidMsg,
) -> [f64; 3] {
    // Basilisk message payloads retain C-compatible arrays at the language
    // boundary. Convert them to fixed-size nalgebra vectors for the control
    // law, then convert the result back to the output payload array.
    let sigma_br = Vector3::from_column_slice(&guidance.sigma_BR); // [-]
    let omega_br_b = Vector3::from_column_slice(&guidance.omega_BR_B); // [rad/s]
    let omega_rn_b = Vector3::from_column_slice(&guidance.omega_RN_B); // [rad/s]
    let domega_rn_b = Vector3::from_column_slice(&guidance.domega_RN_B); // [rad/s^2]
    let known_torque = Vector3::from_column_slice(&known_torque); // [N*m]

    let omega_bn_b = omega_br_b + omega_rn_b; // [rad/s]
    let mut torque = k * sigma_br + p * omega_br_b; // [N*m]
    let angular_momentum = inertia * omega_bn_b; // [N*m*s]
    torque -= omega_rn_b.cross(&angular_momentum); // [N*m]

    let reference_acceleration = domega_rn_b - omega_bn_b.cross(&omega_rn_b); // [rad/s^2]
    torque -= inertia * reference_acceleration; // [N*m]

    let requested_torque = -(torque + known_torque); // [N*m]
    [
        requested_torque[0],
        requested_torque[1],
        requested_torque[2],
    ]
}
