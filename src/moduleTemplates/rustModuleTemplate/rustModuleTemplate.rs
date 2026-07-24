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

//! Basic Rust Basilisk module template.

#![allow(non_snake_case)]

use bsk_messages::*;

/// Rust module configuration and message ports.
#[bsk_build::module]
#[repr(C)]
pub struct RustModuleTemplateConfig {
    /// [-] Python-visible sample counter
    pub dummy: f64,
    /// [-] Positive amount added to the sample counter on each update
    pub increment: f64,
    /// [-] Optional input message
    #[bsk(input, optional)]
    pub dataInMsg: MsgReader<CModuleTemplateMsg>,
    /// [-] Fixed-size array of optional input messages
    #[bsk(input, optional)]
    pub dataInMsgs: [MsgReader<CModuleTemplateMsg>; 2],
    /// [-] Output message
    #[bsk(output)]
    pub dataOutMsg: MsgWriter<CModuleTemplateMsg>,
    /// [-] Fixed-size array of output messages
    #[bsk(output)]
    pub dataOutMsgs: [MsgWriter<CModuleTemplateMsg>; 2],
    /// [-] Test-only fault injection that deliberately panics during update
    pub panicOnUpdate: bool,
}

/// Rust-owned state that is never exposed through C, C++, or Python.
///
/// Unlike the configuration view, this type does not use ``#[repr(C)]`` and
/// may contain ordinary Rust collections, strings, enums, and other safe
/// implementation details.
pub struct RustModuleTemplateState {
    /// [-] History retained across update calls
    update_history: Vec<f64>,
    /// Description of the most recent lifecycle event
    last_event: String,
    /// Internal operating mode represented by a Rust enum
    mode: TemplateMode,
}

#[derive(Debug, PartialEq, Eq)]
enum TemplateMode {
    Idle,
    Running,
}

impl Default for RustModuleTemplateState {
    fn default() -> Self {
        Self {
            update_history: Vec::new(),
            last_event: String::from("created"),
            mode: TemplateMode::Idle,
        }
    }
}

impl BskModule for RustModuleTemplateConfig {
    type State = RustModuleTemplateState;
    type Inputs = RustModuleTemplateInputs;
    type Outputs = RustModuleTemplateOutputs;

    fn init(&mut self, _state: &mut Self::State) -> BskResult<()> {
        self.increment = 1.0; // [-]
        Ok(())
    }

    fn reset(
        &mut self,
        state: &mut Self::State,
        context: &BskContext<'_>,
        current_sim_nanos: u64,
    ) -> BskResult<Self::Outputs> {
        if !self.increment.is_finite() || self.increment <= 0.0 {
            return Err(BskError::new(
                "rustModuleTemplate.increment must be finite and strictly positive",
            ));
        }
        self.dummy = 0.0; // [-]
        state.update_history.clear();
        state.last_event = format!("reset at {current_sim_nanos} ns");
        state.mode = TemplateMode::Running;
        context.logger().info("Variable dummy set to 0 in reset.");
        Ok(RustModuleTemplateOutputs {
            dataOutMsg: CModuleTemplateMsg::default(),
            dataOutMsgs: core::array::from_fn(|_| CModuleTemplateMsg::default()),
        })
    }

    fn update(
        &mut self,
        state: &mut Self::State,
        context: &BskContext<'_>,
        inputs: Self::Inputs,
        current_sim_nanos: u64,
    ) -> BskResult<Self::Outputs> {
        if self.panicOnUpdate {
            panic!("deliberate rustModuleTemplate update panic");
        }

        let mut data_out_msg = inputs.dataInMsg.unwrap_or_default();
        let mut data_out_msgs = inputs
            .dataInMsgs
            .map(|input_message| input_message.unwrap_or_default());

        if state.mode == TemplateMode::Idle {
            context.logger().warning("Update called before reset.");
            state.mode = TemplateMode::Running;
        }
        self.dummy += self.increment;
        state.update_history.push(self.dummy);
        state.last_event = format!(
            "module {} updated at {current_sim_nanos} ns",
            context.module_id()
        );
        data_out_msg.dataVector[0] += self.dummy;
        for output_message in &mut data_out_msgs {
            output_message.dataVector[0] += self.dummy;
        }

        Ok(RustModuleTemplateOutputs {
            dataOutMsg: data_out_msg,
            dataOutMsgs: data_out_msgs,
        })
    }
}

#[cfg(all(test, target_pointer_width = "64"))]
mod tests {
    use super::*;
    use core::mem::{align_of, offset_of, size_of};

    /// Verify that only Python-facing parameters and ports cross the config ABI.
    #[test]
    fn config_abi_contains_only_public_module_fields() {
        assert_eq!(size_of::<RustModuleTemplateConfig>(), 456);
        assert_eq!(align_of::<RustModuleTemplateConfig>(), 8);
        assert_eq!(offset_of!(RustModuleTemplateConfig, dummy), 0);
        assert_eq!(offset_of!(RustModuleTemplateConfig, increment), 8);
        assert_eq!(offset_of!(RustModuleTemplateConfig, dataInMsg), 16);
        assert_eq!(offset_of!(RustModuleTemplateConfig, dataInMsgs), 88);
        assert_eq!(offset_of!(RustModuleTemplateConfig, dataOutMsg), 232);
        assert_eq!(offset_of!(RustModuleTemplateConfig, dataOutMsgs), 304);
        assert_eq!(offset_of!(RustModuleTemplateConfig, panicOnUpdate), 448);
    }

    /// Verify that generated input and output values use the config port names.
    #[test]
    fn generated_io_values_are_named_by_port() {
        let inputs = RustModuleTemplateInputs {
            dataInMsg: None,
            dataInMsgs: [None; 2],
        };
        assert!(inputs.dataInMsg.is_none());
        assert!(inputs.dataInMsgs.iter().all(Option::is_none));

        let outputs = RustModuleTemplateOutputs::default();
        assert_eq!(outputs.dataOutMsg.dataVector, [0.0; 3]);
        assert!(outputs
            .dataOutMsgs
            .iter()
            .all(|message| message.dataVector == [0.0; 3]));
    }

    /// Show how pure Rust tests obtain framework metadata and logging services.
    #[test]
    fn testing_context_provides_runtime_services() {
        let runtime = BskModuleRuntime::for_testing();
        let context = BskContext::for_testing(&runtime);

        assert_eq!(context.module_id(), 0);
        assert_eq!(context.model_tag(), "");
        assert_eq!(context.call_counts(), 0);
        assert_eq!(context.rng_seed(), 0);
        context
            .logger()
            .info("Rust module test context is available.");
    }

    /// Demonstrate that internal module state can use unrestricted Rust types.
    #[test]
    fn internal_state_remains_outside_the_ffi_config() {
        let runtime = BskModuleRuntime::for_testing();
        let context = BskContext::for_testing(&runtime);
        let mut config = RustModuleTemplateConfig {
            dummy: 99.0,    // [-]
            increment: 1.0, // [-]
            dataInMsg: MsgReader::default(),
            dataInMsgs: core::array::from_fn(|_| MsgReader::default()),
            dataOutMsg: MsgWriter::default(),
            dataOutMsgs: core::array::from_fn(|_| MsgWriter::default()),
            panicOnUpdate: false,
        };
        let mut state = RustModuleTemplateState::default();

        config
            .reset(&mut state, &context, 0) // [ns]
            .expect("valid parameters must reset");
        assert_eq!(state.mode, TemplateMode::Running);
        assert!(state.update_history.is_empty());
        assert_eq!(state.last_event, "reset at 0 ns");

        config
            .update(
                &mut state,
                &context,
                RustModuleTemplateInputs {
                    dataInMsg: None,
                    dataInMsgs: [None; 2],
                },
                1, // [ns]
            )
            .expect("update must succeed");
        assert_eq!(state.update_history, vec![1.0]);
        assert_eq!(state.last_event, "module 0 updated at 1 ns");
    }

    /// Demonstrate expected parameter failures without panicking.
    #[test]
    fn invalid_increment_returns_bsk_error() {
        let runtime = BskModuleRuntime::for_testing();
        let context = BskContext::for_testing(&runtime);
        let mut config = RustModuleTemplateConfig {
            dummy: 0.0,     // [-]
            increment: 0.0, // [-]
            dataInMsg: MsgReader::default(),
            dataInMsgs: core::array::from_fn(|_| MsgReader::default()),
            dataOutMsg: MsgWriter::default(),
            dataOutMsgs: core::array::from_fn(|_| MsgWriter::default()),
            panicOnUpdate: false,
        };
        let mut state = RustModuleTemplateState::default();

        let error = match config.reset(&mut state, &context, 0) {
            Ok(_) => panic!("a zero increment must fail reset validation"),
            Err(error) => error,
        };
        assert_eq!(
            error.message(),
            "rustModuleTemplate.increment must be finite and strictly positive"
        );
    }
}
