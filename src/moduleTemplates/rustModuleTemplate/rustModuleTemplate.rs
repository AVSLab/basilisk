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
    /// [-] Optional input message
    #[bsk(input, optional)]
    pub dataInMsg: MsgReader<CModuleTemplateMsg>,
    /// [-] Output message
    #[bsk(output)]
    pub dataOutMsg: MsgWriter<CModuleTemplateMsg>,
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

    fn reset(
        &mut self,
        state: &mut Self::State,
        context: &BskContext<'_>,
        current_sim_nanos: u64,
    ) -> Self::Outputs {
        self.dummy = 0.0; // [-]
        state.update_history.clear();
        state.last_event = format!("reset at {current_sim_nanos} ns");
        state.mode = TemplateMode::Running;
        context.logger().info("Variable dummy set to 0 in reset.");
        RustModuleTemplateOutputs {
            dataOutMsg: CModuleTemplateMsg::default(),
        }
    }

    fn update(
        &mut self,
        state: &mut Self::State,
        context: &BskContext<'_>,
        inputs: Self::Inputs,
        current_sim_nanos: u64,
    ) -> Self::Outputs {
        let mut data_out_msg = inputs.dataInMsg.unwrap_or_default();

        if state.mode == TemplateMode::Idle {
            context.logger().warning("Update called before reset.");
            state.mode = TemplateMode::Running;
        }
        self.dummy += 1.0; // [-]
        state.update_history.push(self.dummy);
        state.last_event = format!(
            "module {} updated at {current_sim_nanos} ns",
            context.module_id()
        );
        data_out_msg.dataVector[0] += self.dummy;

        RustModuleTemplateOutputs {
            dataOutMsg: data_out_msg,
        }
    }
}

#[cfg(all(test, target_pointer_width = "64"))]
mod tests {
    use super::*;
    use core::mem::{align_of, offset_of, size_of};

    /// Verify that only Python-facing parameters and ports cross the config ABI.
    #[test]
    fn config_abi_contains_only_public_module_fields() {
        assert_eq!(size_of::<RustModuleTemplateConfig>(), 152);
        assert_eq!(align_of::<RustModuleTemplateConfig>(), 8);
        assert_eq!(offset_of!(RustModuleTemplateConfig, dummy), 0);
        assert_eq!(offset_of!(RustModuleTemplateConfig, dataInMsg), 8);
        assert_eq!(offset_of!(RustModuleTemplateConfig, dataOutMsg), 80);
    }

    /// Verify that generated input and output values use the config port names.
    #[test]
    fn generated_io_values_are_named_by_port() {
        let inputs = RustModuleTemplateInputs { dataInMsg: None };
        assert!(inputs.dataInMsg.is_none());

        let outputs = RustModuleTemplateOutputs::default();
        assert_eq!(outputs.dataOutMsg.dataVector, [0.0; 3]);
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
        context.logger().info("Rust module test context is available.");
    }

    /// Demonstrate that internal module state can use unrestricted Rust types.
    #[test]
    fn internal_state_remains_outside_the_ffi_config() {
        let runtime = BskModuleRuntime::for_testing();
        let context = BskContext::for_testing(&runtime);
        let mut config = RustModuleTemplateConfig {
            dummy: 99.0, // [-]
            dataInMsg: MsgReader::default(),
            dataOutMsg: MsgWriter::default(),
        };
        let mut state = RustModuleTemplateState::default();

        let _ = config.reset(&mut state, &context, 0); // [ns]
        assert_eq!(state.mode, TemplateMode::Running);
        assert!(state.update_history.is_empty());
        assert_eq!(state.last_event, "reset at 0 ns");

        let _ = config.update(
            &mut state,
            &context,
            RustModuleTemplateInputs { dataInMsg: None },
            1, // [ns]
        );
        assert_eq!(state.update_history, vec![1.0]);
        assert_eq!(state.last_event, "module 0 updated at 1 ns");
    }
}
