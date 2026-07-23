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
    /// [-] Basilisk runtime information
    pub runtime: BskModuleRuntime,
    /// [-] Sample module state
    pub dummy: f64,
    /// [-] Optional input message
    #[bsk(input, optional)]
    pub dataInMsg: MsgReader<CModuleTemplateMsg>,
    /// [-] Output message
    #[bsk(output)]
    pub dataOutMsg: MsgWriter<CModuleTemplateMsg>,
    /// [-] Basilisk logging handle
    pub bskLogger: *mut BSKLogger,
}

impl BskModule for RustModuleTemplateConfig {
    type Inputs = RustModuleTemplateInputs;
    type Outputs = RustModuleTemplateOutputs;

    fn reset(&mut self, _current_sim_nanos: u64) -> Self::Outputs {
        self.dummy = 0.0;
        self.bskLogger.info("Variable dummy set to 0 in reset.");
        RustModuleTemplateOutputs {
            dataOutMsg: CModuleTemplateMsg::default(),
        }
    }

    fn update(&mut self, inputs: Self::Inputs, _current_sim_nanos: u64) -> Self::Outputs {
        let mut data_out_msg = inputs.dataInMsg.unwrap_or_default();

        self.dummy += 1.0;
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

    /// Verify that the generated C++ wrapper retains the established config ABI.
    #[test]
    fn config_abi_layout_matches_compatibility_baseline() {
        assert_eq!(size_of::<RustModuleTemplateConfig>(), 192);
        assert_eq!(align_of::<RustModuleTemplateConfig>(), 8);
        assert_eq!(offset_of!(RustModuleTemplateConfig, runtime), 0);
        assert_eq!(offset_of!(RustModuleTemplateConfig, dummy), 32);
        assert_eq!(offset_of!(RustModuleTemplateConfig, dataInMsg), 40);
        assert_eq!(offset_of!(RustModuleTemplateConfig, dataOutMsg), 112);
        assert_eq!(offset_of!(RustModuleTemplateConfig, bskLogger), 184);
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
}
