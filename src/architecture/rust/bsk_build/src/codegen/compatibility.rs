//  ISC License
//
//  Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
//
//  Permission to use, copy, modify, and/or distribute this software for any
//  purpose with or without fee is hereby granted, provided that the above
//  copyright notice and this permission notice appear in all copies.

//! Compatibility contract for replacing the current source scanner and
//! renderers. These tests intentionally exercise the complete sample-module
//! surface so a future procedural macro can be compared against one baseline.

use std::path::Path;

use super::optionality::apply_input_optionality;
use super::render_header::render_c_header;
use super::render_shim::render_shim;
use super::render_swig::render_swig_interface;
use super::test_support::{info_for, types};
use super::types::ConfigInfo;

fn sample_config() -> ConfigInfo {
    let mut info = info_for(
        "#[bsk_build::module] #[repr(C)] \
         pub struct RustModuleTemplateConfig { \
         pub runtime: BskModuleRuntime, \
         pub dummy: f64, \
         pub dataInMsg: MsgReader<CModuleTemplateMsg>, \
         pub dataOutMsg: MsgWriter<CModuleTemplateMsg>, \
         pub bskLogger: *mut BSKLogger, \
         }",
    );
    let mut diagnostics = Vec::new();
    apply_input_optionality(
        &mut info,
        &types(&["Option<CModuleTemplateMsg>"]),
        &mut diagnostics,
    );
    assert!(diagnostics.is_empty(), "diagnostics: {diagnostics:?}");
    info
}

fn assert_appears_in_order(text: &str, fragments: &[&str]) {
    let mut search_start = 0;
    for fragment in fragments {
        let relative_index = text[search_start..]
            .find(fragment)
            .unwrap_or_else(|| panic!("missing `{fragment}` in:\n{text}"));
        search_start += relative_index + fragment.len();
    }
}

#[test]
fn sample_header_preserves_config_surface_and_field_order() {
    let header = render_c_header(&sample_config(), "rustModuleTemplate");

    assert!(header.contains("#include \"cMsgCInterface/CModuleTemplateMsg_C.h\""));
    assert_appears_in_order(
        &header,
        &[
            "typedef struct RustModuleTemplateConfig {",
            "BskRustModuleRuntime runtime;",
            "double dummy;",
            "CModuleTemplateMsg_C dataInMsg;",
            "CModuleTemplateMsg_C dataOutMsg;",
            "BSKLogger *bskLogger;",
        ],
    );
    assert!(header.contains("BSK_RUST_DECL(rustModuleTemplate, RustModuleTemplateConfig)"));
    assert!(header.contains("RustModuleTemplateConfig() = delete;"));
    assert!(header.contains("~RustModuleTemplateConfig() = delete;"));
}

#[test]
fn legacy_shim_preserves_lifecycle_and_optional_input_behavior() {
    let shim = render_shim(&sample_config(), "rustModuleTemplate");

    assert!(shim.contains("fn New_rustModuleTemplate() -> *mut RustModuleTemplateConfig"));
    assert!(shim.contains("fn Delete_rustModuleTemplate("));
    assert!(shim.contains("fn SelfInit_rustModuleTemplate("));
    assert!(shim.contains("fn Reset_rustModuleTemplate("));
    assert!(shim.contains("fn Update_rustModuleTemplate("));
    assert!(shim.contains("::std::boxed::Box::into_raw(cfg)"));
    assert!(shim.contains("::std::boxed::Box::from_raw(cfg)"));
    assert!(shim.contains("(*cfg).dataOutMsg.init();"));
    assert!(shim.contains(
        "let data_in_msg = (*cfg).dataInMsg.is_linked().then(|| (*cfg).dataInMsg.read());"
    ));
    assert!(!shim.contains("dataInMsg is not connected"));
    assert!(shim.contains(
        "<RustModuleTemplateConfig as BskModule>::update(\
         &mut *cfg, (data_in_msg,), current_sim_nanos)"
    ));
    assert_eq!(shim.matches("(*cfg).dataOutMsg.write(").count(), 2);
}

#[test]
fn sample_swig_delegates_python_wrapper_contract_to_shared_template() {
    let interface = render_swig_interface(
        &sample_config(),
        "rustModuleTemplate",
        Path::new("/generated/rustModuleTemplate.h"),
    );

    assert!(interface.contains("%module rustModuleTemplate"));
    assert!(interface.contains("%import \"cMsgCInterface/CModuleTemplateMsg_C.h\""));
    assert!(
        interface.contains("%rust_wrap_2(rustModuleTemplate, RustModuleTemplateConfig)")
    );
    assert!(!interface.contains("%template(rustModuleTemplate)"));
    assert!(!interface.contains("%pythonappend RustWrapper::RustWrapper"));
    assert!(!interface.contains("%extend RustModuleTemplateConfig"));
}
