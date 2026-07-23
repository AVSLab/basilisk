// ISC License
//
// Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
//
// Permission to use, copy, modify, and/or distribute this software for any
// purpose with or without fee is hereby granted, provided that the above
// copyright notice and this permission notice appear in all copies.

//! Compatibility checks for the cbindgen C-layout renderer.
//!
//! These tests record which Rust representations cbindgen renders directly
//! and the narrow Basilisk-specific boundary handled by ``bsk-build``:
//!
//! - cbindgen renders ordinary ``#[repr(C)]`` config fields, nested structs,
//!   arrays, aliases, mapped ``cfg`` values, concrete C message-port structs,
//!   and optional boxed state.
//! - cbindgen does not resolve ``T::Port`` in the current
//!   ``MsgReader<T>``/``MsgWriter<T>`` abstraction. It emits an undefined
//!   ``Port`` type for that representation.
//!
//! Production code recognizes those generated specialization names and maps
//! them to Basilisk's existing ``<Message>_C`` port types. It also supplies
//! includes, lifecycle declarations, Rust-owned-state opacity, and the shared
//! SWIG wrapper invocation.

#![cfg(feature = "codegen")]

use std::path::{Path, PathBuf};

fn fixture_dir() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("tests")
        .join("fixtures")
        .join("cbindgen_compatibility")
}

fn render_fixture() -> String {
    let fixture = fixture_dir();
    let config_path = fixture.join("cbindgen.toml");
    let config = cbindgen::Config::from_file(&config_path)
        .unwrap_or_else(|error| panic!("could not read {}: {error}", config_path.display()));
    let source_path = fixture.join("compatibility.rs");
    let bindings = cbindgen::Builder::new()
        .with_config(config)
        .with_src(&source_path)
        .generate()
        .unwrap_or_else(|error| {
            panic!(
                "cbindgen could not inspect {}: {error}",
                source_path.display()
            )
        });
    let mut header = Vec::new();
    bindings.write(&mut header);
    String::from_utf8(header).expect("cbindgen generated a non-UTF-8 header")
}

#[test]
fn renders_c_representable_basilisk_config_fields() {
    let header = render_fixture();

    assert!(header.contains("typedef double Gain;"));
    assert!(header.contains("double position[3];"));
    assert!(header.contains("struct NestedConfig nested;"));
    assert!(header.contains("#define VECTOR_LENGTH 3"));
    assert!(header.contains("double vector[VECTOR_LENGTH];"));
    assert!(header.contains("double matrix[2][3];"));
    assert!(header.contains("#if defined(BSK_CBINDGEN_WIDE)"));
    assert!(header.contains("#if !defined(BSK_CBINDGEN_WIDE)"));
    assert!(header.contains("struct ExampleMsg_C input_port;"));
    assert!(header.contains("struct ExampleMsg_C output_port;"));
    assert!(header.contains("struct OwnedState *state;"));
    assert!(header.contains("Mode mode;"));
}

#[test]
fn exposes_associated_message_port_limitation() {
    let header = render_fixture();

    assert!(header.contains("typedef Port MsgReader_ExampleMsg;"));
    assert!(
        !header.contains("typedef struct ExampleMsg_C MsgReader_ExampleMsg;"),
        "cbindgen unexpectedly resolved the associated message-port type; \
         reassess whether the procedural macro still needs a C-facing projection"
    );
}
