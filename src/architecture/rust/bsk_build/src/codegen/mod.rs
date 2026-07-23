// ISC License
//
// Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
//
// Permission to use, copy, modify, and/or distribute this software for any
// purpose with or without fee is hereby granted, provided that the above
// copyright notice and this permission notice appear in all copies.

//! Build-script integration for Rust-backed Basilisk modules.
//!
//! ``cbindgen`` renders Rust's C-compatible types. This module supplies only
//! the Basilisk-specific policy around that output: existing message-port type
//! names, the runtime mirror, Rust-owned state, lifecycle declarations, and
//! the shared SWIG wrapper invocation.

use std::collections::{BTreeMap, BTreeSet};
use std::path::{Path, PathBuf};

const RUNTIME_RUST_TYPE: &str = "BskModuleRuntime";
const RUNTIME_C_TYPE: &str = "BskRustModuleRuntime";
const LOGGER_TYPE: &str = "BSKLogger";
const READER_PREFIX: &str = "MsgReader_";
const WRITER_PREFIX: &str = "MsgWriter_";

#[derive(Debug, Default, PartialEq, Eq)]
struct BindingMetadata {
    /// cbindgen specialization name to existing Basilisk C message-port type.
    port_types: BTreeMap<String, String>,
    /// Top-level config fields backed by ``Option<Box<T>>``.
    owned_state_fields: BTreeSet<String>,
}

/// Generate the C header and SWIG interface for a Rust module.
///
/// ``config_type`` is the exact Rust identifier of the
/// ``#[bsk_build::module]`` configuration struct. The build script is a
/// separate Cargo crate and cannot name the module crate's Rust type directly,
/// so this explicit string is the only module metadata it supplies.
pub fn generate_bindings(config_type: &str) {
    validate_identifier(config_type);

    let manifest_dir = PathBuf::from(
        std::env::var("CARGO_MANIFEST_DIR")
            .expect("bsk-build: CARGO_MANIFEST_DIR not set (is this running in build.rs?)"),
    );
    let out_dir = PathBuf::from(std::env::var("OUT_DIR").expect("bsk-build: OUT_DIR not set"));

    println!("cargo:rerun-if-changed={}", manifest_dir.display());
    println!("cargo:rerun-if-changed=build.rs");
    println!("cargo:rerun-if-env-changed=BSK_HEADER_PATH");
    println!("cargo:rerun-if-env-changed=BSK_INTERFACE_PATH");

    let module_name = module_name(config_type);
    let header_path = std::env::var_os("BSK_HEADER_PATH")
        .map(PathBuf::from)
        .unwrap_or_else(|| out_dir.join(format!("{module_name}.h")));
    let interface_path = std::env::var_os("BSK_INTERFACE_PATH")
        .map(PathBuf::from)
        .unwrap_or_else(|| out_dir.join(format!("{module_name}_rust_wrap.i")));

    let discovery_config = cbindgen_config(config_type);
    let discovered_header = render_bindings(&manifest_dir, discovery_config);
    let metadata = analyze_bindings(&discovered_header, config_type);

    let final_config = final_cbindgen_config(config_type, &module_name, &metadata);
    let generated_header = render_bindings(&manifest_dir, final_config);
    let generated_header = finish_header(generated_header, config_type, &module_name, &metadata);
    write_generated_file(&header_path, &generated_header, "C header");

    let interface = render_swig_interface(config_type, &module_name, &header_path, &metadata);
    write_generated_file(&interface_path, &interface, "SWIG interface");
}

fn validate_identifier(identifier: &str) {
    let mut characters = identifier.chars();
    let valid_first = characters
        .next()
        .is_some_and(|character| character == '_' || character.is_ascii_alphabetic());
    let valid_rest =
        characters.all(|character| character == '_' || character.is_ascii_alphanumeric());
    if !valid_first || !valid_rest {
        panic!("bsk-build: `{identifier}` is not a valid Rust configuration type identifier");
    }
}

fn module_name(config_type: &str) -> String {
    std::env::var_os("BSK_HEADER_PATH")
        .and_then(|path| PathBuf::from(path).file_stem().map(|stem| stem.to_owned()))
        .map(|stem| stem.to_string_lossy().into_owned())
        .or_else(|| std::env::var("CARGO_PKG_NAME").ok())
        .unwrap_or_else(|| config_type.trim_end_matches("Config").to_owned())
        .replace('-', "_")
}

fn cbindgen_config(config_type: &str) -> cbindgen::Config {
    let mut config = cbindgen::Config {
        language: cbindgen::Language::C,
        cpp_compat: true,
        documentation: true,
        documentation_style: cbindgen::DocumentationStyle::Doxy,
        usize_is_size_t: true,
        ..cbindgen::Config::default()
    };
    config.parse.parse_deps = true;
    config.parse.include = Some(vec!["bsk-build".to_owned(), "bsk-messages".to_owned()]);
    config.export.include = vec![config_type.to_owned()];
    config
}

fn final_cbindgen_config(
    config_type: &str,
    module_name: &str,
    metadata: &BindingMetadata,
) -> cbindgen::Config {
    let mut config = cbindgen_config(config_type);

    config.export.exclude = vec![RUNTIME_RUST_TYPE.to_owned(), LOGGER_TYPE.to_owned()];
    config
        .export
        .rename
        .insert(RUNTIME_RUST_TYPE.to_owned(), RUNTIME_C_TYPE.to_owned());
    for (rust_type, c_type) in &metadata.port_types {
        config.export.exclude.push(rust_type.clone());
        config
            .export
            .rename
            .insert(rust_type.clone(), c_type.clone());
    }

    config.includes = vec!["architecture/_GeneralModuleFiles/bsk_rust_module.h".to_owned()];
    config.includes.extend(
        metadata
            .port_types
            .values()
            .collect::<BTreeSet<_>>()
            .into_iter()
            .map(|c_type| format!("cMsgCInterface/{c_type}.h")),
    );
    config.include_guard = Some(header_guard(module_name));
    config.autogen_warning =
        Some("/* Auto-generated by bsk-build from the Rust module configuration. */".to_owned());
    config.export.pre_body.insert(
        config_type.to_owned(),
        format!(
            "#ifdef __cplusplus\n  {config_type}() = delete;\n  ~{config_type}() = delete;\n#endif"
        ),
    );
    config
}

fn render_bindings(manifest_dir: &Path, config: cbindgen::Config) -> String {
    let bindings = cbindgen::Builder::new()
        .with_crate(manifest_dir)
        .with_config(config)
        .generate()
        .unwrap_or_else(|error| {
            panic!(
                "bsk-build: cbindgen could not inspect module crate {}: {error}",
                manifest_dir.display()
            )
        });
    let mut output = Vec::new();
    bindings.write(&mut output);
    String::from_utf8(output).expect("bsk-build: cbindgen produced a non-UTF-8 header")
}

fn analyze_bindings(header: &str, config_type: &str) -> BindingMetadata {
    let config_body = config_body(header, config_type);
    let mut metadata = BindingMetadata::default();
    let enum_types = header
        .lines()
        .filter_map(|line| {
            let mut words = line.split_whitespace();
            match (words.next(), words.next(), words.next()) {
                (Some("enum"), Some(name), None) => Some(name.to_owned()),
                _ => None,
            }
        })
        .collect::<BTreeSet<_>>();

    for line in header.lines() {
        let Some(alias) = line
            .trim()
            .strip_prefix("typedef Port ")
            .and_then(|declaration| declaration.strip_suffix(';'))
        else {
            continue;
        };
        let message_type = alias
            .strip_prefix(READER_PREFIX)
            .or_else(|| alias.strip_prefix(WRITER_PREFIX));
        if let Some(message_type) = message_type {
            metadata
                .port_types
                .insert(alias.to_owned(), format!("{message_type}_C"));
        }
    }

    for line in header.lines() {
        let declaration = line.trim();
        let declaration_words = declaration
            .strip_suffix(';')
            .map(|declaration| declaration.split_whitespace().collect::<Vec<_>>())
            .unwrap_or_default();
        let field_type = match declaration_words.as_slice() {
            [field_type, _field_name] => Some(*field_type),
            ["enum", field_type, _field_name] => Some(*field_type),
            _ => None,
        };
        if field_type.is_some_and(|field_type| enum_types.contains(field_type)) {
            panic!(
                "bsk-build: `{config_type}` exposes enum field `{declaration}`.\n\
                 Store its integer representation in the public config and \
                 convert it to the Rust enum after validating the value."
            );
        }
    }

    for line in config_body.lines() {
        let declaration = line.trim();
        if !declaration.contains('*') || declaration.contains(LOGGER_TYPE) {
            continue;
        }
        if let Some(field_name) = declaration_field_name(declaration) {
            metadata.owned_state_fields.insert(field_name.to_owned());
        }
    }

    metadata
}

fn config_body<'a>(header: &'a str, config_type: &str) -> &'a str {
    let start_marker = format!("typedef struct {config_type} {{");
    let end_marker = format!("}} {config_type};");
    let start = header.find(&start_marker).unwrap_or_else(|| {
        panic!(
            "bsk-build: cbindgen did not emit `{config_type}`.\n\
             Pass the exact `#[bsk_build::module]` config type name to \
             `bsk_build::generate_bindings` in build.rs."
        )
    }) + start_marker.len();
    let relative_end = header[start..].find(&end_marker).unwrap_or_else(|| {
        panic!("bsk-build: cbindgen emitted an incomplete `{config_type}` declaration")
    });
    &header[start..start + relative_end]
}

fn declaration_field_name(declaration: &str) -> Option<&str> {
    declaration
        .strip_suffix(';')?
        .split_whitespace()
        .next_back()
        .map(|field| field.trim_start_matches('*'))
        .filter(|field| !field.is_empty())
}

fn finish_header(
    mut header: String,
    config_type: &str,
    module_name: &str,
    metadata: &BindingMetadata,
) -> String {
    let mut in_config = false;
    let start_marker = format!("typedef struct {config_type} {{");
    let end_marker = format!("}} {config_type};");
    let mut opaque_header = String::with_capacity(header.len());

    for line in header.lines() {
        if line.trim() == start_marker {
            in_config = true;
        }
        if in_config {
            if let Some(field_name) = declaration_field_name(line.trim()) {
                if metadata.owned_state_fields.contains(field_name) {
                    let indentation = &line[..line.len() - line.trim_start().len()];
                    opaque_header.push_str(indentation);
                    opaque_header.push_str("void *");
                    opaque_header.push_str(field_name);
                    opaque_header.push_str(";\n");
                    continue;
                }
            }
        }
        opaque_header.push_str(line);
        opaque_header.push('\n');
        if line.trim() == end_marker {
            in_config = false;
        }
    }
    header = opaque_header;

    let lifecycle = format!("\nBSK_RUST_DECL({module_name}, {config_type})\n\n");
    let guard_end = header
        .rfind("#endif")
        .expect("bsk-build: cbindgen include guard is missing its closing #endif");
    header.insert_str(guard_end, &lifecycle);

    if header.contains("typedef Port ")
        || header.contains("MsgReader<")
        || header.contains("MsgWriter<")
    {
        panic!(
            "bsk-build: cbindgen left an unresolved Rust message-port type in the generated header"
        );
    }
    header
}

fn render_swig_interface(
    config_type: &str,
    module_name: &str,
    header_path: &Path,
    metadata: &BindingMetadata,
) -> String {
    let header = header_path.display();
    let message_imports = metadata
        .port_types
        .values()
        .collect::<BTreeSet<_>>()
        .into_iter()
        .map(|c_type| format!("%import \"cMsgCInterface/{c_type}.h\"\n"))
        .collect::<String>();
    let immutable_fields = metadata
        .owned_state_fields
        .iter()
        .map(|field| format!("%immutable {config_type}::{field};\n"))
        .collect::<String>();

    format!(
        "%module {module_name}\n\
         %{{\n\
         #include \"{header}\"\n\
         %}}\n\
         \n\
         #define BSK_RUST_DECL(name, configType)\n\
         %include \"swig_c_wrap.i\"\n\
         \n\
         {message_imports}\
         {immutable_fields}\
         %nodefaultctor {config_type};\n\
         %nodefaultdtor {config_type};\n\
         %include \"{header}\"\n\
         \n\
         %rust_wrap_2({module_name}, {config_type})\n"
    )
}

fn write_generated_file(path: &Path, contents: &str, description: &str) {
    if let Some(parent) = path.parent() {
        std::fs::create_dir_all(parent).unwrap_or_else(|error| {
            panic!(
                "bsk-build: could not create {} directory {}: {error}",
                description,
                parent.display()
            )
        });
    }
    std::fs::write(path, contents).unwrap_or_else(|error| {
        panic!(
            "bsk-build: could not write generated {} to {}: {error}",
            description,
            path.display()
        )
    });
}

fn header_guard(module_name: &str) -> String {
    let normalized = module_name
        .chars()
        .map(|character| {
            if character.is_ascii_alphanumeric() {
                character.to_ascii_uppercase()
            } else {
                '_'
            }
        })
        .collect::<String>();
    format!("{normalized}_H")
}

#[cfg(test)]
mod tests {
    use super::*;

    const DISCOVERED_HEADER: &str = r#"
typedef struct OwnedState {
  double value;
} OwnedState;

typedef Port MsgReader_InputMsg;
typedef Port MsgWriter_OutputMsg;

typedef struct ExampleConfig {
  BskModuleRuntime runtime;
  MsgReader_InputMsg inputInMsg;
  MsgWriter_OutputMsg outputOutMsg;
  struct OwnedState *state;
  BSKLogger *bskLogger;
} ExampleConfig;
"#;

    #[test]
    fn extracts_only_basilisk_specific_metadata() {
        let metadata = analyze_bindings(DISCOVERED_HEADER, "ExampleConfig");
        assert_eq!(
            metadata.port_types,
            BTreeMap::from([
                ("MsgReader_InputMsg".to_owned(), "InputMsg_C".to_owned()),
                ("MsgWriter_OutputMsg".to_owned(), "OutputMsg_C".to_owned()),
            ])
        );
        assert_eq!(
            metadata.owned_state_fields,
            BTreeSet::from(["state".to_owned()])
        );
    }

    #[test]
    fn makes_owned_state_opaque_and_keeps_lifecycle_inside_guard() {
        let metadata = analyze_bindings(DISCOVERED_HEADER, "ExampleConfig");
        let header = "#ifndef EXAMPLE_H\n#define EXAMPLE_H\n\
                      typedef struct ExampleConfig {\n\
                      \x20 struct OwnedState *state;\n\
                      } ExampleConfig;\n\
                      #endif /* EXAMPLE_H */\n";
        let header = finish_header(header.to_owned(), "ExampleConfig", "example", &metadata);

        assert!(header.contains("void *state;"));
        assert!(!header.contains("struct OwnedState *state;"));
        assert!(header.find("BSK_RUST_DECL(example, ExampleConfig)") < header.rfind("#endif"));
    }

    #[test]
    fn swig_interface_contains_metadata_and_generic_wrapper_only() {
        let metadata = analyze_bindings(DISCOVERED_HEADER, "ExampleConfig");
        let interface = render_swig_interface(
            "ExampleConfig",
            "example",
            Path::new("/tmp/example.h"),
            &metadata,
        );

        assert!(interface.contains("%import \"cMsgCInterface/InputMsg_C.h\""));
        assert!(interface.contains("%import \"cMsgCInterface/OutputMsg_C.h\""));
        assert!(interface.contains("%immutable ExampleConfig::state;"));
        assert!(interface.contains("%rust_wrap_2(example, ExampleConfig)"));
        assert!(!interface.contains("%template(example)"));
    }

    #[test]
    fn rejects_invalid_config_identifier() {
        let result = std::panic::catch_unwind(|| validate_identifier("not::a::type"));
        assert!(result.is_err());
    }

    #[test]
    fn rejects_enum_config_field() {
        let header = "enum Mode\n{\n  First,\n};\n\
                      typedef struct ExampleConfig {\n\
                      \x20 Mode mode;\n\
                      } ExampleConfig;\n";
        let result = std::panic::catch_unwind(|| analyze_bindings(header, "ExampleConfig"));
        assert!(result.is_err());
    }
}
