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
    /// Fixed-size message-port arrays exposed through indexed Python views.
    port_arrays: BTreeMap<String, PortArray>,
    /// Top-level config fields backed by ``Option<Box<T>>``.
    owned_state_fields: BTreeSet<String>,
    /// Python-visible configuration values exposed through generated accessors.
    config_fields: Vec<ConfigField>,
}

#[derive(Debug, PartialEq, Eq)]
struct PortArray {
    c_type: String,
}

#[derive(Debug, PartialEq, Eq)]
struct ConfigField {
    name: String,
    c_type: String,
    dimensions: Vec<String>,
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
    println!("cargo:rerun-if-env-changed=BSK_BINDINGS_TRIGGER_PATH");
    if let Some(trigger_path) = std::env::var_os("BSK_BINDINGS_TRIGGER_PATH") {
        println!(
            "cargo:rerun-if-changed={}",
            PathBuf::from(trigger_path).display()
        );
    }

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
        if let Some((rust_type, field_name)) = port_array_declaration(declaration) {
            if let Some(c_type) = metadata.port_types.get(rust_type) {
                metadata.port_arrays.insert(
                    field_name.to_owned(),
                    PortArray {
                        c_type: c_type.clone(),
                    },
                );
            }
        }
        if !declaration.contains('*') || declaration.contains(LOGGER_TYPE) {
            continue;
        }
        if let Some(field_name) = declaration_field_name(declaration) {
            metadata.owned_state_fields.insert(field_name.to_owned());
        }
    }

    for line in config_body.lines() {
        let declaration = line.trim();
        let Some(field) = config_field_declaration(declaration) else {
            continue;
        };
        if metadata.port_types.contains_key(&field.c_type)
            || metadata.owned_state_fields.contains(&field.name)
        {
            continue;
        }
        metadata.config_fields.push(field);
    }

    metadata
}

fn port_array_declaration(declaration: &str) -> Option<(&str, &str)> {
    let declaration = declaration.strip_suffix(';')?;
    let mut words = declaration.split_whitespace();
    let rust_type = words.next()?;
    let field = words.next()?;
    if words.next().is_some() {
        return None;
    }
    let array_start = field.find('[')?;
    let array_end = field.rfind(']')?;
    if array_end <= array_start + 1 || array_end + 1 != field.len() {
        return None;
    }
    Some((rust_type, &field[..array_start]))
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
    let field = declaration
        .strip_suffix(';')?
        .split_whitespace()
        .next_back()
        .map(|field| field.trim_start_matches('*'))
        .filter(|field| !field.is_empty())?;
    Some(field.split('[').next().unwrap_or(field))
}

fn config_field_declaration(declaration: &str) -> Option<ConfigField> {
    if declaration.contains('*') {
        return None;
    }
    let declaration = declaration.strip_suffix(';')?;
    let mut words = declaration.split_whitespace().collect::<Vec<_>>();
    let field = words.pop()?;
    if words.is_empty() {
        return None;
    }
    let name_end = field.find('[').unwrap_or(field.len());
    let name = &field[..name_end];
    if name.is_empty() {
        return None;
    }
    let mut dimensions = Vec::new();
    let mut remaining = &field[name_end..];
    while !remaining.is_empty() {
        let dimension = remaining.strip_prefix('[')?;
        let end = dimension.find(']')?;
        if end == 0 {
            return None;
        }
        dimensions.push(dimension[..end].to_owned());
        remaining = &dimension[end + 1..];
    }
    Some(ConfigField {
        name: name.to_owned(),
        c_type: words.join(" "),
        dimensions,
    })
}

fn finish_header(
    mut header: String,
    config_type: &str,
    module_name: &str,
    metadata: &BindingMetadata,
) -> String {
    let handle_type = format!("{config_type}Handle");
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

    let lifecycle = format!("\nBSK_RUST_DECL({module_name}, {config_type}, {handle_type})\n\n");
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
    let wrapper_type = format!(
        "RustWrapper<{config_type}, {config_type}Handle, \
         Create_{module_name}, Config_{module_name}, \
         GetConfigField_{module_name}, SetConfigField_{module_name}, \
         ConfigFieldDeprecationDate_{module_name}, \
         ConfigFieldDeprecationMessage_{module_name}, \
         Destroy_{module_name}, Update_{module_name}, \
         SelfInit_{module_name}, Reset_{module_name}>"
    );
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
    let ignored_port_arrays = metadata
        .port_arrays
        .keys()
        .map(|field| format!("%ignore {config_type}::{field};\n"))
        .collect::<String>();
    let ignored_config_fields = metadata
        .config_fields
        .iter()
        .map(|field| format!("%ignore {config_type}::{};\n", field.name))
        .collect::<String>();
    let config_vector_templates = metadata
        .config_fields
        .iter()
        .filter(|field| !field.dimensions.is_empty())
        .map(|field| &field.c_type)
        .collect::<BTreeSet<_>>()
        .into_iter()
        .map(|c_type| {
            format!(
                "%template(__BskRustVector_{}) std::vector<{c_type}>;\n",
                sanitize_identifier(c_type)
            )
        })
        .collect::<String>();
    let config_accessors = metadata
        .config_fields
        .iter()
        .enumerate()
        .map(|(index, field)| render_config_accessor(field, index, module_name, &wrapper_type))
        .collect::<String>();
    let port_array_accessors = metadata
        .port_arrays
        .iter()
        .map(|(field_name, array)| {
            let c_type = &array.c_type;
            let python_property = format!(
                "\x20 %pythoncode %{{\n\
                 \x20   @property\n\
                 \x20   def {field_name}(self):\n\
                 \x20       return [self.__bsk_{field_name}_at(index)\n\
                 \x20               for index in range(self.__bsk_{field_name}_len())]\n\
                 \x20 %}}\n"
            );
            format!(
                "%extend {config_type} {{\n\
                 \x20 {c_type} *__bsk_{field_name}_at(size_t index) {{\n\
                 \x20   const size_t length = sizeof($self->{field_name}) / \
                 sizeof($self->{field_name}[0]);\n\
                 \x20   if (index >= length) {{\n\
                 \x20     throw BasiliskError(\"{module_name}.{field_name} index is out of range\");\n\
                 \x20   }}\n\
                 \x20   return &$self->{field_name}[index];\n\
                 \x20 }}\n\
                 \x20 size_t __bsk_{field_name}_len() {{\n\
                 \x20   return sizeof($self->{field_name}) / sizeof($self->{field_name}[0]);\n\
                 \x20 }}\n\
                 {python_property}\
                 }}\n\
                 \n\
                 %extend {wrapper_type} {{\n\
                 {python_property}\
                 }}\n\
                 \n"
            )
        })
        .collect::<String>();

    format!(
        "%module {module_name}\n\
         %{{\n\
         #include \"{header}\"\n\
         %}}\n\
         \n\
         #define BSK_RUST_DECL(name, configType, handleType)\n\
         %include \"swig_c_wrap.i\"\n\
         %include <std_vector.i>\n\
         \n\
         {message_imports}\
         {immutable_fields}\
         {ignored_port_arrays}\
         {ignored_config_fields}\
         %nodefaultctor {config_type};\n\
         %nodefaultdtor {config_type};\n\
         %include \"{header}\"\n\
         {config_vector_templates}\
         \n\
         %rust_wrap_2({module_name}, {config_type}, {config_type}Handle)\n\
         \n\
         {config_accessors}\
         {port_array_accessors}"
    )
}

fn render_config_accessor(
    field: &ConfigField,
    index: usize,
    module_name: &str,
    wrapper_type: &str,
) -> String {
    let field_name = &field.name;
    let accessor_suffix = uppercase_first(field_name);
    let get_method = format!("get{accessor_suffix}");
    let set_method = format!("set{accessor_suffix}");
    let hidden_getter = format!("__bsk_get_{field_name}");
    let hidden_setter = format!("__bsk_set_{field_name}");
    let hidden_warning = format!("__bsk_warn_{field_name}");
    let hidden_flatten = format!("__bsk_flatten_{field_name}");
    let hidden_reshape = format!("__bsk_reshape_{field_name}");
    let shape_expressions = field
        .dimensions
        .iter()
        .enumerate()
        .map(|(dimension_index, _)| format!("self.__bsk_{field_name}_dim_{dimension_index}()"))
        .collect::<Vec<_>>();
    let (python_getter, python_setter, python_array_helpers) = if field.dimensions.is_empty() {
        (
            format!("self.{hidden_getter}()"),
            format!("self.{hidden_setter}(value)"),
            String::new(),
        )
    } else {
        (
            format!(
                "self.{hidden_reshape}(list(self.{hidden_getter}()), [{}])",
                shape_expressions.join(", ")
            ),
            format!("self.{hidden_setter}(self.{hidden_flatten}(value))"),
            format!(
                "\x20   def {hidden_flatten}(self, value):\n\
                 \x20       if isinstance(value, (str, bytes)) or not hasattr(value, \"__iter__\"):\n\
                 \x20           return [value]\n\
                 \x20       flattened = []\n\
                 \x20       for element in value:\n\
                 \x20           flattened.extend(self.{hidden_flatten}(element))\n\
                 \x20       return flattened\n\
                 \x20   def {hidden_reshape}(self, values, shape):\n\
                 \x20       if len(shape) <= 1:\n\
                 \x20           return values\n\
                 \x20       stride = 1\n\
                 \x20       for dimension in shape[1:]:\n\
                 \x20           stride *= dimension\n\
                 \x20       return [\n\
                 \x20           self.{hidden_reshape}(\n\
                 \x20               values[index * stride:(index + 1) * stride],\n\
                 \x20               shape[1:],\n\
                 \x20           )\n\
                 \x20           for index in range(shape[0])\n\
                 \x20       ]\n"
            ),
        )
    };
    let python_methods = format!(
        "\x20 %pythoncode %{{\n\
         \x20   def {hidden_warning}(self):\n\
         \x20       removal_date = self.__bskConfigFieldDeprecationDate({index})\n\
         \x20       if removal_date:\n\
         \x20           from Basilisk.utilities import deprecated\n\
         \x20           deprecated.deprecationWarn(\n\
         \x20               f\"{{type(self).__module__}}.{{type(self).__name__}}.{field_name}\",\n\
         \x20               removal_date,\n\
         \x20               self.__bskConfigFieldDeprecationMessage({index}),\n\
         \x20           )\n\
         {python_array_helpers}\
         \x20   def {get_method}(self):\n\
         \x20       self.{hidden_warning}()\n\
         \x20       return {python_getter}\n\
         \x20   def {set_method}(self, value):\n\
         \x20       self.{hidden_warning}()\n\
         \x20       {python_setter}\n\
         \x20   {field_name} = property({get_method}, {set_method})\n\
         \x20 %}}\n"
    );

    if field.dimensions.is_empty() {
        let c_type = &field.c_type;
        return format!(
            "%extend {wrapper_type} {{\n\
             \x20 {c_type} {hidden_getter}() {{\n\
             \x20   {c_type} value{{}};\n\
             \x20   $self->__bskGetConfigField({index}, &value, sizeof(value));\n\
             \x20   return value;\n\
             \x20 }}\n\
             \x20 void {hidden_setter}({c_type} value) {{\n\
             \x20   $self->__bskSetConfigField({index}, &value, sizeof(value));\n\
             \x20 }}\n\
             {python_methods}\
             }}\n\
             \n"
        );
    }

    let c_type = &field.c_type;
    let length_expression = field.dimensions.join(" * ");
    let dimension_accessors = field
        .dimensions
        .iter()
        .enumerate()
        .map(|(dimension_index, dimension)| {
            format!(
                "\x20 size_t __bsk_{field_name}_dim_{dimension_index}() {{\n\
                 \x20   return static_cast<size_t>({dimension});\n\
                 \x20 }}\n"
            )
        })
        .collect::<String>();
    if c_type == "bool" {
        return format!(
            "%extend {wrapper_type} {{\n\
             \x20 std::vector<bool> {hidden_getter}() {{\n\
             \x20   const size_t length = static_cast<size_t>({length_expression});\n\
             \x20   auto rawValue = std::make_unique<bool[]>(length);\n\
             \x20   $self->__bskGetConfigField(\n\
             \x20       {index}, rawValue.get(), length * sizeof(bool));\n\
             \x20   std::vector<bool> value;\n\
             \x20   value.reserve(length);\n\
             \x20   for (size_t index = 0; index < length; ++index) {{\n\
             \x20     value.push_back(rawValue[index]);\n\
             \x20   }}\n\
             \x20   return value;\n\
             \x20 }}\n\
             \x20 void {hidden_setter}(const std::vector<bool> &value) {{\n\
             \x20   const size_t length = static_cast<size_t>({length_expression});\n\
             \x20   if (value.size() != length) {{\n\
             \x20     throw BasiliskError(\n\
             \x20         \"{module_name}.{field_name} has the wrong number of values\");\n\
             \x20   }}\n\
             \x20   auto rawValue = std::make_unique<bool[]>(length);\n\
             \x20   for (size_t index = 0; index < length; ++index) {{\n\
             \x20     rawValue[index] = value[index];\n\
             \x20   }}\n\
             \x20   $self->__bskSetConfigField(\n\
             \x20       {index}, rawValue.get(), length * sizeof(bool));\n\
             \x20 }}\n\
             {dimension_accessors}\
             {python_methods}\
             }}\n\
             \n"
        );
    }
    format!(
        "%extend {wrapper_type} {{\n\
         \x20 std::vector<{c_type}> {hidden_getter}() {{\n\
         \x20   const size_t length = static_cast<size_t>({length_expression});\n\
         \x20   std::vector<{c_type}> value(length);\n\
         \x20   $self->__bskGetConfigField(\n\
         \x20       {index}, value.data(), value.size() * sizeof({c_type}));\n\
         \x20   return value;\n\
         \x20 }}\n\
         \x20 void {hidden_setter}(const std::vector<{c_type}> &value) {{\n\
         \x20   const size_t length = static_cast<size_t>({length_expression});\n\
         \x20   if (value.size() != length) {{\n\
         \x20     throw BasiliskError(\n\
         \x20         \"{module_name}.{field_name} has the wrong number of values\");\n\
         \x20   }}\n\
         \x20   $self->__bskSetConfigField(\n\
         \x20       {index}, value.data(), value.size() * sizeof({c_type}));\n\
         \x20 }}\n\
         {dimension_accessors}\
         {python_methods}\
         }}\n\
         \n"
    )
}

fn uppercase_first(name: &str) -> String {
    let mut characters = name.chars();
    let Some(first) = characters.next() else {
        return String::new();
    };
    first.to_uppercase().collect::<String>() + characters.as_str()
}

fn sanitize_identifier(value: &str) -> String {
    value
        .chars()
        .map(|character| {
            if character.is_ascii_alphanumeric() {
                character
            } else {
                '_'
            }
        })
        .collect()
}

fn write_generated_file(path: &Path, contents: &str, description: &str) {
    if std::fs::read_to_string(path).is_ok_and(|existing| existing == contents) {
        return;
    }

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
  double gain;
  double vector[3];
  double matrix[2][3];
  bool flags[2];
  MsgReader_InputMsg inputInMsg;
  MsgReader_InputMsg inputInMsgs[2];
  MsgWriter_OutputMsg outputOutMsg;
  MsgWriter_OutputMsg outputOutMsgs[2];
  struct OwnedState *state;
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
            metadata.port_arrays,
            BTreeMap::from([
                (
                    "inputInMsgs".to_owned(),
                    PortArray {
                        c_type: "InputMsg_C".to_owned(),
                    },
                ),
                (
                    "outputOutMsgs".to_owned(),
                    PortArray {
                        c_type: "OutputMsg_C".to_owned(),
                    },
                ),
            ])
        );
        assert_eq!(
            metadata.owned_state_fields,
            BTreeSet::from(["state".to_owned()])
        );
        assert_eq!(
            metadata.config_fields,
            vec![
                ConfigField {
                    name: "gain".to_owned(),
                    c_type: "double".to_owned(),
                    dimensions: Vec::new(),
                },
                ConfigField {
                    name: "vector".to_owned(),
                    c_type: "double".to_owned(),
                    dimensions: vec!["3".to_owned()],
                },
                ConfigField {
                    name: "matrix".to_owned(),
                    c_type: "double".to_owned(),
                    dimensions: vec!["2".to_owned(), "3".to_owned()],
                },
                ConfigField {
                    name: "flags".to_owned(),
                    c_type: "bool".to_owned(),
                    dimensions: vec!["2".to_owned()],
                },
            ]
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
        assert!(
            header.find("BSK_RUST_DECL(example, ExampleConfig, ExampleConfigHandle)")
                < header.rfind("#endif")
        );
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
        assert!(interface.contains("%ignore ExampleConfig::inputInMsgs;"));
        assert!(interface.contains("%ignore ExampleConfig::outputOutMsgs;"));
        assert!(interface.contains("%ignore ExampleConfig::gain;"));
        assert!(interface.contains("%ignore ExampleConfig::vector;"));
        assert!(interface.contains("%ignore ExampleConfig::matrix;"));
        assert!(interface.contains("%ignore ExampleConfig::flags;"));
        assert!(interface.contains("%extend RustWrapper<ExampleConfig"));
        assert!(interface.contains("%extend ExampleConfig"));
        assert!(interface.contains("InputMsg_C *__bsk_inputInMsgs_at"));
        assert!(interface.contains("OutputMsg_C *__bsk_outputOutMsgs_at"));
        assert!(interface.contains("def inputInMsgs(self):"));
        assert!(interface.contains("def outputOutMsgs(self):"));
        assert!(interface.contains("def getGain(self):"));
        assert!(interface.contains("def setGain(self, value):"));
        assert!(interface.contains("gain = property(getGain, setGain)"));
        assert!(interface.contains("std::vector<double> __bsk_get_vector"));
        assert!(interface.contains("vector = property(getVector, setVector)"));
        assert!(interface.contains("std::vector<double> __bsk_get_matrix"));
        assert!(interface.contains("size_t __bsk_matrix_dim_0()"));
        assert!(interface.contains("size_t __bsk_matrix_dim_1()"));
        assert!(interface.contains("self.__bsk_reshape_matrix("));
        assert!(interface.contains("matrix = property(getMatrix, setMatrix)"));
        assert!(interface.contains("std::vector<bool> __bsk_get_flags"));
        assert!(interface.contains("std::make_unique<bool[]>(length)"));
        assert!(interface.contains("flags = property(getFlags, setFlags)"));
        assert!(interface.contains("ConfigFieldDeprecationDate_example"));
        assert!(interface.contains("%rust_wrap_2(example, ExampleConfig, ExampleConfigHandle)"));
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
