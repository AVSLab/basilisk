//  ISC License
//
//  Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
//
//  Permission to use, copy, modify, and/or distribute this software for any
//  purpose with or without fee is hereby granted, provided that the above
//  copyright notice and this permission notice appear in all copies.

//! `build.rs` codegen: scans a BSK Rust module's config struct and emits its
//! C header, FFI shim, and SWIG interface. See the crate root's doc comment
//! for the user-facing guide; this module (and its submodules) is the
//! implementation.
//!
//! * `types` — the shared data model (`ConfigInfo`, `FieldInfo`,
//!   `MethodInfo`, ...) produced by discovery and consumed by the renderers.
//! * `discovery` — finds the config struct and resolves its fields' Rust
//!   types to a C ABI representation.
//! * `methods` — finds `impl <ConfigType> { pub fn ... }` setter/getter
//!   methods with a C/SWIG-compatible signature.
//! * `optionality` — derives `*InMsg` field optionality from `impl
//!   BskModule`'s `Inputs` tuple.
//! * `render_header` / `render_shim` / `render_swig` — the three build
//!   artifacts' renderers.

use std::path::PathBuf;

mod discovery;
mod methods;
mod optionality;
mod render_header;
mod render_shim;
mod render_swig;
#[cfg(test)]
mod test_support;
mod types;

use discovery::{find_bsk_module_impl, find_struct_by_name, FindStructError, SourceAsts};
use methods::find_config_methods;
use optionality::apply_input_optionality;
use render_header::render_c_header;
use render_shim::render_shim;
use render_swig::render_swig_interface;
use types::{panic_with_diagnostics, ConfigInfo, FieldInfo, NestedCtx};

/// Run from a BSK Rust module's ``build.rs``.
///
/// Reads ``CARGO_MANIFEST_DIR`` and ``OUT_DIR`` from the environment (both
/// are set by Cargo when running build scripts), finds the module's config
/// struct via ``syn`` AST parsing, and writes the C header and shim. The
/// header is written under ``OUT_DIR`` unless ``BSK_HEADER_PATH`` is set.
pub fn generate() {
    let manifest_dir = PathBuf::from(
        std::env::var("CARGO_MANIFEST_DIR")
            .expect("bsk-build: CARGO_MANIFEST_DIR not set (is this running in build.rs?)"),
    );
    let out_dir = PathBuf::from(std::env::var("OUT_DIR").expect("bsk-build: OUT_DIR not set"));

    // Tell Cargo to rerun if source files or this shared build library change.
    println!("cargo:rerun-if-changed=src");
    println!("cargo:rerun-if-changed=build.rs");
    println!("cargo:rerun-if-env-changed=BSK_INCLUDE_DIR");
    println!("cargo:rerun-if-env-changed=BSK_HEADER_PATH");
    println!("cargo:rerun-if-env-changed=BSK_INTERFACE_PATH");

    let src_dir = manifest_dir.join("src");
    let source_asts = SourceAsts::load(&src_dir);

    // The `impl BskModule for <Type>` block is the *only* thing that marks a
    // struct as the module's config — no naming convention required.
    let (struct_name, input_types) = find_bsk_module_impl(&source_asts).unwrap_or_else(|| {
        if !source_asts.diagnostics.is_empty() {
            panic_with_diagnostics(source_asts.diagnostics.clone());
        }
        panic!(
            "bsk-build: no `impl BskModule for <Type>` found under {}.\n\
             Every BSK Rust module's config struct must implement `BskModule`:\n\
             \n\
             \x20   impl BskModule for MyModuleConfig {{\n\
             \x20       type Inputs = (...);\n\
             \x20       type Outputs = (...);\n\
             \x20       fn update(&mut self, inputs: Self::Inputs, t: u64) -> Self::Outputs {{ … }}\n\
             \x20   }}\n",
            src_dir.display()
        )
    });
    let item_struct = match find_struct_by_name(&source_asts, &struct_name) {
        Ok(Some(item)) => item,
        Ok(None) => panic!(
            "bsk-build: found `impl BskModule for {struct_name}`, but no \
             `#[repr(C)] pub struct {struct_name} {{ … }}` definition under {}.\n\
             The type implementing `BskModule` must be a `#[repr(C)]` struct \
             defined in this crate's `src/`.",
            src_dir.display()
        ),
        Err(FindStructError::MissingReprC) => panic!(
            "bsk-build: found `struct {struct_name}`, which implements \
             `BskModule`, but it is missing `#[repr(C)]`.\n\
             Add `#[repr(C)]` immediately above `pub struct {struct_name}`. \
             BSK passes this config struct across the Rust/C++ ABI, so its \
             layout must be explicitly C-compatible."
        ),
    };

    // Collects `#[repr(C)]` structs discovered transitively through
    // by-value struct fields, in dependency order (see `NestedCtx`).
    let mut ctx = NestedCtx {
        source_asts: &source_asts,
        diagnostics: source_asts.diagnostics.clone(),
        nested: Vec::new(),
        in_progress: vec![struct_name.clone()],
    };
    let mut info = ConfigInfo {
        struct_name,
        fields: discovery::extract_fields(&item_struct, &mut ctx, true),
        nested_structs: Vec::new(),
        methods: Vec::new(),
    };
    info.nested_structs = std::mem::take(&mut ctx.nested);

    // Plain `impl <ConfigType> { pub fn ... }` methods (setters/getters with
    // validation or conversion logic, mirroring a hand-written C++ module's
    // `setFoo`/`getFoo` pair) — see `methods::find_config_methods` for which
    // signatures are eligible and how incompatible ones are handled.
    let nested_names: Vec<String> = info.nested_structs.iter().map(|s| s.name.clone()).collect();
    let (methods, method_warnings) =
        find_config_methods(&source_asts, &info.struct_name, &nested_names);
    for warning in &method_warnings {
        println!("cargo:warning={warning}");
    }
    info.methods = methods;

    if !info.fields.iter().any(FieldInfo::is_runtime) {
        ctx.error(
            info.struct_name.clone(),
            format!(
                "has no `runtime: bsk_messages::BskModuleRuntime` field.\n\
             Add it as the first field, e.g.:\n\
             \n\
             \x20   #[repr(C)]\n\
             \x20   pub struct {} {{\n\
             \x20       pub runtime: bsk_messages::BskModuleRuntime,\n\
             \x20       // … other fields …\n\
             \x20   }}\n",
            info.struct_name
            ),
        );
    }
    apply_input_optionality(&mut info, &input_types, &mut ctx.diagnostics);
    if !ctx.diagnostics.is_empty() {
        panic_with_diagnostics(ctx.diagnostics);
    }

    // The module name drives the generated symbol suffix
    // (SelfInit_<name>/Reset_<name>/Update_<name>) and header filename/include
    // guard; it must match the CMake TARGET the generated SWIG `.i` file
    // expects. CMake supplies BSK_HEADER_PATH so this is derived from the
    // same build-tree path it manages as a Cargo byproduct. Direct
    // `cargo build`/`cargo test` (no CMake) falls back to the crate name —
    // that output isn't consumed by anything in that mode.
    let module_name = std::env::var_os("BSK_HEADER_PATH")
        .map(PathBuf::from)
        .and_then(|p| p.file_stem().map(|s| s.to_string_lossy().into_owned()))
        .unwrap_or_else(|| {
            std::env::var("CARGO_PKG_NAME")
                .map(|s| s.replace('-', "_"))
                .unwrap_or_else(|_| info.struct_name.clone())
        });

    // CMake supplies BSK_HEADER_PATH so it can model this as a build
    // byproduct. For direct Cargo builds, keep generated files in OUT_DIR.
    let header_path = std::env::var_os("BSK_HEADER_PATH")
        .map(PathBuf::from)
        .unwrap_or_else(|| out_dir.join(format!("{module_name}.h")));
    if let Some(parent) = header_path.parent() {
        if let Err(e) = std::fs::create_dir_all(parent) {
            panic!(
                "bsk-build: could not create header directory {} for \
                 `BSK_HEADER_PATH`/`OUT_DIR`: {e}\n\
                 Check that the path is valid and that this process has \
                 permission to create it — if `BSK_HEADER_PATH` is set \
                 explicitly (normally by CMake), check its value.",
                parent.display()
            );
        }
    }
    let header = render_c_header(&info, &module_name);
    // A silent failure here would surface much later, and much more
    // confusingly, as SWIG or the C++ compiler failing to find this exact
    // header — fail loudly at the source instead.
    if let Err(e) = std::fs::write(&header_path, &header) {
        panic!(
            "bsk-build: could not write generated header to {}: {e}\n\
             Check that the path is valid and writable — SWIG and the C++ \
             build will fail to find this header otherwise.",
            header_path.display()
        );
    }

    // Write the BSK lifecycle shim to OUT_DIR.
    let shim = render_shim(&info, &module_name);
    let shim_path = out_dir.join("bsk_shim.rs");
    if let Err(e) = std::fs::write(&shim_path, shim) {
        panic!(
            "bsk-build: could not write generated shim to {}: {e}\n\
             `OUT_DIR` is normally managed by Cargo — check that this \
             process has permission to write under it.",
            shim_path.display()
        );
    }

    // CMake supplies BSK_INTERFACE_PATH (same convention as BSK_HEADER_PATH)
    // so it never has to work out message ports/owned-state fields/the
    // config type name itself: the SWIG `.i` file is just another artifact
    // `generate()` produces from `info`, the same one the header/shim already come from.
    // CMake's job becomes "run cargo build, then run swig on the file it wrote".
    let interface_path = std::env::var_os("BSK_INTERFACE_PATH")
        .map(PathBuf::from)
        .unwrap_or_else(|| out_dir.join(format!("{module_name}_rust_wrap.i")));
    if let Some(parent) = interface_path.parent() {
        if let Err(e) = std::fs::create_dir_all(parent) {
            panic!(
                "bsk-build: could not create interface directory {} for \
                 `BSK_INTERFACE_PATH`/`OUT_DIR`: {e}\n\
                 Check that the path is valid and that this process has \
                 permission to create it — if `BSK_INTERFACE_PATH` is set \
                 explicitly (normally by CMake), check its value.",
                parent.display()
            );
        }
    }
    let interface = render_swig_interface(&info, &module_name, &header_path);
    if let Err(e) = std::fs::write(&interface_path, &interface) {
        panic!(
            "bsk-build: could not write generated SWIG interface to {}: {e}\n\
             Check that the path is valid and writable — SWIG will fail to \
             find it otherwise.",
            interface_path.display()
        );
    }
}
