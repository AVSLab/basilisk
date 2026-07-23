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

use std::path::{Path, PathBuf};

#[cfg(test)]
mod compatibility;
mod discovery;
mod methods;
mod optionality;
mod render_header;
mod render_shim;
mod render_swig;
#[cfg(test)]
mod test_support;
mod types;

use discovery::{
    find_bsk_module_impl, find_bsk_module_impl_for, find_marked_module_configs,
    find_struct_by_name, FindStructError, SourceAsts,
};
use methods::find_config_methods;
use optionality::apply_input_optionality;
use render_header::render_c_header;
use render_shim::render_shim;
use render_swig::render_swig_interface;
use types::{panic_with_diagnostics, ConfigInfo, FieldInfo, NestedCtx};

/// Generate bindings for a conventionally laid-out Cargo crate.
///
/// This is equivalent to calling [`generate_from`] with ``"src"``.
pub fn generate() {
    generate_from("src");
}

/// Generate bindings from a Rust source file or directory.
///
/// ``source_path`` is resolved relative to ``CARGO_MANIFEST_DIR``. Pass the
/// crate-root file for a Basilisk-style module layout, or a directory to scan
/// all of its Rust files recursively. ``CARGO_MANIFEST_DIR`` and ``OUT_DIR``
/// are set by Cargo when running build scripts.
pub fn generate_from(source_path: impl AsRef<Path>) {
    let manifest_dir = PathBuf::from(
        std::env::var("CARGO_MANIFEST_DIR")
            .expect("bsk-build: CARGO_MANIFEST_DIR not set (is this running in build.rs?)"),
    );
    let out_dir = PathBuf::from(std::env::var("OUT_DIR").expect("bsk-build: OUT_DIR not set"));
    let source_path = manifest_dir.join(source_path);

    // Tell Cargo to rerun if source files or this shared build library change.
    println!("cargo:rerun-if-changed={}", source_path.display());
    println!("cargo:rerun-if-changed=build.rs");
    println!("cargo:rerun-if-env-changed=BSK_INCLUDE_DIR");
    println!("cargo:rerun-if-env-changed=BSK_HEADER_PATH");
    println!("cargo:rerun-if-env-changed=BSK_INTERFACE_PATH");

    let source_asts = SourceAsts::load(&source_path);

    // Prefer the explicit module attribute. The unmarked fallback keeps existing
    // out-of-tree modules working during the staged procedural-macro migration.
    let marked_configs = find_marked_module_configs(&source_asts);
    if marked_configs.len() > 1 {
        panic!(
            "bsk-build: found more than one `#[bsk_build::module]` struct under {}: {}.\n\
             A Rust crate can expose only one Basilisk module config.",
            source_path.display(),
            marked_configs.join(", ")
        );
    }
    let marked_config = marked_configs.first().map(String::as_str);
    let module_impl = match marked_config {
        Some(config_name) => find_bsk_module_impl_for(&source_asts, Some(config_name)),
        None => find_bsk_module_impl(&source_asts),
    };
    let (struct_name, input_types) = module_impl.unwrap_or_else(|| {
        if !source_asts.diagnostics.is_empty() {
            panic_with_diagnostics(source_asts.diagnostics.clone());
        }
        let marker_help = marked_config.map_or_else(
            || {
                "Add `#[bsk_build::module]` to the config struct and implement \
                 `BskModule` for it."
                    .to_owned()
            },
            |config_name| format!("The marked `{config_name}` struct must implement `BskModule`."),
        );
        panic!(
            "bsk-build: no matching `impl BskModule for <Type>` found under {}.\n\
             {marker_help}\n\
             \n\
             \x20   impl BskModule for MyModuleConfig {{\n\
             \x20       type Inputs = (...);\n\
             \x20       type Outputs = (...);\n\
             \x20       fn update(&mut self, inputs: Self::Inputs, t: u64) -> Self::Outputs {{ … }}\n\
             \x20   }}\n",
            source_path.display()
        )
    });
    let item_struct = match find_struct_by_name(&source_asts, &struct_name) {
        Ok(Some(item)) => item,
        Ok(None) => panic!(
            "bsk-build: found `impl BskModule for {struct_name}`, but no \
             `#[repr(C)] pub struct {struct_name} {{ … }}` definition under {}.\n\
             The type implementing `BskModule` must be a `#[repr(C)]` struct \
             defined in the configured Rust source path.",
            source_path.display()
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
