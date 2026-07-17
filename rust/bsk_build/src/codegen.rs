//  ISC License
//
//  Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
//
//  Permission to use, copy, modify, and/or distribute this software for any
//  purpose with or without fee is hereby granted, provided that the above
//  copyright notice and this permission notice appear in all copies.

//! `build.rs` codegen: scans a BSK Rust module's config struct and emits its
//! C header, FFI shim, and SWIG interface. See the crate root's doc comment
//! for the user-facing guide; this module is the implementation.

use std::collections::BTreeSet;
use std::path::{Path, PathBuf};

use syn::{
    Attribute, Expr, ExprLit, Fields, File, ImplItem, Item, ItemStruct, Lit, Meta, Type, TypeArray,
    TypePath, TypePtr,
};
use walkdir::WalkDir;

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

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
        fields: extract_fields(&item_struct, &mut ctx, true),
        nested_structs: Vec::new(),
    };
    info.nested_structs = std::mem::take(&mut ctx.nested);
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

// ---------------------------------------------------------------------------
// Config struct descriptor
// ---------------------------------------------------------------------------

struct ConfigInfo {
    /// Exact Rust identifier of the `#[repr(C)]` struct implementing
    /// `BskModule` — whatever the user named it, e.g. "mrpRustControllerConfig".
    struct_name: String,
    fields: Vec<FieldInfo>,
    /// `#[repr(C)]` struct types referenced (directly or transitively) by a
    /// by-value struct field, in dependency order — each entry's own fields
    /// only reference entries earlier in this list (or the config struct's
    /// fields), so emitting them in this order satisfies C's
    /// declare-before-use rule.
    nested_structs: Vec<NestedStructInfo>,
}

/// A `#[repr(C)]` struct discovered through a by-value struct field
/// (see "Nested structs" in the module docs). Same field shape as the
/// config struct, minus the parts that only make sense once, at the top
/// level (`runtime`, message ports, owned heap state).
struct NestedStructInfo {
    name: String,
    fields: Vec<FieldInfo>,
}

/// Threaded through field extraction so nested struct discovery can search
/// the crate source and detect by-value reference cycles (which C cannot
/// represent — a struct can't contain itself by value).
struct NestedCtx<'a> {
    source_asts: &'a SourceAsts,
    diagnostics: Vec<Diagnostic>,
    /// Structs fully resolved so far, in dependency order (see
    /// `ConfigInfo::nested_structs`).
    nested: Vec<NestedStructInfo>,
    /// Struct names currently being resolved (a stack), for cycle detection.
    in_progress: Vec<String>,
}

#[derive(Clone, Debug)]
struct Diagnostic {
    context: String,
    message: String,
}

impl NestedCtx<'_> {
    fn error(&mut self, context: impl Into<String>, message: impl Into<String>) {
        self.diagnostics.push(Diagnostic {
            context: context.into(),
            message: message.into(),
        });
    }
}

fn panic_with_diagnostics(mut diagnostics: Vec<Diagnostic>) -> ! {
    diagnostics.sort_by(|left, right| {
        left.context
            .cmp(&right.context)
            .then_with(|| left.message.cmp(&right.message))
    });
    let rendered = diagnostics
        .iter()
        .enumerate()
        .map(|(index, diagnostic)| {
            format!(
                "  {}. `{}` {}",
                index + 1,
                diagnostic.context,
                diagnostic.message
            )
        })
        .collect::<Vec<_>>()
        .join("\n");
    panic!(
        "bsk-build: found {} error(s) while generating the C ABI:\n{}",
        diagnostics.len(),
        rendered
    );
}

/// Whether a `MsgReader<T>`/`MsgWriter<T>` field is an input or output port.
#[derive(Clone, Copy, PartialEq, Eq)]
enum MsgKind {
    Reader,
    Writer,
}

struct FieldInfo {
    name: String, // "attGuidInMsg"
    /// C type string: "double", "AttGuidMsg_C" (for a message field), "BSKLogger *"
    c_type: String,
    /// Raw Rust path last-segment string, e.g. "f64", "BskModuleRuntime".
    /// Empty for `MsgReader`/`MsgWriter` fields — use `msg_kind`/`msg_type`.
    rust_base_type: String,
    /// `Some((Reader|Writer, "AttGuidMsg"))` for `MsgReader<AttGuidMsg>` /
    /// `MsgWriter<AttGuidMsg>` fields; `None` for everything else.
    msg: Option<(MsgKind, String)>,
    doc: String,
    is_optional: bool,
    /// `Option<Box<T>>` owned heap state (see "Owned heap state" in the
    /// module docs). Its own field, rather than inferring it from `c_type
    /// == "void *"` at call sites, so `render_manifest` (and anything else
    /// that needs it) has one unambiguous source of truth.
    is_owned_state: bool,
    /// Non-empty for fixed-size array fields (`[T; N]`, `[[T; N]; M]`, …).
    /// Holds the dimensions outermost-first in C order — so Rust `[[f64; 3]; 4]`
    /// maps to C `double field[4][3]` and `array_dims = [4, 3]`. `c_type`
    /// is always the *scalar* element type (`"double"`, not `"double[3]"`);
    /// `render_field_decls` emits the dimension suffixes after the name.
    /// Empty for every non-array field kind.
    array_dims: Vec<usize>,
}

impl FieldInfo {
    fn is_in_msg(&self) -> bool {
        matches!(&self.msg, Some((MsgKind::Reader, _)))
    }
    fn is_out_msg(&self) -> bool {
        matches!(&self.msg, Some((MsgKind::Writer, _)))
    }
    /// The mandatory `SysModel` runtime mirror field (see module docs).
    fn is_runtime(&self) -> bool {
        self.name == "runtime" && self.rust_base_type == "BskModuleRuntime"
    }
}

// ---------------------------------------------------------------------------
// syn-based discovery: `impl BskModule for <Type>` and the struct it names
// ---------------------------------------------------------------------------

/// Parsed ASTs for every Rust source file in a module crate.
///
/// Discovery and nested-type resolution share this cache so a build script
/// parses each source file exactly once.
struct SourceAsts {
    files: Vec<File>,
    diagnostics: Vec<Diagnostic>,
}

impl SourceAsts {
    fn load(src_dir: &Path) -> Self {
        let mut files = Vec::new();
        let mut diagnostics = Vec::new();
        for entry in WalkDir::new(src_dir)
            .follow_links(false)
            .into_iter()
            .filter_map(|entry| entry.ok())
            .filter(|entry| entry.path().extension().is_some_and(|ext| ext == "rs"))
        {
            let path = entry.path();
            let content = match std::fs::read_to_string(path) {
                Ok(content) => content,
                Err(error) => {
                    diagnostics.push(Diagnostic {
                        context: path.display().to_string(),
                        message: format!("could not read Rust source: {error}"),
                    });
                    continue;
                }
            };
            match syn::parse_file(&content) {
                Ok(file) => files.push(file),
                Err(error) => diagnostics.push(Diagnostic {
                    context: path.display().to_string(),
                    message: format!("could not parse Rust source: {error}"),
                }),
            }
        }
        Self { files, diagnostics }
    }

    /// Returns the first top-level item with the exact Rust identifier.
    fn find_named_item(&self, name: &str) -> Option<&Item> {
        self.files
            .iter()
            .flat_map(|file| file.items.iter())
            .find(|item| match item {
                Item::Struct(item) => item.ident == name,
                Item::Enum(item) => item.ident == name,
                _ => false,
            })
    }
}

/// Searches cached source ASTs for the first
/// `impl BskModule for <Type> { type Inputs = (...); ... }`
/// and returns `(struct_name, input_types)`. `input_types` is the `Inputs`
/// tuple's element types in order (empty for `type Inputs = ();`).
fn find_bsk_module_impl(source_asts: &SourceAsts) -> Option<(String, Vec<Type>)> {
    for ast in &source_asts.files {
        for item in &ast.items {
            let imp = match item {
                Item::Impl(imp) => imp,
                _ => continue,
            };
            let is_bsk_module = imp
                .trait_
                .as_ref()
                .and_then(|(_, path, _)| path.segments.last())
                .map_or(false, |seg| seg.ident == "BskModule");
            if !is_bsk_module {
                continue;
            }
            let struct_name = match type_last_ident(&imp.self_ty) {
                Some(n) => n,
                None => continue,
            };
            let mut input_types = Vec::new();
            for impl_item in &imp.items {
                if let ImplItem::Type(assoc) = impl_item {
                    if assoc.ident == "Inputs" {
                        input_types = tuple_elems(&assoc.ty);
                    }
                }
            }
            return Some((struct_name, input_types));
        }
    }
    None
}

/// A struct with the requested name exists but lacks the ABI representation
/// required for C interop.
#[derive(Debug, PartialEq, Eq)]
enum FindStructError {
    MissingReprC,
}

/// Searches cached source ASTs for a `#[repr(C)]` struct with the exact
/// identifier `name`.
///
/// Returns [`FindStructError::MissingReprC`] when a same-named struct exists
/// but lacks `#[repr(C)]`, allowing callers to tell that user mistake apart
/// from a missing or misspelled type.
fn find_struct_by_name<'a>(
    source_asts: &'a SourceAsts,
    name: &str,
) -> Result<Option<&'a ItemStruct>, FindStructError> {
    match source_asts.find_named_item(name) {
        Some(Item::Struct(item)) if has_repr_c(item) => Ok(Some(item)),
        Some(Item::Struct(_)) => Err(FindStructError::MissingReprC),
        _ => Ok(None),
    }
}

fn has_repr_c(s: &ItemStruct) -> bool {
    s.attrs.iter().any(|attr| {
        if !attr.path().is_ident("repr") {
            return false;
        }
        // repr attribute content as a string — check it contains "C"
        // (handles repr(C), repr(C, packed), etc.)
        let tokens = attr.to_token_stream().to_string();
        tokens.contains(" C ") || tokens.contains("(C)") || tokens.ends_with("C }")
    })
}

/// Extracts field descriptors from a `#[repr(C)]` struct. `allow_module_fields`
/// gates the field kinds that only make sense on the top-level config
/// struct (message ports, `Option<Box<T>>` owned state) — `false` for
/// nested structs discovered through a by-value struct field, which rejects
/// them with a clear error instead of silently generating something
/// meaningless.
fn extract_fields(s: &ItemStruct, ctx: &mut NestedCtx, allow_module_fields: bool) -> Vec<FieldInfo> {
    let named = match &s.fields {
        Fields::Named(f) => &f.named,
        Fields::Unnamed(_) => {
            ctx.error(
                s.ident.to_string(),
                format!(
                    "is a tuple struct, which isn't supported for a generated C ABI.\n\
             Use a named-field struct instead:\n\
             \n\
             \x20   #[repr(C)]\n\
             \x20   pub struct {} {{\n\
             \x20       pub field_name: FieldType,\n\
             \x20   }}",
                    s.ident
                ),
            );
            return Vec::new();
        }
        Fields::Unit => {
            ctx.error(
                s.ident.to_string(),
                format!(
                    "is a unit struct, which isn't supported for a generated C ABI.\n\
             Use a named-field struct instead:\n\
             \n\
             \x20   #[repr(C)]\n\
             \x20   pub struct {} {{\n\
             \x20       pub field_name: FieldType,\n\
             \x20   }}",
                    s.ident
                ),
            );
            return Vec::new();
        }
    };

    let mut result = Vec::new();

    for field in named {
        let name = field
            .ident
            .as_ref()
            .expect("Fields::Named must contain field identifiers")
            .to_string();

        let doc = collect_doc(&field.attrs);

        if let Some((kind, msg_type)) = msg_port_info(&field.ty) {
            if !allow_module_fields {
                ctx.error(
                    format!("{}.{}", s.ident, name),
                    format!(
                        "is a `MsgReader`/`MsgWriter` field, \
                     but `{}` is a nested struct (reached through a by-value \
                     struct field), not the top-level config struct. Message \
                     ports are only meaningful on the config struct that \
                     implements `BskModule`.",
                        s.ident
                    ),
                );
                continue;
            }
            // MsgReader<Foo>/MsgWriter<Foo> — same C layout as `Foo_C`.
            let c_type = format!("{msg_type}_C");
            result.push(FieldInfo {
                name,
                c_type,
                rust_base_type: String::new(),
                msg: Some((kind, msg_type)),
                doc,
                is_optional: false,
                is_owned_state: false,
                array_dims: Vec::new(),
            });
            continue;
        }

        if is_option_box(&field.ty) {
            if !allow_module_fields {
                ctx.error(
                    format!("{}.{}", s.ident, name),
                    format!(
                        "is `Option<Box<T>>`, but `{}` is a \
                     nested struct (reached through a by-value struct field), \
                     not the top-level config struct. Owned heap state relies \
                     on the destructor `bsk-build` generates for the config \
                     struct, which only exists there.",
                        s.ident
                    ),
                );
                continue;
            }
            // Option<Box<T>> — owned heap state (see "Owned heap state" in
            // the module docs). Nullable pointer optimization guarantees
            // this has the same layout/ABI as a raw pointer, so it maps
            // directly to `void *`; the generated destructor (see
            // `render_c_header`) runs Rust's normal drop glue to free it.
            result.push(FieldInfo {
                name,
                c_type: "void *".to_owned(),
                rust_base_type: String::new(),
                msg: None,
                doc,
                is_optional: false,
                is_owned_state: true,
                array_dims: Vec::new(),
            });
            continue;
        }

        let field_ctx = format!("{}.{name}", s.ident);
        let Some((rust_base_type, c_type, array_dims)) = decompose_type(&field.ty, ctx, &field_ctx)
        else {
            continue;
        };
        result.push(FieldInfo {
            name,
            c_type,
            rust_base_type,
            msg: None,
            doc,
            // Filled in by `apply_input_optionality` from the `impl
            // BskModule` block's `Inputs` tuple types — see module docs.
            is_optional: false,
            is_owned_state: false,
            array_dims,
        });
    }
    result
}

/// Recognizes a `MsgReader<Foo>` / `MsgWriter<Foo>` field type and returns
/// `(kind, "Foo")`.
fn msg_port_info(ty: &Type) -> Option<(MsgKind, String)> {
    let Type::Path(TypePath { path, .. }) = ty else { return None };
    let seg = path.segments.last()?;
    let kind = match seg.ident.to_string().as_str() {
        "MsgReader" => MsgKind::Reader,
        "MsgWriter" => MsgKind::Writer,
        _ => return None,
    };
    let syn::PathArguments::AngleBracketed(args) = &seg.arguments else { return None };
    let syn::GenericArgument::Type(inner) = args.args.first()? else { return None };
    type_last_ident(inner).map(|t| (kind, t))
}

/// True for `Option<Box<T>>` (any `T`) — bsk-build's supported pattern for
/// owned heap state (see "Owned heap state" in the module docs). Rust
/// guarantees this has the same size/alignment/ABI as a bare pointer (the
/// null-pointer optimization applies to `Option<Box<T>>` for any `T`), so it
/// is safe to treat as `void *` across the FFI boundary.
fn is_option_box(ty: &Type) -> bool {
    let Type::Path(TypePath { path, .. }) = ty else { return false };
    let Some(seg) = path.segments.last() else { return false };
    if seg.ident != "Option" {
        return false;
    }
    let syn::PathArguments::AngleBracketed(args) = &seg.arguments else { return false };
    let Some(syn::GenericArgument::Type(inner)) = args.args.first() else { return false };
    let Type::Path(TypePath { path: inner_path, .. }) = inner else { return false };
    inner_path.segments.last().is_some_and(|s| s.ident == "Box")
}

// ---------------------------------------------------------------------------
// Input optionality — derived from `impl BskModule`'s `Inputs` tuple types
// ---------------------------------------------------------------------------

/// Marks each `*InMsg` field optional when the matching element of
/// `impl BskModule for <Type> { type Inputs = (...); }` is `Option<Msg>`,
/// instead of requiring a doc-comment annotation.
fn apply_input_optionality(
    info: &mut ConfigInfo,
    input_types: &[Type],
    diagnostics: &mut Vec<Diagnostic>,
) {
    let struct_name = info.struct_name.clone();
    let input_fields: Vec<&mut FieldInfo> =
        info.fields.iter_mut().filter(|f| f.is_in_msg()).collect();

    if input_fields.len() != input_types.len() {
        let field_names: Vec<&str> = input_fields.iter().map(|f| f.name.as_str()).collect();
        let type_names: Vec<String> = input_types
            .iter()
            .map(|t| t.to_token_stream().to_string())
            .collect();
        diagnostics.push(Diagnostic {
            context: struct_name.clone(),
            message: format!(
                "has {} `MsgReader<T>` field(s) \
             ({}) but `impl BskModule for {struct_name}`'s `type Inputs` \
             tuple has {} element(s) ({}). Add or remove a `MsgReader<T>` \
             field, or fix the `Inputs` tuple, so there's exactly one tuple \
             element per input field, in the same order they're declared \
             in the struct.",
            input_fields.len(),
            if field_names.is_empty() { "none".to_owned() } else { field_names.join(", ") },
            input_types.len(),
            if type_names.is_empty() { "none".to_owned() } else { type_names.join(", ") },
            ),
        });
        return;
    }

    for (field, ty) in input_fields.into_iter().zip(input_types.iter()) {
        let (is_optional, inner) = match option_inner(ty) {
            Some(inner) => (true, inner),
            None => (false, ty),
        };

        let expected = field.msg.as_ref().map(|(_, t)| t.clone()).unwrap_or_default();
        if let Some(found) = type_last_ident(inner) {
            if found != expected {
                diagnostics.push(Diagnostic {
                    context: format!("{struct_name}.{}", field.name),
                    message: format!(
                        "is `MsgReader<{expected}>`, but \
                     the matching `Inputs` tuple element resolves to `{found}` \
                     (expected `{expected}`, optionally wrapped in `Option<..>`). \
                     `Inputs` elements must line up with `*InMsg` fields in \
                     declaration order.",
                    ),
                });
                continue;
            }
        }
        field.is_optional = is_optional;
    }
}

/// Splits a tuple type `(A, B)` into `[A, B]`; a non-tuple type is treated as
/// a single-element list (defensive — `Inputs`/`Outputs` are always tuples
/// in generated shims, even `()` and `(A,)`).
fn tuple_elems(ty: &Type) -> Vec<Type> {
    match ty {
        Type::Tuple(t) => t.elems.iter().cloned().collect(),
        other => vec![other.clone()],
    }
}

/// Returns the inner type of `Option<T>`, or `None` if `ty` isn't `Option<_>`.
fn option_inner(ty: &Type) -> Option<&Type> {
    if let Type::Path(TypePath { path, .. }) = ty {
        let seg = path.segments.last()?;
        if seg.ident != "Option" {
            return None;
        }
        if let syn::PathArguments::AngleBracketed(args) = &seg.arguments {
            if let Some(syn::GenericArgument::Type(inner)) = args.args.first() {
                return Some(inner);
            }
        }
    }
    None
}

/// Last path segment of a type, e.g. `BskModuleRuntime` for both
/// `BskModuleRuntime` and `bsk_messages::BskModuleRuntime`.
fn type_last_ident(ty: &Type) -> Option<String> {
    if let Type::Path(TypePath { path, .. }) = ty {
        path.segments.last().map(|s| s.ident.to_string())
    } else {
        None
    }
}

/// Collect all `///` doc-comment lines on a field into one string.
fn collect_doc(attrs: &[Attribute]) -> String {
    let mut parts: Vec<String> = Vec::new();
    for attr in attrs {
        if !attr.path().is_ident("doc") {
            continue;
        }
        if let Meta::NameValue(nv) = &attr.meta {
            if let Expr::Lit(ExprLit {
                lit: Lit::Str(s), ..
            }) = &nv.value
            {
                let text = s.value();
                let trimmed = text.trim().to_owned();
                if !trimmed.is_empty() {
                    parts.push(trimmed);
                }
            }
        }
    }
    parts.join(" ")
}

/// Returns (rust_base_type, c_type) for a non-message, non-owned-state field
/// (those are handled separately by `msg_port_info`/`is_option_box`, above).
/// A pointer `c_type` (e.g. `"BSKLogger *"`) always ends in `*`, which
/// `render_c_header` uses to decide field-declaration spacing.
///
/// * `rust_base_type` — last path segment (e.g. `"f64"`, `"BskModuleRuntime"`)
/// * `c_type`         — full C declaration type (e.g. `"double"`, `"BSKLogger *"`)
///
/// Every branch either resolves to a known-safe C representation or panics —
/// there is no "assume it's already valid C and pass the name through"
/// fallback (see the module docs' closing paragraph).
/// `field_ctx` identifies the field being resolved for error messages, e.g.
/// `"myModuleConfig.weird"` — every panic in this function or anything it
/// calls names both the field and the type it couldn't resolve, so a
/// mistake in one field of a large config struct doesn't require guessing
/// which one.
/// Entry point for a field's type. Peels off any number of `[T; N]` layers
/// (collecting their dimensions outermost-first), then resolves the scalar
/// element type with [`decompose_type_inner`]. Returns
/// `(rust_base_type, c_type, array_dims)` where `array_dims` is empty for a
/// non-array field and `[outer, ..., inner]` for an array (e.g. Rust
/// `[[f64; 3]; 4]` → `c_type = "double"`, `array_dims = [4, 3]`, matching C's
/// `double field[4][3]`). Every dimension must be a bare integer literal; a
/// const/generic length is a build error (see below).
fn decompose_type(
    ty: &Type,
    ctx: &mut NestedCtx,
    field_ctx: &str,
) -> Option<(String, String, Vec<usize>)> {
    let mut dims: Vec<usize> = Vec::new();
    let mut current = ty;
    // Peel [T; N] layers until we reach a non-array type.
    loop {
        match current {
            Type::Array(TypeArray { elem, len, .. }) => {
                let Some(n) = array_len_from_expr(len) else {
                    ctx.error(
                        field_ctx,
                        "has an array length that isn't a plain integer \
                         literal (e.g. `[f64; 3]`). bsk-build evaluates array \
                         lengths at build-script time, before any of the \
                         crate's own `const`s or generics exist to resolve — \
                         const generics, `N * 2`, referencing another `const`, \
                         etc. aren't supported. Use a literal length."
                            .to_owned(),
                    );
                    return None;
                };
                dims.push(n);
                current = elem;
            }
            other => {
                let (rust_base_type, c_type) =
                    decompose_type_inner(other, ctx, false, field_ctx)?;
                return Some((rust_base_type, c_type, dims));
            }
        }
    }
}

/// Evaluates a `[T; <len>]` array-length expression to a `usize`, accepting
/// only a bare integer literal (e.g. `3`) — see `decompose_type`'s doc
/// comment for why more complex expressions aren't supported.
fn array_len_from_expr(len: &Expr) -> Option<usize> {
    match len {
        Expr::Lit(ExprLit { lit: Lit::Int(lit), .. }) => lit.base10_parse::<usize>().ok(),
        _ => None,
    }
}

/// `is_pointer_target` relaxes by-value resolution to also accept types this
/// crate can't fully lay out but that are safe to hold *by pointer* (e.g.
/// `BSKLogger`, an opaque type owned by Basilisk's C++ side) — see
/// `known_opaque_pointer_target`.
fn decompose_type_inner(
    ty: &Type,
    ctx: &mut NestedCtx,
    is_pointer_target: bool,
    field_ctx: &str,
) -> Option<(String, String)> {
    match ty {
        Type::Path(TypePath { path, .. }) => {
            // Take the last path segment: handles both `f64` and
            // `bsk_messages::BskModuleRuntime`.
            let last = path
                .segments
                .last()
                .map(|s| s.ident.to_string())
                .unwrap_or_default();
            // Checked *before* the primitive/special lookup below: SWIG's
            // ownership-transferring pointer setter (see the panic message)
            // applies just as much to `*mut u8`/`*mut f64`/... as it does to
            // a pointer to a struct bsk-build generates — a sized primitive
            // pointee isn't a safer case and must not skip this check.
            if is_pointer_target {
                if known_opaque_pointer_target(&last) {
                    return Some((last.clone(), last));
                }
                // Deliberately does not fall through to primitive/special
                // lookup or `resolve_nested_struct` even when `last` names a
                // valid primitive or nested struct — see the panic message
                // for why raw pointers to anything but the allowlist above
                // aren't allowed.
                ctx.error(
                    field_ctx,
                    format!(
                        "is a raw pointer to `{last}`, \
                     which bsk-build doesn't allow.\n\
                     SWIG generates an ownership-transferring setter for any \
                     pointer field (the Python object passed in is marked as \
                     no longer owning its memory), which is only safe for a \
                     small allowlist of types Basilisk's C++ side keeps \
                     alive independently of Python ({}). `{last}` isn't one \
                     of those.\n\
                     If `{field_ctx}` is meant to hold a `#[repr(C)]` struct \
                     this crate defines, embed it by value instead (see \
                     \"Nested structs\" in the module docs) for safe copy \
                     semantics with no ownership questions. There is \
                     currently no supported way to store a persistent \
                     string/byte-buffer field (e.g. `*const c_char`) on a \
                     config struct, use an enum instead."
                    , OPAQUE_POINTER_TARGETS.join(", "),
                    ),
                );
                return None;
            }
            if let Some(c) = primitive_to_c(&last).or_else(|| special_type_to_c(&last)) {
                return Some((last, c.to_owned()));
            }
            let c = resolve_nested_struct(&last, ctx, field_ctx)?;
            Some((last, c))
        }
        Type::Ptr(TypePtr {
            mutability, elem, ..
        }) => {
            if type_last_ident(elem).as_deref() == Some("c_void") {
                ctx.error(
                    field_ctx,
                    "is a bare `*mut`/`*const c_void` \
                     field — that's not supported, since an untyped raw \
                     pointer bypasses both Rust's ownership and SWIG's type \
                     checking. If `{field_ctx}` holds heap state that \
                     persists across calls, use `Option<Box<T>>` instead \
                     (see \"Owned heap state\" in the module docs); there is \
                     no supported use for a bare `void *` field.",
                );
                return None;
            }
            let (inner_rust, inner_c) = decompose_type_inner(elem, ctx, true, field_ctx)?;
            let c_type = if mutability.is_some() {
                format!("{} *", inner_c)
            } else {
                format!("const {} *", inner_c)
            };
            Some((inner_rust, c_type))
        }
        other => {
            let s = other.to_token_stream().to_string();
            ctx.error(
                field_ctx,
                format!(
                    "has type `{s}`, which isn't a \
                 supported field type.\n\
                 Supported field types are: Rust primitives (f64, i32, \
                 bool, ...), `MsgReader<T>`/`MsgWriter<T>` message ports, \
                 `Option<Box<T>>` owned heap state, `*mut`/`*const` \
                 pointers, or a `#[repr(C)]` struct defined in this crate's \
                 `src/`.\n\
                 If `{field_ctx}` needs to hold `{s}` specifically, it \
                 can't be represented in the generated C header — store it \
                 behind `Option<Box<T>>` instead (see \"Owned heap state\" \
                 in the module docs) and expose only C-compatible data \
                 through other fields."
                ),
            );
            None
        }
    }
}

/// Resolves a bare (by-value) type name that isn't a primitive or special
/// FFI type: it must be a `#[repr(C)]` struct defined in this crate's
/// `src/` (see "Nested structs" in the module docs). Recursively extracts
/// and registers its fields in `ctx.nested`, in dependency order, the first
/// time it's seen; later references just reuse the cached entry.
///
/// `field_ctx` is the field that referenced `name` (e.g.
/// `"myModuleConfig.target"`), included in every panic so the error points
/// at the field to fix, not just the unresolved type name.
fn resolve_nested_struct(name: &str, ctx: &mut NestedCtx, field_ctx: &str) -> Option<String> {
    if ctx.nested.iter().any(|s| s.name == name) {
        return Some(name.to_owned());
    }
    if ctx.in_progress.iter().any(|n| n == name) {
        ctx.error(
            field_ctx,
            format!(
                "(type `{name}`) is involved in a \
             struct-by-value cycle ({} -> ... -> {name}) — a struct can't \
             contain itself by value in C. Break the cycle by making \
             `{field_ctx}` a pointer or, if it's heap state rather than a \
             plain parameter, `Option<Box<T>>`.",
            ctx.in_progress.join(" -> "),
            ),
        );
        return None;
    }
    let source_asts = ctx.source_asts;
    if matches!(source_asts.find_named_item(name), Some(Item::Enum(_))) {
        ctx.error(
            field_ctx,
            format!(
                "has type `{name}`, a Rust `enum` — \
             enums aren't a supported field type, regardless of `#[repr(...)]`.\n\
             The problem isn't representation: a C `enum` backed by, say, \
             `#[repr(u8)]` has the same layout as `u8`. The problem is that \
             SWIG's default field setter for an enum-typed member accepts \
             any integer, including ones that don't name a declared variant \
             — so Python could put a value into `{field_ctx}` that isn't a \
             valid `{name}`, and matching on it in Rust would be undefined \
             behavior.\n\
             Use the enum's underlying integer type as the field instead \
             (e.g. `pub {last}: u8` if `{name}` is `#[repr(u8)]`), and \
             convert it to `{name}` inside `update`/`reset` with a checked \
             conversion (e.g. a manual `match` with a fallback, or \
             `TryFrom`) instead of trusting the raw value.",
            last = field_ctx.rsplit('.').next().unwrap_or(field_ctx),
            ),
        );
        return None;
    }
    let item = match find_struct_by_name(source_asts, name) {
        Ok(Some(item)) => item,
        Ok(None) => {
            ctx.error(
                field_ctx,
                format!(
                    "has type `{name}`, which isn't a \
             supported field type.\n\
             Supported field types are: Rust primitives (f64, i32, bool, \
             ...), `MsgReader<T>`/`MsgWriter<T>` message ports, \
             `Option<Box<T>>` owned heap state, `*mut`/`*const` pointers, or \
             another `#[repr(C)]` struct defined in this crate's `src/`.\n\
             `{name}` matched none of these. If you meant to reference a \
             struct defined elsewhere in `{field_ctx}`'s crate, check that \
             it's spelled correctly and annotated `#[repr(C)]` — bsk-build \
             only looks for `#[repr(C)]` structs under this crate's `src/`."
                ),
            );
            return None;
        }
        Err(FindStructError::MissingReprC) => {
            ctx.error(
                field_ctx,
                format!(
                    "has type `{name}`, a struct defined in \
             this crate's `src/`, but `{name}` is missing `#[repr(C)]`.\n\
             Add `#[repr(C)]` immediately above `pub struct {name}`. \
             By-value nested structs are included in the generated C ABI and \
             therefore require an explicit C-compatible layout."
                ),
            );
            return None;
        }
    };
    ctx.in_progress.push(name.to_owned());
    let error_count = ctx.diagnostics.len();
    let fields = extract_fields(&item, ctx, false);
    ctx.in_progress.pop();
    if ctx.diagnostics.len() == error_count {
        ctx.nested.push(NestedStructInfo {
            name: name.to_owned(),
            fields,
        });
        Some(name.to_owned())
    } else {
        None
    }
}

/// Types this crate never has (or needs) the full layout of, but that are
/// safe to reference *by pointer* because Basilisk's C++ side owns the real
/// instance and keeps it alive independently of Python — the Rust side only
/// ever forwards the pointer. Extend this list only for similarly opaque,
/// long-lived-on-the-C++-side BSK types. A `#[repr(C)]` struct this crate
/// itself generates (nested or top-level) is never added here: those are
/// meant to be used by value (see "Nested structs" in the module docs), and
/// a raw pointer to one hits the panic in `decompose_type_inner` instead.
const OPAQUE_POINTER_TARGETS: &[&str] = &["BSKLogger"];

fn known_opaque_pointer_target(name: &str) -> bool {
    OPAQUE_POINTER_TARGETS.contains(&name)
}

/// `bsk-messages` types whose Rust name intentionally differs from their C
/// counterpart (`bsk_rust_module.h` calls it `BskRustModuleRuntime` — the
/// `Rust` disambiguates it as a Rust-module-only ABI detail, unlike
/// `BSKLogger` which is a shared BSK type and keeps its name unchanged).
fn special_type_to_c(rust: &str) -> Option<&'static str> {
    Some(match rust {
        "BskModuleRuntime" => "BskRustModuleRuntime",
        _ => return None,
    })
}

fn primitive_to_c(rust: &str) -> Option<&'static str> {
    Some(match rust {
        "f64" => "double",
        "f32" => "float",
        "i64" => "int64_t",
        "u64" => "uint64_t",
        "i32" => "int32_t",
        "u32" => "uint32_t",
        "i16" => "int16_t",
        "u16" => "uint16_t",
        "i8" => "int8_t",
        "u8" => "uint8_t",
        "bool" => "bool",
        "c_void" => "void",
        _ => return None,
    })
}

// ---------------------------------------------------------------------------
// C header renderer
// ---------------------------------------------------------------------------

/// Renders one field declaration per line, e.g. `"    double K;"`, doc
/// comments right-aligned as trailing Doxygen `/*!< ... */`. Shared by the
/// config struct and nested struct renderers.
fn render_field_decls(fields: &[FieldInfo]) -> String {
    let mut out = String::new();
    for f in fields {
        // "BSKLogger *" + name -> "BSKLogger *bskLogger" (no space after
        // `*`); non-pointer types get the usual single space.
        let sep = if f.c_type.ends_with('*') { "" } else { " " };
        // C's array-length suffix attaches after the field name
        // (`double name[3];`, `double name[4][3];`), never after the type.
        let array_suffix: String = f.array_dims.iter().map(|n| format!("[{n}]")).collect();
        let decl = format!("    {}{sep}{}{array_suffix};", f.c_type, f.name);

        if f.doc.is_empty() {
            out.push_str(&format!("{decl}\n"));
        } else {
            out.push_str(&format!("{decl:<52} /*!< {} */\n", f.doc));
        }
    }
    out
}

/// Renders a nested struct's `typedef struct` (see "Nested structs" in the
/// module docs) — a plain C struct with no BSK-specific machinery, since
/// only the top-level config struct gets the runtime field, message
/// includes, and destructor.
fn render_nested_struct(s: &NestedStructInfo) -> String {
    format!(
        "\n\
         /*! @brief Nested struct type, generated from the Rust source by \
         bsk-build. Do not edit by hand. */\n\
         typedef struct {name} {{\n\
         {fields}\
         }} {name};\n",
        name = s.name,
        fields = render_field_decls(&s.fields),
    )
}

fn render_c_header(info: &ConfigInfo, module: &str) -> String {
    let cfg_type = &info.struct_name;

    // Include-guard: insert underscores before each uppercase run.
    let guard = to_include_guard(module);

    // Determine which cMsgCInterface headers to include.
    let msg_includes: BTreeSet<String> = info
        .fields
        .iter()
        .filter(|f| f.msg.is_some())
        .map(|f| format!("cMsgCInterface/{}.h", f.c_type))
        .collect();

    let mut h = String::new();

    h.push_str(&format!(
        "/*\n\
         * Auto-generated by bsk-build from src/*.rs.\n\
         * Source of truth: `pub struct {cfg_type}` in the Rust source.\n\
         * Do not edit manually — re-run `cargo build` to regenerate.\n\
         */\n\
         #ifndef {guard}\n\
         #define {guard}\n\
         \n\
         #include \"architecture/_GeneralModuleFiles/bsk_rust_module.h\"\n"
    ));
    for inc in &msg_includes {
        h.push_str(&format!("#include \"{inc}\"\n"));
    }

    // Nested struct types (see "Nested structs" in the module docs) must be
    // fully declared before the config struct uses them by value; `ctx`
    // populated `info.nested_structs` in dependency order during field
    // extraction, so a simple in-order emission satisfies that.
    for nested in &info.nested_structs {
        h.push_str(&render_nested_struct(nested));
    }

    h.push_str(&format!(
        "\n\
         /*! @brief Config struct for the {module} Basilisk module (Rust implementation).\n\
         *\n\
         *  This struct is generated from the Rust source by bsk-build.\n\
         *  Do not edit the C header by hand; edit the Rust struct instead.\n\
         */\n\
         typedef struct {cfg_type} {{\n"
    ));

    h.push_str(&render_field_decls(&info.fields));

    // The C++ constructor calls Init_<module> so `BskModule::init()` runs at
    // construction time — before Python configures any fields. This is the Rust
    // equivalent of a C++ module's constructor (see BskModule::init docs).
    // The destructor runs the config struct's ordinary Rust drop glue (freeing
    // any `Option<Box<T>>` owned-state field) — see "Owned heap state" in the
    // module docs and `BSK_RUST_DECL`'s `Drop_<module>` declaration.
    // Both are only visible to C++ (SWIG); plain-C consumers see an ordinary struct.
    h.push_str(
        "#ifdef __cplusplus\n\
         \x20   ",
    );
    h.push_str(cfg_type);
    h.push_str(
        "();\n\
         \x20   ~",
    );
    h.push_str(cfg_type);
    h.push_str(
        "();\n\
         #endif\n",
    );

    h.push_str(&format!(
        "}} {cfg_type};\n\
         \n\
         BSK_RUST_DECL({module}, {cfg_type})\n\
         \n\
         #ifdef __cplusplus\n\
         extern \"C-unwind\" void Init_{module}({cfg_type}* cfg);\n\
         inline {cfg_type}::{cfg_type}() {{ Init_{module}(this); }}\n\
         inline {cfg_type}::~{cfg_type}() {{ Drop_{module}(this); }}\n\
         #endif\n\
         \n\
         #endif /* {guard} */\n"
    ));

    h
}

/// camelCase → CAMEL_CASE_H for include guards.
fn to_include_guard(name: &str) -> String {
    let mut out = String::new();
    for (i, ch) in name.chars().enumerate() {
        if ch.is_uppercase() && i > 0 {
            out.push('_');
        }
        out.push(ch.to_ascii_uppercase());
    }
    out.push_str("_H");
    out
}

// ---------------------------------------------------------------------------
// Shim renderer
// ---------------------------------------------------------------------------

fn camel_to_snake(s: &str) -> String {
    let mut out = String::new();
    for (i, ch) in s.chars().enumerate() {
        if ch.is_uppercase() && i > 0 {
            out.push('_');
        }
        out.push(ch.to_ascii_lowercase());
    }
    out
}

/// Renders the SWIG `.i` file `bsk_add_rust_module.cmake` points SWIG at
/// (see the `BSK_INTERFACE_PATH` handling in `generate`). This is the only
/// generator of Rust-module `.i` files — there is no hand-written-`.i`
/// escape hatch (unlike C modules' `%c_wrap` family in `swig_c_wrap.i`) — so
/// the boilerplate that wraps `RustWrapper` around the module's lifecycle
/// functions is written out directly here instead of via a SWIG macro.
fn render_swig_interface(info: &ConfigInfo, module: &str, header_path: &Path) -> String {
    let cfg_type = &info.struct_name;
    let header_abs = header_path.display();

    // %import registers each message type in SWIG's type system so that
    // struct member getters return a pointer to the embedded field
    // (in-place access) rather than a by-value copy. Without this,
    // `ctrl.attGuidInMsg.subscribeTo()` would silently modify a temporary
    // copy instead of the real field. `BTreeSet` both de-duplicates (two
    // ports of the same message type) and gives a stable order.
    let msg_imports: BTreeSet<&str> = info
        .fields
        .iter()
        .filter(|f| f.is_in_msg() || f.is_out_msg())
        .map(|f| f.c_type.as_str())
        .collect();
    let mut import_lines = String::new();
    for t in &msg_imports {
        import_lines.push_str(&format!("%import \"cMsgCInterface/{t}.h\"\n"));
    }

    // Owned-state fields (Rust `Option<Box<T>>`) are Rust-internal; nothing
    // else may safely write to them: the destructor `render_c_header`
    // generates runs Rust's drop glue over whatever bytes are there, so a
    // Python-side write (even just `= None`) either leaks the previously
    // owned box or, if aliased to some other pointer entirely, frees memory
    // Rust's allocator never allocated. `%immutable` hides only the setter;
    // the (harmless) getter is unaffected. Must appear before `%include` so
    // SWIG applies it while parsing the struct.
    let mut immutable_lines = String::new();
    for f in info.fields.iter().filter(|f| f.is_owned_state) {
        immutable_lines.push_str(&format!("%immutable {cfg_type}::{};\n", f.name));
    }

    format!(
        "%module {module}\n\
         %{{\n\
         #include \"{header_abs}\"\n\
         %}}\n\
         \n\
         /* BSK_RUST_DECL emits extern \"C\" declarations for the C compiler; the\n\
          * %ignore lines below hide them from Python and SWIG never needs to see\n\
          * the declarations themselves. Define it empty here (matching its real\n\
          * 2-arg signature) so headers that use it parse cleanly under SWIG. */\n\
         #define BSK_RUST_DECL(name, configType)\n\
         \n\
         %include \"swig_c_wrap.i\"\n\
         \n\
         {import_lines}\n\
         {immutable_lines}\n\
         %include \"{header_abs}\"\n\
         \n\
         %ignore Update_{module};\n\
         %ignore SelfInit_{module};\n\
         %ignore Reset_{module};\n\
         // Drop_{module} (BSK_RUST_DECL) is only ever called from the generated\n\
         // header's inline destructor (see render_c_header); hide it from Python\n\
         // so nothing can call it directly and double-drop the config's owned\n\
         // state.\n\
         %ignore Drop_{module};\n\
         \n\
         // Default no-op Reset. Templated so it loses overload resolution to a\n\
         // real Reset_{module} the module may define (same trick %c_wrap_3 uses\n\
         // for C modules — see swig_c_wrap.i).\n\
         %inline %{{\n\
         \x20 template <typename T> inline void Reset_{module}(T, uint64_t, const BskRustModuleRuntime*) {{}}\n\
         %}}\n\
         \n\
         // RustWrapper(TConfig* config) takes ownership of the pointer; the\n\
         // Python object for that config shouldn't also think it owns the memory.\n\
         %pythonappend RustWrapper::RustWrapper %{{\n\
         \x20   if (len(args)) > 0:\n\
         \x20       args[0].thisown = False\n\
         %}}\n\
         \n\
         %template({module}) RustWrapper<{cfg_type},Update_{module},SelfInit_{module},Reset_{module}>;\n\
         \n\
         %extend {cfg_type} {{\n\
         \x20 %pythoncode %{{\n\
         \x20   def createWrapper(self):\n\
         \x20       return {module}(self)\n\
         \x20 %}}\n\
         }}\n"
    )
}

fn render_shim(info: &ConfigInfo, module: &str) -> String {
    let cfg_type = &info.struct_name;

    let inputs: Vec<&FieldInfo> = info.fields.iter().filter(|f| f.is_in_msg()).collect();
    let outputs: Vec<&FieldInfo> = info.fields.iter().filter(|f| f.is_out_msg()).collect();

    let out_vars: Vec<String> = outputs.iter().map(|f| camel_to_snake(&f.name)).collect();
    let out_lhs = match out_vars.len() {
        0 => "let ()".to_owned(),
        1 => format!("let ({},)", out_vars[0]),
        _ => format!("let ({})", out_vars.join(", ")),
    };

    // Does the config struct expose a bskLogger field for per-module verbosity?
    let has_logger = info.fields.iter().any(|f| f.name == "bskLogger");
    // Expression to pass as the receiver of `BskLoggerExt::bsk_error` below.
    // Falls back to a null pointer, which `BskLoggerExt` resolves to the
    // process-wide default logger — see bsk-messages.
    let logger_expr = if has_logger {
        "(*cfg).bskLogger"
    } else {
        "::core::ptr::null_mut()"
    };

    let mut s = String::new();
    s.push_str("// Auto-generated by bsk-build — do not edit.\n\n");

    // Init — called from the generated C++ default constructor before Python
    // touches the struct. This is the Rust equivalent of a C++ module's
    // constructor: sets non-zero parameter defaults and initial state.
    s.push_str(&format!(
        "#[no_mangle]\npub unsafe extern \"C-unwind\" fn Init_{module}(\n    cfg: *mut {cfg_type},\n) {{\n\
         \x20   <{cfg_type} as BskModule>::init(&mut *cfg);\n\
         }}\n\n"
    ));

    // SelfInit — called at InitializeSimulation(), after Python has configured
    // the struct. Per Basilisk's cModules-3.rst: "The SelfInit function should
    // only do the above steps [Msg_C_init calls]." The shim handles all of them;
    // there is no BskModule::self_init() trait method.
    s.push_str(&format!(
        "#[no_mangle]\npub unsafe extern \"C-unwind\" fn SelfInit_{module}(\n    cfg: *mut {cfg_type},\n    runtime: *const BskModuleRuntime,\n) {{\n\
         \x20   (*cfg).runtime = ::core::ptr::read(runtime);\n"
    ));
    for f in &outputs {
        s.push_str(&format!("    (*cfg).{field}.init();\n", field = f.name));
    }
    s.push_str("}\n\n");

    // Reset
    s.push_str(&format!(
        "#[no_mangle]\npub unsafe extern \"C-unwind\" fn Reset_{module}(\n    cfg: *mut {cfg_type},\n    current_sim_nanos: u64,\n    runtime: *const BskModuleRuntime,\n) {{\n\
         \x20   (*cfg).runtime = ::core::ptr::read(runtime);\n"
    ));
    for f in &inputs {
        if !f.is_optional {
            s.push_str(&format!(
                "    if !(*cfg).{field}.is_linked() {{\n\
                 \x20       bsk_messages::BskLoggerExt::bsk_error({logger_expr}, \"[{module}] {field} is not connected\");\n\
                 \x20   }}\n",
                field = f.name,
            ));
        }
    }
    // Capture and write the outputs returned by reset(), just as Update does,
    // so every output port is initialised before the first UpdateState tick.
    s.push_str(&format!(
        "    {out_lhs}: <{cfg_type} as BskModule>::Outputs = \
         <{cfg_type} as BskModule>::reset(&mut *cfg, current_sim_nanos);\n",
    ));
    for f in &outputs {
        let var = camel_to_snake(&f.name);
        s.push_str(&format!(
            "    (*cfg).{field}.write(&{var}, (*cfg).runtime.module_id(), current_sim_nanos);\n",
            field = f.name,
        ));
    }
    s.push_str("}\n\n");

    // Update
    s.push_str(&format!(
        "#[no_mangle]\npub unsafe extern \"C-unwind\" fn Update_{module}(\n    cfg: *mut {cfg_type},\n    current_sim_nanos: u64,\n    runtime: *const BskModuleRuntime,\n) {{\n\
         \x20   (*cfg).runtime = ::core::ptr::read(runtime);\n"
    ));
    for f in &inputs {
        let var = camel_to_snake(&f.name);
        if f.is_optional {
            s.push_str(&format!(
                "    let {var} = (*cfg).{field}.is_linked().then(|| (*cfg).{field}.read());\n",
                field = f.name,
            ));
        } else {
            s.push_str(&format!(
                "    if !(*cfg).{field}.is_linked() {{\n\
                 \x20       bsk_messages::BskLoggerExt::bsk_error({logger_expr}, \"[{module}] {field} is not connected\");\n\
                 \x20   }}\n\
                 \x20   let {var} = (*cfg).{field}.read();\n",
                field = f.name,
            ));
        }
    }

    let input_vars: Vec<String> = inputs.iter().map(|f| camel_to_snake(&f.name)).collect();
    let input_tuple = match input_vars.len() {
        0 => "()".to_owned(),
        1 => format!("({},)", input_vars[0]),
        _ => format!("({})", input_vars.join(", ")),
    };

    s.push_str(&format!(
        "    {out_lhs}: <{cfg_type} as BskModule>::Outputs = \
         <{cfg_type} as BskModule>::update(&mut *cfg, {input_tuple}, current_sim_nanos);\n",
    ));

    for f in &outputs {
        let var = camel_to_snake(&f.name);
        s.push_str(&format!(
            "    (*cfg).{field}.write(&{var}, (*cfg).runtime.module_id(), current_sim_nanos);\n",
            field = f.name,
        ));
    }
    s.push_str("}\n\n");

    // Called from the generated header's `~{cfg_type}()` (see
    // `render_c_header`) — runs the config struct's ordinary drop glue
    // (freeing any `Option<Box<T>>` owned-state field) without deallocating
    // the struct itself; the C++ side does that immediately afterward.
    s.push_str(&format!(
        "#[no_mangle]\npub unsafe extern \"C-unwind\" fn Drop_{module}(cfg: *mut {cfg_type}) {{\n\
         \x20   ::core::ptr::drop_in_place(cfg);\n\
         }}\n",
    ));

    s
}

// Needed for `.to_token_stream()` — used both to detect `#[repr(C)]` and to
// render unsupported types as normal Rust syntax in error messages, instead
// of `syn`'s much noisier derived `Debug` output.
use syn::__private::ToTokens;

#[cfg(test)]
mod tests {
    use super::*;

    /// No test here defines a nested struct in a *separate* file, so this
    /// placeholder directory is never actually read; see
    /// `nested_repr_c_struct_field_is_generated_before_parent` for a test
    /// that sets up a real one.
    fn empty_src_dir() -> PathBuf {
        PathBuf::from("/nonexistent-bsk-build-test-src-dir")
    }

    fn info_for(struct_src: &str) -> ConfigInfo {
        info_for_in(struct_src, &empty_src_dir())
    }

    fn info_for_in(struct_src: &str, src_dir: &Path) -> ConfigInfo {
        let item: ItemStruct = syn::parse_str(struct_src).expect("test struct must parse");
        let struct_name = item.ident.to_string();
        let source_asts = SourceAsts::load(src_dir);
        let mut ctx = NestedCtx {
            source_asts: &source_asts,
            diagnostics: source_asts.diagnostics.clone(),
            nested: Vec::new(),
            in_progress: vec![struct_name.clone()],
        };
        let fields = extract_fields(&item, &mut ctx, true);
        if !ctx.diagnostics.is_empty() {
            panic_with_diagnostics(ctx.diagnostics);
        }
        ConfigInfo {
            struct_name,
            fields,
            nested_structs: ctx.nested,
        }
    }

    fn diagnostics_for(struct_src: &str) -> Vec<Diagnostic> {
        diagnostics_for_in(struct_src, &empty_src_dir())
    }

    fn diagnostics_for_in(struct_src: &str, src_dir: &Path) -> Vec<Diagnostic> {
        let item: ItemStruct = syn::parse_str(struct_src).expect("test struct must parse");
        let source_asts = SourceAsts::load(src_dir);
        let mut ctx = NestedCtx {
            source_asts: &source_asts,
            diagnostics: Vec::new(),
            nested: Vec::new(),
            in_progress: vec![item.ident.to_string()],
        };
        extract_fields(&item, &mut ctx, true);
        ctx.diagnostics
    }

    #[test]
    fn named_struct_without_repr_c_is_reported_separately() {
        let test_dir = std::env::temp_dir().join(format!(
            "bsk-build-missing-repr-c-{}",
            std::process::id()
        ));
        std::fs::create_dir_all(&test_dir).expect("test source directory must be created");
        std::fs::write(
            test_dir.join("module.rs"),
            "pub struct MissingReprC { pub value: f64 }",
        )
        .expect("test source must be written");

        let source_asts = SourceAsts::load(&test_dir);
        assert!(matches!(
            find_struct_by_name(&source_asts, "MissingReprC"),
            Err(FindStructError::MissingReprC)
        ));

        std::fs::remove_dir_all(test_dir).expect("test source directory must be removed");
    }

    #[test]
    fn source_ast_cache_reports_parse_failures() {
        let test_dir = std::env::temp_dir().join(format!(
            "bsk-build-parse-error-{}",
            std::process::id()
        ));
        std::fs::create_dir_all(&test_dir).expect("test source directory must be created");
        std::fs::write(test_dir.join("broken.rs"), "pub struct Broken {")
            .expect("invalid test source must be written");

        let source_asts = SourceAsts::load(&test_dir);
        assert!(source_asts.files.is_empty());
        assert_eq!(source_asts.diagnostics.len(), 1);
        assert!(source_asts.diagnostics[0]
            .message
            .starts_with("could not parse Rust source:"));

        std::fs::remove_dir_all(test_dir).expect("test source directory must be removed");
    }

    #[test]
    fn independent_field_errors_are_collected_together() {
        let item: ItemStruct = syn::parse_str(
            "#[repr(C)] pub struct MyConfig { \
             pub runtime: bsk_messages::BskModuleRuntime, \
             pub bytePtr: *mut u8, \
             pub voidPtr: *mut core::ffi::c_void, \
             }",
        )
        .expect("test struct must parse");
        let source_asts = SourceAsts::load(&empty_src_dir());
        let mut ctx = NestedCtx {
            source_asts: &source_asts,
            diagnostics: Vec::new(),
            nested: Vec::new(),
            in_progress: vec!["MyConfig".to_owned()],
        };

        let fields = extract_fields(&item, &mut ctx, true);
        assert_eq!(fields.len(), 1);
        assert_eq!(ctx.diagnostics.len(), 2);
        assert!(ctx
            .diagnostics
            .iter()
            .any(|diagnostic| diagnostic.context == "MyConfig.bytePtr"));
        assert!(ctx
            .diagnostics
            .iter()
            .any(|diagnostic| diagnostic.context == "MyConfig.voidPtr"));
    }

    #[test]
    fn tuple_struct_is_rejected() {
        let diagnostics = diagnostics_for("#[repr(C)] pub struct TupleConfig(pub f64);");
        assert_eq!(diagnostics.len(), 1);
        assert_eq!(diagnostics[0].context, "TupleConfig");
        assert!(diagnostics[0]
            .message
            .starts_with("is a tuple struct, which isn't supported"));
    }

    #[test]
    fn unit_struct_is_rejected() {
        let diagnostics = diagnostics_for("#[repr(C)] pub struct UnitConfig;");
        assert_eq!(diagnostics.len(), 1);
        assert_eq!(diagnostics[0].context, "UnitConfig");
        assert!(diagnostics[0]
            .message
            .starts_with("is a unit struct, which isn't supported"));
    }

    /// `Option<Box<T>>` owned-state fields map to a nullable `void *`, and
    /// the generated header wires a C++ destructor to a `Drop_<module>`
    /// call so cleanup happens automatically (see "Owned heap state" in the
    /// module docs) rather than through a hand-written `Cleanup_*` function
    /// or a custom SWIG destructor.
    #[test]
    fn owned_state_field_maps_to_nullable_void_ptr() {
        let info = info_for(
            "#[repr(C)] pub struct MyConfig { \
             pub runtime: bsk_messages::BskModuleRuntime, \
             pub state: Option<Box<MyState>>, \
             }",
        );
        let field = info
            .fields
            .iter()
            .find(|f| f.name == "state")
            .expect("state field must be detected");
        assert_eq!(field.c_type, "void *");
        assert!(!field.is_in_msg());
        assert!(!field.is_out_msg());
    }

    #[test]
    fn header_declares_and_defines_cpp_constructor_and_destructor() {
        let info = info_for(
            "#[repr(C)] pub struct MyConfig { \
             pub runtime: bsk_messages::BskModuleRuntime, \
             pub state: Option<Box<MyState>>, \
             }",
        );
        let header = render_c_header(&info, "myModule");
        // Struct must have a tag name (not anonymous) for the in-class declarations
        // to be legal C++.
        assert!(header.contains("typedef struct MyConfig {"));
        // Both declarations appear inside the struct body.
        assert!(header.contains("MyConfig();"));
        assert!(header.contains("~MyConfig();"));
        // Inline definitions appear after BSK_RUST_DECL.
        assert!(header.contains("extern \"C-unwind\" void Init_myModule(MyConfig* cfg);"));
        assert!(header.contains("inline MyConfig::MyConfig() { Init_myModule(this); }"));
        assert!(header.contains("inline MyConfig::~MyConfig() { Drop_myModule(this); }"));
        assert!(header.contains("void *state;"), "header:\n{header}");
        assert!(header.contains("BSK_RUST_DECL(myModule, MyConfig)"));
    }

    /// The generated `.i` file must `%import` each message type exactly
    /// once (even with two ports of the same type), `%immutable` each
    /// owned-state field, and instantiate `RustWrapper` with the config
    /// struct's real name — the whole point of generating this from `info`
    /// instead of a CMake-side regex scan (see `bsk_add_rust_module.cmake`).
    #[test]
    fn swig_interface_imports_dedup_and_marks_owned_state_immutable() {
        let info = info_for(
            "#[repr(C)] pub struct MyController { \
             pub runtime: bsk_messages::BskModuleRuntime, \
             pub state: Option<Box<MyState>>, \
             pub attGuidInMsg: MsgReader<AttGuidMsg>, \
             pub cmdTorqueOutMsg: MsgWriter<CmdTorqueBodyMsg>, \
             pub feedforwardTorqueInMsg: MsgReader<CmdTorqueBodyMsg>, \
             }",
        );
        let i_file = render_swig_interface(&info, "myModule", Path::new("/tmp/myModule.h"));

        assert!(i_file.contains("%module myModule"));
        assert!(i_file.contains("#include \"/tmp/myModule.h\""));
        assert!(i_file.contains(
            "%template(myModule) RustWrapper<MyController,Update_myModule,SelfInit_myModule,Reset_myModule>;"
        ));
        assert!(i_file.contains("%ignore Drop_myModule;"));
        assert!(
            i_file.contains("%import \"cMsgCInterface/CmdTorqueBodyMsg_C.h\""),
            "i_file:\n{i_file}"
        );
        // Only one %import per distinct message type, despite two ports.
        assert_eq!(i_file.matches("CmdTorqueBodyMsg_C.h").count(), 1);
        assert!(i_file.contains("%import \"cMsgCInterface/AttGuidMsg_C.h\""));
        assert!(i_file.contains("%immutable MyController::state;"));
    }

    #[test]
    fn shim_generates_init_and_drop_functions() {
        let info = info_for(
            "#[repr(C)] pub struct MyConfig { \
             pub runtime: bsk_messages::BskModuleRuntime, \
             pub state: Option<Box<MyState>>, \
             }",
        );
        let shim = render_shim(&info, "myModule");
        // Init_ calls BskModule::init() — the constructor hook.
        assert!(shim.contains("fn Init_myModule("), "Init_ must be in shim:\n{shim}");
        assert!(shim.contains("<MyConfig as BskModule>::init(&mut *cfg)"));
        // Drop_ calls drop_in_place for owned-state cleanup.
        assert!(shim.contains("fn Drop_myModule(cfg: *mut MyConfig)"));
        assert!(shim.contains("::core::ptr::drop_in_place(cfg);"));
    }

    #[test]
    fn self_init_shim_only_does_port_init_no_trait_call() {
        // Per Basilisk's cModules-3.rst: SelfInit should ONLY call Msg_C_init
        // on output ports. The shim handles this; there is no BskModule trait
        // method for SelfInit.
        let info = info_for(
            "#[repr(C)] pub struct MyController { \
             pub runtime: bsk_messages::BskModuleRuntime, \
             pub attGuidInMsg: MsgReader<AttGuidMsg>, \
             pub cmdTorqueOutMsg: MsgWriter<CmdTorqueBodyMsg>, \
             }",
        );
        let shim = render_shim(&info, "myModule");
        let self_init_fn = {
            let start = shim.find("fn SelfInit_myModule").expect("SelfInit_ must exist");
            let end = shim[start..].find("\n}\n").map(|i| start + i + 3).unwrap_or(shim.len());
            &shim[start..end]
        };
        // Must call .init() on the output port.
        assert!(
            self_init_fn.contains("(*cfg).cmdTorqueOutMsg.init()"),
            "SelfInit shim must init output ports:\n{self_init_fn}"
        );
        // Must NOT contain a BskModule trait method call — only BskModuleRuntime
        // appears (in the parameter type), never a `<Type as BskModule>::` dispatch.
        assert!(
            !self_init_fn.contains("as BskModule>"),
            "SelfInit shim must not call any BskModule trait method:\n{self_init_fn}"
        );
    }
    #[test]
    fn reset_shim_writes_all_outputs_like_update() {
        // The generated Reset_ shim must capture reset()'s return value and
        // write every output port — exactly as Update_ does — so output
        // messages are valid before the first UpdateState tick, and so the
        // type system forces the module author to return something for every
        // output rather than silently leaving a port uninitialised.
        let info = info_for(
            "#[repr(C)] pub struct MyController { \
             pub runtime: bsk_messages::BskModuleRuntime, \
             pub attGuidInMsg: MsgReader<AttGuidMsg>, \
             pub cmdTorqueOutMsg: MsgWriter<CmdTorqueBodyMsg>, \
             pub statusOutMsg: MsgWriter<DeviceCmdMsg>, \
             }",
        );
        let shim = render_shim(&info, "myModule");

        let reset_fn = {
            let start = shim.find("fn Reset_myModule").expect("Reset_ must exist");
            let end = shim[start..].find("\n}\n").map(|i| start + i + 3).unwrap_or(shim.len());
            &shim[start..end]
        };

        // reset() return value must be destructured
        assert!(
            reset_fn.contains("let (cmd_torque_out_msg, status_out_msg)"),
            "reset shim must destructure outputs:\n{reset_fn}"
        );
        // Each output must be written by the shim
        assert!(
            reset_fn.contains("(*cfg).cmdTorqueOutMsg.write(&cmd_torque_out_msg,"),
            "reset shim must write cmdTorqueOutMsg:\n{reset_fn}"
        );
        assert!(
            reset_fn.contains("(*cfg).statusOutMsg.write(&status_out_msg,"),
            "reset shim must write statusOutMsg:\n{reset_fn}"
        );
    }

    #[test]
    #[should_panic(expected = "`MyConfig.raw` is a bare `*mut`/`*const c_void` field")]
    fn bare_void_pointer_field_is_rejected() {
        // A bare `*mut c_void` (the old, unsafe pattern) is not the
        // `Option<Box<T>>` owned-state pattern, and must be rejected rather
        // than silently accepted as a plain pointer field — see "Owned heap
        // state" in the module docs. The panic must also name the
        // offending field so it's obvious which one to fix.
        info_for(
            "#[repr(C)] pub struct MyConfig { \
             pub runtime: bsk_messages::BskModuleRuntime, \
             pub raw: *mut core::ffi::c_void, \
             }",
        );
    }

    #[test]
    fn nested_repr_c_struct_field_is_generated_before_parent() {
        let tmp = std::env::temp_dir().join(format!(
            "bsk_build_test_nested_{}_{}",
            std::process::id(),
            line!()
        ));
        std::fs::create_dir_all(&tmp).unwrap();
        std::fs::write(
            tmp.join("types.rs"),
            "#[repr(C)] pub struct Vec2 { pub x: f64, pub y: f64 }",
        )
        .unwrap();

        let info = info_for_in(
            "#[repr(C)] pub struct MyConfig { \
             pub runtime: bsk_messages::BskModuleRuntime, \
             pub pos: Vec2, \
             }",
            &tmp,
        );

        let pos = info.fields.iter().find(|f| f.name == "pos").unwrap();
        assert_eq!(pos.c_type, "Vec2");

        assert_eq!(info.nested_structs.len(), 1);
        assert_eq!(info.nested_structs[0].name, "Vec2");
        assert_eq!(info.nested_structs[0].fields.len(), 2);

        let header = render_c_header(&info, "myModule");
        let vec2_decl = header.find("typedef struct Vec2").expect("Vec2 must be declared");
        let myconfig_decl = header
            .find("typedef struct MyConfig")
            .expect("MyConfig must be declared");
        assert!(
            vec2_decl < myconfig_decl,
            "Vec2 must be declared before MyConfig uses it by value:\n{header}"
        );

        std::fs::remove_dir_all(&tmp).ok();
    }

    #[test]
    fn fixed_size_array_of_primitives_is_supported() {
        let info = info_for(
            "#[repr(C)] pub struct MyConfig { \
             pub runtime: bsk_messages::BskModuleRuntime, \
             pub maxTorques: [f64; 3], \
             }",
        );

        let field = info.fields.iter().find(|f| f.name == "maxTorques").unwrap();
        // `c_type` is the scalar element type; dimensions are in `array_dims`.
        assert_eq!(field.c_type, "double");
        assert_eq!(field.array_dims, [3]);

        let header = render_c_header(&info, "myModule");
        assert!(
            header.contains("double maxTorques[3];"),
            "expected a C array field declaration, got:\n{header}"
        );
    }

    #[test]
    fn fixed_size_2d_array_of_primitives_is_supported() {
        // [[f64; 3]; 4] in Rust == double field[4][3] in C (outermost dim first).
        let info = info_for(
            "#[repr(C)] pub struct MyConfig { \
             pub runtime: bsk_messages::BskModuleRuntime, \
             pub dcm: [[f64; 3]; 3], \
             }",
        );

        let field = info.fields.iter().find(|f| f.name == "dcm").unwrap();
        assert_eq!(field.c_type, "double");
        assert_eq!(field.array_dims, [3, 3]);

        let header = render_c_header(&info, "myModule");
        assert!(
            header.contains("double dcm[3][3];"),
            "expected a 2D C array field declaration, got:\n{header}"
        );
    }

    #[test]
    fn fixed_size_array_of_nested_struct_is_supported() {
        let tmp = std::env::temp_dir().join(format!(
            "bsk_build_test_array_nested_{}_{}",
            std::process::id(),
            line!()
        ));
        std::fs::create_dir_all(&tmp).unwrap();
        std::fs::write(
            tmp.join("types.rs"),
            "#[repr(C)] pub struct Vec2 { pub x: f64, pub y: f64 }",
        )
        .unwrap();

        let info = info_for_in(
            "#[repr(C)] pub struct MyConfig { \
             pub runtime: bsk_messages::BskModuleRuntime, \
             pub corners: [Vec2; 4], \
             }",
            &tmp,
        );

        let field = info.fields.iter().find(|f| f.name == "corners").unwrap();
        assert_eq!(field.c_type, "Vec2");
        assert_eq!(field.array_dims, [4]);
        assert_eq!(info.nested_structs.len(), 1);

        let header = render_c_header(&info, "myModule");
        assert!(
            header.contains("Vec2 corners[4];"),
            "expected a C array-of-struct field declaration, got:\n{header}"
        );

        std::fs::remove_dir_all(&tmp).ok();
    }

    #[test]
    #[should_panic(expected = "`MyConfig.thing` has an array length that isn't a plain integer literal")]
    fn non_literal_array_length_is_rejected() {
        // `N` is written as a bare identifier in the array type — syn parses
        // this as `Expr::Path`, not a literal, so `array_len_from_expr`
        // returns `None` and we must emit the clear "not a plain integer
        // literal" error rather than an opaque codegen failure.
        info_for(
            "#[repr(C)] pub struct MyConfig { \
             pub runtime: bsk_messages::BskModuleRuntime, \
             pub thing: [f64; N], \
             }",
        );
    }

    #[test]
    #[should_panic(
        expected = "`MyConfig.thing` has type `SomeTypeThatDoesNotExistAnywhere`, which isn't a supported field type"
    )]
    fn unresolvable_bare_type_name_is_rejected() {
        // The panic must name the field (`MyConfig.thing`), not just the
        // unresolved type, so a mistake in one field of a large config
        // struct doesn't require guessing which one.
        info_for(
            "#[repr(C)] pub struct MyConfig { \
             pub runtime: bsk_messages::BskModuleRuntime, \
             pub thing: SomeTypeThatDoesNotExistAnywhere, \
             }",
        );
    }

    #[test]
    #[should_panic(expected = "Message ports are only meaningful")]
    fn msg_port_field_on_nested_struct_is_rejected() {
        // Leaving the tmp dir behind on panic is harmless (OS temp cleanup).
        let tmp = std::env::temp_dir().join(format!(
            "bsk_build_test_nested_msg_{}_{}",
            std::process::id(),
            line!()
        ));
        std::fs::create_dir_all(&tmp).unwrap();
        std::fs::write(
            tmp.join("types.rs"),
            "#[repr(C)] pub struct Bad { pub reading: bsk_messages::MsgReader<bsk_messages::AttGuidMsg> }",
        )
        .unwrap();

        info_for_in(
            "#[repr(C)] pub struct MyConfig { \
             pub runtime: bsk_messages::BskModuleRuntime, \
             pub bad: Bad, \
             }",
            &tmp,
        );
    }

    #[test]
    #[should_panic(expected = "`Inner.cPtr` is a raw pointer to `Deep`, which bsk-build doesn't allow")]
    fn raw_pointer_inside_a_nested_struct_is_also_rejected() {
        // The restriction in `raw_pointer_to_repr_c_struct_is_rejected` must
        // hold no matter how deep the pointer field is nested -- here it's
        // two levels down (MyConfig -> inner: Inner -> cPtr: *mut Deep), not
        // on the top-level config struct itself. `decompose_type_inner` has
        // no "is this the top-level struct" special case, so this should
        // already be caught by the same check; this test pins that down
        // rather than leaving it as an inference from the code.
        let tmp = std::env::temp_dir().join(format!(
            "bsk_build_test_nested_ptr_{}_{}",
            std::process::id(),
            line!()
        ));
        std::fs::create_dir_all(&tmp).unwrap();
        std::fs::write(
            tmp.join("types.rs"),
            "#[repr(C)] pub struct Deep { pub v: f64 } \
             #[repr(C)] pub struct Inner { pub cPtr: *mut Deep }",
        )
        .unwrap();

        info_for_in(
            "#[repr(C)] pub struct MyConfig { \
             pub runtime: bsk_messages::BskModuleRuntime, \
             pub inner: Inner, \
             }",
            &tmp,
        );
    }

    #[test]
    fn known_opaque_type_is_allowed_by_pointer_but_not_by_value() {
        // *mut BSKLogger (the real field shape used by every example
        // module) must keep working without needing BSKLogger's own
        // #[repr(C)] definition in the crate.
        let info = info_for(
            "#[repr(C)] pub struct MyConfig { \
             pub runtime: bsk_messages::BskModuleRuntime, \
             pub bskLogger: *mut BSKLogger, \
             }",
        );
        let field = info.fields.iter().find(|f| f.name == "bskLogger").unwrap();
        assert_eq!(field.c_type, "BSKLogger *");
    }

    #[test]
    fn raw_pointer_to_repr_c_struct_is_rejected() {
        // A raw pointer to a plain `#[repr(C)]` struct (as opposed to the
        // `BSKLogger`-style opaque-pointer allowlist) would get SWIG's
        // default ownership-transferring pointer setter with no
        // corresponding deallocation on the Rust side — a leak at best.
        // The struct should be embedded by value instead (see "Nested
        // structs"), so this must be rejected rather than silently
        // resolving `Other` as a nested struct.
        let tmp = std::env::temp_dir().join(format!(
            "bsk_build_test_ptr_to_struct_{}_{}",
            std::process::id(),
            line!()
        ));
        std::fs::create_dir_all(&tmp).unwrap();
        std::fs::write(
            tmp.join("types.rs"),
            "#[repr(C)] pub struct Other { pub v: f64 }",
        )
        .unwrap();

        let diagnostics = diagnostics_for_in(
            "#[repr(C)] pub struct MyConfig { \
             pub runtime: bsk_messages::BskModuleRuntime, \
             pub otherPtr: *mut Other, \
             }",
            &tmp,
        );
        assert_eq!(diagnostics.len(), 1);
        assert_eq!(diagnostics[0].context, "MyConfig.otherPtr");
        assert!(diagnostics[0]
            .message
            .starts_with("is a raw pointer to `Other`, which bsk-build doesn't allow"));
        std::fs::remove_dir_all(tmp).unwrap();
    }

    #[test]
    fn raw_pointer_to_primitive_is_rejected() {
        // A pointee that resolves via `primitive_to_c` (e.g. `u8`) must not
        // skip the same ownership-transfer check applied to struct pointees
        // (`raw_pointer_to_repr_c_struct_is_rejected`) — SWIG generates the
        // identical `SWIG_POINTER_DISOWN` setter for `uint8_t *` as it does
        // for a pointer to a wrapped struct, so a primitive pointee isn't a
        // safer case and must be checked before the primitive lookup, not
        // after.
        let diagnostics = diagnostics_for(
            "#[repr(C)] pub struct MyConfig { \
             pub runtime: bsk_messages::BskModuleRuntime, \
             pub raw: *mut u8, \
             }",
        );
        assert_eq!(diagnostics.len(), 1);
        assert_eq!(diagnostics[0].context, "MyConfig.raw");
        assert!(diagnostics[0]
            .message
            .starts_with("is a raw pointer to `u8`, which bsk-build doesn't allow"));
    }

    #[test]
    fn enum_field_is_rejected_with_enum_specific_message() {
        // A `#[repr(u8)]` enum has the same layout as `u8`, but SWIG's
        // default setter for an enum-typed field accepts any integer, not
        // just ones matching a declared variant — so this must be rejected
        // with an explanation distinct from "not a supported type" (which
        // would incorrectly suggest the fix is `#[repr(C)]`, as if `Mode`
        // were meant to be a nested struct).
        let tmp = std::env::temp_dir().join(format!(
            "bsk_build_test_enum_field_{}_{}",
            std::process::id(),
            line!()
        ));
        std::fs::create_dir_all(&tmp).unwrap();
        std::fs::write(
            tmp.join("types.rs"),
            "#[repr(u8)] pub enum Mode { Idle = 0, Active = 1 }",
        )
        .unwrap();

        let diagnostics = diagnostics_for_in(
            "#[repr(C)] pub struct MyConfig { \
             pub runtime: bsk_messages::BskModuleRuntime, \
             pub mode: Mode, \
             }",
            &tmp,
        );
        assert_eq!(diagnostics.len(), 1);
        assert_eq!(diagnostics[0].context, "MyConfig.mode");
        assert!(diagnostics[0]
            .message
            .starts_with("has type `Mode`, a Rust `enum`"));
        std::fs::remove_dir_all(tmp).unwrap();
    }

    #[test]
    fn known_opaque_type_by_value_is_rejected() {
        // BSKLogger's full layout isn't known to this crate, so it can only
        // be used by pointer, never by value.
        let diagnostics = diagnostics_for(
            "#[repr(C)] pub struct MyConfig { \
             pub runtime: bsk_messages::BskModuleRuntime, \
             pub logger: BSKLogger, \
             }",
        );
        assert_eq!(diagnostics.len(), 1);
        assert_eq!(diagnostics[0].context, "MyConfig.logger");
        assert!(diagnostics[0]
            .message
            .starts_with("has type `BSKLogger`, which isn't a supported field type"));
    }

    #[test]
    fn unsupported_field_type_error_uses_rust_syntax_not_debug_dump() {
        // The error must render the type the way it appears in source
        // (`& 'static str`), not `syn`'s derived `Debug` output — a wall of
        // `Type::Reference { and_token: ... }` is not something a module
        // author should have to decode.
        let diagnostics = diagnostics_for(
            "#[repr(C)] pub struct MyConfig { \
             pub runtime: bsk_messages::BskModuleRuntime, \
             pub weird: &'static str, \
             }",
        );
        assert_eq!(diagnostics.len(), 1);
        assert_eq!(diagnostics[0].context, "MyConfig.weird");
        assert!(diagnostics[0]
            .message
            .starts_with("has type `& 'static str`, which isn't a supported field type"));
    }

    fn types(strs: &[&str]) -> Vec<Type> {
        strs.iter()
            .map(|s| syn::parse_str::<Type>(s).expect("test type must parse"))
            .collect()
    }

    #[test]
    fn input_field_count_mismatch_names_fields_and_types() {
        // When the `*InMsg` field count and the `Inputs` tuple length
        // disagree, the message must list both sides by name so the
        // mismatch is obvious without cross-referencing the struct
        // definition and the `impl BskModule` block by hand.
        let mut info = info_for(
            "#[repr(C)] pub struct MyConfig { \
             pub runtime: bsk_messages::BskModuleRuntime, \
             pub fooInMsg: bsk_messages::MsgReader<bsk_messages::AttGuidMsg>, \
             }",
        );
        let mut diagnostics = Vec::new();
        apply_input_optionality(&mut info, &types(&["FooMsg", "BarMsg"]), &mut diagnostics);
        assert_eq!(diagnostics.len(), 1);
        assert!(diagnostics[0].message.contains(
            "has 1 `MsgReader<T>` field(s) (fooInMsg) but `impl BskModule for MyConfig`'s \
             `type Inputs` tuple has 2 element(s) (FooMsg, BarMsg)"
        ));
    }
}
