//  ISC License
//
//  Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
//
//  Permission to use, copy, modify, and/or distribute this software for any
//  purpose with or without fee is hereby granted, provided that the above
//  copyright notice and this permission notice appear in all copies.

//! syn-based discovery: parses a module crate's source tree, finds its
//! `#[bsk_build::module]` config (or a legacy `impl BskModule` fallback), and
//! resolves each field's Rust type to a C ABI representation (primitives,
//! message ports, owned heap state, fixed-size arrays, and nested
//! `#[repr(C)]` structs).

use std::path::Path;

use syn::{
    Attribute, Expr, ExprLit, Fields, File, ImplItem, Item, ItemStruct, Lit, Meta, Type,
    TypeArray, TypePath, TypePtr,
};
use walkdir::WalkDir;

use super::types::{Diagnostic, FieldInfo, MsgKind, NestedCtx, NestedStructInfo};

// Needed for `.to_token_stream()` — used both to detect `#[repr(C)]` and to
// render unsupported types as normal Rust syntax in error messages, instead
// of `syn`'s much noisier derived `Debug` output.
use syn::__private::ToTokens;

/// Parsed ASTs for every Rust source file in a module crate.
///
/// Discovery and nested-type resolution share this cache so a build script
/// parses each source file exactly once.
pub(super) struct SourceAsts {
    pub(super) files: Vec<File>,
    pub(super) diagnostics: Vec<Diagnostic>,
}

impl SourceAsts {
    pub(super) fn load(source_path: &Path) -> Self {
        let mut files = Vec::new();
        let mut diagnostics = Vec::new();
        for entry in WalkDir::new(source_path)
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
    pub(super) fn find_named_item(&self, name: &str) -> Option<&Item> {
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

/// Returns the names of structs explicitly marked as Basilisk module configs.
pub(super) fn find_marked_module_configs(source_asts: &SourceAsts) -> Vec<String> {
    source_asts
        .files
        .iter()
        .flat_map(|file| file.items.iter())
        .filter_map(|item| match item {
            Item::Struct(item) if has_bsk_module_attribute(item) => {
                Some(item.ident.to_string())
            }
            _ => None,
        })
        .collect()
}

fn has_bsk_module_attribute(item: &ItemStruct) -> bool {
    item.attrs.iter().any(|attribute| {
        attribute
            .path()
            .segments
            .last()
            .is_some_and(|segment| segment.ident == "module")
    })
}

/// Searches cached source ASTs for the first
/// `impl BskModule for <Type> { type Inputs = (...); ... }`
/// and returns `(struct_name, input_types)`. `input_types` is the `Inputs`
/// tuple's element types in order (empty for `type Inputs = ();`).
pub(super) fn find_bsk_module_impl(source_asts: &SourceAsts) -> Option<(String, Vec<Type>)> {
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
                .is_some_and(|seg| seg.ident == "BskModule");
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
                        input_types = super::optionality::tuple_elems(&assoc.ty);
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
pub(super) enum FindStructError {
    MissingReprC,
}

/// Searches cached source ASTs for a `#[repr(C)]` struct with the exact
/// identifier `name`.
///
/// Returns [`FindStructError::MissingReprC`] when a same-named struct exists
/// but lacks `#[repr(C)]`, allowing callers to tell that user mistake apart
/// from a missing or misspelled type.
pub(super) fn find_struct_by_name<'a>(
    source_asts: &'a SourceAsts,
    name: &str,
) -> Result<Option<&'a ItemStruct>, FindStructError> {
    match source_asts.find_named_item(name) {
        Some(Item::Struct(item)) if has_repr_c(item) => Ok(Some(item)),
        Some(Item::Struct(_)) => Err(FindStructError::MissingReprC),
        _ => Ok(None),
    }
}

pub(super) fn has_repr_c(s: &ItemStruct) -> bool {
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
pub(super) fn extract_fields(
    s: &ItemStruct,
    ctx: &mut NestedCtx,
    allow_module_fields: bool,
) -> Vec<FieldInfo> {
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
            // `render_header::render_c_header`) runs Rust's normal drop
            // glue to free it.
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
            // Filled in by `optionality::apply_input_optionality` from the
            // `impl BskModule` block's `Inputs` tuple types — see module docs.
            is_optional: false,
            is_owned_state: false,
            array_dims,
        });
    }
    result
}

/// Recognizes a `MsgReader<Foo>` / `MsgWriter<Foo>` field type and returns
/// `(kind, "Foo")`.
pub(super) fn msg_port_info(ty: &Type) -> Option<(MsgKind, String)> {
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
pub(super) fn is_option_box(ty: &Type) -> bool {
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

/// Returns (rust_base_type, c_type) for a non-message, non-owned-state field
/// (those are handled separately by `msg_port_info`/`is_option_box`, above).
/// A pointer `c_type` (e.g. `"BSKLogger *"`) always ends in `*`, which
/// `render_header::render_c_header` uses to decide field-declaration spacing.
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
pub(super) fn decompose_type(
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
pub(super) fn array_len_from_expr(len: &Expr) -> Option<usize> {
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
                 pointers, or a `#[repr(C)]` struct defined in the configured \
                 Rust source path.\n\
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
/// FFI type: it must be a `#[repr(C)]` struct defined in the configured
/// Rust source path (see "Nested structs" in the module docs). Recursively extracts
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
             another `#[repr(C)]` struct defined in the configured Rust source path.\n\
             `{name}` matched none of these. If you meant to reference a \
             struct defined elsewhere in `{field_ctx}`'s crate, check that \
             it's spelled correctly and annotated `#[repr(C)]` — bsk-build \
             only looks for `#[repr(C)]` structs in the configured Rust source path."
                ),
            );
            return None;
        }
        Err(FindStructError::MissingReprC) => {
            ctx.error(
                field_ctx,
                format!(
                    "has type `{name}`, a struct defined in the configured \
             Rust source path, but `{name}` is missing `#[repr(C)]`.\n\
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
    let fields = extract_fields(item, ctx, false);
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
pub(super) const OPAQUE_POINTER_TARGETS: &[&str] = &["BSKLogger"];

pub(super) fn known_opaque_pointer_target(name: &str) -> bool {
    OPAQUE_POINTER_TARGETS.contains(&name)
}

/// `bsk-messages` types whose Rust name intentionally differs from their C
/// counterpart (`bsk_rust_module.h` calls it `BskRustModuleRuntime` — the
/// `Rust` disambiguates it as a Rust-module-only ABI detail, unlike
/// `BSKLogger` which is a shared BSK type and keeps its name unchanged).
pub(super) fn special_type_to_c(rust: &str) -> Option<&'static str> {
    Some(match rust {
        "BskModuleRuntime" => "BskRustModuleRuntime",
        _ => return None,
    })
}

pub(super) fn primitive_to_c(rust: &str) -> Option<&'static str> {
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

/// Last path segment of a type, e.g. `BskModuleRuntime` for both
/// `BskModuleRuntime` and `bsk_messages::BskModuleRuntime`.
pub(super) fn type_last_ident(ty: &Type) -> Option<String> {
    if let Type::Path(TypePath { path, .. }) = ty {
        path.segments.last().map(|s| s.ident.to_string())
    } else {
        None
    }
}

/// Collect all `///` doc-comment lines on a field into one string.
pub(super) fn collect_doc(attrs: &[Attribute]) -> String {
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::codegen::test_support::*;

    #[test]
    fn qualified_attribute_marks_the_module_config() {
        let source_asts = SourceAsts {
            files: vec![syn::parse_file(
                "#[bsk_build::module] #[repr(C)] \
                 pub struct ControllerConfig { \
                 pub runtime: BskModuleRuntime }",
            )
            .expect("test source must parse")],
            diagnostics: Vec::new(),
        };

        assert_eq!(
            find_marked_module_configs(&source_asts),
            vec!["ControllerConfig"]
        );
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
    fn source_ast_cache_accepts_a_single_source_file() {
        let test_file = std::env::temp_dir().join(format!(
            "bsk-build-single-source-{}.rs",
            std::process::id()
        ));
        std::fs::write(
            &test_file,
            "#[repr(C)] pub struct SingleSourceConfig { pub value: f64 }",
        )
        .expect("test source must be written");

        let source_asts = SourceAsts::load(&test_file);
        assert_eq!(source_asts.files.len(), 1);
        assert!(source_asts.diagnostics.is_empty());

        std::fs::remove_file(test_file).expect("test source file must be removed");
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
    /// the generated `Delete_<module>` function runs Rust drop glue and
    /// returns the config allocation to Rust automatically (see "Owned heap
    /// state" in the module docs).
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
}
