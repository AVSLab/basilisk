//  ISC License
//
//  Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
//
//  Permission to use, copy, modify, and/or distribute this software for any
//  purpose with or without fee is hereby granted, provided that the above
//  copyright notice and this permission notice appear in all copies.

//! Shared data model produced by discovery (`discovery.rs`, `methods.rs`,
//! `optionality.rs`) and consumed by the renderers (`render_header.rs`,
//! `render_shim.rs`, `render_swig.rs`).

use super::discovery::SourceAsts;

// ---------------------------------------------------------------------------
// Config struct descriptor
// ---------------------------------------------------------------------------

pub(super) struct ConfigInfo {
    /// Exact Rust identifier of the `#[repr(C)]` struct implementing
    /// `BskModule` — whatever the user named it, e.g. "mrpRustControllerConfig".
    pub(super) struct_name: String,
    pub(super) fields: Vec<FieldInfo>,
    /// `#[repr(C)]` struct types referenced (directly or transitively) by a
    /// by-value struct field, in dependency order — each entry's own fields
    /// only reference entries earlier in this list (or the config struct's
    /// fields), so emitting them in this order satisfies C's
    /// declare-before-use rule.
    pub(super) nested_structs: Vec<NestedStructInfo>,
    /// `pub fn` methods on `impl <ConfigType> { ... }` (not `impl BskModule`)
    /// with a C/SWIG-compatible signature — see `methods::find_config_methods`.
    pub(super) methods: Vec<MethodInfo>,
}

/// A `#[repr(C)]` struct discovered through a by-value struct field
/// (see "Nested structs" in the module docs). Same field shape as the
/// config struct, minus the parts that only make sense once, at the top
/// level (`runtime`, message ports, owned heap state).
pub(super) struct NestedStructInfo {
    pub(super) name: String,
    pub(super) fields: Vec<FieldInfo>,
}

/// Threaded through field extraction so nested struct discovery can search
/// the crate source and detect by-value reference cycles (which C cannot
/// represent — a struct can't contain itself by value).
pub(super) struct NestedCtx<'a> {
    pub(super) source_asts: &'a SourceAsts,
    pub(super) diagnostics: Vec<Diagnostic>,
    /// Structs fully resolved so far, in dependency order (see
    /// `ConfigInfo::nested_structs`).
    pub(super) nested: Vec<NestedStructInfo>,
    /// Struct names currently being resolved (a stack), for cycle detection.
    pub(super) in_progress: Vec<String>,
}

#[derive(Clone, Debug)]
pub(super) struct Diagnostic {
    pub(super) context: String,
    pub(super) message: String,
}

impl NestedCtx<'_> {
    pub(super) fn error(&mut self, context: impl Into<String>, message: impl Into<String>) {
        self.diagnostics.push(Diagnostic {
            context: context.into(),
            message: message.into(),
        });
    }
}

pub(super) fn panic_with_diagnostics(mut diagnostics: Vec<Diagnostic>) -> ! {
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
pub(super) enum MsgKind {
    Reader,
    Writer,
}

pub(super) struct FieldInfo {
    pub(super) name: String, // "attGuidInMsg"
    /// C type string: "double", "AttGuidMsg_C" (for a message field), "BSKLogger *"
    pub(super) c_type: String,
    /// Raw Rust path last-segment string, e.g. "f64", "BskModuleRuntime".
    /// Empty for `MsgReader`/`MsgWriter` fields — use `msg_kind`/`msg_type`.
    pub(super) rust_base_type: String,
    /// `Some((Reader|Writer, "AttGuidMsg"))` for `MsgReader<AttGuidMsg>` /
    /// `MsgWriter<AttGuidMsg>` fields; `None` for everything else.
    pub(super) msg: Option<(MsgKind, String)>,
    pub(super) doc: String,
    pub(super) is_optional: bool,
    /// `Option<Box<T>>` owned heap state (see "Owned heap state" in the
    /// module docs). Its own field, rather than inferring it from `c_type
    /// == "void *"` at call sites, so `render_manifest` (and anything else
    /// that needs it) has one unambiguous source of truth.
    pub(super) is_owned_state: bool,
    /// Non-empty for fixed-size array fields (`[T; N]`, `[[T; N]; M]`, …).
    /// Holds the dimensions outermost-first in C order — so Rust `[[f64; 3]; 4]`
    /// maps to C `double field[4][3]` and `array_dims = [4, 3]`. `c_type`
    /// is always the *scalar* element type (`"double"`, not `"double[3]"`);
    /// `render_header::render_field_decls` emits the dimension suffixes
    /// after the name.
    /// Empty for every non-array field kind.
    pub(super) array_dims: Vec<usize>,
}

impl FieldInfo {
    pub(super) fn is_in_msg(&self) -> bool {
        matches!(&self.msg, Some((MsgKind::Reader, _)))
    }
    pub(super) fn is_out_msg(&self) -> bool {
        matches!(&self.msg, Some((MsgKind::Writer, _)))
    }
    /// The mandatory `SysModel` runtime mirror field (see module docs).
    pub(super) fn is_runtime(&self) -> bool {
        self.name == "runtime" && self.rust_base_type == "BskModuleRuntime"
    }
}

// ---------------------------------------------------------------------------
// Config methods — see `methods.rs` for how these are discovered
// ---------------------------------------------------------------------------

pub(super) struct MethodParam {
    pub(super) name: String,
    /// Original Rust element type, exactly as written (e.g. `"f64"`,
    /// `"Vec2"`) — used to reconstruct valid Rust syntax for the generated
    /// shim, which calls the *real* method with its *real* argument types.
    pub(super) rust_base_type: String,
    /// C element type (e.g. `"double"`, `"Vec2"`) — used for the C++ header
    /// declaration. Scalar even for an array parameter/return; see
    /// `array_dims`, matching `FieldInfo`'s convention.
    pub(super) c_type: String,
    /// Empty for a scalar parameter/return; `[outer, ..., inner]` for a
    /// fixed-size array, matching `FieldInfo::array_dims`.
    pub(super) array_dims: Vec<usize>,
}

pub(super) struct MethodInfo {
    /// Exact Rust identifier — used verbatim as the C++ member function
    /// name (and therefore the Python-visible name via SWIG's smart-pointer
    /// forwarding). No case conversion: module authors are expected to name
    /// these to match the sibling C++ module already, the same way message
    /// port and config field names are kept as-is rather than
    /// snake_cased (see the module docs' naming guidance).
    pub(super) name: String,
    /// `&mut self` (true) or `&self` (false); selects whether the generated
    /// C++ member function is `const`.
    pub(super) is_mut: bool,
    pub(super) params: Vec<MethodParam>,
    /// `None` for a `()`-returning method (most setters); `Some` for a
    /// getter. Never has a non-empty `array_dims` — see `methods.rs`'s
    /// module-level comment for why arrays can't be returned by value.
    pub(super) ret: Option<MethodParam>,
    pub(super) doc: String,
}
