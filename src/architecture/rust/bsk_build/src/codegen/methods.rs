//  ISC License
//
//  Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
//
//  Permission to use, copy, modify, and/or distribute this software for any
//  purpose with or without fee is hereby granted, provided that the above
//  copyright notice and this permission notice appear in all copies.

//! Config methods — `impl <ConfigType> { pub fn ... }` (not `impl BskModule`)
//!
//! A hand-written C++ module often exposes a `setFoo`/`getFoo` pair instead of
//! a plain public field when the setter needs to validate or convert the
//! value (e.g. rejecting a non-positive gain). Rust modules get the same
//! capability: any `pub fn` in a plain `impl <ConfigType> { ... }` block
//! (found alongside, but distinct from, the `impl BskModule for <ConfigType>`
//! block) with a C/SWIG-compatible signature is exposed as a same-named
//! public C++ member function on the generated config struct, which SWIG's
//! smart-pointer support (`RustWrapper::operator->()`) then forwards to
//! Python automatically — no `%extend` needed, and no separate Python-visible
//! name to keep in sync with the Rust one.
//!
//! A signature is compatible when:
//! * the receiver is `&self` or `&mut self` (not by-value `self`);
//! * it has no generic parameters;
//! * every non-`self` parameter is a simple identifier pattern with a
//!   primitive, fixed-size-array, or (already-registered) nested `#[repr(C)]`
//!   struct type — the same type vocabulary `discovery::decompose_type`
//!   accepts for fields, minus message ports and owned state, which aren't
//!   meaningful as a bare parameter;
//! * the return type is `()`, a primitive, or a nested struct — not a
//!   fixed-size array, which C/C++ cannot return by value (unlike a
//!   parameter, which decays to a pointer instead of actually being passed
//!   by value, arrays have no by-value return convention at all).
//!
//! An incompatible method is *skipped* (not a hard build error): it just
//! isn't reachable from Python this way, and `generate()` prints a
//! `cargo:warning=` explaining why so the mismatch doesn't go unnoticed
//! silently. Mark a method `#[doc(hidden)]` to suppress the warning for a
//! method that's intentionally Rust-only (e.g. a helper only ever called
//! from `update`/`reset`, or from this crate's own `#[cfg(test)]` code).

use syn::{Attribute, FnArg, ImplItem, Item, Meta, Pat, ReturnType, Type, TypeArray, TypePath, Visibility};

use super::discovery::{array_len_from_expr, collect_doc, primitive_to_c, special_type_to_c, type_last_ident, SourceAsts};
use super::types::{MethodInfo, MethodParam};

// Needed for `.to_token_stream()` in the diagnostic messages below.
use syn::__private::ToTokens;

/// Reserved shim symbol prefixes `bsk-build` itself generates
/// (`New_<module>`, `Delete_<module>`, `SelfInit_<module>`, ...) — a config method literally
/// named one of these would collide with them once suffixed with the same
/// `_<module>`, so it's rejected same as any other incompatible signature.
const RESERVED_METHOD_NAMES: &[&str] = &["New", "Delete", "SelfInit", "Reset", "Update"];

/// Scans `source_asts` for a plain `impl <struct_name> { ... }` block (any
/// number of them) and returns every `pub fn` with a C/SWIG-compatible
/// signature, plus a human-readable warning for each `pub fn` that was
/// found but skipped (see the module-level comment above for the exact
/// rules). `nested_names` is the set of nested `#[repr(C)]` struct names
/// already registered via field discovery — the only struct types a method
/// parameter/return may reference by value.
pub(super) fn find_config_methods(
    source_asts: &SourceAsts,
    struct_name: &str,
    nested_names: &[String],
) -> (Vec<MethodInfo>, Vec<String>) {
    let mut methods = Vec::new();
    let mut warnings = Vec::new();

    for ast in &source_asts.files {
        for item in &ast.items {
            let Item::Impl(imp) = item else { continue };
            // Only a plain `impl <struct_name> { ... }` — `impl BskModule
            // for <struct_name>` (and any other trait impl) is out of scope;
            // those methods belong to the trait, not this ad hoc surface.
            if imp.trait_.is_some() {
                continue;
            }
            if type_last_ident(&imp.self_ty).as_deref() != Some(struct_name) {
                continue;
            }
            for impl_item in &imp.items {
                let ImplItem::Fn(f) = impl_item else { continue };
                if !matches!(f.vis, Visibility::Public(_)) {
                    continue;
                }
                if has_doc_hidden(&f.attrs) {
                    continue;
                }
                let method_name = f.sig.ident.to_string();

                let mut inputs = f.sig.inputs.iter();
                let is_mut = match inputs.next() {
                    Some(FnArg::Receiver(r)) if r.reference.is_some() => r.mutability.is_some(),
                    Some(FnArg::Receiver(_)) => {
                        warnings.push(format!(
                            "bsk-build: `{struct_name}::{method_name}` takes `self` by value; \
                             only `&self`/`&mut self` methods can be exposed to C/SWIG. Skipping \
                             this method (mark it `#[doc(hidden)]` to silence this warning)."
                        ));
                        continue;
                    }
                    // No receiver at all — an associated function (e.g. a
                    // constructor-style helper), not an instance method.
                    // Not a candidate; not a mistake either, so no warning.
                    _ => continue,
                };

                if !f.sig.generics.params.is_empty() {
                    warnings.push(format!(
                        "bsk-build: `{struct_name}::{method_name}` is generic; bsk-build cannot \
                         generate a fixed C ABI for a generic function. Skipping this method \
                         (mark it `#[doc(hidden)]` to silence this warning)."
                    ));
                    continue;
                }
                if RESERVED_METHOD_NAMES.contains(&method_name.as_str()) {
                    warnings.push(format!(
                        "bsk-build: `{struct_name}::{method_name}` has the same name as a \
                         lifecycle shim function bsk-build itself generates. Skipping this \
                         method — rename it (mark it `#[doc(hidden)]` if it's intentionally \
                         Rust-only)."
                    ));
                    continue;
                }

                let mut params = Vec::new();
                let mut incompatible = false;
                for arg in inputs {
                    let FnArg::Typed(pat_type) = arg else { continue };
                    let Pat::Ident(pat_ident) = pat_type.pat.as_ref() else {
                        warnings.push(format!(
                            "bsk-build: `{struct_name}::{method_name}` has a parameter pattern \
                             bsk-build doesn't support (only a plain `name: Type` parameter is \
                             allowed). Skipping this method (mark it `#[doc(hidden)]` to \
                             silence this warning)."
                        ));
                        incompatible = true;
                        break;
                    };
                    let param_name = pat_ident.ident.to_string();
                    let Some((rust_base_type, c_type, array_dims)) =
                        try_resolve_simple_type(&pat_type.ty, nested_names)
                    else {
                        warnings.push(format!(
                            "bsk-build: `{struct_name}::{method_name}` has parameter `{param_name}` \
                             of type `{}`, which bsk-build cannot expose to C/SWIG (supported: \
                             primitives, fixed-size arrays, or an already-registered nested \
                             `#[repr(C)]` struct). Skipping this method (mark it `#[doc(hidden)]` \
                             to silence this warning).",
                            pat_type.ty.to_token_stream(),
                        ));
                        incompatible = true;
                        break;
                    };
                    params.push(MethodParam {
                        name: param_name,
                        rust_base_type,
                        c_type,
                        array_dims,
                    });
                }
                if incompatible {
                    continue;
                }

                let ret = match &f.sig.output {
                    ReturnType::Default => None,
                    ReturnType::Type(_, ty) => {
                        let Some((rust_base_type, c_type, array_dims)) =
                            try_resolve_simple_type(ty, nested_names)
                        else {
                            warnings.push(format!(
                                "bsk-build: `{struct_name}::{method_name}` returns `{}`, which \
                                 bsk-build cannot expose to C/SWIG (supported: `()`, a \
                                 primitive, or an already-registered nested `#[repr(C)]` \
                                 struct). Skipping this method (mark it `#[doc(hidden)]` to \
                                 silence this warning).",
                                ty.to_token_stream(),
                            ));
                            continue;
                        };
                        if !array_dims.is_empty() {
                            warnings.push(format!(
                                "bsk-build: `{struct_name}::{method_name}` returns a fixed-size \
                                 array (`{}`); C/C++ cannot return an array by value (only take \
                                 one as a parameter, which decays to a pointer instead). \
                                 Skipping this method (mark it `#[doc(hidden)]` to silence this \
                                 warning).",
                                ty.to_token_stream(),
                            ));
                            continue;
                        }
                        Some(MethodParam {
                            name: String::new(),
                            rust_base_type,
                            c_type,
                            array_dims,
                        })
                    }
                };

                methods.push(MethodInfo {
                    name: method_name,
                    is_mut,
                    params,
                    ret,
                    doc: collect_doc(&f.attrs),
                });
            }
        }
    }

    (methods, warnings)
}

fn has_doc_hidden(attrs: &[Attribute]) -> bool {
    attrs.iter().any(|attr| {
        if !attr.path().is_ident("doc") {
            return false;
        }
        matches!(&attr.meta, Meta::List(list) if list.tokens.to_string() == "hidden")
    })
}

/// A lighter-weight, non-diagnostic-emitting relative of
/// `discovery::decompose_type` for method parameter/return types: peels
/// `[T; N]` layers (bailing on a non-literal length, same as
/// `decompose_type`), then accepts a primitive, a `bsk-messages` special
/// type, or a name already in `nested_names` — and returns `None` for
/// anything else (pointers, `MsgReader`/`MsgWriter`, `Option<..>`,
/// generics, ...) instead of pushing a build-breaking diagnostic, since an
/// incompatible method signature is a warning here, not an error.
fn try_resolve_simple_type(
    ty: &Type,
    nested_names: &[String],
) -> Option<(String, String, Vec<usize>)> {
    let mut dims: Vec<usize> = Vec::new();
    let mut current = ty;
    loop {
        match current {
            Type::Array(TypeArray { elem, len, .. }) => {
                let n = array_len_from_expr(len)?;
                dims.push(n);
                current = elem;
            }
            Type::Path(TypePath { path, .. }) => {
                let last = path.segments.last()?.ident.to_string();
                if let Some(c) = primitive_to_c(&last).or_else(|| special_type_to_c(&last)) {
                    return Some((last, c.to_owned(), dims));
                }
                if nested_names.iter().any(|n| n == &last) {
                    return Some((last.clone(), last, dims));
                }
                return None;
            }
            _ => return None,
        }
    }
}

/// Builds the Rust array-type syntax for `array_dims`/`base` (e.g.
/// `base = "f64"`, `dims = [4, 3]` -> `"[[f64; 3]; 4]"`), matching the
/// `FieldInfo::array_dims` convention (outermost dimension first). Returns
/// `base` unchanged for a scalar (`dims` empty).
pub(super) fn rust_array_type(base: &str, dims: &[usize]) -> String {
    let mut s = base.to_owned();
    for &n in dims.iter().rev() {
        s = format!("[{s}; {n}]");
    }
    s
}
