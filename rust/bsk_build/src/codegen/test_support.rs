//  ISC License
//
//  Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
//
//  Permission to use, copy, modify, and/or distribute this software for any
//  purpose with or without fee is hereby granted, provided that the above
//  copyright notice and this permission notice appear in all copies.

//! Test-only helpers shared by every `codegen` submodule's `#[cfg(test)]
//! mod tests`, so each one can build a [`ConfigInfo`] (or collect
//! diagnostics) from an inline struct definition without repeating the
//! `SourceAsts`/`NestedCtx` plumbing `generate()` itself uses.

#![cfg(test)]

use std::path::{Path, PathBuf};

use syn::{ItemStruct, Type};

use super::discovery::SourceAsts;
use super::types::{ConfigInfo, Diagnostic, NestedCtx};

/// No test here defines a nested struct in a *separate* file, so this
/// placeholder directory is never actually read; see
/// `render_header::tests::nested_repr_c_struct_field_is_generated_before_parent`
/// for a test that sets up a real one.
pub(super) fn empty_src_dir() -> PathBuf {
    PathBuf::from("/nonexistent-bsk-build-test-src-dir")
}

pub(super) fn info_for(struct_src: &str) -> ConfigInfo {
    info_for_in(struct_src, &empty_src_dir())
}

pub(super) fn info_for_in(struct_src: &str, src_dir: &Path) -> ConfigInfo {
    let item: ItemStruct = syn::parse_str(struct_src).expect("test struct must parse");
    let struct_name = item.ident.to_string();
    let source_asts = SourceAsts::load(src_dir);
    let mut ctx = NestedCtx {
        source_asts: &source_asts,
        diagnostics: source_asts.diagnostics.clone(),
        nested: Vec::new(),
        in_progress: vec![struct_name.clone()],
    };
    let fields = super::discovery::extract_fields(&item, &mut ctx, true);
    if !ctx.diagnostics.is_empty() {
        super::types::panic_with_diagnostics(ctx.diagnostics);
    }
    ConfigInfo {
        struct_name,
        fields,
        nested_structs: ctx.nested,
        methods: Vec::new(),
    }
}

pub(super) fn diagnostics_for(struct_src: &str) -> Vec<Diagnostic> {
    diagnostics_for_in(struct_src, &empty_src_dir())
}

pub(super) fn diagnostics_for_in(struct_src: &str, src_dir: &Path) -> Vec<Diagnostic> {
    let item: ItemStruct = syn::parse_str(struct_src).expect("test struct must parse");
    let source_asts = SourceAsts::load(src_dir);
    let mut ctx = NestedCtx {
        source_asts: &source_asts,
        diagnostics: Vec::new(),
        nested: Vec::new(),
        in_progress: vec![item.ident.to_string()],
    };
    super::discovery::extract_fields(&item, &mut ctx, true);
    ctx.diagnostics
}

/// Parses a list of type strings, e.g. `types(&["FooMsg", "BarMsg"])`, for
/// tests that need `syn::Type` values (matching an `Inputs` tuple) without
/// parsing a whole struct.
pub(super) fn types(strs: &[&str]) -> Vec<Type> {
    strs.iter()
        .map(|s| syn::parse_str::<Type>(s).expect("test type must parse"))
        .collect()
}
