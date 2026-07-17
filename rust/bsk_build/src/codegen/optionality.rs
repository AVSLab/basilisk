//  ISC License
//
//  Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
//
//  Permission to use, copy, modify, and/or distribute this software for any
//  purpose with or without fee is hereby granted, provided that the above
//  copyright notice and this permission notice appear in all copies.

//! Input optionality — derived from `impl BskModule`'s `Inputs` tuple types.

use syn::{Type, TypePath};

use super::discovery::type_last_ident;
use super::types::{ConfigInfo, Diagnostic};

// Needed for `.to_token_stream()` in the diagnostic message below.
use syn::__private::ToTokens;

/// Marks each `*InMsg` field optional when the matching element of
/// `impl BskModule for <Type> { type Inputs = (...); }` is `Option<Msg>`,
/// instead of requiring a doc-comment annotation.
pub(super) fn apply_input_optionality(
    info: &mut ConfigInfo,
    input_types: &[Type],
    diagnostics: &mut Vec<Diagnostic>,
) {
    let struct_name = info.struct_name.clone();
    let input_fields: Vec<&mut super::types::FieldInfo> =
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
pub(super) fn tuple_elems(ty: &Type) -> Vec<Type> {
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

#[cfg(test)]
mod tests {
    use super::super::test_support::{info_for, types};
    use super::*;

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
