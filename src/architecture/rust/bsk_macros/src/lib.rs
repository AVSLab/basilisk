// ISC License
//
// Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
//
// Permission to use, copy, modify, and/or distribute this software for any
// purpose with or without fee is hereby granted, provided that the above
// copyright notice and this permission notice appear in all copies.
//
// THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
// WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
// MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
// ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
// WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
// ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
// OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

//! Procedural macros for native Basilisk Rust modules.

use proc_macro::TokenStream;
use syn::{parse_macro_input, Fields, ItemStruct, Type, Visibility};

/// Mark and validate a Basilisk module's top-level configuration struct.
#[proc_macro_attribute]
pub fn module(arguments: TokenStream, input: TokenStream) -> TokenStream {
    let _ = parse_macro_input!(arguments as syn::parse::Nothing);
    let original_input = input.clone();
    let input = parse_macro_input!(input as ItemStruct);
    validate_module_config(&input)
        .map(|()| original_input)
        .unwrap_or_else(|error| error.into_compile_error().into())
}

fn validate_module_config(input: &ItemStruct) -> syn::Result<()> {
    if !matches!(input.vis, Visibility::Public(_)) {
        return Err(syn::Error::new_spanned(
            &input.ident,
            "a Basilisk module config must be declared `pub` for C++/SWIG access",
        ));
    }
    if !input.generics.params.is_empty() {
        return Err(syn::Error::new_spanned(
            &input.generics,
            "a Basilisk module config cannot have generic parameters",
        ));
    }
    if !has_repr_c(input) {
        return Err(syn::Error::new_spanned(
            &input.ident,
            "a Basilisk module config must use `#[repr(C)]`",
        ));
    }

    let fields = match &input.fields {
        Fields::Named(fields) => &fields.named,
        Fields::Unnamed(fields) => {
            return Err(syn::Error::new_spanned(
                fields,
                "a Basilisk module config must use named fields",
            ));
        }
        Fields::Unit => {
            return Err(syn::Error::new_spanned(
                &input.ident,
                "a Basilisk module config must use named fields",
            ));
        }
    };

    for field in fields {
        if !matches!(field.vis, Visibility::Public(_)) {
            return Err(syn::Error::new_spanned(
                field,
                "Basilisk module config fields must be declared `pub` for C++/SWIG access",
            ));
        }
    }

    let has_runtime = fields.iter().any(|field| {
        field.ident.as_ref().is_some_and(|ident| ident == "runtime")
            && type_last_ident(&field.ty).is_some_and(|ident| ident == "BskModuleRuntime")
    });
    if !has_runtime {
        return Err(syn::Error::new_spanned(
            &input.ident,
            "a Basilisk module config requires a `runtime: BskModuleRuntime` field",
        ));
    }

    Ok(())
}

fn has_repr_c(input: &ItemStruct) -> bool {
    input.attrs.iter().any(|attribute| {
        if !attribute.path().is_ident("repr") {
            return false;
        }
        let mut found = false;
        let parsed = attribute.parse_nested_meta(|meta| {
            if meta.path.is_ident("C") {
                found = true;
            }
            Ok(())
        });
        parsed.is_ok() && found
    })
}

fn type_last_ident(field_type: &Type) -> Option<&syn::Ident> {
    match field_type {
        Type::Path(type_path) => type_path.path.segments.last().map(|segment| &segment.ident),
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use syn::parse_quote;

    #[test]
    fn accepts_public_repr_c_config_with_runtime() {
        let input: ItemStruct = parse_quote! {
            #[repr(C)]
            pub struct ControllerConfig {
                pub runtime: bsk_messages::BskModuleRuntime,
                pub gain: f64,
            }
        };

        assert!(validate_module_config(&input).is_ok());
    }

    #[test]
    fn rejects_config_without_repr_c() {
        let input: ItemStruct = parse_quote! {
            pub struct ControllerConfig {
                pub runtime: BskModuleRuntime,
            }
        };

        let error = validate_module_config(&input).expect_err("missing repr(C) must fail");
        assert!(error.to_string().contains("must use `#[repr(C)]`"));
    }

    #[test]
    fn rejects_config_without_runtime() {
        let input: ItemStruct = parse_quote! {
            #[repr(C)]
            pub struct ControllerConfig {
                pub gain: f64,
            }
        };

        let error = validate_module_config(&input).expect_err("missing runtime must fail");
        assert!(error
            .to_string()
            .contains("requires a `runtime: BskModuleRuntime` field"));
    }

    #[test]
    fn rejects_private_config_field() {
        let input: ItemStruct = parse_quote! {
            #[repr(C)]
            pub struct ControllerConfig {
                pub runtime: BskModuleRuntime,
                gain: f64,
            }
        };

        let error = validate_module_config(&input).expect_err("private field must fail");
        assert!(error.to_string().contains("fields must be declared `pub`"));
    }
}
