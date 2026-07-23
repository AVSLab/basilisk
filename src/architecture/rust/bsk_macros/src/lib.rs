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
use proc_macro2::TokenStream as TokenStream2;
use quote::{format_ident, quote};
use syn::{parse_macro_input, Field, Fields, ItemStruct, LitStr, Type, Visibility};

/// Mark and validate a Basilisk module's top-level configuration struct.
#[proc_macro_attribute]
pub fn module(arguments: TokenStream, input: TokenStream) -> TokenStream {
    let _ = parse_macro_input!(arguments as syn::parse::Nothing);
    let input = parse_macro_input!(input as ItemStruct);
    expand_module(input)
        .map(TokenStream::from)
        .unwrap_or_else(|error| error.into_compile_error().into())
}

fn expand_module(input: ItemStruct) -> syn::Result<TokenStream2> {
    validate_module_config(&input)?;

    let config_type = &input.ident;
    let fields = match &input.fields {
        Fields::Named(fields) => &fields.named,
        _ => unreachable!("validated module config must have named fields"),
    };
    let input_fields: Vec<&Field> = fields
        .iter()
        .filter(|field| type_last_ident(&field.ty).is_some_and(|ident| ident == "MsgReader"))
        .collect();
    let output_fields: Vec<&Field> = fields
        .iter()
        .filter(|field| type_last_ident(&field.ty).is_some_and(|ident| ident == "MsgWriter"))
        .collect();

    let module_name = module_name();
    let init_function = format_ident!("Init_{module_name}");
    let self_init_function = format_ident!("SelfInit_{module_name}");
    let reset_function = format_ident!("Reset_{module_name}");
    let update_function = format_ident!("Update_{module_name}");
    let drop_function = format_ident!("Drop_{module_name}");

    let logger = fields
        .iter()
        .find(|field| {
            field
                .ident
                .as_ref()
                .is_some_and(|ident| ident == "bskLogger")
        })
        .and_then(|field| field.ident.as_ref())
        .map_or_else(
            || quote!(::core::ptr::null_mut()),
            |field_name| quote!((*config).#field_name),
        );

    let input_ports: Vec<TokenStream2> = input_fields
        .iter()
        .filter_map(|field| field.ident.as_ref())
        .map(|field_name| quote!(&mut (*config).#field_name))
        .collect();
    let missing_input_messages: Vec<LitStr> = input_fields
        .iter()
        .filter_map(|field| field.ident.as_ref())
        .map(|field_name| {
            LitStr::new(
                &format!("[{module_name}] {field_name} is not connected"),
                field_name.span(),
            )
        })
        .collect();

    let input_values: Vec<syn::Ident> = (0..input_fields.len())
        .map(|index| format_ident!("Input{index}"))
        .collect();
    let input_messages: Vec<syn::Ident> = (0..input_fields.len())
        .map(|index| format_ident!("Message{index}"))
        .collect();
    let input_port_variables: Vec<syn::Ident> = (0..input_fields.len())
        .map(|index| format_ident!("port{index}"))
        .collect();
    let input_indices: Vec<syn::Index> = (0..input_fields.len()).map(syn::Index::from).collect();
    let input_tuple_trait = format_ident!("__BskInputTupleFor{config_type}");
    let validate_inputs_function =
        format_ident!("__bsk_validate_inputs_for_{}", config_type.to_string());
    let read_inputs_function = format_ident!("__bsk_read_inputs_for_{}", config_type.to_string());
    let input_adapter = if input_fields.is_empty() {
        TokenStream2::new()
    } else {
        quote! {
            #[cfg(not(test))]
            trait #input_tuple_trait<Ports>: Sized {
                fn validate(
                    ports: Ports,
                    logger: *mut ::bsk_build::BSKLogger,
                    missing_messages: &[&str],
                );
                fn read(
                    ports: Ports,
                    logger: *mut ::bsk_build::BSKLogger,
                    missing_messages: &[&str],
                ) -> Self;
            }

            #[cfg(not(test))]
            impl<'bsk_ports, #(#input_values, #input_messages),*>
                #input_tuple_trait<(
                    #(&'bsk_ports mut ::bsk_build::MsgReader<#input_messages>,)*
                )> for (#(#input_values,)*)
            where
                #(
                    #input_messages: ::bsk_build::Msg,
                    #input_values: ::bsk_build::BskModuleInput<#input_messages>,
                )*
            {
                fn validate(
                    ports: (
                        #(&'bsk_ports mut ::bsk_build::MsgReader<#input_messages>,)*
                    ),
                    logger: *mut ::bsk_build::BSKLogger,
                    missing_messages: &[&str],
                ) {
                    let (#(#input_port_variables,)*) = ports;
                    #(
                        <#input_values as ::bsk_build::BskModuleInput<#input_messages>>::validate(
                            #input_port_variables,
                            logger,
                            missing_messages[#input_indices],
                        );
                    )*
                }

                fn read(
                    ports: (
                        #(&'bsk_ports mut ::bsk_build::MsgReader<#input_messages>,)*
                    ),
                    logger: *mut ::bsk_build::BSKLogger,
                    missing_messages: &[&str],
                ) -> Self {
                    let (#(#input_port_variables,)*) = ports;
                    (
                        #(
                            <#input_values as ::bsk_build::BskModuleInput<#input_messages>>::read(
                                #input_port_variables,
                                logger,
                                missing_messages[#input_indices],
                            ),
                        )*
                    )
                }
            }

            #[cfg(not(test))]
            #[allow(non_snake_case)]
            fn #validate_inputs_function<Ports>(
                ports: Ports,
                logger: *mut ::bsk_build::BSKLogger,
                missing_messages: &[&str],
            )
            where
                <#config_type as ::bsk_build::BskModule>::Inputs:
                    #input_tuple_trait<Ports>,
            {
                <<#config_type as ::bsk_build::BskModule>::Inputs as
                    #input_tuple_trait<Ports>>::validate(
                        ports,
                        logger,
                        missing_messages,
                    );
            }

            #[cfg(not(test))]
            #[allow(non_snake_case)]
            fn #read_inputs_function<Ports>(
                ports: Ports,
                logger: *mut ::bsk_build::BSKLogger,
                missing_messages: &[&str],
            ) -> <#config_type as ::bsk_build::BskModule>::Inputs
            where
                <#config_type as ::bsk_build::BskModule>::Inputs:
                    #input_tuple_trait<Ports>,
            {
                <<#config_type as ::bsk_build::BskModule>::Inputs as
                    #input_tuple_trait<Ports>>::read(
                        ports,
                        logger,
                        missing_messages,
                    )
            }
        }
    };

    let validate_inputs = if input_ports.is_empty() {
        TokenStream2::new()
    } else {
        quote! {
            #validate_inputs_function(
                (#(#input_ports,)*),
                #logger,
                &[#(#missing_input_messages,)*],
            );
        }
    };
    let read_inputs = if input_ports.is_empty() {
        quote! {
            let inputs: <#config_type as ::bsk_build::BskModule>::Inputs = ();
        }
    } else {
        quote! {
            let inputs: <#config_type as ::bsk_build::BskModule>::Inputs =
                #read_inputs_function(
                    (#(#input_ports,)*),
                    #logger,
                    &[#(#missing_input_messages,)*],
                );
        }
    };

    let output_fields: Vec<&syn::Ident> = output_fields
        .iter()
        .filter_map(|field| field.ident.as_ref())
        .collect();
    let output_variables: Vec<syn::Ident> = output_fields
        .iter()
        .map(|field_name| format_ident!("{}", camel_to_snake(&field_name.to_string())))
        .collect();
    let initialize_outputs = output_fields
        .iter()
        .map(|field_name| quote!((*config).#field_name.init();));
    let write_reset_outputs =
        output_fields
            .iter()
            .zip(output_variables.iter())
            .map(|(field_name, variable)| {
                quote! {
                    (*config).#field_name.write(
                        &#variable,
                        (*config).runtime.module_id(),
                        current_sim_nanos,
                    );
                }
            });
    let write_update_outputs =
        output_fields
            .iter()
            .zip(output_variables.iter())
            .map(|(field_name, variable)| {
                quote! {
                    (*config).#field_name.write(
                        &#variable,
                        (*config).runtime.module_id(),
                        current_sim_nanos,
                    );
                }
            });

    Ok(quote! {
        #input

        #input_adapter

        #[cfg(not(test))]
        #[allow(non_snake_case)]
        #[no_mangle]
        pub unsafe extern "C-unwind" fn #init_function(config: *mut #config_type) {
            <#config_type as ::bsk_build::BskModule>::init(&mut *config);
        }

        #[cfg(not(test))]
        #[allow(non_snake_case)]
        #[no_mangle]
        pub unsafe extern "C-unwind" fn #self_init_function(
            config: *mut #config_type,
            runtime: *const ::bsk_build::BskModuleRuntime,
        ) {
            (*config).runtime = ::core::ptr::read(runtime);
            #(#initialize_outputs)*
        }

        #[cfg(not(test))]
        #[allow(non_snake_case)]
        #[no_mangle]
        pub unsafe extern "C-unwind" fn #reset_function(
            config: *mut #config_type,
            current_sim_nanos: u64,
            runtime: *const ::bsk_build::BskModuleRuntime,
        ) {
            (*config).runtime = ::core::ptr::read(runtime);
            #validate_inputs
            let (#(#output_variables,)*):
                <#config_type as ::bsk_build::BskModule>::Outputs =
                <#config_type as ::bsk_build::BskModule>::reset(
                    &mut *config,
                    current_sim_nanos,
                );
            #(#write_reset_outputs)*
        }

        #[cfg(not(test))]
        #[allow(non_snake_case)]
        #[no_mangle]
        pub unsafe extern "C-unwind" fn #update_function(
            config: *mut #config_type,
            current_sim_nanos: u64,
            runtime: *const ::bsk_build::BskModuleRuntime,
        ) {
            (*config).runtime = ::core::ptr::read(runtime);
            #read_inputs
            let (#(#output_variables,)*):
                <#config_type as ::bsk_build::BskModule>::Outputs =
                <#config_type as ::bsk_build::BskModule>::update(
                    &mut *config,
                    inputs,
                    current_sim_nanos,
                );
            #(#write_update_outputs)*
        }

        #[cfg(not(test))]
        #[allow(non_snake_case)]
        #[no_mangle]
        pub unsafe extern "C-unwind" fn #drop_function(config: *mut #config_type) {
            ::core::ptr::drop_in_place(config);
        }
    })
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

fn module_name() -> String {
    std::env::var_os("BSK_HEADER_PATH")
        .and_then(|path| {
            std::path::PathBuf::from(path)
                .file_stem()
                .map(|stem| stem.to_owned())
        })
        .map(|stem| stem.to_string_lossy().into_owned())
        .or_else(|| std::env::var("CARGO_PKG_NAME").ok())
        .unwrap_or_else(|| "bsk_rust_module".to_owned())
        .replace('-', "_")
}

fn camel_to_snake(name: &str) -> String {
    let mut output = String::new();
    for (index, character) in name.chars().enumerate() {
        if character.is_uppercase() && index > 0 {
            output.push('_');
        }
        output.push(character.to_ascii_lowercase());
    }
    output
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
    fn expansion_generates_lifecycle_and_port_adaptation() {
        let input: ItemStruct = parse_quote! {
            #[repr(C)]
            pub struct ControllerConfig {
                pub runtime: BskModuleRuntime,
                pub inputInMsg: MsgReader<InputMsg>,
                pub outputOutMsg: MsgWriter<OutputMsg>,
                pub bskLogger: *mut BSKLogger,
            }
        };

        let expanded = expand_module(input)
            .expect("valid module must expand")
            .to_string();
        for lifecycle in ["Init_", "SelfInit_", "Reset_", "Update_", "Drop_"] {
            assert!(expanded.contains(lifecycle), "expanded module: {expanded}");
        }
        assert!(expanded.contains("BskModuleInput"));
        assert!(expanded.contains("inputInMsg is not connected"));
        assert!(expanded.contains("outputOutMsg . init"));
        assert_eq!(expanded.matches("outputOutMsg . write").count(), 2);
        assert!(!expanded.contains("include !"));
    }

    #[test]
    fn input_adapter_is_generated_for_the_config_arity() {
        let mut fields = String::from("pub runtime: BskModuleRuntime,");
        for index in 0..12 {
            fields.push_str(&format!(
                "pub input{index}InMsg: MsgReader<Input{index}Msg>,"
            ));
        }
        let input: ItemStruct = syn::parse_str(&format!(
            "#[repr(C)] pub struct ManyInputsConfig {{ {fields} }}"
        ))
        .expect("test config must parse");

        let expanded = expand_module(input)
            .expect("valid module must expand")
            .to_string();
        assert!(expanded.contains("Input11"));
        assert!(expanded.contains("Message11"));
        assert!(expanded.contains("port11"));
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
