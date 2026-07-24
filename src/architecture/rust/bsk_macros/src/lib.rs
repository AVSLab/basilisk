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
use syn::{
    parse_macro_input, AngleBracketedGenericArguments, Attribute, Expr, Field, Fields,
    GenericArgument, ItemStruct, LitStr, PathArguments, Type, Visibility,
};

/// Mark and validate a Basilisk module's top-level configuration struct.
///
/// Message fields must use ``#[bsk(input)]``,
/// ``#[bsk(input, optional)]``, or ``#[bsk(output)]``. For a config named
/// ``MyModuleConfig``, this attribute generates ``MyModuleInputs`` and
/// ``MyModuleOutputs`` with corresponding named message-value fields, plus
/// the Basilisk C ABI construction, destruction, and lifecycle entry points.
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

    let mut input = input;
    let config_type = input.ident.clone();
    let (input_fields, output_fields) = extract_message_ports(&mut input)?;
    let fields = match &input.fields {
        Fields::Named(fields) => &fields.named,
        _ => unreachable!("validated module config must have named fields"),
    };
    let inputs_type = io_type_name(&config_type, "Inputs");
    let outputs_type = io_type_name(&config_type, "Outputs");
    let handle_type = format_ident!("{config_type}Handle");
    let instance_type = format_ident!("__Bsk{config_type}Instance");

    let module_name = module_name();
    let create_function = format_ident!("Create_{module_name}");
    let config_function = format_ident!("Config_{module_name}");
    let destroy_function = format_ident!("Destroy_{module_name}");
    let self_init_function = format_ident!("SelfInit_{module_name}");
    let reset_function = format_ident!("Reset_{module_name}");
    let update_function = format_ident!("Update_{module_name}");
    let assert_io_types_function =
        format_ident!("__bsk_assert_io_types_for_{}", config_type.to_string());
    let guard_instance_function =
        format_ident!("__bsk_guard_instance_for_{}", config_type.to_string());
    let initialize_config_fields: Vec<TokenStream2> = fields
        .iter()
        .map(|field| {
            let field_name = field
                .ident
                .as_ref()
                .expect("validated module config must have named fields");
            let initializer = match message_port_type(&field.ty) {
                Some((_, _, PortShape::Array(_))) => {
                    quote!(::core::array::from_fn(|_| {
                        ::core::default::Default::default()
                    }))
                }
                _ => quote!(::core::default::Default::default()),
            };
            quote!(#field_name: #initializer)
        })
        .collect();

    let input_names: Vec<&syn::Ident> = input_fields.iter().map(|field| &field.name).collect();
    let input_types: Vec<TokenStream2> = input_fields.iter().map(MessagePort::value_type).collect();
    let input_docs: Vec<TokenStream2> = input_fields
        .iter()
        .map(|field| {
            let docs = &field.docs;
            quote!(#(#docs)*)
        })
        .collect();
    let missing_input_messages: Vec<LitStr> = input_fields
        .iter()
        .map(|field| {
            let suffix = match &field.shape {
                PortShape::Single => " is not connected",
                PortShape::Array(_) => "",
            };
            LitStr::new(
                &format!("[{module_name}] {}{suffix}", field.name),
                field.name.span(),
            )
        })
        .collect();
    let validate_inputs = input_fields
        .iter()
        .zip(missing_input_messages.iter())
        .filter(|(field, _)| !field.optional)
        .map(|(field, missing_message)| {
            let field_name = &field.name;
            let message_type = &field.message_type;
            match &field.shape {
                PortShape::Single => quote! {
                    <#message_type as ::bsk_build::BskModuleInput<#message_type>>::validate(
                        &mut (*config).#field_name,
                        #missing_message,
                    )?;
                },
                PortShape::Array(_) => quote! {
                    for (index, port) in (*config).#field_name.iter_mut().enumerate() {
                        let missing_message =
                            ::std::format!("{}[{}] is not connected", #missing_message, index);
                        <#message_type as ::bsk_build::BskModuleInput<#message_type>>::validate(
                            port,
                            &missing_message,
                        )?;
                    }
                },
            }
        });
    let read_inputs = input_fields
        .iter()
        .zip(input_types.iter())
        .zip(missing_input_messages.iter())
        .map(|((field, input_type), missing_message)| {
            let field_name = &field.name;
            let message_type = &field.message_type;
            let input_element_type = field.element_value_type();
            match &field.shape {
                PortShape::Single => quote! {
                    #field_name:
                        <#input_type as ::bsk_build::BskModuleInput<#message_type>>::read(
                            &mut (*config).#field_name,
                            #missing_message,
                        )?
                },
                PortShape::Array(_) => quote! {
                    #field_name: {
                        let mut values: #input_type =
                            ::core::array::from_fn(
                                |_| ::core::default::Default::default()
                            );
                        for (index, (port, value)) in (*config)
                            .#field_name
                            .iter_mut()
                            .zip(values.iter_mut())
                            .enumerate()
                        {
                            let missing_message =
                                ::std::format!("{}[{}] is not connected", #missing_message, index);
                            *value =
                                <#input_element_type as
                                    ::bsk_build::BskModuleInput<#message_type>>::read(
                                        port,
                                        &missing_message,
                                    )?;
                        }
                        values
                    }
                },
            }
        });

    let output_names: Vec<&syn::Ident> = output_fields.iter().map(|field| &field.name).collect();
    let output_types: Vec<TokenStream2> =
        output_fields.iter().map(MessagePort::value_type).collect();
    let output_initializers = output_fields.iter().map(|field| {
        let field_name = &field.name;
        let initializer = field.default_value();
        quote!(#field_name: #initializer)
    });
    let output_docs: Vec<TokenStream2> = output_fields
        .iter()
        .map(|field| {
            let docs = &field.docs;
            quote!(#(#docs)*)
        })
        .collect();
    let initialize_outputs = output_fields.iter().map(|field| {
        let field_name = &field.name;
        match &field.shape {
            PortShape::Single => quote!((*config).#field_name.init();),
            PortShape::Array(_) => quote! {
                for port in &mut (*config).#field_name {
                    port.init();
                }
            },
        }
    });
    let write_reset_outputs = output_fields.iter().map(|field| {
        let field_name = &field.name;
        match &field.shape {
            PortShape::Single => quote! {
                (*config).#field_name.write(
                    &outputs.#field_name,
                    context.module_id(),
                    current_sim_nanos,
                );
            },
            PortShape::Array(_) => quote! {
                for (port, value) in (*config)
                    .#field_name
                    .iter_mut()
                    .zip(outputs.#field_name.iter())
                {
                    port.write(
                        value,
                        context.module_id(),
                        current_sim_nanos,
                    );
                }
            },
        }
    });
    let write_update_outputs = output_fields.iter().map(|field| {
        let field_name = &field.name;
        match &field.shape {
            PortShape::Single => quote! {
                (*config).#field_name.write(
                    &outputs.#field_name,
                    context.module_id(),
                    current_sim_nanos,
                );
            },
            PortShape::Array(_) => quote! {
                for (port, value) in (*config)
                    .#field_name
                    .iter_mut()
                    .zip(outputs.#field_name.iter())
                {
                    port.write(
                        value,
                        context.module_id(),
                        current_sim_nanos,
                    );
                }
            },
        }
    });

    Ok(quote! {
        #input

        #[cfg(not(panic = "unwind"))]
        compile_error!(
            "Basilisk Rust modules require panic=\"unwind\" so generated FFI \
             boundaries can contain panics; set panic = \"unwind\" in the \
             workspace dev and release profiles"
        );

        /// Named message values supplied to this module's `update` method.
        #[allow(non_camel_case_types, non_snake_case)]
        pub struct #inputs_type {
            #(
                #input_docs
                pub #input_names: #input_types,
            )*
        }

        /// Named message values returned by this module's `reset` and `update` methods.
        #[allow(non_camel_case_types, non_snake_case)]
        pub struct #outputs_type {
            #(
                #output_docs
                pub #output_names: #output_types,
            )*
        }

        impl ::core::default::Default for #outputs_type {
            fn default() -> Self {
                Self {
                    #(#output_initializers,)*
                }
            }
        }

        /// Opaque C handle for this module's Rust-owned instance.
        #[doc(hidden)]
        #[repr(C)]
        pub struct #handle_type {
            _private: [u8; 0],
        }

        #[doc(hidden)]
        struct #instance_type {
            config: #config_type,
            state: <#config_type as ::bsk_build::BskModule>::State,
            poisoned_by: ::core::option::Option<&'static str>,
        }

        #[doc(hidden)]
        fn #guard_instance_function(
            instance: &mut #instance_type,
            operation: &'static str,
            action: impl FnOnce(&mut #instance_type) -> ::bsk_build::BskResult<()>,
        ) -> *mut ::bsk_build::BskRustError {
            if let ::core::option::Option::Some(poisoned_by) = instance.poisoned_by {
                return ::bsk_build::BskRustError::__poisoned(operation, poisoned_by);
            }
            let (error, panicked) =
                ::bsk_build::__ffi_boundary_with_status(operation, || action(instance));
            if panicked {
                instance.poisoned_by = ::core::option::Option::Some(operation);
            }
            error
        }

        #[doc(hidden)]
        #[allow(dead_code, non_snake_case)]
        fn #assert_io_types_function(
            config: &mut #config_type,
            state: &mut <#config_type as ::bsk_build::BskModule>::State,
            context: &::bsk_build::BskContext<'_>,
            inputs: #inputs_type,
            current_sim_nanos: u64,
        ) -> ::bsk_build::BskResult<#outputs_type> {
            <#config_type as ::bsk_build::BskModule>::update(
                config,
                state,
                context,
                inputs,
                current_sim_nanos,
            )
        }

        #[cfg(not(test))]
        #[allow(non_snake_case)]
        #[no_mangle]
        pub unsafe extern "C" fn #create_function(
            output_handle: *mut *mut #handle_type,
        ) -> *mut ::bsk_build::BskRustError {
            if output_handle.is_null() {
                return ::bsk_build::BskRustError::__invalid_argument(concat!(
                    stringify!(#create_function),
                    ": output handle pointer must not be null",
                ));
            }
            unsafe {
                output_handle.write(::core::ptr::null_mut());
            }
            ::bsk_build::__ffi_boundary(stringify!(#create_function), || {
                let mut instance = ::std::boxed::Box::new(#instance_type {
                    config: #config_type {
                        #(#initialize_config_fields,)*
                    },
                    state: ::core::default::Default::default(),
                    poisoned_by: ::core::option::Option::None,
                });
                <#config_type as ::bsk_build::BskModule>::init(
                    &mut instance.config,
                    &mut instance.state,
                )?;
                unsafe {
                    output_handle.write(
                        ::std::boxed::Box::into_raw(instance).cast::<#handle_type>(),
                    );
                }
                Ok(())
            })
        }

        #[cfg(not(test))]
        #[allow(non_snake_case)]
        #[no_mangle]
        pub unsafe extern "C" fn #config_function(
            handle: *mut #handle_type,
        ) -> *mut #config_type {
            if handle.is_null() {
                return ::core::ptr::null_mut();
            }
            let instance = handle.cast::<#instance_type>();
            ::core::ptr::addr_of_mut!((*instance).config)
        }

        #[cfg(not(test))]
        #[allow(non_snake_case)]
        #[no_mangle]
        pub unsafe extern "C" fn #destroy_function(
            handle: *mut #handle_type,
        ) -> *mut ::bsk_build::BskRustError {
            if handle.is_null() {
                return ::core::ptr::null_mut();
            }
            ::bsk_build::__ffi_boundary(stringify!(#destroy_function), || {
                unsafe {
                    drop(::std::boxed::Box::from_raw(handle.cast::<#instance_type>()));
                }
                Ok(())
            })
        }

        #[cfg(not(test))]
        #[allow(non_snake_case)]
        #[no_mangle]
        pub unsafe extern "C" fn #self_init_function(
            handle: *mut #handle_type,
            context: *const ::bsk_build::BskModuleContext,
        ) -> *mut ::bsk_build::BskRustError {
            if handle.is_null() {
                return ::bsk_build::BskRustError::__invalid_argument(concat!(
                    stringify!(#self_init_function),
                    ": module handle must not be null",
                ));
            }
            if context.is_null() {
                return ::bsk_build::BskRustError::__invalid_argument(concat!(
                    stringify!(#self_init_function),
                    ": lifecycle context must not be null",
                ));
            }
            let instance = unsafe { &mut *handle.cast::<#instance_type>() };
            #guard_instance_function(
                instance,
                stringify!(#self_init_function),
                |instance| {
                    let config = &mut instance.config;
                    let _context = unsafe { ::bsk_build::BskContext::__from_raw(context) };
                    #(#initialize_outputs)*
                    Ok(())
                },
            )
        }

        #[cfg(not(test))]
        #[allow(non_snake_case)]
        #[no_mangle]
        pub unsafe extern "C" fn #reset_function(
            handle: *mut #handle_type,
            current_sim_nanos: u64,
            context: *const ::bsk_build::BskModuleContext,
        ) -> *mut ::bsk_build::BskRustError {
            if handle.is_null() {
                return ::bsk_build::BskRustError::__invalid_argument(concat!(
                    stringify!(#reset_function),
                    ": module handle must not be null",
                ));
            }
            if context.is_null() {
                return ::bsk_build::BskRustError::__invalid_argument(concat!(
                    stringify!(#reset_function),
                    ": lifecycle context must not be null",
                ));
            }
            let instance = unsafe { &mut *handle.cast::<#instance_type>() };
            #guard_instance_function(
                instance,
                stringify!(#reset_function),
                |instance| {
                    let config = &mut instance.config;
                    let context = unsafe { ::bsk_build::BskContext::__from_raw(context) };
                    #(#validate_inputs)*
                    let outputs: #outputs_type =
                        <#config_type as ::bsk_build::BskModule>::reset(
                            config,
                            &mut instance.state,
                            &context,
                            current_sim_nanos,
                        )?;
                    #(#write_reset_outputs)*
                    Ok(())
                },
            )
        }

        #[cfg(not(test))]
        #[allow(non_snake_case)]
        #[no_mangle]
        pub unsafe extern "C" fn #update_function(
            handle: *mut #handle_type,
            current_sim_nanos: u64,
            context: *const ::bsk_build::BskModuleContext,
        ) -> *mut ::bsk_build::BskRustError {
            if handle.is_null() {
                return ::bsk_build::BskRustError::__invalid_argument(concat!(
                    stringify!(#update_function),
                    ": module handle must not be null",
                ));
            }
            if context.is_null() {
                return ::bsk_build::BskRustError::__invalid_argument(concat!(
                    stringify!(#update_function),
                    ": lifecycle context must not be null",
                ));
            }
            let instance = unsafe { &mut *handle.cast::<#instance_type>() };
            #guard_instance_function(
                instance,
                stringify!(#update_function),
                |instance| {
                    let config = &mut instance.config;
                    let context = unsafe { ::bsk_build::BskContext::__from_raw(context) };
                    let inputs: #inputs_type = #inputs_type {
                        #(#read_inputs,)*
                    };
                    let outputs: #outputs_type =
                        <#config_type as ::bsk_build::BskModule>::update(
                            config,
                            &mut instance.state,
                            &context,
                            inputs,
                            current_sim_nanos,
                        )?;
                    #(#write_update_outputs)*
                    Ok(())
                },
            )
        }

    })
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum PortDirection {
    Input,
    Output,
}

struct MessagePort {
    name: syn::Ident,
    message_type: Type,
    shape: PortShape,
    optional: bool,
    docs: Vec<Attribute>,
}

enum PortShape {
    Single,
    Array(Expr),
}

impl MessagePort {
    fn element_value_type(&self) -> TokenStream2 {
        let message_type = &self.message_type;
        if self.optional {
            quote!(::core::option::Option<#message_type>)
        } else {
            quote!(#message_type)
        }
    }

    fn value_type(&self) -> TokenStream2 {
        let element_type = self.element_value_type();
        match &self.shape {
            PortShape::Single => element_type,
            PortShape::Array(length) => quote!([#element_type; #length]),
        }
    }

    fn default_value(&self) -> TokenStream2 {
        match &self.shape {
            PortShape::Single => quote!(::core::default::Default::default()),
            PortShape::Array(_) => {
                quote!(::core::array::from_fn(|_| {
                    ::core::default::Default::default()
                }))
            }
        }
    }
}

fn extract_message_ports(
    input: &mut ItemStruct,
) -> syn::Result<(Vec<MessagePort>, Vec<MessagePort>)> {
    let fields = match &mut input.fields {
        Fields::Named(fields) => &mut fields.named,
        _ => unreachable!("validated module config must have named fields"),
    };
    let mut inputs = Vec::new();
    let mut outputs = Vec::new();

    for field in fields {
        let port = parse_message_port(field)?;
        if let Some((direction, port)) = port {
            match direction {
                PortDirection::Input => inputs.push(port),
                PortDirection::Output => outputs.push(port),
            }
        }
    }

    Ok((inputs, outputs))
}

fn parse_message_port(field: &mut Field) -> syn::Result<Option<(PortDirection, MessagePort)>> {
    let annotation_indices: Vec<usize> = field
        .attrs
        .iter()
        .enumerate()
        .filter_map(|(index, attribute)| attribute.path().is_ident("bsk").then_some(index))
        .collect();
    let port_type = message_port_type(&field.ty);

    if annotation_indices.is_empty() {
        if port_type.is_some() {
            return Err(syn::Error::new_spanned(
                field,
                "message ports require `#[bsk(input)]`, \
                 `#[bsk(input, optional)]`, or `#[bsk(output)]`",
            ));
        }
        return Ok(None);
    }
    if annotation_indices.len() > 1 {
        return Err(syn::Error::new_spanned(
            field,
            "a message port must have exactly one `#[bsk(...)]` annotation",
        ));
    }

    let annotation = field.attrs[annotation_indices[0]].clone();
    let mut direction = None;
    let mut optional = false;
    annotation.parse_nested_meta(|meta| {
        let candidate = if meta.path.is_ident("input") {
            Some(PortDirection::Input)
        } else if meta.path.is_ident("output") {
            Some(PortDirection::Output)
        } else if meta.path.is_ident("optional") {
            if optional {
                return Err(meta.error("duplicate `optional` argument"));
            }
            optional = true;
            return Ok(());
        } else {
            return Err(meta.error("expected `input`, `output`, or `optional` in this annotation"));
        };

        if direction
            .replace(candidate.expect("direction candidate"))
            .is_some()
        {
            return Err(meta.error("choose exactly one of `input` or `output`"));
        }
        Ok(())
    })?;

    let direction = direction.ok_or_else(|| {
        syn::Error::new_spanned(
            &annotation,
            "a message port annotation requires `input` or `output`",
        )
    })?;
    if optional && direction != PortDirection::Input {
        return Err(syn::Error::new_spanned(
            &annotation,
            "`optional` is valid only for an input port",
        ));
    }

    let (type_direction, message_type, shape) = port_type.ok_or_else(|| {
        syn::Error::new_spanned(
            &field.ty,
            "an annotated input must use `MsgReader<Message>` or \
             `[MsgReader<Message>; N]`, and an annotated output must use \
             `MsgWriter<Message>` or `[MsgWriter<Message>; N]`",
        )
    })?;
    if direction != type_direction {
        let expected = match direction {
            PortDirection::Input => "MsgReader<Message>` or `[MsgReader<Message>; N]",
            PortDirection::Output => "MsgWriter<Message>` or `[MsgWriter<Message>; N]",
        };
        return Err(syn::Error::new_spanned(
            &field.ty,
            format!("this annotation requires `{expected}`"),
        ));
    }

    let name = field
        .ident
        .clone()
        .expect("validated module config must have named fields");
    let docs = field
        .attrs
        .iter()
        .filter(|attribute| attribute.path().is_ident("doc"))
        .cloned()
        .collect();
    field
        .attrs
        .retain(|attribute| !attribute.path().is_ident("bsk"));

    Ok(Some((
        direction,
        MessagePort {
            name,
            message_type,
            shape,
            optional,
            docs,
        },
    )))
}

fn message_port_type(field_type: &Type) -> Option<(PortDirection, Type, PortShape)> {
    if let Type::Array(array) = field_type {
        let (direction, message_type) = direct_message_port_type(&array.elem)?;
        return Some((direction, message_type, PortShape::Array(array.len.clone())));
    }

    let (direction, message_type) = direct_message_port_type(field_type)?;
    Some((direction, message_type, PortShape::Single))
}

fn direct_message_port_type(field_type: &Type) -> Option<(PortDirection, Type)> {
    let type_path = match field_type {
        Type::Path(type_path) => type_path,
        _ => return None,
    };
    let segment = type_path.path.segments.last()?;
    let direction = if segment.ident == "MsgReader" {
        PortDirection::Input
    } else if segment.ident == "MsgWriter" {
        PortDirection::Output
    } else {
        return None;
    };
    let arguments = match &segment.arguments {
        PathArguments::AngleBracketed(arguments) => arguments,
        _ => return None,
    };
    let message_type = single_type_argument(arguments)?;
    Some((direction, message_type.clone()))
}

fn single_type_argument(arguments: &AngleBracketedGenericArguments) -> Option<&Type> {
    if arguments.args.len() != 1 {
        return None;
    }
    match arguments.args.first()? {
        GenericArgument::Type(argument) => Some(argument),
        _ => None,
    }
}

fn io_type_name(config_type: &syn::Ident, suffix: &str) -> syn::Ident {
    let config_name = config_type.to_string();
    let module_name = config_name
        .strip_suffix("Config")
        .filter(|name| !name.is_empty())
        .unwrap_or(&config_name);
    format_ident!("{module_name}{suffix}", span = config_type.span())
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
        if let Type::Ptr(pointer) = &field.ty {
            return Err(syn::Error::new_spanned(
                pointer,
                "raw pointer config fields are unsupported; use value fields for Python \
                 parameters and `BskModule::State` for internal Rust state",
            ));
        }
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

#[cfg(test)]
mod tests {
    use super::*;
    use syn::parse_quote;

    #[test]
    fn accepts_public_repr_c_config() {
        let input: ItemStruct = parse_quote! {
            #[repr(C)]
            pub struct ControllerConfig {
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
                #[bsk(input, optional)]
                pub inputInMsg: MsgReader<InputMsg>,
                #[bsk(output)]
                pub outputOutMsg: MsgWriter<OutputMsg>,
            }
        };

        let expanded = expand_module(input)
            .expect("valid module must expand")
            .to_string();
        for lifecycle in [
            "Create_",
            "Config_",
            "Destroy_",
            "SelfInit_",
            "Reset_",
            "Update_",
        ] {
            assert!(expanded.contains(lifecycle), "expanded module: {expanded}");
        }
        assert!(expanded.contains("Box :: new"));
        assert!(expanded.contains("struct ControllerConfigHandle"));
        assert!(expanded.contains("struct __BskControllerConfigInstance"));
        assert!(expanded.contains("BskModule > :: State"));
        assert!(expanded.contains("BskModule > :: init"));
        assert!(expanded.contains("BskContext :: __from_raw"));
        assert!(expanded.contains("BskModuleContext"));
        assert!(expanded.contains("BskRustError"));
        assert!(expanded.contains("BskResult < ControllerOutputs >"));
        assert!(expanded.contains("__ffi_boundary"));
        assert_eq!(expanded.matches("__ffi_boundary_with_status").count(), 1);
        assert!(expanded.contains("__bsk_guard_instance_for_ControllerConfig"));
        assert!(expanded.contains("poisoned_by"));
        assert!(expanded.contains("__poisoned"));
        assert!(expanded.contains("output_handle"));
        assert!(expanded.contains("panic = \"unwind\""));
        assert!(expanded.contains("extern \"C\""));
        assert!(!expanded.contains("C-unwind"));
        assert!(expanded.contains("Box :: into_raw"));
        assert!(expanded.contains("Box :: from_raw"));
        assert!(!expanded.contains("New_"));
        assert!(!expanded.contains("Delete_"));
        assert!(expanded.contains("BskModuleInput"));
        assert!(expanded.contains("struct ControllerInputs"));
        assert!(expanded.contains("struct ControllerOutputs"));
        assert!(expanded.contains("inputInMsg : :: core :: option :: Option < InputMsg >"));
        assert!(expanded.contains("outputOutMsg : OutputMsg"));
        assert!(expanded.contains("__bsk_assert_io_types_for_ControllerConfig"));
        assert!(expanded.contains("inputInMsg is not connected"));
        assert!(expanded.contains("outputOutMsg . init"));
        assert_eq!(expanded.matches("outputOutMsg . write").count(), 2);
        assert!(!expanded.contains("include !"));
        assert!(!expanded.contains("bsk (input"));
        assert!(!expanded.contains("bsk (output"));
    }

    #[test]
    fn named_inputs_are_generated_for_every_annotated_port() {
        let mut fields = String::new();
        for index in 0..12 {
            fields.push_str(&format!(
                "#[bsk(input)] pub input{index}InMsg: MsgReader<Input{index}Msg>,"
            ));
        }
        let input: ItemStruct = syn::parse_str(&format!(
            "#[repr(C)] pub struct ManyInputsConfig {{ {fields} }}"
        ))
        .expect("test config must parse");

        let expanded = expand_module(input)
            .expect("valid module must expand")
            .to_string();
        assert!(expanded.contains("struct ManyInputsInputs"));
        assert!(expanded.contains("input11InMsg : Input11Msg"));
        assert!(expanded.contains("input11InMsg : < Input11Msg as"));
    }

    #[test]
    fn fixed_port_arrays_generate_named_values_and_lifecycle_loops() {
        let input: ItemStruct = parse_quote! {
            #[repr(C)]
            pub struct ControllerConfig {
                #[bsk(input)]
                pub requiredInMsgs: [MsgReader<InputMsg>; 2],
                #[bsk(input, optional)]
                pub optionalInMsgs: [MsgReader<InputMsg>; 3],
                #[bsk(output)]
                pub outputOutMsgs: [MsgWriter<OutputMsg>; 2],
            }
        };

        let expanded = expand_module(input)
            .expect("fixed message-port arrays must expand")
            .to_string();
        assert!(expanded.contains("requiredInMsgs : [InputMsg ; 2]"));
        assert!(
            expanded.contains("optionalInMsgs : [:: core :: option :: Option < InputMsg > ; 3]")
        );
        assert!(expanded.contains("outputOutMsgs : [OutputMsg ; 2]"));
        assert!(expanded.contains("requiredInMsgs . iter_mut () . enumerate ()"));
        assert!(expanded.contains("optionalInMsgs . iter_mut ()"));
        assert!(expanded.contains("zip (values . iter_mut ())"));
        assert!(expanded.contains("for port in & mut (* config) . outputOutMsgs"));
        assert!(expanded.contains("zip (outputs . outputOutMsgs . iter ())"));
        assert!(expanded.contains("core :: array :: from_fn"));
        assert!(expanded.contains("[{}] is not connected"));
        assert!(!expanded.contains("requiredInMsgs is not connected"));
    }

    #[test]
    fn rejects_unannotated_message_port() {
        let input: ItemStruct = parse_quote! {
            #[repr(C)]
            pub struct ControllerConfig {
                pub inputInMsg: MsgReader<InputMsg>,
            }
        };

        let error = expand_module(input).expect_err("unannotated port must fail");
        assert!(error.to_string().contains("message ports require"));
    }

    #[test]
    fn rejects_unannotated_message_port_array() {
        let input: ItemStruct = parse_quote! {
            #[repr(C)]
            pub struct ControllerConfig {
                pub inputInMsgs: [MsgReader<InputMsg>; 2],
            }
        };

        let error = expand_module(input).expect_err("unannotated port array must fail");
        assert!(error.to_string().contains("message ports require"));
    }

    #[test]
    fn rejects_dynamic_message_port_collection() {
        let input: ItemStruct = parse_quote! {
            #[repr(C)]
            pub struct ControllerConfig {
                #[bsk(input)]
                pub inputInMsgs: Vec<MsgReader<InputMsg>>,
            }
        };

        let error = expand_module(input).expect_err("dynamic port collection must fail");
        assert!(error.to_string().contains("[MsgReader<Message>; N]"));
    }

    #[test]
    fn rejects_optional_output_port() {
        let input: ItemStruct = parse_quote! {
            #[repr(C)]
            pub struct ControllerConfig {
                #[bsk(output, optional)]
                pub outputOutMsg: MsgWriter<OutputMsg>,
            }
        };

        let error = expand_module(input).expect_err("optional output must fail");
        assert!(error
            .to_string()
            .contains("`optional` is valid only for an input port"));
    }

    #[test]
    fn rejects_annotation_that_disagrees_with_port_type() {
        let input: ItemStruct = parse_quote! {
            #[repr(C)]
            pub struct ControllerConfig {
                #[bsk(output)]
                pub inputInMsg: MsgReader<InputMsg>,
            }
        };

        let error = expand_module(input).expect_err("mismatched annotation must fail");
        assert!(error
            .to_string()
            .contains("this annotation requires `MsgWriter<Message>`"));
    }

    #[test]
    fn rejects_config_without_repr_c() {
        let input: ItemStruct = parse_quote! {
            pub struct ControllerConfig {
                pub gain: f64,
            }
        };

        let error = validate_module_config(&input).expect_err("missing repr(C) must fail");
        assert!(error.to_string().contains("must use `#[repr(C)]`"));
    }

    #[test]
    fn accepts_config_without_framework_fields() {
        let input: ItemStruct = parse_quote! {
            #[repr(C)]
            pub struct ControllerConfig {
                pub gain: f64,
            }
        };

        assert!(validate_module_config(&input).is_ok());
    }

    #[test]
    fn rejects_private_config_field() {
        let input: ItemStruct = parse_quote! {
            #[repr(C)]
            pub struct ControllerConfig {
                gain: f64,
            }
        };

        let error = validate_module_config(&input).expect_err("private field must fail");
        assert!(error.to_string().contains("fields must be declared `pub`"));
    }

    #[test]
    fn rejects_raw_pointer_config_field() {
        let input: ItemStruct = parse_quote! {
            #[repr(C)]
            pub struct ControllerConfig {
                pub state: *mut f64,
            }
        };

        let error = validate_module_config(&input).expect_err("raw pointer must fail");
        assert!(error.to_string().contains("raw pointer config fields"));
    }

    #[test]
    fn rejects_logger_config_field() {
        let input: ItemStruct = parse_quote! {
            #[repr(C)]
            pub struct ControllerConfig {
                pub bskLogger: *mut BSKLogger,
            }
        };

        let error = validate_module_config(&input).expect_err("logger belongs in BskContext");
        assert!(error.to_string().contains("raw pointer config fields"));
    }
}
