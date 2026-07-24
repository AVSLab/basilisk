/*
 ISC License

 Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

 Permission to use, copy, modify, and/or distribute this software for any
 purpose with or without fee is hereby granted, provided that the above
 copyright notice and this permission notice appear in all copies.

 THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
*/

use proc_macro2::{Group, Ident, Literal, TokenStream, TokenTree};
use regex::Regex;
use std::{
    collections::BTreeSet,
    env,
    error::Error,
    ffi::OsStr,
    fmt::Write as _,
    fs,
    path::{Path, PathBuf},
};

const GENERATED_BINDINGS_FILE: &str = "bsk_message_bindings.rs";

fn main() {
    if let Err(error) = generate_bindings() {
        panic!("bsk-messages could not generate Rust message bindings: {error}");
    }
}

fn generate_bindings() -> Result<(), Box<dyn Error>> {
    for variable in [
        "BSK_CMSG_DIR",
        "BSK_SRC_ROOT",
        "BSK_C_SYSTEM_INCLUDE_DIRS",
        "LIBCLANG_PATH",
        "VIRTUAL_ENV",
        "CONDA_PREFIX",
        "pythonLocation",
        "Python_ROOT_DIR",
    ] {
        println!("cargo:rerun-if-env-changed={variable}");
    }

    configure_bundled_libclang();

    let manifest_dir = PathBuf::from(
        env::var_os("CARGO_MANIFEST_DIR")
            .ok_or("Cargo did not provide CARGO_MANIFEST_DIR to the bsk-messages build script")?,
    );
    let repository_root = manifest_dir
        .ancestors()
        .nth(4)
        .ok_or("bsk-messages is not located under the Basilisk src tree")?;
    let source_root = env_path("BSK_SRC_ROOT").unwrap_or_else(|| repository_root.join("src"));
    let c_message_dir = env_path("BSK_CMSG_DIR")
        .unwrap_or_else(|| repository_root.join("dist3/autoSource/cMsgCInterface"));

    if !source_root.is_dir() {
        return Err(format!(
            "Basilisk source directory does not exist: {}",
            source_root.display()
        )
        .into());
    }
    if !c_message_dir.is_dir() {
        return Err(format!(
            "generated C message directory does not exist: {}. Run \
             `python3 conanfile.py --rustModules True` first",
            c_message_dir.display()
        )
        .into());
    }
    // Watching the directory catches newly added or removed message headers;
    // watching each file below catches content changes with precise paths in
    // Cargo's diagnostics.
    println!("cargo:rerun-if-changed={}", c_message_dir.display());

    let mut headers = fs::read_dir(&c_message_dir)?
        .filter_map(Result::ok)
        .map(|entry| entry.path())
        .filter(|path| {
            path.extension() == Some(OsStr::new("h"))
                && path
                    .file_stem()
                    .is_some_and(|stem| stem.to_string_lossy().ends_with("_C"))
        })
        .collect::<Vec<_>>();
    headers.sort();
    if headers.is_empty() {
        return Err(format!(
            "no generated *_C.h message headers were found under {}",
            c_message_dir.display()
        )
        .into());
    }
    for header in &headers {
        println!("cargo:rerun-if-changed={}", header.display());
    }

    let output_dir = PathBuf::from(
        env::var_os("OUT_DIR").ok_or("Cargo did not provide OUT_DIR to bsk-messages")?,
    );
    let wrapper_path = output_dir.join("bsk_messages_wrapper.h");
    fs::write(&wrapper_path, render_wrapper(&headers))?;

    // Rust modules bind only Basilisk's C message ABI. Parsing these headers
    // as C also avoids coupling bindgen's libclang version to the host C++
    // standard-library implementation.
    let mut bindings_builder = bindgen::Builder::default()
        .header(wrapper_path.display().to_string())
        .use_core()
        .prepend_enum_name(false)
        .merge_extern_blocks(true)
        .derive_default(true)
        .allowlist_type(".*MsgPayload|.*Msg_C|MsgHeader|BSKLogger|logLevel_t")
        .allowlist_function(".*Msg_C_.*")
        .opaque_type("BSKLogger")
        .clang_arg(format!("-I{}", source_root.display()))
        .clang_arg(format!("-I{}", c_message_dir.display()))
        .clang_args(["-std=c11", "-x", "c"]);
    if let Some(include_dirs) = env::var_os("BSK_C_SYSTEM_INCLUDE_DIRS") {
        for include_dir in env::split_paths(&include_dirs) {
            bindings_builder = bindings_builder
                .clang_arg("-isystem")
                .clang_arg(include_dir.to_string_lossy().into_owned());
        }
    }
    let bindings = bindings_builder
        .generate()
        .map_err(|_| "bindgen could not parse the generated Basilisk C message headers")?;

    let mut generated = rewrite_binding_names(&bindings.to_string())?;
    generated.push('\n');
    generated.push_str(&render_message_trait_impls(&generated)?);

    fs::write(output_dir.join(GENERATED_BINDINGS_FILE), generated)?;
    Ok(())
}

fn rewrite_binding_names(bindings: &str) -> Result<String, Box<dyn Error>> {
    let tokens = bindings.parse::<TokenStream>()?;
    let rewritten = rewrite_token_stream(tokens);
    let syntax = syn::parse2::<syn::File>(rewritten)?;
    Ok(prettyplease::unparse(&syntax))
}

fn rewrite_token_stream(tokens: TokenStream) -> TokenStream {
    tokens
        .into_iter()
        .map(|token| match token {
            TokenTree::Group(group) => {
                let mut rewritten =
                    Group::new(group.delimiter(), rewrite_token_stream(group.stream()));
                rewritten.set_span(group.span());
                TokenTree::Group(rewritten)
            }
            TokenTree::Ident(identifier) => {
                let original = identifier.to_string();
                let rewritten = match original.as_str() {
                    "payload" => "data".to_owned(),
                    "payloadPointer" => "dataPointer".to_owned(),
                    _ => original
                        .strip_suffix("MsgPayload")
                        .map_or(original.clone(), |prefix| format!("{prefix}Msg")),
                };
                TokenTree::Ident(Ident::new(&rewritten, identifier.span()))
            }
            TokenTree::Literal(literal) => TokenTree::Literal(rewrite_string_literal(literal)),
            other => other,
        })
        .collect()
}

fn rewrite_string_literal(literal: Literal) -> Literal {
    let Ok(value) = syn::parse_str::<syn::LitStr>(&literal.to_string()) else {
        return literal;
    };
    let rewritten = value
        .value()
        .replace("MsgPayload", "Msg")
        .replace("::payloadPointer", "::dataPointer")
        .replace("::payload", "::data");
    let mut rewritten_literal = Literal::string(&rewritten);
    rewritten_literal.set_span(literal.span());
    rewritten_literal
}

fn env_path(variable: &str) -> Option<PathBuf> {
    env::var_os(variable)
        .filter(|value| !value.is_empty())
        .map(PathBuf::from)
}

fn render_wrapper(headers: &[PathBuf]) -> String {
    let mut wrapper = String::from(
        "/* Auto-generated bsk-messages bindgen wrapper. */\n\
         #include <stdint.h>\n\
         #include <stddef.h>\n\n",
    );
    for header in headers {
        let normalized = header.to_string_lossy().replace('\\', "/");
        wrapper.push_str(&format!("#include \"{normalized}\"\n"));
    }
    wrapper
}

fn render_message_trait_impls(bindings: &str) -> Result<String, Box<dyn Error>> {
    let message_pattern = Regex::new(r"pub struct (\w+Msg)_C\b")?;
    let message_types = message_pattern
        .captures_iter(bindings)
        .map(|captures| captures[1].to_owned())
        .collect::<BTreeSet<_>>();
    if message_types.is_empty() {
        return Err("bindgen did not produce any Basilisk message types".into());
    }

    let mut implementations =
        String::from("// Auto-generated `Msg` trait impl for every Basilisk message type.\n");
    for message_type in message_types {
        writeln!(implementations, "impl Msg for {message_type} {{")?;
        writeln!(implementations, "    type Port = {message_type}_C;")?;
        writeln!(implementations, "    #[inline(always)]")?;
        writeln!(
            implementations,
            "    fn __is_linked(port: &mut {message_type}_C) -> bool {{ unsafe {{ \
             {message_type}_C_isLinked(port) != 0 }} }}"
        )?;
        writeln!(implementations, "    #[inline(always)]")?;
        writeln!(
            implementations,
            "    fn __read(port: &mut {message_type}_C) -> Self {{ unsafe {{ \
             {message_type}_C_read(port) }} }}"
        )?;
        writeln!(implementations, "    #[inline(always)]")?;
        writeln!(
            implementations,
            "    fn __init(port: &mut {message_type}_C) {{ unsafe {{ \
             {message_type}_C_init(port) }} }}"
        )?;
        writeln!(implementations, "    #[inline(always)]")?;
        writeln!(
            implementations,
            "    fn __write(data: &Self, port: &mut {message_type}_C, module_id: i64, \
             current_sim_nanos: u64) {{"
        )?;
        writeln!(
            implementations,
            "        unsafe {{ {message_type}_C_write(data as *const {message_type} as \
             *mut {message_type}, port, module_id, current_sim_nanos) }}"
        )?;
        writeln!(implementations, "    }}")?;
        writeln!(implementations, "}}")?;
    }
    Ok(implementations)
}

fn configure_bundled_libclang() {
    // Xcode's current SDK headers can use compiler builtins that are unknown
    // to the libclang bundled in the Python wheel. Let clang-sys locate
    // Xcode's matching library on macOS instead.
    if cfg!(target_os = "macos") || env::var_os("LIBCLANG_PATH").is_some() {
        return;
    }

    for variable in [
        "VIRTUAL_ENV",
        "CONDA_PREFIX",
        "pythonLocation",
        "Python_ROOT_DIR",
    ] {
        let Some(prefix) = env_path(variable) else {
            continue;
        };
        if let Some(native_dir) = find_python_libclang(&prefix) {
            env::set_var("LIBCLANG_PATH", native_dir);
            return;
        }
    }
}

fn find_python_libclang(prefix: &Path) -> Option<PathBuf> {
    let windows_candidate = prefix.join("Lib/site-packages/clang/native");
    if contains_libclang(&windows_candidate) {
        return Some(windows_candidate);
    }

    let lib_dir = prefix.join("lib");
    for entry in fs::read_dir(lib_dir).ok()?.filter_map(Result::ok) {
        let candidate = entry.path().join("site-packages/clang/native");
        if contains_libclang(&candidate) {
            return Some(candidate);
        }
    }
    None
}

fn contains_libclang(directory: &Path) -> bool {
    fs::read_dir(directory)
        .ok()
        .into_iter()
        .flatten()
        .filter_map(Result::ok)
        .any(|entry| {
            entry
                .file_name()
                .to_string_lossy()
                .to_ascii_lowercase()
                .starts_with("libclang")
        })
}
