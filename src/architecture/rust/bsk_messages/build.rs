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

use regex::Regex;
use std::{
    collections::BTreeSet,
    env,
    error::Error,
    ffi::OsStr,
    fmt::Write as _,
    fs, io,
    path::{Path, PathBuf},
};

mod build_support;

use build_support::MessageBindingCallbacks;

const GENERATED_BINDINGS_FILE: &str = "bsk_message_bindings.rs";

fn main() {
    if let Err(error) = generate_bindings() {
        panic!("bsk-messages could not generate Rust message bindings: {error}");
    }
}

fn generate_bindings() -> Result<(), Box<dyn Error>> {
    for variable in [
        "BSK_CMSG_DIR",
        "BSK_CMSG_DIRS",
        "BSK_CMSG_DIRS_FILE",
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
    let explicit_source_root = env_path("BSK_SRC_ROOT");
    let explicit_c_message_dirs = match env_path("BSK_CMSG_DIRS_FILE") {
        Some(path) => Some(read_message_directory_file(&path)?),
        None => {
            env_paths("BSK_CMSG_DIRS").or_else(|| env_path("BSK_CMSG_DIR").map(|path| vec![path]))
        }
    };
    let repository_root = if explicit_source_root.is_none() || explicit_c_message_dirs.is_none() {
        Some(
            manifest_dir
                .ancestors()
                .nth(4)
                .ok_or("bsk-messages is not located under the Basilisk src tree")?,
        )
    } else {
        None
    };
    let source_root = explicit_source_root.unwrap_or_else(|| {
        repository_root
            .expect("repository fallback required for the Basilisk source root")
            .join("src")
    });
    let c_message_dirs = explicit_c_message_dirs.unwrap_or_else(|| {
        vec![repository_root
            .expect("repository fallback required for generated C messages")
            .join("dist3/autoSource/cMsgCInterface")]
    });

    if !source_root.is_dir() {
        return Err(format!(
            "Basilisk source directory does not exist: {}",
            source_root.display()
        )
        .into());
    }
    let mut headers_by_name = std::collections::BTreeMap::<String, PathBuf>::new();
    for c_message_dir in &c_message_dirs {
        if !c_message_dir.is_dir() {
            return Err(format!(
                "generated C message directory does not exist: {}. Run \
                 `python3 conanfile.py` first; Rust modules need not be enabled",
                c_message_dir.display()
            )
            .into());
        }
        // Watching the directory catches newly added or removed message headers;
        // watching each file below catches content changes with precise paths in
        // Cargo's diagnostics.
        println!("cargo:rerun-if-changed={}", c_message_dir.display());
        for header in fs::read_dir(c_message_dir)?
            .filter_map(Result::ok)
            .map(|entry| entry.path())
            .filter(|path| {
                path.extension() == Some(OsStr::new("h"))
                    && path
                        .file_stem()
                        .is_some_and(|stem| stem.to_string_lossy().ends_with("_C"))
            })
        {
            let name = header
                .file_name()
                .and_then(OsStr::to_str)
                .ok_or_else(|| {
                    format!(
                        "generated C message header has a non-UTF-8 name: {}",
                        header.display()
                    )
                })?
                .to_owned();
            if let Some(existing) = headers_by_name.insert(name.clone(), header.clone()) {
                return Err(format!(
                    "duplicate C message interface `{name}` was found at {} and {}",
                    existing.display(),
                    header.display()
                )
                .into());
            }
        }
    }
    let headers = headers_by_name.into_values().collect::<Vec<_>>();
    if headers.is_empty() {
        let searched = c_message_dirs
            .iter()
            .map(|path| path.display().to_string())
            .collect::<Vec<_>>()
            .join(", ");
        return Err(
            format!("no generated *_C.h message headers were found under {searched}").into(),
        );
    }
    for header in &headers {
        println!("cargo:rerun-if-changed={}", header.display());
    }

    let output_dir = PathBuf::from(
        env::var_os("OUT_DIR").ok_or("Cargo did not provide OUT_DIR to bsk-messages")?,
    );
    let wrapper_path = output_dir.join("bsk_messages_wrapper.h");
    write_if_changed(&wrapper_path, &render_wrapper(&headers))?;

    // Rust modules bind only Basilisk's C message ABI. Parsing these headers
    // as C also avoids coupling bindgen's libclang version to the host C++
    // standard-library implementation.
    let mut bindings_builder = bindgen::Builder::default()
        .formatter(bindgen::Formatter::Prettyplease)
        .header(wrapper_path.display().to_string())
        // Track every header libclang reads, including payload headers included
        // transitively by generated *_C.h files. Without these callbacks Cargo
        // can reuse bindings after an extension edits an existing payload.
        .parse_callbacks(Box::new(bindgen::CargoCallbacks::new()))
        .parse_callbacks(Box::new(MessageBindingCallbacks))
        .use_core()
        .prepend_enum_name(false)
        .merge_extern_blocks(true)
        .derive_default(true)
        .allowlist_type(".*MsgPayload|.*Msg_C|MsgHeader|BSKLogger|logLevel_t")
        .allowlist_function(".*Msg_C_.*")
        .opaque_type("BSKLogger")
        .clang_arg(format!("-I{}", source_root.display()))
        .clang_args(["-std=c11", "-x", "c"]);
    for c_message_dir in &c_message_dirs {
        bindings_builder = bindings_builder.clang_arg(format!("-I{}", c_message_dir.display()));
    }
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

    let mut generated = bindings.to_string();
    generated.push('\n');
    generated.push_str(&render_message_trait_impls(&generated)?);

    write_if_changed(&output_dir.join(GENERATED_BINDINGS_FILE), &generated)?;
    Ok(())
}

fn write_if_changed(path: &Path, contents: &str) -> io::Result<()> {
    if fs::read_to_string(path).is_ok_and(|existing| existing == contents) {
        return Ok(());
    }
    fs::write(path, contents)
}

fn env_path(variable: &str) -> Option<PathBuf> {
    env::var_os(variable)
        .filter(|value| !value.is_empty())
        .map(PathBuf::from)
}

fn env_paths(variable: &str) -> Option<Vec<PathBuf>> {
    env::var_os(variable)
        .filter(|value| !value.is_empty())
        .map(|value| env::split_paths(&value).collect())
}

fn read_message_directory_file(path: &Path) -> Result<Vec<PathBuf>, Box<dyn Error>> {
    // CMake supplies one path per line so native path-list separators never
    // have to cross its command/list boundary.
    println!("cargo:rerun-if-changed={}", path.display());
    let contents = fs::read_to_string(path).map_err(|error| {
        format!(
            "could not read C-message directory file {}: {error}",
            path.display()
        )
    })?;
    let directories = contents
        .lines()
        .filter(|line| !line.trim().is_empty())
        .map(PathBuf::from)
        .collect::<Vec<_>>();
    if directories.is_empty() {
        return Err(format!(
            "C-message directory file {} did not contain any paths",
            path.display()
        )
        .into());
    }
    Ok(directories)
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
        writeln!(
            implementations,
            "// SAFETY: bindgen generated both `{message_type}` and `{message_type}_C` from the"
        )?;
        writeln!(
            implementations,
            "// same Basilisk C-message header, and every method delegates to its matching C function."
        )?;
        writeln!(implementations, "unsafe impl Msg for {message_type} {{")?;
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
            "    fn __is_initialized(port: &{message_type}_C) -> bool {{ \
             !port.payloadPointer.is_null() && !port.headerPointer.is_null() }}"
        )?;
        writeln!(implementations, "    #[inline(always)]")?;
        writeln!(
            implementations,
            "    fn __port_pointers(port: &{message_type}_C) -> (*const (), *const ()) {{ \
             (port.payloadPointer.cast::<()>() as *const (), \
              port.headerPointer.cast::<()>() as *const ()) }}"
        )?;
        writeln!(implementations, "    #[inline(always)]")?;
        writeln!(
            implementations,
            "    unsafe fn __read(port: &mut {message_type}_C) -> Self {{ unsafe {{ \
             {message_type}_C_read(port) }} }}"
        )?;
        writeln!(implementations, "    #[inline(always)]")?;
        writeln!(
            implementations,
            "    unsafe fn __init(port: &mut {message_type}_C) {{ unsafe {{ \
             {message_type}_C_init(port) }} }}"
        )?;
        writeln!(implementations, "    #[inline(always)]")?;
        writeln!(
            implementations,
            "    unsafe fn __write(data: &Self, port: &mut {message_type}_C, module_id: i64, \
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
