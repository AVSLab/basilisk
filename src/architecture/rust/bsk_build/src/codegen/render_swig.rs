//  ISC License
//
//  Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
//
//  Permission to use, copy, modify, and/or distribute this software for any
//  purpose with or without fee is hereby granted, provided that the above
//  copyright notice and this permission notice appear in all copies.

//! SWIG interface renderer: turns a [`ConfigInfo`] into the
//! `<ModuleName>_rust_wrap.i` file CMake points SWIG at.

use std::collections::BTreeSet;
use std::path::Path;

use super::types::ConfigInfo;

/// Renders the SWIG `.i` file `bsk_add_rust_module.cmake` points SWIG at
/// (see the `BSK_INTERFACE_PATH` handling in `generate`). This is the only
/// generator of Rust-module `.i` files — there is no hand-written-`.i`
/// escape hatch (unlike C modules' `%c_wrap` family in `swig_c_wrap.i`) — so
/// the boilerplate that wraps `RustWrapper` around the module's lifecycle
/// functions is written out directly here instead of via a SWIG macro.
pub(super) fn render_swig_interface(info: &ConfigInfo, module: &str, header_path: &Path) -> String {
    let cfg_type = &info.struct_name;
    let header_abs = header_path.display();

    // %import registers each message type in SWIG's type system so that
    // struct member getters return a pointer to the embedded field
    // (in-place access) rather than a by-value copy. Without this,
    // `ctrl.attGuidInMsg.subscribeTo()` would silently modify a temporary
    // copy instead of the real field. `BTreeSet` both de-duplicates (two
    // ports of the same message type) and gives a stable order.
    let msg_imports: BTreeSet<&str> = info
        .fields
        .iter()
        .filter(|f| f.is_in_msg() || f.is_out_msg())
        .map(|f| f.c_type.as_str())
        .collect();
    let mut import_lines = String::new();
    for t in &msg_imports {
        import_lines.push_str(&format!("%import \"cMsgCInterface/{t}.h\"\n"));
    }

    // Owned-state fields (Rust `Option<Box<T>>`) are Rust-internal; nothing
    // else may safely write to them. `Delete_<module>` runs Rust's drop glue,
    // so a Python-side write (even just `= None`) either leaks the previously
    // owned box or, if aliased to some other pointer entirely, frees memory
    // Rust's allocator never allocated. `%immutable` hides only the setter;
    // the (harmless) getter is unaffected. Must appear before `%include` so
    // SWIG applies it while parsing the struct.
    let mut immutable_lines = String::new();
    for f in info.fields.iter().filter(|f| f.is_owned_state) {
        immutable_lines.push_str(&format!("%immutable {cfg_type}::{};\n", f.name));
    }

    // Each config method's raw extern "C-unwind" shim (see
    // `methods::find_config_methods`) is only ever called from its own
    // inline C++ member function definition (`render_header::render_method_inline_def`);
    // hide it from Python the same way New_/Delete_/etc. are hidden, so only
    // the member function itself (forwarded to Python via
    // `RustWrapper::operator->()`) is reachable.
    let mut method_ignores = String::new();
    for m in &info.methods {
        method_ignores.push_str(&format!("%ignore {}_{module};\n", m.name));
    }

    format!(
        "%module {module}\n\
         %{{\n\
         #include \"{header_abs}\"\n\
         %}}\n\
         \n\
         /* BSK_RUST_DECL emits extern \"C\" declarations for the C compiler; the\n\
          * %ignore lines below hide them from Python and SWIG never needs to see\n\
          * the declarations themselves. Define it empty here (matching its real\n\
          * 2-arg signature) so headers that use it parse cleanly under SWIG. */\n\
         #define BSK_RUST_DECL(name, configType)\n\
         \n\
         %include \"swig_c_wrap.i\"\n\
         \n\
         {import_lines}\n\
         {immutable_lines}\n\
         %nodefaultctor {cfg_type};\n\
         %nodefaultdtor {cfg_type};\n\
         %include \"{header_abs}\"\n\
         \n\
         %ignore New_{module};\n\
         %ignore Delete_{module};\n\
         %ignore Update_{module};\n\
         %ignore SelfInit_{module};\n\
         %ignore Reset_{module};\n\
         {method_ignores}\
         \n\
         // Default no-op Reset. Templated so it loses overload resolution to a\n\
         // real Reset_{module} the module may define (same trick %c_wrap_3 uses\n\
         // for C modules — see swig_c_wrap.i).\n\
         %inline %{{\n\
         \x20 template <typename T> inline void Reset_{module}(T, uint64_t, const BskRustModuleRuntime*) {{}}\n\
         %}}\n\
         \n\
         // RustWrapper(TConfig* config) takes ownership of the pointer; the\n\
         // Python object for that config shouldn't also think it owns the memory.\n\
         %pythonappend RustWrapper::RustWrapper %{{\n\
         \x20   if (len(args)) > 0:\n\
         \x20       args[0].thisown = False\n\
         %}}\n\
         \n\
         %template({module}) RustWrapper<{cfg_type},New_{module},Delete_{module},Update_{module},SelfInit_{module},Reset_{module}>;\n\
         \n\
         %extend {cfg_type} {{\n\
         \x20 {cfg_type}() {{\n\
         \x20   return New_{module}();\n\
         \x20 }}\n\
         \x20 ~{cfg_type}() {{\n\
         \x20   Delete_{module}($self);\n\
         \x20 }}\n\
         \x20 %pythoncode %{{\n\
         \x20   def createWrapper(self):\n\
         \x20       return {module}(self)\n\
         \x20 %}}\n\
         }}\n"
    )
}

#[cfg(test)]
mod tests {
    use super::super::test_support::info_for;
    use super::*;

    /// The generated `.i` file must `%import` each message type exactly
    /// once (even with two ports of the same type), `%immutable` each
    /// owned-state field, and instantiate `RustWrapper` with the config
    /// struct's real name — the whole point of generating this from `info`
    /// instead of a CMake-side regex scan (see `bsk_add_rust_module.cmake`).
    #[test]
    fn swig_interface_imports_dedup_and_marks_owned_state_immutable() {
        let info = info_for(
            "#[repr(C)] pub struct MyController { \
             pub runtime: bsk_messages::BskModuleRuntime, \
             pub state: Option<Box<MyState>>, \
             pub attGuidInMsg: MsgReader<AttGuidMsg>, \
             pub cmdTorqueOutMsg: MsgWriter<CmdTorqueBodyMsg>, \
             pub feedforwardTorqueInMsg: MsgReader<CmdTorqueBodyMsg>, \
             }",
        );
        let i_file = render_swig_interface(&info, "myModule", Path::new("/tmp/myModule.h"));

        assert!(i_file.contains("%module myModule"));
        assert!(i_file.contains("#include \"/tmp/myModule.h\""));
        assert!(i_file.contains(
            "%template(myModule) RustWrapper<MyController,New_myModule,Delete_myModule,Update_myModule,SelfInit_myModule,Reset_myModule>;"
        ));
        assert!(i_file.contains("%nodefaultctor MyController;"));
        assert!(i_file.contains("%nodefaultdtor MyController;"));
        assert!(i_file.contains("%ignore New_myModule;"));
        assert!(i_file.contains("%ignore Delete_myModule;"));
        assert!(i_file.contains("return New_myModule();"));
        assert!(i_file.contains("Delete_myModule($self);"));
        assert!(
            i_file.contains("%import \"cMsgCInterface/CmdTorqueBodyMsg_C.h\""),
            "i_file:\n{i_file}"
        );
        // Only one %import per distinct message type, despite two ports.
        assert_eq!(i_file.matches("CmdTorqueBodyMsg_C.h").count(), 1);
        assert!(i_file.contains("%import \"cMsgCInterface/AttGuidMsg_C.h\""));
        assert!(i_file.contains("%immutable MyController::state;"));
    }
}
