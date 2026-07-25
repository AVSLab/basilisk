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

use std::env;
use std::path::PathBuf;

fn main() {
    if env::var_os("CARGO_FEATURE_FFI_TESTS").is_none() {
        return;
    }

    let manifest_dir =
        PathBuf::from(env::var_os("CARGO_MANIFEST_DIR").expect("Cargo manifest directory"));
    let source_root = manifest_dir.join("../../..");
    let utilities = source_root.join("architecture/utilities");
    let sources = [
        utilities.join("linearAlgebra.c"),
        utilities.join("orbitalMotion.c"),
        utilities.join("rigidBodyKinematics.c"),
    ];

    let mut build = cc::Build::new();
    build.include(&source_root).warnings(false);
    if env::var("CARGO_CFG_TARGET_ENV").as_deref() == Ok("msvc") {
        // Match the definition applied by Basilisk's CMake configuration so
        // MSVC's math header exposes constants such as M_PI.
        build.std("c17").define("_USE_MATH_DEFINES", None);
    } else {
        // Basilisk's GCC and Clang builds use GNU C17. The GNU extensions
        // expose the math constants used by the existing C utilities.
        build.std("gnu17");
    }
    for source in &sources {
        println!("cargo:rerun-if-changed={}", source.display());
        build.file(source);
    }
    build.compile("bsk_utilities_ffi_tests");
}
