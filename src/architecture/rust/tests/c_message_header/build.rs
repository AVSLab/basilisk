/*
 ISC License

 Copyright (c) 2026, Care Weather Technologies

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

use std::path::PathBuf;

fn main() {
    println!("cargo:rustc-check-cfg=cfg(link_cmsg)");
    let manifest_dir = PathBuf::from(std::env::var("CARGO_MANIFEST_DIR").unwrap());
    let repository = manifest_dir.join("../../../../..");
    let archive = repository.join("dist3/Basilisk/libcMsgCInterface.a");
    println!("cargo:rerun-if-changed={}", archive.display());
    if archive.is_file() {
        let directory = archive
            .parent()
            .expect("archive path includes its directory");
        println!("cargo:rustc-cfg=link_cmsg");
        println!("cargo:rustc-link-search=native={}", directory.display());
        println!("cargo:rustc-link-lib=static=cMsgCInterface");
        return;
    }
    if repository.join("dist3/CMakeCache.txt").is_file() {
        panic!(
            "dist3 is configured but {} is missing; rebuild Basilisk before testing message headers",
            archive.display()
        );
    }
}
