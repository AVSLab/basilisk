# ISC License
#
# Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
#
# Permission to use, copy, modify, and/or distribute this software for any
# purpose with or without fee is hereby granted, provided that the above
# copyright notice and this permission notice appear in all copies.
#
# THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
# WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
# ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
# WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
# ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
# OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

# Single source of truth shared with bsk-sdk for the Rust build integration.
# Keep BSK_RUST_MIN_VERSION aligned with workspace.package.rust-version in
# src/Cargo.toml and the crate version aligned across the bsk-* support crates.
set(BSK_RUST_MIN_VERSION "1.89")
set(BSK_RUST_SUPPORT_CRATE_VERSION "0.1.0")

set(BSK_CORROSION_VERSION "0.6.1")
set(BSK_CORROSION_GIT_TAG "1499b14e4906a2890f5cee1547c8848db261753d")
