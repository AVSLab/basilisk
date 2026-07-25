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

foreach(_required_variable
        BSK_RUST_BINDINGS_TRIGGER
        BSK_RUST_HEADER
        BSK_RUST_INTERFACE)
  if(NOT DEFINED ${_required_variable} OR "${${_required_variable}}" STREQUAL "")
    message(FATAL_ERROR
      "bskEnsureRustBindings.cmake requires ${_required_variable}")
  endif()
endforeach()

# Cargo does not know that build.rs writes these files outside OUT_DIR. Keep a
# stable watched file, and change it when a generated binding is removed while
# Cargo's own artifacts remain current. Cargo then reruns build.rs and restores
# both files.
if(NOT EXISTS "${BSK_RUST_BINDINGS_TRIGGER}")
  get_filename_component(_trigger_directory
                         "${BSK_RUST_BINDINGS_TRIGGER}"
                         DIRECTORY)
  file(MAKE_DIRECTORY "${_trigger_directory}")
  file(WRITE "${BSK_RUST_BINDINGS_TRIGGER}" "")
endif()

if(NOT EXISTS "${BSK_RUST_HEADER}" OR NOT EXISTS "${BSK_RUST_INTERFACE}")
  file(APPEND "${BSK_RUST_BINDINGS_TRIGGER}" "regenerate\n")
endif()
