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

include_guard(GLOBAL)

# Corrosion expands this executable override as a command list, preserving the
# original Cargo arguments, environment, working directory, and dependencies.
function(bsk_configure_rust_build_output rust_target)
  get_target_property(_cargo_executable Rust::Cargo IMPORTED_LOCATION)
  if(NOT _cargo_executable OR NOT Python3_EXECUTABLE)
    message(FATAL_ERROR "Rust build output requires the selected Cargo and Python executables.")
  endif()
  set(_cargo_command "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_FUNCTION_LIST_DIR}/runCargo.py")
  cmake_language(GET_MESSAGE_LOG_LEVEL _log_level)
  if(_log_level MATCHES "^(VERBOSE|DEBUG|TRACE)$")
    list(APPEND _cargo_command --verbose)
  endif()
  list(APPEND _cargo_command "${_cargo_executable}")
  set_property(TARGET "${rust_target}" PROPERTY INTERFACE_CORROSION_CARGO "${_cargo_command}")
endfunction()
