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

if(NOT DEFINED BSK_RUST_MODULE_FILE OR BSK_RUST_MODULE_FILE STREQUAL "")
  message(FATAL_ERROR "BSK_RUST_MODULE_FILE is required")
endif()
if(NOT DEFINED BSK_NM_EXECUTABLE OR BSK_NM_EXECUTABLE STREQUAL "")
  message(FATAL_ERROR "BSK_NM_EXECUTABLE is required")
endif()

execute_process(
  COMMAND "${BSK_NM_EXECUTABLE}" -u "${BSK_RUST_MODULE_FILE}"
  RESULT_VARIABLE _nm_result
  OUTPUT_VARIABLE _nm_output
  ERROR_VARIABLE _nm_error
)
if(NOT _nm_result EQUAL 0)
  message(FATAL_ERROR
    "Could not inspect Rust module symbols in ${BSK_RUST_MODULE_FILE}:\n"
    "${_nm_error}")
endif()

string(
  REGEX MATCHALL
  "[A-Za-z_][A-Za-z0-9_]*Msg_C_(init|isLinked|read|write)"
  _unresolved_message_symbols
  "${_nm_output}")
if(_unresolved_message_symbols)
  list(REMOVE_DUPLICATES _unresolved_message_symbols)
  list(SORT _unresolved_message_symbols)
  list(JOIN _unresolved_message_symbols "\n  " _unresolved_message_list)
  message(FATAL_ERROR
    "Rust module ${BSK_RUST_MODULE_FILE} contains unresolved Basilisk "
    "C-message symbols:\n  ${_unresolved_message_list}")
endif()
