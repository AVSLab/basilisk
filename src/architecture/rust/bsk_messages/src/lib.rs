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

#![allow(
    non_snake_case,
    non_camel_case_types,
    non_upper_case_globals,
    dead_code,
    clippy::all
)]

pub use bsk_build::{
    BskConfigValue, BskContext, BskError, BskLoggerRef, BskModule, BskModuleRuntime, BskResult,
    Msg, MsgReader, MsgWriter,
};

#[cfg(test)]
#[path = "../build_support.rs"]
mod build_support;

include!(concat!(env!("OUT_DIR"), "/bsk_message_bindings.rs"));

#[cfg(test)]
mod input_subscription_tests {
    use super::*;

    /// The generated restore operation changes only subscription metadata.
    #[test]
    fn restore_preserves_inline_payload_and_source_contents() {
        let mut source = CModuleTemplateMsg_C::default();
        source.payload.dataVector = [1.0, 2.0, 3.0]; // [-]
        source.header.isWritten = 1;
        source.header.timeWritten = 27; // [ns]
        let data_pointer = core::ptr::addr_of_mut!(source.payload);
        let header_pointer = core::ptr::addr_of_mut!(source.header);
        let mut reader = CModuleTemplateMsg_C::default();
        reader.payload.dataVector = [4.0, 5.0, 6.0]; // [-]
        reader.header.timeWritten = 19; // [ns]
        reader.header.moduleID = 42;

        for linked in [true, false] {
            // SAFETY: Both pointers describe the same live source allocation;
            // no source pointer may be dereferenced by this operation.
            unsafe {
                CModuleTemplateMsg::__restore_subscription(
                    &mut reader,
                    data_pointer.cast(),
                    header_pointer.cast(),
                    linked,
                );
            }
            assert_eq!(reader.payloadPointer, data_pointer);
            assert_eq!(reader.headerPointer, header_pointer);
            assert_eq!(reader.header.isLinked, i64::from(linked));
            assert_eq!(reader.payload.dataVector, [4.0, 5.0, 6.0]); // [-]
            assert_eq!(reader.header.timeWritten, 19); // [ns]
            assert_eq!(reader.header.moduleID, 42);
            assert_eq!(source.payload.dataVector, [1.0, 2.0, 3.0]); // [-]
            assert_eq!(source.header.isWritten, 1);
            assert_eq!(source.header.timeWritten, 27); // [ns]
        }
    }
}
