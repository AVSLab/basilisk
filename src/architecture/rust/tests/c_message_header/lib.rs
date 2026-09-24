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

//! Header queries through the generated `Msg` impl and the built C library.
//! The mock port in `bsk-build` does not cover this path.

#[cfg(all(test, link_cmsg))]
use bsk_messages::{
    BskContext, BskModuleRuntime, CModuleTemplateMsg, CModuleTemplateMsg_C, Msg, MsgReader,
};

#[cfg(all(test, not(link_cmsg)))]
#[test]
fn c_message_library_absent() {}

#[cfg(all(test, link_cmsg))]
#[test]
fn reader_header_queries_use_the_c_message() {
    let runtime = BskModuleRuntime::for_testing();
    let mut source = CModuleTemplateMsg_C::default();
    // SAFETY: `source` is a live C message container. Init and write only
    // update that container.
    unsafe { CModuleTemplateMsg::__init(&mut source) };
    let payload = CModuleTemplateMsg {
        dataVector: [1.0, 2.0, 3.0],
    };
    let first_written_at_ns = 1_000; // [ns]
    unsafe { CModuleTemplateMsg::__write(&payload, &mut source, 7, first_written_at_ns) };

    let mut reader = MsgReader::<CModuleTemplateMsg>::default();
    // SAFETY: `reader` is transparent over `CModuleTemplateMsg_C`, and
    // `source` outlives the queries below.
    unsafe {
        bsk_messages::CModuleTemplateMsg_C_subscribe(
            core::ptr::from_mut(&mut reader).cast(),
            &mut source,
        );
    }
    let bindings = [reader.__capture_binding()];
    // SAFETY: The binding describes this reader and `source`, both still alive.
    let context = unsafe { BskContext::for_testing(&runtime).__with_input_bindings(&bindings) };
    assert!(reader
        .is_written(&context)
        .expect("subscribed source must be queryable"));
    assert_eq!(
        reader
            .time_written(&context)
            .expect("subscribed source must be queryable"),
        first_written_at_ns
    );
    assert_eq!(
        reader
            .module_id(&context)
            .expect("a written source must expose its module ID"),
        7
    );
    assert_eq!(
        reader
            .read(&context)
            .expect("written source must be readable")
            .dataVector,
        [1.0, 2.0, 3.0]
    );

    let revised = CModuleTemplateMsg {
        dataVector: [4.0, 5.0, 6.0],
    };
    let revised_written_at_ns = 2_500; // [ns]
    unsafe { CModuleTemplateMsg::__write(&revised, &mut source, 9, revised_written_at_ns) };
    assert!(reader
        .is_written(&context)
        .expect("republished source must stay written"));
    assert_eq!(
        reader
            .time_written(&context)
            .expect("republished source must be queryable"),
        revised_written_at_ns
    );
    assert_eq!(
        reader
            .module_id(&context)
            .expect("republished source must expose its module ID"),
        9
    );
    assert_eq!(
        reader
            .read(&context)
            .expect("republished source must be readable")
            .dataVector,
        [4.0, 5.0, 6.0]
    );

    let mut unpublished = CModuleTemplateMsg_C::default();
    unsafe { CModuleTemplateMsg::__init(&mut unpublished) };
    let mut unread = MsgReader::<CModuleTemplateMsg>::default();
    unsafe {
        bsk_messages::CModuleTemplateMsg_C_subscribe(
            core::ptr::from_mut(&mut unread).cast(),
            &mut unpublished,
        );
    }
    let unpublished_bindings = [unread.__capture_binding()];
    let unpublished_context =
        unsafe { BskContext::for_testing(&runtime).__with_input_bindings(&unpublished_bindings) };
    assert!(!unread
        .is_written(&unpublished_context)
        .expect("linked unpublished source must be queryable"));
    assert_eq!(
        unread
            .time_written(&unpublished_context)
            .expect("linked unpublished source must be queryable"),
        0
    );
    let error = unread
        .module_id(&unpublished_context)
        .expect_err("an unpublished source has no module ID");
    assert!(error
        .to_string()
        .contains("unwritten Basilisk input message"));
}
