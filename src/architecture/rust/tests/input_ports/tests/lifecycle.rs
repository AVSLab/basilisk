// ISC License
//
// Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
//
// Permission to use, copy, modify, and/or distribute this software for any
// purpose with or without fee is hereby granted, provided that the above
// copyright notice and this permission notice appear in all copies.

//! Source-lifetime regressions through actual generated FFI entry points.
use bsk_build::{
    BskModuleContext, BskRustError, BskRustErrorKind, BskRustError_kind, BskRustError_message,
    Destroy_BskRustError, MsgReader,
};
use bsk_input_port_tests::*;

fn result(error: *mut BskRustError) -> Result<(), (BskRustErrorKind, String)> {
    if error.is_null() {
        return Ok(());
    }
    // SAFETY: The fixture returned this owning error handle; copy its diagnostic
    // before releasing it exactly once.
    unsafe {
        let kind = BskRustError_kind(error);
        let message = std::ffi::CStr::from_ptr(BskRustError_message(error))
            .to_string_lossy()
            .into_owned();
        Destroy_BskRustError(error);
        Err((kind, message))
    }
}

struct Module {
    handle: *mut ReaderConfigHandle,
    context: BskModuleContext,
}

impl Module {
    fn new() -> Self {
        // SAFETY: The context's integer and raw pointer fields all permit zero.
        // The mock module does not log or require a model tag.
        let context = unsafe { std::mem::zeroed() };
        let mut handle = std::ptr::null_mut();
        // SAFETY: The handle output and borrowed context remain valid throughout
        // each ABI call. This helper owns and later destroys the created handle.
        unsafe {
            result(Create_bsk_input_port_tests(&mut handle)).unwrap();
            result(SelfInit_bsk_input_port_tests(handle, &context)).unwrap();
        }
        Self { handle, context }
    }

    fn config(&mut self) -> &mut ReaderConfig {
        // SAFETY: The handle is live and exclusively borrowed through this helper.
        unsafe { &mut *Config_bsk_input_port_tests(self.handle) }
    }

    fn call(&mut self, update: bool) -> Result<(), (BskRustErrorKind, String)> {
        let time = 0; // [ns]
                      // SAFETY: Tests retain all subscribed sources for the call and only
                      // change subscriptions between calls, as the C++/Python caller does.
        unsafe {
            result(if update {
                Update_bsk_input_port_tests(self.handle, time, &self.context)
            } else {
                Reset_bsk_input_port_tests(self.handle, time, &self.context)
            })
        }
    }
}

impl Drop for Module {
    fn drop(&mut self) {
        // SAFETY: This helper uniquely owns the handle, even after a panic.
        result(unsafe { Destroy_bsk_input_port_tests(self.handle) }).unwrap();
    }
}

/// Snapshot pointer identity without touching the source allocation.
fn pointers(reader: &mut MsgReader<TestMessage>) -> (*const (), *const ()) {
    use bsk_build::Msg;
    // SAFETY: MsgReader is repr(transparent) over its declared Port type.
    let port = unsafe { &*(reader as *mut MsgReader<TestMessage>).cast::<TestPort>() };
    TestMessage::__port_pointers(port)
}

#[test]
/// Supported subscriptions refresh between calls, and a harmless round-trip move is allowed.
fn original_reader_and_automatic_inputs_follow_external_rebinding() {
    for update in [false, true] {
        let first = Source::new(17);
        let second = Source::new(29);
        let mut module = Module::new();
        // SAFETY: Both sources outlive their subscriptions and callbacks.
        unsafe { first.subscribe(&mut module.config().dataInMsg) };
        module.call(update).unwrap();
        assert_eq!(module.config().value, 17);
        if update {
            assert_eq!(module.config().automaticValue, 17);
        }
        unsafe { second.subscribe(&mut module.config().dataInMsg) };
        drop(first);
        module.call(update).unwrap();
        assert_eq!(module.config().value, 29);
        if update {
            assert_eq!(module.config().automaticValue, 29);
        }
        module.config().action = 6;
        module.call(update).unwrap();
        assert_eq!(module.config().value, 29);
    }
}

#[test]
/// Moving out, unsubscribing, and moving back cannot bypass the live-source check.
fn retained_reader_cannot_read_after_original_source_is_released() {
    for update in [false, true] {
        let source = Source::new(17);
        let mut module = Module::new();
        // SAFETY: Source remains alive until the original slot is unsubscribed.
        unsafe { source.subscribe(&mut module.config().dataInMsg) };
        let original = pointers(&mut module.config().dataInMsg);
        module.config().action = 1;
        let error = module.call(update).unwrap_err();
        assert!(error.1.contains("subscription has been restored"));
        assert!(module.config().rejected);
        assert_eq!(pointers(&mut module.config().dataInMsg), original);

        // Simulate Python unsubscribe releasing its keep-alive reference.
        module.config().dataInMsg = MsgReader::default();
        drop(source);
        module.config().action = 4;
        let before = read_count();
        module.call(update).unwrap();
        assert!(module.config().rejected);
        assert_eq!(
            read_count(),
            before,
            "retained reader must not enter the raw read"
        );

        // Reinstalling the stale reader in the original slot must not bless it
        // for this callback or for the next callback's newly captured context.
        module.config().action = 3;
        let error = module.call(update).unwrap_err();
        assert!(error.1.contains("subscription has been restored"));
        assert!(module.config().rejected);
        assert!(!module.config().dataInMsg.is_linked());
        assert_eq!(read_count(), before);
        module.config().action = 0;
        assert!(module.call(update).unwrap_err().1.contains("unlinked"));
        assert_eq!(read_count(), before);
    }
}

#[test]
/// Cleanup restores all changed slots and preserves the callback's error or panic policy.
fn swaps_restore_every_scalar_and_array_slot_on_success_error_or_panic() {
    for update in [false, true] {
        for failure in [0, 1, 2] {
            let sources = [Source::new(11), Source::new(22), Source::new(33)];
            let mut module = Module::new();
            // SAFETY: These sources outlive the module and all callbacks.
            unsafe {
                sources[0].subscribe(&mut module.config().dataInMsg);
                sources[1].subscribe(&mut module.config().dataInMsgs[0]);
                sources[2].subscribe(&mut module.config().dataInMsgs[1]);
            }
            let original = [
                pointers(&mut module.config().dataInMsg),
                pointers(&mut module.config().dataInMsgs[0]),
                pointers(&mut module.config().dataInMsgs[1]),
            ];
            module.config().action = 2;
            module.config().failure = failure;
            let error = module.call(update).unwrap_err();
            assert!(module.config().rejected);
            assert_eq!(pointers(&mut module.config().dataInMsg), original[0]);
            assert_eq!(pointers(&mut module.config().dataInMsgs[0]), original[1]);
            assert_eq!(pointers(&mut module.config().dataInMsgs[1]), original[2]);
            match failure {
                0 => assert!(error.1.contains("subscription has been restored")),
                1 => assert_eq!(error.1, "intentional callback error"),
                2 => {
                    assert_eq!(error.0, BskRustErrorKind::Panic);
                    assert!(error.1.contains("intentional callback panic"));
                }
                _ => unreachable!(),
            }
            module.config().action = 0;
            module.config().failure = 0;
            if failure == 2 {
                assert!(module
                    .call(update)
                    .unwrap_err()
                    .1
                    .contains("previous panic"));
            } else {
                module.call(update).unwrap();
                assert_eq!(module.config().value, 11);
            }
        }
    }
}

#[test]
/// Even default replacement must leave Python's retained subscription intact on return.
fn replacing_a_reader_with_default_restores_its_subscription() {
    for update in [false, true] {
        let source = Source::new(17);
        let mut module = Module::new();
        // SAFETY: Source outlives the module.
        unsafe { source.subscribe(&mut module.config().dataInMsg) };
        let original = pointers(&mut module.config().dataInMsg);
        module.config().action = 5;
        assert!(module
            .call(update)
            .unwrap_err()
            .1
            .contains("subscription has been restored"));
        assert_eq!(pointers(&mut module.config().dataInMsg), original);
        module.config().action = 0;
        module.call(update).unwrap();
        assert_eq!(module.config().value, 17);
    }
}

#[test]
/// Init cannot turn readers saved from a destroyed instance into valid subscriptions.
fn init_rejects_retained_scalar_and_array_readers_before_exposing_a_handle() {
    for slot in 0..4 {
        for failure in 0..3 {
            let sources = [Source::new(11), Source::new(22), Source::new(33)];
            let mut first = Module::new();
            // SAFETY: Sources outlive the first instance's subscriptions and calls.
            unsafe {
                sources[0].subscribe(&mut first.config().dataInMsg);
                sources[1].subscribe(&mut first.config().dataInMsgs[0]);
                sources[2].subscribe(&mut first.config().dataInMsgs[1]);
            }
            first.config().action = 7;
            assert!(first
                .call(false)
                .unwrap_err()
                .1
                .contains("subscription has been restored"));
            drop(first);
            drop(sources);

            set_init_behavior(slot, failure);
            let before = read_count();
            let mut handle = std::ptr::null_mut();
            // SAFETY: The output slot is valid. Module callbacks use only safe
            // Rust; the mock message never dereferences released source memory.
            let creation = result(unsafe { Create_bsk_input_port_tests(&mut handle) });
            let exposed_handle = !handle.is_null();
            if exposed_handle {
                // Avoid leaking an incorrectly accepted instance if this regresses.
                result(unsafe { Destroy_bsk_input_port_tests(handle) }).unwrap();
            }
            let error = creation.expect_err("init must not accept a retained input reader");
            assert!(
                !exposed_handle,
                "failed construction must leave a null handle"
            );
            assert_eq!(read_count(), before, "init must never enter the raw reader");
            assert_eq!(
                dropped_input_links(),
                Some([false; 3]),
                "restore every scalar/array slot before dropping a rejected instance"
            );
            match failure {
                0 => {
                    assert_eq!(error.0, BskRustErrorKind::Expected);
                    assert!(error.1.contains("subscription has been restored"));
                }
                1 => {
                    assert_eq!(error.0, BskRustErrorKind::Expected);
                    assert_eq!(error.1, "intentional init error");
                }
                2 => {
                    assert_eq!(error.0, BskRustErrorKind::Panic);
                    assert!(error.1.contains("intentional init panic"));
                }
                _ => unreachable!(),
            }

            // Rejection must not poison future instances or change normal
            // caller-owned subscriptions, which are established after init.
            let source = Source::new(47);
            let mut fresh = Module::new();
            unsafe { source.subscribe(&mut fresh.config().dataInMsg) };
            fresh.call(true).unwrap();
            assert_eq!(fresh.config().automaticValue, 47);
        }
    }
}
