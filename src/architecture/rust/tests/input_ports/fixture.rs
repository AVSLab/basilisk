// ISC License
//
// Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
//
// Permission to use, copy, modify, and/or distribute this software for any
// purpose with or without fee is hereby granted, provided that the above
// copyright notice and this permission notice appear in all copies.

//! Test-only module exercising generated lifecycle code without a C++ library.
//! The mock port resolves opaque source identities without dereferencing pointers.
//! Its reader is naturally Send, so the lifetime regressions remain possible
//! without bypassing the module-state thread-transfer requirement.
#![allow(non_snake_case)]

use bsk_build::{BskContext, BskError, BskModule, BskResult, Msg, MsgReader};
use std::cell::RefCell;
use std::collections::HashMap;

thread_local! {
    static SOURCE_VALUES: RefCell<HashMap<usize, TestMessage>> = RefCell::default();
    static READ_COUNT: RefCell<usize> = const { RefCell::new(0) };
}

/// Test payload with a non-physical marker value.
#[derive(Clone, Copy, Default)]
pub struct TestMessage(pub u64);

#[derive(Default)]
pub struct TestPort {
    data: usize,
    header: usize,
    linked: bool,
}

// SAFETY: This mock uses only TestPort's Rust representation, never C++.
// Source addresses are opaque registry keys, never dereferenced. Reads obtain
// copied values from the test registry; restore only copies subscription
// metadata. No output ports use this test message.
unsafe impl Msg for TestMessage {
    type Port = TestPort;

    fn __is_linked(port: &mut TestPort) -> bool {
        port.linked
    }
    fn __is_initialized(port: &TestPort) -> bool {
        port.data != 0 && port.header != 0
    }
    fn __port_pointers(port: &TestPort) -> (*const (), *const ()) {
        (port.data as *const (), port.header as *const ())
    }
    unsafe fn __restore_subscription(
        port: &mut TestPort,
        data: *const (),
        header: *const (),
        linked: bool,
    ) {
        port.data = data as usize;
        port.header = header as usize;
        port.linked = linked;
    }
    unsafe fn __read(port: &mut TestPort) -> Self {
        READ_COUNT.with_borrow_mut(|count| *count += 1);
        SOURCE_VALUES.with_borrow(|sources| {
            *sources
                .get(&port.data)
                .expect("read of a released mock source")
        })
    }
    unsafe fn __init(_port: &mut TestPort) {
        unreachable!("this fixture has no output ports")
    }
    unsafe fn __write(_data: &Self, _port: &mut TestPort, _id: i64, _time: u64) {
        unreachable!("this fixture has no output ports")
    }
}

/// Source allocation, owned by the simulated foreign caller, not the reader.
pub struct Source(Box<TestMessage>);

impl Source {
    pub fn new(value: u64) -> Self {
        let source = Self(Box::new(TestMessage(value)));
        SOURCE_VALUES
            .with_borrow_mut(|sources| sources.insert(source.pointer() as usize, *source.0));
        source
    }

    pub fn pointer(&self) -> *const TestMessage {
        &*self.0
    }

    /// Simulate a C/Python subscription between lifecycle calls.
    ///
    /// # Safety
    /// Keep this source alive until the original configuration port is
    /// unsubscribed. Do not connect or release it during a lifecycle callback.
    pub unsafe fn subscribe(&self, reader: &mut MsgReader<TestMessage>) {
        // SAFETY: MsgReader is repr(transparent) over TestPort.
        let port = unsafe { &mut *(reader as *mut MsgReader<TestMessage>).cast::<TestPort>() };
        port.data = self.pointer() as usize;
        port.header = port.data;
        port.linked = true;
    }
}

impl Drop for Source {
    fn drop(&mut self) {
        SOURCE_VALUES.with_borrow_mut(|sources| sources.remove(&(self.pointer() as usize)));
    }
}

pub fn read_count() -> usize {
    READ_COUNT.with_borrow(|count| *count)
}

#[derive(Default)]
pub struct ReaderState {
    saved: Option<MsgReader<TestMessage>>,
}

/// Hostile module actions are kept in an architecture test fixture, not the tutorial.
#[bsk_build::module]
#[repr(C)]
pub struct ReaderConfig {
    #[bsk(optional)]
    pub dataInMsg: MsgReader<TestMessage>,
    #[bsk(optional)]
    pub dataInMsgs: [MsgReader<TestMessage>; 2],
    pub action: u32,
    pub failure: u32,
    pub rejected: bool,
    pub value: u64,
    pub automaticValue: u64,
}

impl ReaderConfig {
    fn exercise(
        &mut self,
        state: &mut ReaderState,
        context: &BskContext<'_>,
    ) -> BskResult<ReaderOutputs> {
        match self.action {
            0 => self.value = self.dataInMsg.read(context)?.0,
            1 => {
                state.saved = Some(std::mem::take(&mut self.dataInMsg));
                self.rejected = state.saved.as_mut().unwrap().read(context).is_err();
            }
            2 => {
                std::mem::swap(&mut self.dataInMsg, &mut self.dataInMsgs[0]);
                self.dataInMsgs.swap(0, 1);
                self.rejected = self.dataInMsg.read(context).is_err()
                    && self
                        .dataInMsgs
                        .iter_mut()
                        .all(|port| port.read(context).is_err());
            }
            3 => {
                self.dataInMsg = state.saved.take().unwrap();
                self.rejected = self.dataInMsg.read(context).is_err();
            }
            4 => self.rejected = state.saved.as_mut().unwrap().read(context).is_err(),
            5 => self.dataInMsg = MsgReader::default(),
            6 => {
                // A temporary move out and back is harmless if never read while moved.
                let temporary = std::mem::take(&mut self.dataInMsg);
                self.dataInMsg = temporary;
                self.value = self.dataInMsg.read(context)?.0;
            }
            _ => unreachable!(),
        }
        match self.failure {
            1 => Err(BskError::new("intentional callback error")),
            2 => panic!("intentional callback panic"),
            _ => Ok(ReaderOutputs::default()),
        }
    }
}

impl BskModule for ReaderConfig {
    type State = ReaderState;
    type Inputs = ReaderInputs;
    type Outputs = ReaderOutputs;

    fn reset(
        &mut self,
        state: &mut ReaderState,
        context: &BskContext<'_>,
        _time: u64,
    ) -> BskResult<Self::Outputs> {
        self.exercise(state, context)
    }

    fn update(
        &mut self,
        state: &mut ReaderState,
        context: &BskContext<'_>,
        inputs: ReaderInputs,
        _time: u64,
    ) -> BskResult<Self::Outputs> {
        self.automaticValue = inputs.dataInMsg.map_or(0, |message| message.0);
        self.exercise(state, context)
    }
}
