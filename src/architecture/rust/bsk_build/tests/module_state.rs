// ISC License
//
// Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
//
// Permission to use, copy, modify, and/or distribute this software for any
// purpose with or without fee is hereby granted, provided that the above
// copyright notice and this permission notice appear in all copies.

//! Exercise exclusive state handoff, without requiring shared concurrent access.

use bsk_build::{BskContext, BskModule, BskModuleRuntime, BskResult};
use std::cell::RefCell;
use std::sync::{Arc, Mutex};
use std::thread::{self, ThreadId};

#[derive(Default)]
struct NumericalState {
    // RefCell deliberately makes this state !Sync, while preserving Send.
    samples: RefCell<Vec<u64>>,
    label: String,
    // The test observes callbacks and destruction from outside the owned state.
    // This mutex is test instrumentation, not a module-state requirement.
    threads: Arc<Mutex<Vec<ThreadId>>>,
}

impl Drop for NumericalState {
    fn drop(&mut self) {
        self.threads.lock().unwrap().push(thread::current().id());
    }
}

struct NumericalModule;

impl BskModule for NumericalModule {
    type State = NumericalState;
    type Inputs = u64;
    type Outputs = u64;

    fn init(&mut self, state: &mut Self::State) -> BskResult<()> {
        state.label = String::from("constructed");
        state.threads.lock().unwrap().push(thread::current().id());
        Ok(())
    }

    fn reset(
        &mut self,
        state: &mut Self::State,
        _context: &BskContext<'_>,
        _time: u64,
    ) -> BskResult<Self::Outputs> {
        state.samples.borrow_mut().clear();
        state.threads.lock().unwrap().push(thread::current().id());
        Ok(0)
    }

    fn update(
        &mut self,
        state: &mut Self::State,
        _context: &BskContext<'_>,
        input: u64,
        _time: u64,
    ) -> BskResult<Self::Outputs> {
        state.samples.borrow_mut().push(input);
        state.label = String::from("updated");
        state.threads.lock().unwrap().push(thread::current().id());
        Ok(state.samples.borrow().iter().sum())
    }
}

/// Construct on the caller, reset/update on a worker, then destroy on the caller.
#[test]
fn send_but_not_sync_state_supports_exclusive_worker_handoff() {
    let caller = thread::current().id();
    let mut module = NumericalModule;
    let mut state = NumericalState::default();
    let observed_threads = Arc::clone(&state.threads);
    module.init(&mut state).unwrap();

    let (state, worker) = thread::spawn(move || {
        let worker = thread::current().id();
        let runtime = BskModuleRuntime::for_testing();
        let context = BskContext::for_testing(&runtime);
        let time = 0; // [ns]
        module.reset(&mut state, &context, time).unwrap();
        assert_eq!(module.update(&mut state, &context, 17, time).unwrap(), 17);
        assert_eq!(module.update(&mut state, &context, 29, time).unwrap(), 46);
        (state, worker)
    })
    .join()
    .expect("the worker should return the exclusively owned state");

    assert_ne!(caller, worker);
    assert_eq!(*state.samples.borrow(), [17, 29]);
    assert_eq!(state.label, "updated");
    drop(state);
    assert_eq!(
        *observed_threads.lock().unwrap(),
        [caller, worker, worker, worker, caller]
    );
}

/// Scalars, arrays, owned containers, and thread-safe shared ownership need no manual impl.
#[test]
fn conventional_private_state_types_are_send_by_default() {
    fn require_state<T: Default + Send>() {}
    require_state::<()>();
    require_state::<f64>();
    require_state::<[f64; 3]>();
    require_state::<Vec<f64>>();
    require_state::<String>();
    require_state::<Box<[f64; 3]>>();
    require_state::<Arc<Vec<f64>>>();
    require_state::<RefCell<Vec<f64>>>();
    require_state::<NumericalState>();
}
