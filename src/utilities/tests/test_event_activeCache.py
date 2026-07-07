#
#  ISC License
#
#  Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
#
#  Permission to use, copy, modify, and/or distribute this software for any
#  purpose with or without fee is hereby granted, provided that the above
#  copyright notice and this permission notice appear in all copies.
#
#   Unit Test Script
#   Module Name:        SimulationBaseClass active-event cache (issue #455)
#   Author:             robotrocketscience (https://github.com/robotrocketscience)
#   Creation Date:      2026-07-07
#
"""Guards the cached ``activeEvents()`` list against staleness.

The optimization only holds up if the cache is invalidated at *every* point that
can change which events are active. The central test asserts the invariant
``activeEvents() == {events with eventActive True}`` after each kind of mutation,
plus the end-to-end behaviour (trigger times, counts, self-reactivation) that a
running simulation depends on.
"""
from Basilisk.utilities import SimulationBaseClass, macros


def _trueActive(sim):
    return {e.eventName for e in sim.eventMap.values() if e.eventActive}


def _cachedActive(sim):
    return {e.eventName for e in sim.activeEvents()}


def _assertConsistent(sim):
    # activeEvents() must equal the true active set, and every returned event
    # must actually be active.
    assert _cachedActive(sim) == _trueActive(sim)
    assert all(e.eventActive for e in sim.activeEvents())


def _addEvent(sim, name, active, cond=False, action=None):
    sim.createNewEvent(
        name, macros.sec2nano(1.0), active,
        conditionFunction=(cond if callable(cond) else (lambda s, c=cond: c)),
        actionFunction=(action or (lambda s: None)),
    )


def test_cache_reflects_createNewEvent():
    sim = SimulationBaseClass.SimBaseClass()
    assert _cachedActive(sim) == set()
    _addEvent(sim, "a", True)
    _assertConsistent(sim); assert _cachedActive(sim) == {"a"}
    _addEvent(sim, "b", False)
    _assertConsistent(sim); assert _cachedActive(sim) == {"a"}
    _addEvent(sim, "c", True)
    _assertConsistent(sim); assert _cachedActive(sim) == {"a", "c"}


def test_cache_reflects_setEventActivity():
    sim = SimulationBaseClass.SimBaseClass()
    _addEvent(sim, "a", True); _addEvent(sim, "b", False)
    _ = sim.activeEvents()  # prime the cache
    sim.setEventActivity("b", True)
    _assertConsistent(sim); assert _cachedActive(sim) == {"a", "b"}
    sim.setEventActivity("a", False)
    _assertConsistent(sim); assert _cachedActive(sim) == {"b"}


def test_cache_reflects_setAllButCurrent():
    sim = SimulationBaseClass.SimBaseClass()
    for n in ("a", "b", "c"):
        _addEvent(sim, n, True)
    _ = sim.activeEvents()  # prime
    sim.setAllButCurrentEventActivity("b", False)
    _assertConsistent(sim); assert _cachedActive(sim) == {"b"}


def test_noop_setEventActivity_does_not_rebuild_cache():
    # Setting an event to the activity state it already has must not invalidate
    # the cache (no unnecessary O(total events) rebuild), while a real change must.
    sim = SimulationBaseClass.SimBaseClass()
    _addEvent(sim, "a", True); _addEvent(sim, "b", False)
    cached = sim.activeEvents()  # prime
    sim.setEventActivity("a", True)   # already active -> no-op
    assert sim.activeEvents() is cached
    sim.setEventActivity("b", False)  # already inactive -> no-op
    assert sim.activeEvents() is cached
    sim.setEventActivity("b", True)   # real change -> rebuild
    assert sim.activeEvents() is not cached
    _assertConsistent(sim); assert _cachedActive(sim) == {"a", "b"}


def test_noop_setAllButCurrent_does_not_rebuild_cache():
    # A blanket set that changes nothing must not rebuild the cache; one that
    # actually flips an event's activity must.
    sim = SimulationBaseClass.SimBaseClass()
    for n in ("a", "b", "c"):
        _addEvent(sim, n, True)
    cached = sim.activeEvents()  # prime, all active
    sim.setAllButCurrentEventActivity("b", True)   # a, c already active -> no-op
    assert sim.activeEvents() is cached
    sim.setAllButCurrentEventActivity("b", False)  # real change -> rebuild
    assert sim.activeEvents() is not cached
    _assertConsistent(sim); assert _cachedActive(sim) == {"b"}


def test_cache_reflects_trigger_deactivation():
    # An event that triggers deactivates itself; it must leave the active set.
    sim = SimulationBaseClass.SimBaseClass()
    _addEvent(sim, "fires", True, cond=True)
    _addEvent(sim, "stays", True, cond=False)
    _ = sim.activeEvents()  # prime with both active
    sim.eventMap["fires"].checkEvent(sim)
    _assertConsistent(sim); assert _cachedActive(sim) == {"stays"}
    assert sim.eventMap["fires"].occurCounter == 1


def test_cache_handles_self_reactivating_event():
    # Action re-activates the event via setEventActivity: it must stay active.
    sim = SimulationBaseClass.SimBaseClass()

    def reactivate(s):
        s.setEventActivity("loop", True)

    _addEvent(sim, "loop", True, cond=True, action=reactivate)
    _ = sim.activeEvents()
    sim.eventMap["loop"].checkEvent(sim)
    _assertConsistent(sim); assert _cachedActive(sim) == {"loop"}
    assert sim.eventMap["loop"].occurCounter == 1


def test_returned_list_not_corrupted_by_later_mutation():
    # A snapshot taken before a mutation may legitimately be stale, but a fresh
    # query after invalidation must be correct (no aliasing bug).
    sim = SimulationBaseClass.SimBaseClass()
    _addEvent(sim, "a", True)
    first = sim.activeEvents()
    _addEvent(sim, "b", True)
    second = sim.activeEvents()
    assert {e.eventName for e in second} == {"a", "b"}
    assert first is not second  # cache was rebuilt, not mutated in place


def test_end_to_end_trigger_times_and_counts():
    # Full run with no C++ modules: a conditionTime event must fire once at the
    # right time and then stay inactive; an unrelated inactive event must not fire.
    sim = SimulationBaseClass.SimBaseClass()
    proc = sim.CreateNewProcess("p")
    proc.addTask(sim.CreateNewTask("t", macros.sec2nano(0.1)))
    fired = []
    sim.createNewEvent(
        "boom", macros.sec2nano(0.1), True,
        conditionFunction=(lambda s: s.TotalSim.CurrentNanos >= macros.sec2nano(1.0)),
        actionFunction=(lambda s: fired.append(s.TotalSim.CurrentNanos)),
    )
    _addEvent(sim, "idle", False, cond=True)  # would fire if ever checked
    sim.InitializeSimulation()
    sim.showProgressBar = False
    sim.ConfigureStopTime(macros.sec2nano(2.0))
    sim.ExecuteSimulation()
    assert len(fired) == 1
    assert fired[0] >= macros.sec2nano(1.0)
    assert sim.eventMap["boom"].occurCounter == 1
    assert sim.eventMap["idle"].occurCounter == 0
    assert _cachedActive(sim) == set()  # both inactive at end


def test_end_to_end_periodic_self_reactivating():
    # An event that re-activates itself each trigger should fire many times.
    sim = SimulationBaseClass.SimBaseClass()
    proc = sim.CreateNewProcess("p")
    proc.addTask(sim.CreateNewTask("t", macros.sec2nano(0.1)))
    hits = []

    def act(s):
        hits.append(s.TotalSim.CurrentNanos)
        s.setEventActivity("tick", True)  # re-arm

    sim.createNewEvent(
        "tick", macros.sec2nano(0.5), True,
        conditionFunction=(lambda s: True),
        actionFunction=act,
    )
    sim.InitializeSimulation()
    sim.showProgressBar = False
    sim.ConfigureStopTime(macros.sec2nano(3.0))
    sim.ExecuteSimulation()
    # checked every 0.5 s over [0, 3] s -> fires repeatedly, stays active
    assert len(hits) >= 5
    assert _cachedActive(sim) == {"tick"}


def test_duplicate_createNewEvent_does_not_corrupt_cache():
    # A duplicate name warns and returns early without adding; the primed cache
    # must remain correct (the early return must not leave a stale/invalid cache).
    sim = SimulationBaseClass.SimBaseClass()
    _addEvent(sim, "a", True)
    assert _cachedActive(sim) == {"a"}  # prime
    import warnings
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        _addEvent(sim, "a", True)  # duplicate -> skipped
    _assertConsistent(sim)
    assert _cachedActive(sim) == {"a"}
    assert len(sim.eventMap) == 1


def test_conditionTime_event_end_to_end():
    # conditionTime events use a constructor-generated conditionFunction; verify
    # they fire once at/after the target time and then leave the active set.
    sim = SimulationBaseClass.SimBaseClass()
    proc = sim.CreateNewProcess("p")
    proc.addTask(sim.CreateNewTask("t", macros.sec2nano(0.1)))
    fired = []
    sim.createNewEvent(
        "timed", macros.sec2nano(0.1), True,
        conditionTime=macros.sec2nano(1.3),
        actionFunction=(lambda s: fired.append(s.TotalSim.CurrentNanos)),
    )
    sim.InitializeSimulation()
    sim.showProgressBar = False
    sim.ConfigureStopTime(macros.sec2nano(2.0))
    sim.ExecuteSimulation()
    assert len(fired) == 1
    assert fired[0] >= macros.sec2nano(1.3)
    assert _cachedActive(sim) == set()


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn(); print(f"{name} PASSED")
