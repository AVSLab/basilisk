# Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
# This file is distributed under the ISC License in LICENSE.

"""Defer effector naming warnings until a simulation lifecycle call completes."""

from contextlib import contextmanager
from itertools import count
from threading import Lock, local


_deferredManagers = {}
_deferralLock = Lock()
_activeScopes = {}
_scopeIdentifiers = count(1)
_callingScope = local()


class _ReportScope:
    """Retain managers and their native owners until the outer Python call returns."""

    def __init__(self):
        self.identifier = next(_scopeIdentifiers)
        self.managers = {}
        self.owners = []


def _releaseManagers(managers):
    """Release one deferral per manager while the caller holds the registry lock."""
    for address in managers:
        _deferredManagers[address] -= 1
        if _deferredManagers[address] == 0:
            del _deferredManagers[address]


def isReportDeferred(manager):
    """Check the native manager identity across Python proxies and worker threads."""
    with _deferralLock:
        return int(manager.this) in _deferredManagers


@contextmanager
def deferReports(models, simulation=None):
    """Report scheduled managers' warnings after successful initialization or reset.

    :param models: Models participating in the lifecycle call. Owners are retained
        for the entire scope so their native manager addresses remain valid.
    :param simulation: Optional native simulation whose initialization workers
        must inherit this reporting scope.

    Nested simulations join the outer reporting scope, including when a native
    worker calls back into Python. Independent calls on other threads keep their
    own scope. Successful nested calls retain their managers and owners until the
    outer call completes, even if the nested simulation itself is discarded.
    Every counter is released before reporting, even when a warning becomes an
    exception. An initialization error is propagated without reporting warnings.
    """
    from Basilisk.architecture import sim_model

    owners = list(models)
    managers = {}
    for model in owners:
        manager = getattr(model, "dynManager", None)
        if getattr(manager, "_reportLegacyAutomaticEffectorNaming", None) is not None:
            managers.setdefault(int(manager.this), manager)

    previousScope = getattr(_callingScope, "current", None)
    with _deferralLock:
        scope = previousScope or _activeScopes.get(sim_model.SimModel._getCurrentPythonExecutionContext())
        isRoot = scope is None
        if isRoot:
            scope = _ReportScope()
            scope.managers.update(managers)
            scope.owners.extend(owners)
            _activeScopes[scope.identifier] = scope
        for address in managers:
            _deferredManagers[address] = _deferredManagers.get(address, 0) + 1
    _callingScope.current = scope
    previousContext = None
    succeeded = False
    try:
        if simulation is not None:
            previousContext = simulation._getPythonExecutionContext()
            simulation._setPythonExecutionContext(scope.identifier)
        yield
        succeeded = True
    finally:
        if previousContext is not None:
            simulation._setPythonExecutionContext(previousContext)
        _callingScope.current = previousScope
        with _deferralLock:
            if isRoot:
                _releaseManagers(scope.managers)
                del _activeScopes[scope.identifier]
            elif succeeded:
                # Transfer each new manager's guard to the root. Existing root
                # managers already own a guard, so release only the nested copy.
                _releaseManagers(managers.keys() & scope.managers.keys())
                scope.managers.update(managers)
                scope.owners.extend(owners)
            else:
                _releaseManagers(managers)

    if isRoot:
        for manager in scope.managers.values():
            # The generator and contextlib.__exit__ add two frames to the usual
            # lifecycle method -> manager reporter -> deprecation helper path.
            manager._reportLegacyAutomaticEffectorNaming(stacklevel=6)
