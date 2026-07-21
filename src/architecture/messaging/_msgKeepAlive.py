#
#  ISC License
#
#  Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
#
#  Permission to use, copy, modify, and/or distribute this software for any
#  purpose with or without fee is hereby granted, provided that the above
#  copyright notice and this permission notice appear in all copies.
#
#  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
#  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
#  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
#  ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
#  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
#  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
#  OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
#

"""Keep-alive helpers for C-message (``Msg_C``) readers -- issue #1433.

When a ``Msg_C`` reader (for example a C module's ``dataInMsg``) subscribes to a
stand-alone source, it stores *raw pointers* into that source's memory. If the
only Python reference to the source then goes out of scope, Python may
garbage-collect it and the subscriber is left reading freed memory.

Likewise, a recorder created from a ``Msg_C`` stores raw pointers into that
message's storage. The recorder therefore retains the stand-alone message or
embedded-message owner for as long as the recorder exists.

Issue #676 fixed the C++ ``ReadFunctor`` direction by hanging the keep-alive on
that reader's C++ destructor. A ``Msg_C`` is a plain C struct with no destructor,
so this module keeps the source alive from the Python side instead.

The retention target follows the Python object that owns the source storage:

* a stand-alone message source retains the source proxy itself;
* an embedded ``Msg_C`` source retains its owning config or module wrapper through
  a transferable lease, because the embedded proxy is transient and does not own
  the source storage.

Where the reference is stored depends on how the subscriber is owned:

* a stand-alone ``Msg_C`` subscriber pins the source on its own persistent proxy;
* an embedded ``Msg_C`` subscriber pins the source in a hidden dictionary on its
  owning config or C-module wrapper.

In both cases the reference is also released on an explicit ``unsubscribe()`` or
when the subscriber re-subscribes to a different source.

Recorders store their retention target directly on the persistent recorder proxy.
The reference is released automatically with the recorder.

All access happens on the Python side under the GIL (module construction,
recorder construction, subscribe/unsubscribe calls, and ``weakref`` finalizers
all run in the interpreter thread), so the registries need no additional locking.
"""

import weakref

#: Embedded ``Msg_C`` C-address (``int``) -> (weak owner handle, owner token).
_owner_by_address = {}

#: Attribute name used to pin a retention target on a persistent SWIG proxy.
_PIN_ATTR = "_bskKeepAliveSource"

#: Attribute name used for the module-owned keep-alive dictionary.
_MODULE_PIN_ATTR = "_bskMsgKeepAlive"

#: Attribute names used to make module registration idempotent and finalizable.
_MODULE_ADDRS_ATTR = "_bskMsgKeepAliveAddrs"
_MODULE_TOKEN_ATTR = "_bskMsgKeepAliveToken"
_MODULE_OWNER_HANDLE_ATTR = "_bskMsgKeepAliveOwner"


class _OwnerLease:
    """Strong reference to the current owner of embedded message storage."""

    def __init__(self, owner):
        self.owner = owner


class _OwnerHandle:
    """Track an embedded message owner and every active lease on that owner."""

    def __init__(self, owner):
        self._owner_ref = weakref.ref(owner)
        self._leases = weakref.WeakSet()

    @property
    def owner(self):
        """Return the current storage owner, or ``None`` after its collection."""
        return self._owner_ref()

    def lease(self):
        """Return a strong, transferable reference to the current owner."""
        owner = self.owner
        if owner is None:
            return None
        lease = _OwnerLease(owner)
        self._leases.add(lease)
        return lease

    def transfer(self, owner):
        """Move this handle and all active leases to a new storage owner."""
        self._owner_ref = weakref.ref(owner)
        for lease in self._leases:
            lease.owner = owner


def registerModule(module):
    """Register the embedded ``Msg_C`` fields owned by ``module`` or a config.

    The C-module and config SWIG wrappers call this from their Python constructors.
    SWIG returns fresh non-owning proxies for embedded C messages on each attribute
    access, so this registry lets later ``subscribeTo()`` calls recover the storage
    owner from an embedded message's stable C address.
    """
    if hasattr(module, _MODULE_ADDRS_ATTR):
        return

    _register_owner(module, _OwnerHandle(module), {})


def transferModuleOwner(config, module):
    """Transfer config-owned message storage and keep-alives to ``module``.

    Existing leases are retargeted so subscriptions established before
    ``createWrapper()`` retain the module after its C wrapper takes ownership of the
    config storage.
    """
    if not hasattr(config, _MODULE_OWNER_HANDLE_ATTR):
        registerModule(config)

    handle = getattr(config, _MODULE_OWNER_HANDLE_ATTR)
    pins = getattr(config, _MODULE_PIN_ATTR)
    object.__setattr__(config, _MODULE_PIN_ATTR, {})

    handle.transfer(module)
    _register_owner(module, handle, pins)


def retainSource(subscriber, source):
    """Retain ``source`` for as long as ``subscriber`` (a ``Msg_C``) reads it.

    Replaces any previously retained source for the same subscriber, dropping the
    old reference.
    """
    target = _retention_target(source)
    owner = _owner_of(subscriber)

    # Replace the pin directly. Python installs the new reference before dropping
    # the old one, so a weakref callback triggered by old-source destruction sees
    # the native subscription and its keep-alive in a consistent state.
    if owner is not None:
        getattr(owner, _MODULE_PIN_ATTR)[int(subscriber.this)] = target
    elif getattr(subscriber, "thisown", False):
        # Stand-alone subscriber: persistent, owning proxy -- pin on the proxy.
        object.__setattr__(subscriber, _PIN_ATTR, target)
    else:
        # Unknown non-owning subscriber. This is unusual, but retaining on the
        # proxy is still the least surprising fallback for Python-created objects.
        object.__setattr__(subscriber, _PIN_ATTR, target)


def releaseSource(subscriber):
    """Drop the retained source for ``subscriber`` (no-op if none is held)."""
    owner = _owner_of(subscriber)
    if owner is not None:
        pins = getattr(owner, _MODULE_PIN_ATTR, None)
        if pins is not None:
            pins.pop(int(subscriber.this), None)
    else:
        try:
            delattr(subscriber, _PIN_ATTR)
        except AttributeError:
            pass


def retainRecorderSource(recorder, source):
    """Retain the storage owner of ``source`` for the lifetime of ``recorder``."""
    object.__setattr__(recorder, _PIN_ATTR, _retention_target(source))


def copyRecorderSource(recorder, sourceRecorder):
    """Copy the source retention target when a Python recorder is copied."""
    try:
        target = vars(sourceRecorder).get(_PIN_ATTR)
    except TypeError:
        return
    if target is not None:
        object.__setattr__(recorder, _PIN_ATTR, target)


def retainRecorderConstructorSource(recorder, source):
    """Retain a direct ``Msg_C`` constructor source or copy an existing pin."""
    if _looks_like_c_msg(source):
        retainRecorderSource(recorder, source)
    else:
        copyRecorderSource(recorder, source)


def _retention_target(source):
    """Return the Python owner that must stay alive for ``source`` to be valid."""
    handle = _owner_handle_of(source)
    if handle is None:
        return source
    lease = handle.lease()
    return lease if lease is not None else source


def _owner_of(msg):
    """Return the current storage owner for embedded ``Msg_C`` ``msg``."""
    handle = _owner_handle_of(msg)
    return handle.owner if handle is not None else None


def _owner_handle_of(msg):
    """Return the transferable owner handle for embedded ``Msg_C`` ``msg``."""
    if not _looks_like_c_msg(msg) or getattr(msg, "thisown", False):
        return None

    entry = _owner_by_address.get(int(msg.this))
    if entry is None:
        return None

    handle_ref, _ = entry
    return handle_ref()


def _register_owner(owner, handle, pins):
    """Register ``owner`` and its embedded message addresses with ``handle``."""
    token = object()
    addresses = tuple(int(msg.this) for msg in _embedded_c_msgs(owner))

    object.__setattr__(owner, _MODULE_OWNER_HANDLE_ATTR, handle)
    object.__setattr__(owner, _MODULE_TOKEN_ATTR, token)
    object.__setattr__(owner, _MODULE_PIN_ATTR, pins)
    object.__setattr__(owner, _MODULE_ADDRS_ATTR, addresses)

    handle_ref = weakref.ref(handle)
    for address in addresses:
        _owner_by_address[address] = (handle_ref, token)

    weakref.finalize(owner, _release_module_owner, addresses, token)


def _embedded_c_msgs(module):
    """Yield embedded ``Msg_C`` proxies exposed as SWIG properties on ``module``."""
    seen = set()
    for cls in type(module).mro():
        for name, attr in cls.__dict__.items():
            if name in seen or not isinstance(attr, property):
                continue
            seen.add(name)
            try:
                value = getattr(module, name)
            except Exception:
                continue
            if _looks_like_c_msg(value) and not getattr(value, "thisown", False):
                yield value


def _looks_like_c_msg(value):
    """Return ``True`` for generated ``*Msg_C`` SWIG proxy objects."""
    return (
        type(value).__name__.endswith("Msg_C")
        and hasattr(value, "this")
        and hasattr(value, "header")
        and hasattr(value, "payload")
    )


def _release_module_owner(addresses, token):
    """Forget ownership entries that still belong to the finalized owner."""
    for addr in addresses:
        entry = _owner_by_address.get(addr)
        if entry is not None and entry[1] is token:
            _owner_by_address.pop(addr, None)
