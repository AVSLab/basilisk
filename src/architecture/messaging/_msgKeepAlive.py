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

"""Keep-alive helpers for C-message (``Msg_C``) subscribers -- issue #1433.

When a ``Msg_C`` reader (for example a C module's ``dataInMsg``) subscribes to a
stand-alone source, it stores *raw pointers* into that source's memory. If the
only Python reference to the source then goes out of scope, Python may
garbage-collect it and the subscriber is left reading freed memory.

Issue #676 fixed the C++ ``ReadFunctor`` direction by hanging the keep-alive on
that reader's C++ destructor. A ``Msg_C`` is a plain C struct with no destructor,
so this module keeps the source alive from the Python side instead.

The retention target is the Python object that owns the source storage:

* a stand-alone message source retains the source proxy itself;
* a module-embedded ``Msg_C`` source retains the owning module wrapper, because
  the embedded proxy is transient and does not own the source storage.

Where the reference is stored depends on how the subscriber is owned:

* a stand-alone ``Msg_C`` subscriber pins the source on its own persistent proxy;
* a module-embedded ``Msg_C`` subscriber pins the source in a hidden dictionary on
  the owning C-module wrapper.

In both cases the reference is also released on an explicit ``unsubscribe()`` or
when the subscriber re-subscribes to a different source.

All access happens on the Python side under the GIL (module construction,
subscribe/unsubscribe calls, and ``weakref`` finalizers all run in the interpreter
thread), so the registries need no additional locking.
"""

import weakref

#: Embedded ``Msg_C`` C-address (``int``) -> (weak module owner, owner token).
_owner_by_address = {}

#: Attribute name used to pin a source on a stand-alone subscriber proxy.
_PIN_ATTR = "_bskKeepAliveSource"

#: Attribute name used for the module-owned keep-alive dictionary.
_MODULE_PIN_ATTR = "_bskMsgKeepAlive"

#: Attribute names used to make module registration idempotent and finalizable.
_MODULE_ADDRS_ATTR = "_bskMsgKeepAliveAddrs"
_MODULE_TOKEN_ATTR = "_bskMsgKeepAliveToken"


def registerModule(module):
    """Register the embedded ``Msg_C`` fields owned by ``module``.

    The C-module SWIG wrapper calls this once from its Python constructor. SWIG
    returns fresh non-owning proxies for embedded C messages on each attribute
    access, so this registry lets later ``subscribeTo()`` calls recover the owning
    module from an embedded message's stable C address.
    """
    if hasattr(module, _MODULE_ADDRS_ATTR):
        return

    token = object()
    pins = {}
    addresses = []

    object.__setattr__(module, _MODULE_TOKEN_ATTR, token)
    object.__setattr__(module, _MODULE_PIN_ATTR, pins)

    module_ref = weakref.ref(module)
    for msg in _embedded_c_msgs(module):
        addr = int(msg.this)
        _owner_by_address[addr] = (module_ref, token)
        addresses.append(addr)

    object.__setattr__(module, _MODULE_ADDRS_ATTR, tuple(addresses))
    weakref.finalize(module, _release_module_owner, tuple(addresses), token)


def retainSource(subscriber, source):
    """Retain ``source`` for as long as ``subscriber`` (a ``Msg_C``) reads it.

    Replaces any previously retained source for the same subscriber, dropping the
    old reference.
    """
    target = _retention_target(source)
    releaseSource(subscriber)

    owner = _owner_of(subscriber)
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


def _retention_target(source):
    """Return the Python owner that must stay alive for ``source`` to be valid."""
    owner = _owner_of(source)
    return owner if owner is not None else source


def _owner_of(msg):
    """Return the owning C-module wrapper for embedded ``Msg_C`` ``msg``."""
    if not _looks_like_c_msg(msg) or getattr(msg, "thisown", False):
        return None

    entry = _owner_by_address.get(int(msg.this))
    if entry is None:
        return None

    owner_ref, _ = entry
    return owner_ref()


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
    """Forget module ownership entries that still belong to the finalized module."""
    for addr in addresses:
        entry = _owner_by_address.get(addr)
        if entry is not None and entry[1] is token:
            _owner_by_address.pop(addr, None)
