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

"""Keep-alive registry for C-message (``Msg_C``) subscribers -- issue #1433.

When a ``Msg_C`` reader (for example a C module's ``dataInMsg``) subscribes to a
stand-alone source, it stores *raw pointers* into that source's memory. If the
only Python reference to the source then goes out of scope, Python may
garbage-collect it and the subscriber is left reading freed memory.

Issue #676 fixed the C++ ``ReadFunctor`` direction by hanging the keep-alive on
that reader's C++ destructor. A ``Msg_C`` is a plain C struct with no destructor,
so this module keeps the source alive from the Python side instead.

How the source reference is *released* depends on how the subscriber is owned:

* **Stand-alone ``Msg_C`` subscriber** -- its SWIG proxy owns the underlying
  struct (``thisown`` is true) and is a single persistent object, so the source is
  simply pinned as an attribute on that proxy and dies with it.
* **Module-embedded ``Msg_C`` subscriber** (the common case, e.g. ``mod.dataInMsg``)
  -- its proxy is *transient*: SWIG hands out a fresh, non-owning proxy on every
  attribute access, so an attribute pinned on it is immediately lost and the proxy
  has no usable destruction hook. The source is therefore pinned in a process-wide
  registry keyed by the subscriber struct's stable C-address, and released when the
  owning C module is garbage-collected (its wrapper *does* have a real destructor;
  :func:`armModuleCleanup` arms a finalizer over the module config's address range).

In both cases the reference is also released on an explicit ``unsubscribe()`` or
when the subscriber re-subscribes to a different source.

All access happens on the Python side under the GIL (subscribe/unsubscribe calls
and ``weakref`` finalizers all run in the interpreter thread), so the registry
needs no additional locking.
"""

import weakref

#: subscriber ``Msg_C`` C-address (``int``) -> retained source Python object.
#: Holds only *module-embedded* subscribers; stand-alone subscribers pin their
#: source as an attribute on their own proxy instead.
_registry = {}

#: Attribute name used to pin a source on a stand-alone subscriber proxy.
_PIN_ATTR = "_bskKeepAliveSource"


def retainSource(subscriber, source):
    """Retain ``source`` for as long as ``subscriber`` (a ``Msg_C``) reads it.

    Replaces any previously retained source for the same subscriber, dropping the
    old reference.
    """
    if getattr(subscriber, "thisown", False):
        # Stand-alone subscriber: persistent, owning proxy -- pin on the proxy.
        setattr(subscriber, _PIN_ATTR, source)
    else:
        # Module-embedded subscriber: transient proxy -- pin in the registry by
        # the subscriber struct's stable address.
        _registry[int(subscriber.this)] = source


def releaseSource(subscriber):
    """Drop the retained source for ``subscriber`` (no-op if none is held)."""
    if getattr(subscriber, "thisown", False):
        try:
            delattr(subscriber, _PIN_ATTR)
        except AttributeError:
            pass
    else:
        _registry.pop(int(subscriber.this), None)


def armModuleCleanup(module):
    """Arm a finalizer that releases every embedded subscriber's source when
    ``module`` (a C-module wrapper) is garbage-collected.

    ``module`` must expose ``getConfigAddress()`` / ``getConfigSize()`` (provided by
    the ``CWrapper`` SWIG template). All of the module's embedded ``Msg_C`` members
    live within ``[address, address + size)``, so releasing that address range frees
    exactly this module's registered sources and no others.
    """
    lo = int(module.getConfigAddress())
    hi = lo + int(module.getConfigSize())
    weakref.finalize(module, _releaseRange, lo, hi)


def _releaseRange(lo, hi):
    """Release all registry entries whose subscriber address is in ``[lo, hi)``."""
    for addr in [a for a in _registry if lo <= a < hi]:
        _registry.pop(addr, None)
