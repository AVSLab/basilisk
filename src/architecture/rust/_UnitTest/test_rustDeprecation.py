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

import ctypes

import pytest

from Basilisk.utilities import deprecated

rustModuleTemplate = pytest.importorskip(
    "Basilisk.moduleTemplates.rustModuleTemplate",
    reason="Rust module support is not enabled in this Basilisk build",
)


@pytest.mark.parametrize(
    ("removal_date", "warning_type"),
    [
        ("2999/01/01", deprecated.BSKDeprecationWarning),
        ("2000/01/01", deprecated.BSKUrgentDeprecationWarning),
    ],
)
def test_rust_module_deprecation_warning(monkeypatch, removal_date, warning_type):
    """Issue normal and urgent module warnings on the appropriate date."""
    module_type = rustModuleTemplate.rustModuleTemplate
    date_method = next(
        name
        for name in module_type.__dict__
        if name.endswith("__bskModuleDeprecationDate")
    )
    message_method = next(
        name
        for name in module_type.__dict__
        if name.endswith("__bskModuleDeprecationMessage")
    )
    monkeypatch.setattr(module_type, date_method, lambda self: removal_date)
    monkeypatch.setattr(
        module_type,
        message_method,
        lambda self: "Use replacementModule instead.",
    )

    with pytest.warns(warning_type, match="Use replacementModule instead."):
        module = module_type()
    assert module.thisown is True


def test_rust_module_deprecation_metadata_abi():
    """Export module metadata for the wrapper without exposing raw Python APIs."""
    extension = ctypes.CDLL(rustModuleTemplate._rustModuleTemplate.__file__)
    deprecation_date = extension.ModuleDeprecationDate_rustModuleTemplate
    deprecation_date.argtypes = []
    deprecation_date.restype = ctypes.c_char_p
    deprecation_message = extension.ModuleDeprecationMessage_rustModuleTemplate
    deprecation_message.argtypes = []
    deprecation_message.restype = ctypes.c_char_p

    assert deprecation_date() is None
    assert deprecation_message() is None
    assert not hasattr(
        rustModuleTemplate,
        "ModuleDeprecationDate_rustModuleTemplate",
    )
    assert not hasattr(
        rustModuleTemplate,
        "ModuleDeprecationMessage_rustModuleTemplate",
    )
