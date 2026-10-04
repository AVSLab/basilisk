# ISC License
#
# Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
#
# Permission to use, copy, modify, and/or distribute this software for any
# purpose with or without fee is hereby granted, provided that the above
# copyright notice and this permission notice appear in all copies.
#
# THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
# WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
# ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
# WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
# ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
# OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

"""Check native message loading without changing the public Python interfaces."""

import importlib
import importlib.machinery
from pathlib import Path
import pkgutil
import subprocess
import sys
import textwrap

from Basilisk.architecture import messaging


def test_message_bindings_share_one_native_file():
    """All payload modules retain their identities and share one native file."""
    module_names = {
        entry.name
        for entry in pkgutil.iter_modules(messaging.__path__)
        if entry.name.endswith("Payload") and not entry.name.startswith("_")
    }
    assert "SCStatesMsgPayload" in module_names
    module_names.add("messagingSupport")
    native_paths = set()
    for name in module_names:
        wrapper = importlib.import_module(f"{messaging.__name__}.{name}")
        full_name = f"{messaging.__name__}._{name}"
        native = importlib.import_module(full_name)
        assert native.__spec__.name == full_name
        assert getattr(wrapper, f"_{name}") is native
        assert getattr(messaging, f"_{name}") is native
        native_paths.add(Path(native.__file__).resolve())

    assert len(native_paths) == 1
    packaged_binaries = {
        path.resolve()
        for directory in messaging.__path__
        for path in Path(directory).iterdir()
        if any(path.name.endswith(suffix) for suffix in importlib.machinery.EXTENSION_SUFFIXES)
    }
    assert packaged_binaries == native_paths


def test_recorders_work_before_explicit_messaging_import():
    """A fresh process registers module-output recorders and preserves imports."""
    code = textwrap.dedent("""\
        import importlib
        from Basilisk.simulation import spacecraft
        sc = spacecraft.Spacecraft()
        recorder = sc.scStateOutMsg.recorder()

        from Basilisk.architecture import messaging
        assert isinstance(recorder, messaging.SCStatesMsgRecorder)
        payload = messaging.SCStatesMsgPayload()
        payload.r_BN_N = [1.0, 2.0, 3.0]  # [m]
        write_time = 42  # [ns]
        sc.scStateOutMsg.write(payload, write_time)
        recorder.UpdateState(write_time)
        assert recorder.times().tolist() == [write_time]
        assert recorder.r_BN_N.tolist() == [[1.0, 2.0, 3.0]]  # [m]

        native = importlib.import_module('Basilisk.architecture.messaging._SCStatesMsgPayload')
        importlib.reload(messaging)
        assert importlib.import_module(native.__name__) is native
        assert isinstance(sc.scStateOutMsg.recorder(), messaging.SCStatesMsgRecorder)
        """)
    result = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True)
    assert result.returncode == 0, result.stdout + result.stderr
