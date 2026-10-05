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

import pytest

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
        original_spec = native.__spec__
        assert importlib.reload(native) is native
        assert native.__spec__ is original_spec
        assert native.__file__ == original_spec.origin
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
        import importlib.machinery

        def check_shim_loading(loader, module):
            assert not (loader.name.startswith('Basilisk.architecture.messaging._')
                        and loader.name.endswith(('Payload', '_messagingSupport'))), loader.name
            return original_exec(loader, module)

        original_exec = importlib.machinery.SourceFileLoader.exec_module
        importlib.machinery.SourceFileLoader.exec_module = check_shim_loading
        from Basilisk.simulation import spacecraft
        importlib.machinery.SourceFileLoader.exec_module = original_exec
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


@pytest.mark.parametrize("first_import", ["package", "private"])
@pytest.mark.parametrize("second_import", ["private", "other_private", "package"])
@pytest.mark.parametrize("fail_first", [False, True], ids=["success", "retry"])
def test_concurrent_message_imports_wait_for_native_initialization(
        tmp_path, first_import, second_import, fail_first):
    """Wait for native initialization and recover from a failed first import.

    :param tmp_path: Directory for an isolated package used in failure-injection cases.
    :param first_import: Start by importing the messaging package or its private extension.
    :param second_import: Import the same binding, another binding, or the messaging package concurrently.
    :param fail_first: Inject an initialization failure before testing a clean retry.
    """
    package_name = messaging.__name__
    if fail_first:
        # Keep injected failures inside a small package using the real loader
        # and native bindings. Basilisk's top-level initializer imports messaging
        # eagerly, so failing it also invalidates unrelated ancestor packages.
        package_name = "messageImportProbe"
        package = tmp_path / package_name
        package.mkdir()
        (package / "__init__.py").write_text(
            f"__path__.append({str(Path(messaging.__file__).parent)!r})\n"
            "from ._load_messages import load_message_module\n"
            "load_message_module('messagingSupport')\n"
            "load_message_module('AttRefMsgPayload')\n"
            "load_message_module('SCStatesMsgPayload')\n",
            encoding="utf-8",
        )
    code = textwrap.dedent("""\
        import importlib
        import importlib.machinery
        import os
        import sys
        import threading

        package_name = sys.argv[4]
        sys.path.insert(0, sys.argv[5])
        native_name = package_name + '._AttRefMsgPayload'
        public_name = package_name + '.AttRefMsgPayload'
        first_name = package_name if sys.argv[1] == 'package' else native_name
        second_name = {
            'private': native_name,
            'other_private': package_name + '._SCStatesMsgPayload',
            'package': package_name,
        }[sys.argv[2]]
        fail_first = sys.argv[3] == 'True'
        if fail_first and os.name == 'nt':
            # The probe bypasses Basilisk.__init__, including its DLL setup.
            # Retain the directory handle across failed imports and retries.
            dll_directory = os.add_dll_directory(sys.argv[6])
        timeout = 10.0  # [s]
        blocking_window = 0.2  # [s]
        first_ready = threading.Event()
        release = threading.Event()
        second_started = threading.Event()
        second_done = threading.Event()
        modules = {}
        errors = {}
        attempts = []
        original_exec = importlib.machinery.ExtensionFileLoader.exec_module

        def paused_exec(loader, module):
            if loader.name == native_name:
                attempts.append(module)
                if len(attempts) == 1:
                    first_ready.set()
                    assert release.wait(timeout), 'Native initialization was never released'
                    if fail_first:
                        raise RuntimeError('Injected native initialization failure')
            return original_exec(loader, module)

        def import_module(label, name):
            if label == 'second':
                second_started.set()
            try:
                modules[label] = importlib.import_module(name)
            except BaseException as error:
                errors[label] = error
            finally:
                if label == 'first':
                    first_ready.set()
                else:
                    second_done.set()

        importlib.machinery.ExtensionFileLoader.exec_module = paused_exec
        first = threading.Thread(target=import_module, args=('first', first_name), daemon=True)
        second = threading.Thread(target=import_module, args=('second', second_name), daemon=True)
        try:
            first.start()
            assert first_ready.wait(timeout), 'Native initialization did not start'
            if 'first' in errors:
                raise errors['first']
            second.start()
            assert second_started.wait(timeout), 'Concurrent import did not start'
            returned_early = second_done.wait(blocking_window)
        finally:
            release.set()
            first.join(timeout)
            if second.ident is not None:
                second.join(timeout)
            importlib.machinery.ExtensionFileLoader.exec_module = original_exec

        assert not first.is_alive() and not second.is_alive(), 'Import deadlock'
        assert not returned_early, 'Concurrent import returned an unfinished native module'
        assert second_done.is_set()
        if fail_first:
            assert isinstance(errors.get('first'), RuntimeError), errors
            assert str(errors['first']) == 'Injected native initialization failure'
            if 'second' in errors:
                # A waiting import may fail with the first package import on
                # some Python versions. Its next attempt must initialize cleanly.
                assert isinstance(errors['second'], ImportError), errors
                assert len(attempts) == 1
                assert native_name not in sys.modules
                assert public_name not in sys.modules
                importlib.machinery.ExtensionFileLoader.exec_module = paused_exec
                try:
                    modules['second'] = importlib.import_module(second_name)
                finally:
                    importlib.machinery.ExtensionFileLoader.exec_module = original_exec
                del errors['second']
            assert set(errors) == {'first'}, errors
            assert len(attempts) == 2
        else:
            assert not errors, errors
            assert len(attempts) == 1

        native = importlib.import_module(native_name)
        messaging = importlib.import_module(package_name)
        assert modules['second'] is importlib.import_module(second_name)
        assert native is attempts[-1]
        assert messaging._AttRefMsgPayload is native
        assert all(not getattr(module.__spec__, '_initializing', False) for module in attempts)
        # A failed package import can leave earlier native bindings cached.
        # Retrying must also attach those bindings to the new package object.
        for name, module in tuple(sys.modules.items()):
            if name.startswith(package_name + '._') and isinstance(
                    module.__loader__, importlib.machinery.ExtensionFileLoader):
                assert getattr(messaging, name.rsplit('.', 1)[1]) is module

        if fail_first:
            payload = native.new_AttRefMsgPayload()
            try:
                assert native.AttRefMsgPayload_sigma_RN_get(payload) == [0.0, 0.0, 0.0]
            finally:
                native.delete_AttRefMsgPayload(payload)
                payload.own(False)
        else:
            proxy = importlib.import_module(public_name)
            assert proxy._AttRefMsgPayload is native
            payload = proxy.AttRefMsgPayload()
            payload.sigma_RN = [0.1, 0.2, 0.3]  # [-] MRP attitude
            assert native.AttRefMsgPayload_sigma_RN_get(payload) == [0.1, 0.2, 0.3]
            message = messaging.AttRefMsg().write(payload)
            recorder = message.recorder()
            recorder.UpdateState(0)  # [ns]
            assert recorder.sigma_RN.tolist() == [[0.1, 0.2, 0.3]]
        importlib.reload(messaging)
        assert importlib.import_module(native_name) is native
        """)
    subprocess_timeout = 60  # [s]
    result = subprocess.run(
        [sys.executable, "-c", code, first_import, second_import,
         str(fail_first), package_name, str(tmp_path), str(Path(messaging.__file__).resolve().parents[2])],
        capture_output=True, text=True, timeout=subprocess_timeout,
    )
    assert result.returncode == 0, result.stdout + result.stderr
