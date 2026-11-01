# Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
# Distributed under the ISC license; see LICENSE.

"""Exercise numeric vectors shared by payload and module SWIG bindings."""

import itertools
import os
from pathlib import Path
import subprocess
import sys

import Basilisk
import pytest


@pytest.mark.parametrize("import_order", list(itertools.permutations(
    ["simpleInstrument", "zeroWindModel", "linkBudget"]
)))
def test_storage_vectors_survive_module_imports(import_order):
    """Message reads and lookup-table vectors work after each module import."""
    script = r'''
import importlib
import sys

from Basilisk.architecture import messaging

payload = messaging.DataStorageStatusMsgPayload()
payload.storedData = messaging.DoubleVector([400.0, 1200.0])  # [bit]
message = messaging.DataStorageStatusMsg().write(payload)
reader = messaging.DataStorageStatusMsgReader()
reader.subscribeTo(message)

for module_name in sys.argv[1:]:
    module = importlib.import_module("Basilisk.simulation." + module_name)
    # Each read must use the current shared payload registration, not a proxy
    # created before the import that happens to keep a working Python class.
    for result in (message.read(), reader()):
        values = result.storedData
        assert values[0] == 400.0  # [bit]
        assert values[-1] == 1200.0  # [bit]
        assert list(values[:]) == [400.0, 1200.0]  # [bit]
        values[0] = 500.0  # [bit]
        assert result.storedData[0] == 500.0  # [bit]
        values[0] = 400.0  # [bit]
    if module_name == "zeroWindModel":
        values = module.VectorDouble([1.0, 2.0])  # [-]
        assert values[0] == 1.0
        values[1] = 3.0  # [-]
        assert list(values[:]) == [1.0, 3.0]
    if module_name == "linkBudget":
        table = module.LookupTable()
        table.f_l = messaging.DoubleVector([2.2, 8.4])  # [GHz]
        assert table.f_l[0] == 2.2  # [GHz]
        table.f_l[1] = 8.5  # [GHz]
        assert list(table.f_l[:]) == [2.2, 8.5]  # [GHz]
print("Numeric-vector checks passed")
'''
    environment = os.environ.copy()
    package_parent = str(Path(Basilisk.__file__).resolve().parent.parent)
    environment["PYTHONPATH"] = os.pathsep.join(filter(None, [package_parent, environment.get("PYTHONPATH")]))
    environment["PYTHONMALLOC"] = "debug"
    result = subprocess.run(
        [sys.executable, "-X", "faulthandler", "-c", script, *import_order],
        env=environment, capture_output=True, text=True, timeout=120,  # [s]
    )
    assert result.returncode == 0, result.stdout + result.stderr
    assert "Numeric-vector checks passed" in result.stdout
