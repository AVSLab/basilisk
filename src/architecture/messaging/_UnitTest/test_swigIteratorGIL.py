# Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
# Distributed under the ISC license; see LICENSE.

"""Check shared SWIG iterator bindings under different module import orders."""

import os
from pathlib import Path
import subprocess
import sys

import Basilisk
import pytest


@pytest.mark.parametrize("simulation_first", [False, True])
def test_swig_iterator_holds_gil_across_import_orders(simulation_first, tmp_path):
    """Allocation and boundary errors remain safe regardless of the active wrapper."""
    script = r'''
import importlib
import sys

if sys.argv[1] == "True":
    from Basilisk.utilities import SimulationBaseClass
from Basilisk.architecture import messaging
from Basilisk.simulation import dualHingedRigidBodyStateEffector
if sys.argv[1] == "False":
    from Basilisk.utilities import SimulationBaseClass

source = dualHingedRigidBodyStateEffector.DualHingedRigidBodyStateEffector()
vector = source.dualHingedRigidBodyConfigLogOutMsgs
iterator = vector.iterator()
assert iterator.value().getPayloadAddress() == vector[0].getPayloadAddress()
assert (iterator + 1).value().getPayloadAddress() == vector[1].getPayloadAddress()
assert iterator.copy().value().getPayloadAddress() == vector[0].getPayloadAddress()

def expect_stop(operation):
    try:
        operation()
    except StopIteration:
        return
    raise AssertionError("Expected StopIteration")

exhausted = vector.iterator()
exhausted.incr(len(vector))
expect_stop(exhausted.value)
expect_stop(exhausted.incr)
expect_stop(lambda: exhausted.advance(1))
expect_stop(lambda: exhausted.__iadd__(1))
expect_stop(lambda: exhausted + 1)
start = vector.iterator()
expect_stop(start.decr)
expect_stop(lambda: start.advance(-1))
expect_stop(lambda: start.__isub__(1))
expect_stop(lambda: start - 1)

# Also exercise the threaded entry points directly, independent of which
# Python SwigPyIterator class was registered last in the shared SWIG runtime.
for name in ("Basilisk.architecture._py_sys_model",
             "Basilisk.architecture.messaging._SCStatesMsgPayload",
             "Basilisk.architecture.messaging._messagingSupport"):
    native = importlib.import_module(name)
    assert native.SwigPyIterator_value(iterator).getPayloadAddress() == vector[0].getPayloadAddress()
    expect_stop(lambda: native.SwigPyIterator_value(exhausted))
    expect_stop(lambda: native.SwigPyIterator_incr(exhausted))
    expect_stop(lambda: native.SwigPyIterator_decr(start))

numbers = messaging.DoubleVector([1.0, 2.0])  # [-]
assert numbers.begin().value() == 1.0
print("Iterator checks passed")
'''
    environment = os.environ.copy()
    package_parent = str(Path(Basilisk.__file__).resolve().parent.parent)
    environment["PYTHONPATH"] = os.pathsep.join(filter(None, [package_parent, environment.get("PYTHONPATH")]))
    environment["PYTHONMALLOC"] = "debug"
    environment["MPLCONFIGDIR"] = str(tmp_path / "matplotlib")
    result = subprocess.run(
        [sys.executable, "-X", "faulthandler", "-c", script, str(simulation_first)],
        env=environment, capture_output=True, text=True, timeout=120,  # [s]
    )
    assert result.returncode == 0, result.stdout + result.stderr
    assert "Iterator checks passed" in result.stdout
