# Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
# Distributed under the ISC license; see LICENSE.

"""Verify ordinary message wiring and pointer-vector ownership across imports."""

import os
from pathlib import Path
import subprocess
import sys

import Basilisk
import pytest


@pytest.mark.skipif(not Basilisk.hasBuildFeature("vizInterface"), reason="Requires vizInterface")
@pytest.mark.parametrize("vizard_first", [False, True])
@pytest.mark.parametrize("message_type, field", [("THROutputMsg", "thrInMsgs"),
                                                ("RWConfigLogMsg", "rwInMsgs")])
def test_shared_message_vectors_preserve_wiring(vizard_first, message_type, field):
    """Temporary nested views accept wiring and raw pointer copies remain unowned."""
    script = r'''
import gc
import sys
import weakref

if sys.argv[1] == "True":
    from Basilisk.simulation import dataFileToViz, vizInterface
from Basilisk.architecture import messaging
if sys.argv[1] == "False":
    from Basilisk.simulation import vizInterface, dataFileToViz

message_type, field = sys.argv[2:]
message_class = getattr(messaging, message_type)

def check_nested_wiring():
    consumer = vizInterface.VizInterface()
    source = message_class()
    state = messaging.SCStatesMsg()
    entry = vizInterface.VizSpacecraftData()
    consumer.scData.push_back(entry)
    # Both intermediate spacecraft-data proxies disappear before the calls.
    # The live enclosing module still owns the readers' native storage.
    consumer.scData[0].scStateInMsg.subscribeTo(state)
    getattr(consumer.scData[0], field).append(source.addSubscriber())
    assert consumer.scData[0].scStateInMsg.isSubscribedTo(state)
    assert getattr(consumer.scData[0], field)[0].isSubscribedTo(source)
    # Copying a configured spacecraft entry also retains its input sources.
    entry.scStateInMsg.subscribeTo(state)
    getattr(entry, field).append(source.addSubscriber())
    consumer.scData = vizInterface.VizSCVector([entry])
    assert consumer.scData[0].scStateInMsg.isSubscribedTo(state)
    assert getattr(consumer.scData[0], field)[0].isSubscribedTo(source)
    return [weakref.ref(consumer), weakref.ref(source), weakref.ref(state)]

references = check_nested_wiring()
gc.collect()
assert all(reference() is None for reference in references)

# Exercise both classes, including whichever is no longer the active SWIG
# registration after the final import.
for module in (messaging, dataFileToViz):
    source = message_class()
    reader_vector = getattr(module, message_type + "InMsgsVector")()
    reader_vector.append(source.addSubscriber())
    assert reader_vector[0].isSubscribedTo(source)
    reader_vector.clear()

    pointers = getattr(module, message_type + "OutMsgsPtrVector")([source])
    for output in (pointers[0], next(iter(pointers))):
        try:
            output._redirection_owner()
        except ReferenceError as error:
            assert "storage owner is unknown" in str(error)
        else:
            raise AssertionError("A raw pointer-vector copy claimed to own its pointee")
print("Shared-vector checks passed")
'''
    environment = os.environ.copy()
    package_parent = str(Path(Basilisk.__file__).resolve().parent.parent)
    environment["PYTHONPATH"] = os.pathsep.join(filter(None, [package_parent, environment.get("PYTHONPATH")]))
    environment["PYTHONMALLOC"] = "debug"
    result = subprocess.run(
        [sys.executable, "-X", "faulthandler", "-c", script, str(vizard_first), message_type, field],
        env=environment, capture_output=True, text=True, timeout=120,  # [s]
    )
    assert result.returncode == 0, result.stdout + result.stderr
    assert "Shared-vector checks passed" in result.stdout


@pytest.mark.parametrize("late_module", [
    "simpleInstrument",
    pytest.param("dataFileToViz", marks=pytest.mark.skipif(
        not Basilisk.hasBuildFeature("vizInterface"), reason="Requires vizInterface")),
])
@pytest.mark.parametrize("operation", ["copy", "incr", "decr", "advance",
                                       "add", "subtract", "iadd", "isub"])
def test_message_iterator_owner_survives_late_import(late_module, operation):
    """Late imports preserve iterator ownership, redirection, and deletion checks."""
    script = r'''
import gc
import importlib
import sys
import weakref

import pytest
from Basilisk.architecture import messaging
from Basilisk.simulation import dualHingedRigidBodyStateEffector, spacecraft

source = dualHingedRigidBodyStateEffector.DualHingedRigidBodyStateEffector()
reference = weakref.ref(source)
vector = source.dualHingedRigidBodyConfigLogOutMsgs
operation = sys.argv[2]
iterator = vector.end() if operation in ("decr", "isub") else vector.begin()

# Import after constructing the iterator so SWIG replaces its shared wrapper
# registration while a live instance of the previous class still exists.
importlib.import_module("Basilisk.simulation." + sys.argv[1])
operations = {
    "copy": lambda: iterator.copy(),
    "incr": lambda: iterator.incr(),
    "decr": lambda: iterator.decr(),
    "advance": lambda: iterator.advance(1),
    "add": lambda: iterator + 1,
    "subtract": lambda: iterator - (-1),
    "iadd": lambda: iterator.__iadd__(1),
    "isub": lambda: iterator.__isub__(1),
}
result = operations[operation]()
output = result.value()
index = 0 if operation == "copy" else 1
address = vector[index].getPayloadAddress()
assert output.getPayloadAddress() == address
# Returned wrappers must themselves propagate ownership to further results.
assert result.copy().copy().value()._redirection_owner() is source
# Subtraction of iterators returns a distance, not an iterator to tag.
assert result - vector.begin() == index

destination = spacecraft.Spacecraft()
destination.scStateOutMsg = output
del source, vector
gc.collect()
assert reference() is not None
payload = messaging.SCStatesMsgPayload()
payload.r_BN_N = [1.0, 2.0, 3.0]  # [m]
destination.scStateOutMsg.write(payload)
assert list(reference().dualHingedRigidBodyConfigLogOutMsgs[index].read().r_BN_N) == [1.0, 2.0, 3.0]

# Patching the newly registered class must leave other vector results alone.
numbers = messaging.DoubleVector([1.0, 2.0])  # [-]
assert (numbers.begin().copy() + 1).value() == 2.0
assert numbers.end() - numbers.begin() == 2
message = messaging.NavTransMsg()
readers = messaging.NavTransMsgInMsgsVector([message.addSubscriber()])
reader = readers.begin().copy().value()
assert reader.thisown
reader.unsubscribe()
assert readers[0].isSubscribedTo(message)

del destination
gc.collect()
assert reference() is None
for access in (result.value, result.copy, output._redirection_owner):
    with pytest.raises(ReferenceError, match="module owning this message has been deleted"):
        access()
print("Late-import iterator checks passed")
'''
    environment = os.environ.copy()
    package_parent = str(Path(Basilisk.__file__).resolve().parent.parent)
    environment["PYTHONPATH"] = os.pathsep.join(filter(None, [package_parent, environment.get("PYTHONPATH")]))
    environment["PYTHONMALLOC"] = "debug"
    result = subprocess.run(
        [sys.executable, "-X", "faulthandler", "-c", script, late_module, operation],
        env=environment, capture_output=True, text=True, timeout=120,  # [s]
    )
    assert result.returncode == 0, result.stdout + result.stderr
    assert "Late-import iterator checks passed" in result.stdout
