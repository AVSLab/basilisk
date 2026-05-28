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
"""
Unit tests for NumbaModel.

Section  Topic
-------  ----------------------------------------------------------------------
1        Basic scalar I/O + memory namespace (smoke test)
2        Auto-convert scalar / list memory at Reset time
3        Post-Reset convenience writes via memory namespace
4        CurrentSimNanos parameter
5        IsLinked: scalar reader (unlinked, linked, becomes linked mid-sim)
6        Message chain: producer -> consumer (scalar I/O, memory)
7        List readers: all linked, partial link (IsLinked array)
8        List writers
9        Dict readers: key introspection, data round-trip, unlinked error
10       Dict writers
11       2-D memory array
12       moduleID parameter
13       Re-wiring: subscribeTo after Reset
14       Memory persistence across resets
15       Error cases (incl. empty containers, empty memory)
16       Dict reader IsLinked keyed access
17       bskLogger proxy
"""
import subprocess
import sys
import textwrap

import numpy as np
import pytest

from Basilisk.architecture import messaging, bskLogging
from Basilisk.utilities import SimulationBaseClass

try:
    import Basilisk.architecture.numbaModel as numbaModelModule
    from Basilisk.architecture.numbaModel import NumbaModel, _NBMODEL_CFUNC_CACHE
    couldImport = True
except Exception:
    numbaModelModule = None
    NumbaModel = object
    _NBMODEL_CFUNC_CACHE = {}
    couldImport = False

pytestmark = pytest.mark.skipif(
    not couldImport,
    reason="Compiled Basilisk without numba support or numba not available",
)


# ---------------------------------------------------------------------------
# Helper: build a minimal sim, add a model, initialize and run for N ticks
# ---------------------------------------------------------------------------
def _run(model, dt_ns=5, n_ticks=5):
    scSim = SimulationBaseClass.SimBaseClass()
    proc  = scSim.CreateNewProcess("proc")
    proc.addTask(scSim.CreateNewTask("task", dt_ns))
    scSim.AddModelToTask("task", model)
    scSim.InitializeSimulation()
    scSim.ConfigureStopTime(dt_ns * (n_ticks - 1))
    scSim.ExecuteSimulation()
    return scSim


# ===========================================================================
# 1. Basic scalar I/O + memory namespace
# ===========================================================================
class BasicModel(NumbaModel):
    """Minimal scalar input/output model with one integer memory field."""

    def __init__(self):
        """Declare one optional reader, one writer, and the step counter."""
        super().__init__()
        self.dataInMsg  = messaging.CModuleTemplateMsgReader()
        self.dataOutMsg = messaging.CModuleTemplateMsg()
        self.memory.step = np.int32(0)

    @staticmethod
    def UpdateStateImpl(dataInMsgPayload, dataInMsgIsLinked,
                        dataOutMsgPayload, CurrentSimNanos, memory):
        """Forward the input vector when linked and increment the step count."""
        vec = np.zeros(3)
        if dataInMsgIsLinked:
            vec[:] = dataInMsgPayload.dataVector
        memory.step += np.int32(1)
        out = vec.copy()
        out[0] += memory.step
        dataOutMsgPayload.dataVector[:] = out


def test_basicModel():
    """Smoke-test scalar I/O and persistent memory updates."""
    mod = BasicModel()
    mod.ModelTag = "basic"
    _run(mod, dt_ns=5, n_ticks=5)

    out = mod.dataOutMsg.read()
    assert out.dataVector[0] == pytest.approx(5.0)   # step counts 5 ticks
    assert mod.memory.step == 5


# ===========================================================================
# 2. Auto-convert scalar / list memory at Reset time
# ===========================================================================
class AutoConvertModel(NumbaModel):
    """Model used to verify scalar and list memory auto-conversion."""

    def __init__(self):
        """Declare one writer and memory fields with Python-native defaults."""
        super().__init__()
        self.dataOutMsg = messaging.CModuleTemplateMsg()
        self.memory.kp   = 2.0              # plain float → scalar field
        self.memory.bias = [0.1, 0.2, 0.3]  # list        → 1-D array field

    @staticmethod
    def UpdateStateImpl(dataOutMsgPayload, memory):
        """Write the gain-plus-bias result into the outgoing message."""
        for i in range(3):
            dataOutMsgPayload.dataVector[i] = memory.kp + memory.bias[i]


def test_autoConvertMemory():
    """Check scalar and list memory fields are converted at reset."""
    mod = AutoConvertModel()
    _run(mod, dt_ns=5, n_ticks=1)

    assert float(mod.memory.kp)  == pytest.approx(2.0)
    assert mod.memory.bias.shape == (3,)

    out = mod.dataOutMsg.read()
    np.testing.assert_allclose(out.dataVector, [2.1, 2.2, 2.3])


# ===========================================================================
# 3. Post-Reset convenience writes via memory namespace
# ===========================================================================
def test_postResetConvenienceWrite():
    """Check post-reset memory writes update existing fields and reject new ones."""
    mod = BasicModel()
    _run(mod, dt_ns=5, n_ticks=1)

    # Scalar convenience write propagates to the underlying buffer
    mod.memory.step = np.int32(99)
    assert mod.memory.step == 99

    mod.memory.step = np.int32(42)
    assert mod.memory.step == 42

    # Setting a non-existent field after Reset must raise
    with pytest.raises(AttributeError, match="does not exist after Reset"):
        mod.memory.new_field = 1


# ===========================================================================
# 4. CurrentSimNanos parameter
# ===========================================================================
class TimeRecorderModel(NumbaModel):
    """Records the last simulation time and a tick counter in memory."""
    def __init__(self):
        """Declare one writer plus timestamp and counter memory fields."""
        super().__init__()
        self.dataOutMsg    = messaging.CModuleTemplateMsg()
        self.memory.last_ns    = np.uint64(0)
        self.memory.tick_count = np.int32(0)

    @staticmethod
    def UpdateStateImpl(dataOutMsgPayload, CurrentSimNanos, memory):
        """Store the current simulation time and publish it as a float."""
        memory.last_ns     = CurrentSimNanos
        memory.tick_count += np.int32(1)
        dataOutMsgPayload.dataVector[0] = float(CurrentSimNanos)


def test_currentSimNanos():
    """Check the compiled model receives the current simulation time."""
    mod = TimeRecorderModel(); mod.ModelTag = "timer"
    # ticks at t = 0, 5, 10 ns
    _run(mod, dt_ns=5, n_ticks=3)

    assert int(mod.memory.last_ns)   == 10   # last tick at 10 ns
    assert mod.memory.tick_count     == 3
    assert mod.dataOutMsg.read().dataVector[0] == pytest.approx(10.0)


# ===========================================================================
# 5. IsLinked flag - scalar reader
# ===========================================================================
class GuardedReaderModel(NumbaModel):
    """Forwards payload when linked; writes sentinel −1 when unlinked."""
    def __init__(self):
        """Declare one guarded reader, one writer, and a linkage flag."""
        super().__init__()
        self.dataInMsg  = messaging.CModuleTemplateMsgReader()
        self.dataOutMsg = messaging.CModuleTemplateMsg()
        self.memory.ever_linked = np.int32(0)

    @staticmethod
    def UpdateStateImpl(dataInMsgPayload, dataInMsgIsLinked,
                        dataOutMsgPayload, memory):
        """Copy the input when linked and publish a sentinel otherwise."""
        if dataInMsgIsLinked:
            memory.ever_linked = np.int32(1)
            dataOutMsgPayload.dataVector[:] = dataInMsgPayload.dataVector
        else:
            dataOutMsgPayload.dataVector[0] = -1.0


def test_islinkedUnlinked():
    """Unlinked reader → IsLinked is False; cfunc takes the else branch."""
    mod = GuardedReaderModel(); mod.ModelTag = "unlinked"
    _run(mod, dt_ns=5, n_ticks=2)
    assert mod.memory.ever_linked == 0
    assert mod.dataOutMsg.read().dataVector[0] == pytest.approx(-1.0)


def test_islinkedLinked():
    """Subscribed reader → IsLinked is True; payload is forwarded."""
    src = messaging.CModuleTemplateMsg()
    mod = GuardedReaderModel(); mod.ModelTag = "linked"
    mod.dataInMsg.subscribeTo(src)

    scSim = SimulationBaseClass.SimBaseClass()
    proc  = scSim.CreateNewProcess("proc")
    proc.addTask(scSim.CreateNewTask("task", 5))
    scSim.AddModelToTask("task", mod)
    scSim.InitializeSimulation()

    p = messaging.CModuleTemplateMsgPayload()
    p.dataVector = [7.0, 8.0, 9.0]
    src.write(p)

    scSim.ConfigureStopTime(5)
    scSim.ExecuteSimulation()

    assert mod.memory.ever_linked == 1
    np.testing.assert_allclose(mod.dataOutMsg.read().dataVector, [7.0, 8.0, 9.0])


def test_islinkedBecomesLinkedMidSim():
    """Reader starts unlinked; subscribed after first tick - cfunc picks it up."""
    src = messaging.CModuleTemplateMsg()
    mod = GuardedReaderModel(); mod.ModelTag = "midlink"

    scSim = SimulationBaseClass.SimBaseClass()
    proc  = scSim.CreateNewProcess("proc")
    proc.addTask(scSim.CreateNewTask("task", 5))
    scSim.AddModelToTask("task", mod)
    scSim.InitializeSimulation()

    scSim.ConfigureStopTime(5)       # t=0 - still unlinked
    scSim.ExecuteSimulation()
    assert mod.memory.ever_linked == 0
    assert mod.dataOutMsg.read().dataVector[0] == pytest.approx(-1.0)

    # Subscribe mid-sim (no Reset)
    mod.dataInMsg.subscribeTo(src)
    p = messaging.CModuleTemplateMsgPayload()
    p.dataVector = [2.0, 3.0, 4.0]
    src.write(p)

    scSim.ConfigureStopTime(10)      # t=5 - now linked
    scSim.ExecuteSimulation()
    assert mod.memory.ever_linked == 1
    np.testing.assert_allclose(mod.dataOutMsg.read().dataVector, [2.0, 3.0, 4.0])


# ===========================================================================
# 6. Message chain: producer → consumer
# ===========================================================================
class ProducerModel(NumbaModel):
    """Source model used to exercise chained message passing."""

    def __init__(self):
        """Declare one writer and a counter stored in persistent memory."""
        super().__init__()
        self.dataOutMsg = messaging.CModuleTemplateMsg()
        self.memory.count = 0.0

    @staticmethod
    def UpdateStateImpl(dataOutMsgPayload, memory):
        """Increment the counter and publish three scaled copies."""
        memory.count += 1.0
        dataOutMsgPayload.dataVector[0] = memory.count
        dataOutMsgPayload.dataVector[1] = memory.count * 2.0
        dataOutMsgPayload.dataVector[2] = memory.count * 3.0


class ConsumerModel(NumbaModel):
    """Sink model that scales the producer output by a fixed factor."""

    def __init__(self):
        """Declare one input reader and one output writer."""
        super().__init__()
        self.dataInMsg  = messaging.CModuleTemplateMsgReader()
        self.dataOutMsg = messaging.CModuleTemplateMsg()

    @staticmethod
    def UpdateStateImpl(dataInMsgPayload, dataInMsgIsLinked, dataOutMsgPayload):
        """Copy the input vector into the output after multiplying by ten."""
        if dataInMsgIsLinked:
            for i in range(3):
                dataOutMsgPayload.dataVector[i] = dataInMsgPayload.dataVector[i] * 10.0


def test_messageChain():
    """Check a NumbaModel output can feed another NumbaModel input."""
    scSim = SimulationBaseClass.SimBaseClass()
    proc  = scSim.CreateNewProcess("proc")
    proc.addTask(scSim.CreateNewTask("task", 5))

    prod = ProducerModel(); prod.ModelTag = "prod"
    cons = ConsumerModel(); cons.ModelTag = "cons"
    scSim.AddModelToTask("task", prod)
    scSim.AddModelToTask("task", cons)

    cons.dataInMsg.subscribeTo(prod.dataOutMsg)

    scSim.InitializeSimulation()
    scSim.ConfigureStopTime(20)  # t=0,5,10,15,20 → 5 ticks
    scSim.ExecuteSimulation()

    out = cons.dataOutMsg.read()
    # prod ran 5 times → count=5; cons scales ×10
    assert out.dataVector[0] == pytest.approx(50.0)
    assert out.dataVector[1] == pytest.approx(100.0)
    assert out.dataVector[2] == pytest.approx(150.0)


# ===========================================================================
# 7. List of messages - readers
# ===========================================================================
class MultiInModel(NumbaModel):
    """Two input readers summed into one output."""
    def __init__(self):
        """Declare a two-reader container feeding one output message."""
        super().__init__()
        self.dataInMsg  = [messaging.CModuleTemplateMsgReader(),
                           messaging.CModuleTemplateMsgReader()]
        self.dataOutMsg = messaging.CModuleTemplateMsg()

    @staticmethod
    def UpdateStateImpl(dataInMsgPayload, dataInMsgIsLinked, dataOutMsgPayload):
        """Sum the first component from each linked reader."""
        total = 0.0
        for i in range(2):
            if dataInMsgIsLinked[i]:
                total += dataInMsgPayload[i].dataVector[0]
        dataOutMsgPayload.dataVector[0] = total


def test_listReaderBothLinked():
    """Both readers subscribed → sum of both sources."""
    scSim = SimulationBaseClass.SimBaseClass()
    proc  = scSim.CreateNewProcess("proc")
    proc.addTask(scSim.CreateNewTask("task", 5))

    src0 = messaging.CModuleTemplateMsg()
    src1 = messaging.CModuleTemplateMsg()
    mod  = MultiInModel(); mod.ModelTag = "multi"

    scSim.AddModelToTask("task", mod)
    mod.dataInMsg[0].subscribeTo(src0)
    mod.dataInMsg[1].subscribeTo(src1)
    scSim.InitializeSimulation()

    p0 = messaging.CModuleTemplateMsgPayload(); p0.dataVector = [3.0, 0.0, 0.0]
    p1 = messaging.CModuleTemplateMsgPayload(); p1.dataVector = [7.0, 0.0, 0.0]
    src0.write(p0); src1.write(p1)

    scSim.ConfigureStopTime(5)
    scSim.ExecuteSimulation()

    assert mod.dataOutMsg.read().dataVector[0] == pytest.approx(10.0)


def test_listReaderPartialLink():
    """reader[0] subscribed, reader[1] unlinked → only reader[0] contributes."""
    src = messaging.CModuleTemplateMsg()
    mod = MultiInModel(); mod.ModelTag = "partial"

    scSim = SimulationBaseClass.SimBaseClass()
    proc  = scSim.CreateNewProcess("proc")
    proc.addTask(scSim.CreateNewTask("task", 5))
    scSim.AddModelToTask("task", mod)
    mod.dataInMsg[0].subscribeTo(src)
    # dataInMsg[1] intentionally left unlinked

    scSim.InitializeSimulation()

    p = messaging.CModuleTemplateMsgPayload(); p.dataVector = [4.0, 0.0, 0.0]
    src.write(p)

    scSim.ConfigureStopTime(5)
    scSim.ExecuteSimulation()

    # IsLinked[0]=True (4.0), IsLinked[1]=False (0.0) → sum = 4.0
    assert mod.dataOutMsg.read().dataVector[0] == pytest.approx(4.0)


def test_listReaderNoneLinked():
    """Both readers unlinked → output stays zero."""
    mod = MultiInModel(); mod.ModelTag = "nonelinked"
    _run(mod, dt_ns=5, n_ticks=1)
    assert mod.dataOutMsg.read().dataVector[0] == pytest.approx(0.0)


# ===========================================================================
# 8. List of messages - writers
# ===========================================================================
class MultiOutModel(NumbaModel):
    """One input reader → two indexed output writers."""
    def __init__(self):
        """Declare one reader and two output messages in a list."""
        super().__init__()
        self.dataInMsg  = messaging.CModuleTemplateMsgReader()
        self.dataOutMsg = [messaging.CModuleTemplateMsg(),
                           messaging.CModuleTemplateMsg()]

    @staticmethod
    def UpdateStateImpl(dataInMsgPayload, dataInMsgIsLinked, dataOutMsgPayload):
        """Write the input value and its doubled copy to the outputs."""
        v = 0.0
        if dataInMsgIsLinked:
            v = dataInMsgPayload.dataVector[0]
        dataOutMsgPayload[0].dataVector[0] = v
        dataOutMsgPayload[1].dataVector[0] = v * 2.0


def test_listWriter():
    """Check list-based output messages are written by index."""
    scSim = SimulationBaseClass.SimBaseClass()
    proc  = scSim.CreateNewProcess("proc")
    proc.addTask(scSim.CreateNewTask("task", 5))

    src = messaging.CModuleTemplateMsg()
    mod = MultiOutModel(); mod.ModelTag = "multiout"
    scSim.AddModelToTask("task", mod)
    mod.dataInMsg.subscribeTo(src)
    scSim.InitializeSimulation()

    p = messaging.CModuleTemplateMsgPayload(); p.dataVector = [5.0, 0.0, 0.0]
    src.write(p)

    scSim.ConfigureStopTime(5)
    scSim.ExecuteSimulation()

    assert mod.dataOutMsg[0].read().dataVector[0] == pytest.approx(5.0)
    assert mod.dataOutMsg[1].read().dataVector[0] == pytest.approx(10.0)


# ===========================================================================
# 9. Dict of messages - readers
# ===========================================================================
class DictInModel(NumbaModel):
    """Dictionary-based input reader model used for key-order tests."""

    def __init__(self):
        """Declare two named readers and one output message."""
        super().__init__()
        self.dataInMsg = {
            'a': messaging.CModuleTemplateMsgReader(),
            'b': messaging.CModuleTemplateMsgReader(),
        }
        self.dataOutMsg = messaging.CModuleTemplateMsg()

    @staticmethod
    def UpdateStateImpl(dataInMsgPayload, dataOutMsgPayload):
        """Route the named inputs into separate output vector components."""
        dataOutMsgPayload.dataVector[0] = dataInMsgPayload['a'].dataVector[0]
        dataOutMsgPayload.dataVector[1] = dataInMsgPayload['b'].dataVector[0]


def test_dictReaderKeys():
    """Key order is preserved and accessible from Python after Reset."""
    src_a = messaging.CModuleTemplateMsg()
    src_b = messaging.CModuleTemplateMsg()
    mod = DictInModel(); mod.ModelTag = "dictIn"
    scSim = SimulationBaseClass.SimBaseClass()
    proc = scSim.CreateNewProcess("p"); proc.addTask(scSim.CreateNewTask("t", 5))
    scSim.AddModelToTask("t", mod)
    mod.dataInMsg['a'].subscribeTo(src_a)
    mod.dataInMsg['b'].subscribeTo(src_b)
    scSim.InitializeSimulation()
    assert mod._dataInMsgKeys == ('a', 'b')


def test_dictReaderData():
    """Payload from each dict key is routed to the correct output field."""
    src_a = messaging.CModuleTemplateMsg()
    src_b = messaging.CModuleTemplateMsg()
    mod = DictInModel(); mod.ModelTag = "dictInData"

    scSim = SimulationBaseClass.SimBaseClass()
    proc  = scSim.CreateNewProcess("p")
    proc.addTask(scSim.CreateNewTask("t", 5))
    scSim.AddModelToTask("t", mod)
    mod.dataInMsg['a'].subscribeTo(src_a)
    mod.dataInMsg['b'].subscribeTo(src_b)
    scSim.InitializeSimulation()

    p_a = messaging.CModuleTemplateMsgPayload(); p_a.dataVector = [3.0, 0.0, 0.0]
    p_b = messaging.CModuleTemplateMsgPayload(); p_b.dataVector = [7.0, 0.0, 0.0]
    src_a.write(p_a); src_b.write(p_b)

    scSim.ConfigureStopTime(5)
    scSim.ExecuteSimulation()

    out = mod.dataOutMsg.read()
    assert out.dataVector[0] == pytest.approx(3.0)   # from 'a'
    assert out.dataVector[1] == pytest.approx(7.0)   # from 'b'


def test_errorUnlinkedWithoutIslinkedGuard():
    """Declaring ``InMsgPayload`` without ``InMsgIsLinked`` crashes at reset if a reader is unlinked."""
    src_a = messaging.CModuleTemplateMsg()
    mod = DictInModel(); mod.ModelTag = "dictUnguarded"
    mod.dataInMsg['a'].subscribeTo(src_a)
    # 'b' intentionally left unlinked - no IsLinked guard in UpdateStateImpl

    scSim = SimulationBaseClass.SimBaseClass()
    proc  = scSim.CreateNewProcess("p")
    proc.addTask(scSim.CreateNewTask("t", 5))
    scSim.AddModelToTask("t", mod)

    with pytest.raises(RuntimeError):
        scSim.InitializeSimulation()


# ===========================================================================
# 10. Dict of messages - writers
# ===========================================================================
class DictOutModel(NumbaModel):
    """One input reader → two named output messages."""
    def __init__(self):
        """Declare one reader and two named output writers."""
        super().__init__()
        self.dataInMsg  = messaging.CModuleTemplateMsgReader()
        self.dataOutMsg = {
            'pos': messaging.CModuleTemplateMsg(),
            'neg': messaging.CModuleTemplateMsg(),
        }

    @staticmethod
    def UpdateStateImpl(dataInMsgPayload, dataInMsgIsLinked, dataOutMsgPayload):
        """Write positive and negative copies of the input value."""
        v = 0.0
        if dataInMsgIsLinked:
            v = dataInMsgPayload.dataVector[0]
        dataOutMsgPayload['pos'].dataVector[0] =  v
        dataOutMsgPayload['neg'].dataVector[0] = -v


def test_dictWriter():
    """Check dictionary-based output messages are written by key."""
    src = messaging.CModuleTemplateMsg()
    mod = DictOutModel(); mod.ModelTag = "dictOut"

    scSim = SimulationBaseClass.SimBaseClass()
    proc  = scSim.CreateNewProcess("p")
    proc.addTask(scSim.CreateNewTask("t", 5))
    scSim.AddModelToTask("t", mod)
    mod.dataInMsg.subscribeTo(src)
    scSim.InitializeSimulation()

    p = messaging.CModuleTemplateMsgPayload(); p.dataVector = [6.0, 0.0, 0.0]
    src.write(p)

    scSim.ConfigureStopTime(5)
    scSim.ExecuteSimulation()

    assert mod.dataOutMsg['pos'].read().dataVector[0] == pytest.approx( 6.0)
    assert mod.dataOutMsg['neg'].read().dataVector[0] == pytest.approx(-6.0)


# ===========================================================================
# 11. 2-D memory array
# ===========================================================================
class MatrixMemoryModel(NumbaModel):
    """Stores a 3×3 diagonal matrix; outputs the diagonal each tick."""
    def __init__(self):
        """Declare one output message and a diagonal matrix memory field."""
        super().__init__()
        self.dataOutMsg  = messaging.CModuleTemplateMsg()
        self.memory.diag = np.diag([1.0, 2.0, 3.0])   # shape (3, 3)

    @staticmethod
    def UpdateStateImpl(dataOutMsgPayload, memory):
        """Copy the matrix diagonal into the output vector."""
        for i in range(3):
            dataOutMsgPayload.dataVector[i] = memory.diag[i, i]


def test_2dMemoryArrayRead():
    """Cfunc reads 2-D memory field correctly."""
    mod = MatrixMemoryModel(); mod.ModelTag = "matmem"
    _run(mod, dt_ns=5, n_ticks=1)
    np.testing.assert_allclose(mod.dataOutMsg.read().dataVector, [1.0, 2.0, 3.0])


def test_2dMemoryArrayPythonWrite():
    """Python-side assignment to a 2-D memory field propagates to the C buffer."""
    mod = MatrixMemoryModel(); mod.ModelTag = "matwrite"
    _run(mod, dt_ns=5, n_ticks=1)

    mod.memory.diag = np.diag([4.0, 5.0, 6.0])
    assert mod.memory.diag[0, 0] == pytest.approx(4.0)
    assert mod.memory.diag[1, 1] == pytest.approx(5.0)
    assert mod.memory.diag[2, 2] == pytest.approx(6.0)


# ===========================================================================
# 12. moduleID parameter
# ===========================================================================
class ModuleIDModel(NumbaModel):
    """Helper model that captures the module ID seen by the compiled kernel."""

    def __init__(self):
        """Declare one writer and an integer memory slot for the module ID."""
        super().__init__()
        self.dataOutMsg = messaging.CModuleTemplateMsg()
        self.memory.captured_id = np.int64(0)

    @staticmethod
    def UpdateStateImpl(dataOutMsgPayload, memory, moduleID):
        """Store the provided module identifier in persistent memory."""
        memory.captured_id = moduleID


def test_moduleIdParameter():
    """Check the compiled model receives the owning module identifier."""
    mod = ModuleIDModel(); mod.ModelTag = "mid"
    _run(mod, dt_ns=5, n_ticks=1)
    # Python modules receive negative IDs from the sim framework
    assert mod.memory.captured_id != 0
    assert int(mod.memory.captured_id) == mod.moduleID


# ===========================================================================
# 13. Re-wiring: subscribeTo after Reset picks up new payload
# ===========================================================================
def test_rewireAfterReset():
    """Switching sources mid-sim is transparent - no Reset required."""
    src1 = messaging.CModuleTemplateMsg()
    src2 = messaging.CModuleTemplateMsg()

    mod = ConsumerModel(); mod.ModelTag = "rewire"
    scSim = SimulationBaseClass.SimBaseClass()
    proc  = scSim.CreateNewProcess("proc")
    proc.addTask(scSim.CreateNewTask("task", 5))
    scSim.AddModelToTask("task", mod)
    mod.dataInMsg.subscribeTo(src1)

    scSim.InitializeSimulation()

    p1 = messaging.CModuleTemplateMsgPayload(); p1.dataVector = [1.0, 0.0, 0.0]
    src1.write(p1)

    scSim.ConfigureStopTime(5)
    scSim.ExecuteSimulation()
    assert mod.dataOutMsg.read().dataVector[0] == pytest.approx(10.0)  # src1 × 10

    # Re-wire to src2 without calling Reset
    p2 = messaging.CModuleTemplateMsgPayload(); p2.dataVector = [3.0, 0.0, 0.0]
    src2.write(p2)
    mod.dataInMsg.subscribeTo(src2)

    scSim.ConfigureStopTime(10)
    scSim.ExecuteSimulation()
    assert mod.dataOutMsg.read().dataVector[0] == pytest.approx(30.0)  # src2 × 10


# ===========================================================================
# 14. Multiple resets - memory re-initialized to __init__ values
# ===========================================================================
def test_multipleResets():
    """Reset always continues from the current buffer value; cfunc writes and
    Python writes are treated identically."""
    mod = BasicModel(); mod.ModelTag = "multireset"
    _run(mod, dt_ns=5, n_ticks=3)
    assert mod.memory.step == 3           # cfunc incremented 3 times

    # Second sim: Reset syncs buffer → __dict__, so step resumes from 3
    scSim2 = SimulationBaseClass.SimBaseClass()
    proc   = scSim2.CreateNewProcess("proc")
    proc.addTask(scSim2.CreateNewTask("task", 5))
    scSim2.AddModelToTask("task", mod)
    scSim2.InitializeSimulation()          # Reset fires; memory picks up 3
    assert mod.memory.step == 3

    scSim2.ConfigureStopTime(10)           # 3 more ticks: t=0,5,10
    scSim2.ExecuteSimulation()
    assert mod.memory.step == 6           # 3 + 3


def test_memoryWritePersistsThroughReset():
    """Both Python-side writes and cfunc-internal writes survive a subsequent Reset."""
    mod = BasicModel(); mod.ModelTag = "memwrite"
    _run(mod, dt_ns=5, n_ticks=2)
    assert mod.memory.step == 2           # cfunc wrote 2

    # Python-side override
    mod.memory.step = np.int32(10)
    assert mod.memory.step == 10

    # Reset picks up 10 (last write wins, regardless of source)
    scSim2 = SimulationBaseClass.SimBaseClass()
    proc   = scSim2.CreateNewProcess("proc")
    proc.addTask(scSim2.CreateNewTask("task", 5))
    scSim2.AddModelToTask("task", mod)
    scSim2.InitializeSimulation()
    assert mod.memory.step == 10

    scSim2.ConfigureStopTime(5)           # 2 ticks
    scSim2.ExecuteSimulation()
    assert mod.memory.step == 12          # 10 + 2


# ===========================================================================
# 15. Error cases
# ===========================================================================
def test_errorNotOverridden():
    """Check Reset fails when ``UpdateStateImpl`` is not overridden."""
    class Bad(NumbaModel):
        """Model that intentionally omits ``UpdateStateImpl``."""
        pass
    with pytest.raises(NotImplementedError, match="must override UpdateStateImpl"):
        Bad().Reset()


def test_errorNotStaticmethod():
    """Check Reset fails when ``UpdateStateImpl`` is not static."""
    class Bad(NumbaModel):
        """Model with a non-static compiled entry point."""

        def UpdateStateImpl(self):
            """Stand-in invalid implementation used for error handling tests."""
            pass
    with pytest.raises(TypeError, match="@staticmethod"):
        Bad().Reset()


def test_errorMissingInMsg():
    """Check Reset fails when a referenced input message is missing."""
    class Bad(NumbaModel):
        """Model whose implementation references a missing input reader."""

        def __init__(self):
            """Leave out ``dataInMsg`` on purpose for the regression test."""
            super().__init__()
            # forgot to add self.dataInMsg

        @staticmethod
        def UpdateStateImpl(dataInMsgPayload):
            """Dummy implementation used only to trigger setup validation."""
            pass

    with pytest.raises(AttributeError, match="dataInMsg"):
        Bad().Reset()


def test_errorIslinkedWithoutPayload():
    """An IsLinked flag without a matching Payload param leaves the variable
    undefined in the generated wrapper, which Numba catches as a TypingError."""
    import numba
    class Bad(NumbaModel):
        """Model that declares ``IsLinked`` without the payload parameter."""

        def __init__(self):
            """Declare one reader and one writer for the malformed signature."""
            super().__init__()
            self.dataInMsg  = messaging.CModuleTemplateMsgReader()
            self.dataOutMsg = messaging.CModuleTemplateMsg()

        @staticmethod
        def UpdateStateImpl(dataInMsgIsLinked, dataOutMsgPayload):
            """Intentionally omit the payload parameter to provoke a typing error."""
            pass   # missing dataInMsgPayload

    with pytest.raises(numba.core.errors.TypingError, match="dataInMsgIsLinked"):
        Bad().Reset()


def test_errorUnrecognisedParam():
    """Check Reset fails when the compiled signature has an unknown parameter."""
    class Bad(NumbaModel):
        """Model with an unsupported parameter name in the compiled signature."""

        def __init__(self):
            """Provide the minimal base initialization for the error case."""
            super().__init__()

        @staticmethod
        def UpdateStateImpl(mystery):
            """Expose an unknown parameter name for validation testing."""
            pass

    with pytest.raises(AttributeError, match="does not match any known pattern"):
        Bad().Reset()


def test_emptyListReader():
    """Empty list attribute is allowed; cfunc receives a zero-element array (never iterated)."""
    class EmptyListInModel(NumbaModel):
        """Model that exercises an empty list of input readers."""

        def __init__(self):
            """Declare an empty reader list, one writer, and a counter."""
            super().__init__()
            self.dataInMsg  = []
            self.dataOutMsg = messaging.CModuleTemplateMsg()
            self.memory.count = np.int32(0)

        @staticmethod
        def UpdateStateImpl(dataInMsgPayload, dataOutMsgPayload, memory):
            """Increment a counter without ever indexing the empty input list."""
            memory.count += np.int32(1)
            dataOutMsgPayload.dataVector[0] = float(memory.count)

    mod = EmptyListInModel(); mod.ModelTag = "emptylist"
    _run(mod, dt_ns=5, n_ticks=3)
    assert mod.memory.count == 3
    assert mod.dataOutMsg.read().dataVector[0] == pytest.approx(3.0)


def test_emptyMemory():
    """No memory fields defined: 'memory' param still compiles as a valid (ignored) Record."""
    class EmptyMemoryModel(NumbaModel):
        """Model that requests the memory record without declaring fields."""

        def __init__(self):
            """Declare only the output message for the empty-memory case."""
            super().__init__()
            self.dataOutMsg = messaging.CModuleTemplateMsg()
            # no self.memory.* fields defined

        @staticmethod
        def UpdateStateImpl(dataOutMsgPayload, memory):
            """Ignore the empty memory record and publish a constant."""
            dataOutMsgPayload.dataVector[0] = 42.0

    mod = EmptyMemoryModel(); mod.ModelTag = "emptymem"
    _run(mod, dt_ns=5, n_ticks=1)
    assert mod.dataOutMsg.read().dataVector[0] == pytest.approx(42.0)


# ===========================================================================
# 16. bskLogger proxy - bskLog / bskLog1 inside UpdateStateImpl
# ===========================================================================
def test_dictIslinkedKeyed():
    """Dict reader IsLinked is accessible by key name: dataInMsgIsLinked['a']."""
    class DictIsLinkedModel(NumbaModel):
        """Dictionary-reader model used to verify keyed ``IsLinked`` access."""

        def __init__(self):
            """Declare two named readers and one output message."""
            super().__init__()
            self.dataInMsg = {
                'a': messaging.CModuleTemplateMsgReader(),
                'b': messaging.CModuleTemplateMsgReader(),
            }
            self.dataOutMsg = messaging.CModuleTemplateMsg()

        @staticmethod
        def UpdateStateImpl(dataInMsgPayload, dataInMsgIsLinked, dataOutMsgPayload):
            """Forward only the dictionary entries whose link flags are true."""
            if dataInMsgIsLinked['a']:
                dataOutMsgPayload.dataVector[0] = dataInMsgPayload['a'].dataVector[0]
            if dataInMsgIsLinked['b']:
                dataOutMsgPayload.dataVector[1] = dataInMsgPayload['b'].dataVector[0]

    src_a = messaging.CModuleTemplateMsg()
    src_b = messaging.CModuleTemplateMsg()
    mod = DictIsLinkedModel(); mod.ModelTag = "dictlnk"

    scSim = SimulationBaseClass.SimBaseClass()
    proc = scSim.CreateNewProcess("p"); proc.addTask(scSim.CreateNewTask("t", 5))
    scSim.AddModelToTask("t", mod)
    mod.dataInMsg['a'].subscribeTo(src_a)   # 'b' left unlinked

    scSim.InitializeSimulation()

    p_a = messaging.CModuleTemplateMsgPayload(); p_a.dataVector = [3.0, 0.0, 0.0]
    p_b = messaging.CModuleTemplateMsgPayload(); p_b.dataVector = [7.0, 0.0, 0.0]
    src_a.write(p_a); src_b.write(p_b)

    scSim.ConfigureStopTime(5); scSim.ExecuteSimulation()

    out = mod.dataOutMsg.read()
    assert out.dataVector[0] == pytest.approx(3.0)   # 'a' linked → forwarded
    assert out.dataVector[1] == pytest.approx(0.0)   # 'b' unlinked → untouched

    # Now link 'b' mid-sim and verify it becomes accessible
    mod.dataInMsg['b'].subscribeTo(src_b)
    scSim.ConfigureStopTime(10); scSim.ExecuteSimulation()

    out2 = mod.dataOutMsg.read()
    assert out2.dataVector[0] == pytest.approx(3.0)
    assert out2.dataVector[1] == pytest.approx(7.0)


def test_bsklogger():
    """bskLog / bskLog1 inside UpdateStateImpl should not crash; logic must run."""
    class LoggingModel(NumbaModel):
        """Model that exercises the ``bskLogger`` proxy from compiled code."""

        def __init__(self):
            """Declare one writer and a small integer step counter."""
            super().__init__()
            self.dataOutMsg = messaging.CModuleTemplateMsg()
            self.memory.step = np.int32(0)

        @staticmethod
        def UpdateStateImpl(dataOutMsgPayload, bskLogger, memory):
            """Log two messages and publish the incremented step count."""
            memory.step += np.int32(1)
            bskLogger.info("step update")
            bskLogger.bskLog1(bskLogging.BSK_WARNING, "step:", memory.step)
            dataOutMsgPayload.dataVector[0] = float(memory.step)

    mod = LoggingModel(); mod.ModelTag = "logsimple"
    _run(mod, dt_ns=5, n_ticks=3)
    assert mod.memory.step == 3
    assert mod.dataOutMsg.read().dataVector[0] == pytest.approx(3.0)


# ===========================================================================
# 18. Cache correctness — stale-cache stress tests
#
# These tests intentionally probe edge cases where the fingerprint-based cache
# could produce stale or colliding entries.
# ===========================================================================

# ---------------------------------------------------------------------------
# Shared model classes used across multiple tests
# ---------------------------------------------------------------------------

class _ConstWriterModel(NumbaModel):
    """Writes a configurable constant from memory to CModuleTemplateMsg."""
    def __init__(self, value: float):
        """Declare the output message and initialize the stored constant."""
        super().__init__()
        self.dataOutMsg   = messaging.CModuleTemplateMsg()
        self.memory.val = np.float64(value)

    @staticmethod
    def UpdateStateImpl(dataOutMsgPayload, memory):
        """Publish the constant stored in persistent memory."""
        dataOutMsgPayload.dataVector[0] = memory.val


class _DtypeVaryReaderModel(NumbaModel):
    """Same class but reader type varies per instance (injected via constructor)."""
    def __init__(self, readerFactory):
        """Instantiate the injected reader type and a fixed output message."""
        super().__init__()
        self.dataInMsg  = readerFactory()
        self.dataOutMsg = messaging.CModuleTemplateMsg()
        self.memory.val = np.float64(99.0)

    @staticmethod
    def UpdateStateImpl(dataInMsgPayload, dataInMsgIsLinked, dataOutMsgPayload, memory):
        """Ignore the payload contents and publish the sentinel value."""
        # Deliberately does NOT access dataInMsgPayload fields (they differ by dtype).
        # The reader is there solely to force different dtypeHash values.
        dataOutMsgPayload.dataVector[0] = memory.val


class _LoggerCacheModel(NumbaModel):
    """Model that emits one warning through the compiled logger proxy."""

    def __init__(self):
        """Declare a writer so the model has a visible execution side effect."""
        super().__init__()
        self.dataOutMsg = messaging.CModuleTemplateMsg()

    @staticmethod
    def UpdateStateImpl(dataOutMsgPayload, bskLogger):
        """Emit a warning and write a sentinel value."""
        bskLogger.warning("logger cache warning")
        dataOutMsgPayload.dataVector[0] = 1.0


def _runWithLoggerLevel(model, logLevel, dt_ns=5, n_ticks=1):
    """Run one model while setting the owning simulation logger level."""
    scSim = SimulationBaseClass.SimBaseClass()
    scSim.bskLogger.setLevel(logLevel)
    proc = scSim.CreateNewProcess("proc")
    proc.addTask(scSim.CreateNewTask("task", dt_ns))
    scSim.AddModelToTask("task", model)
    scSim.InitializeSimulation()
    scSim.ConfigureStopTime(dt_ns * (n_ticks - 1))
    scSim.ExecuteSimulation()
    return scSim


# ---------------------------------------------------------------------------
# 18a. Different dtype, same class → no fingerprint collision
# ---------------------------------------------------------------------------

def test_cacheNoDtypeCollision():
    """Same class, different reader dtypes → distinct fingerprints, no in-process collision.

    Regression test for the location-based idHash bug: md5(file:qualname) produced
    the same key for all instances of a class regardless of message-type configuration.
    """
    _NBMODEL_CFUNC_CACHE.clear()

    modA = _DtypeVaryReaderModel(messaging.CModuleTemplateMsgReader)
    modA.ModelTag = "dtypeA"
    _run(modA, dt_ns=5, n_ticks=1)

    modB = _DtypeVaryReaderModel(messaging.AttGuidMsgReader)
    modB.ModelTag = "dtypeB"
    _run(modB, dt_ns=5, n_ticks=1)

    assert len(_NBMODEL_CFUNC_CACHE) >= 2, (
        "Different reader dtypes must produce distinct fingerprints; "
        "old location-based idHash would have produced only 1 entry here"
    )
    assert modA._cfunc is not modB._cfunc, (
        "Instances with different dtypes must not share the same cfunc object"
    )
    # Both should still produce the correct output
    assert modA.dataOutMsg.read().dataVector[0] == pytest.approx(99.0)
    assert modB.dataOutMsg.read().dataVector[0] == pytest.approx(99.0)


# ---------------------------------------------------------------------------
# 18b. Same dtype, same class → single cache entry; cfunc is shared
# ---------------------------------------------------------------------------

def test_cacheSameDtypeSharesCfunc():
    """Two instances with identical dtype config share exactly one cfunc."""
    _NBMODEL_CFUNC_CACHE.clear()

    modA = _ConstWriterModel(1.0); modA.ModelTag = "sharedA"
    modB = _ConstWriterModel(2.0); modB.ModelTag = "sharedB"
    _run(modA, dt_ns=5, n_ticks=1)

    cache_before = len(_NBMODEL_CFUNC_CACHE)
    _run(modB, dt_ns=5, n_ticks=1)
    cache_after = len(_NBMODEL_CFUNC_CACHE)

    assert cache_after == cache_before, (
        "Second instance with same dtype must hit in-process cache, not add a new entry"
    )
    assert modA._cfunc is modB._cfunc, "Same dtype config must reuse the same cfunc object"


# ---------------------------------------------------------------------------
# 18c. Different logger levels, same class → runtime level respected, cfunc shared
# ---------------------------------------------------------------------------

def test_cacheLoggerLevelRuntime(monkeypatch, tmp_path, capfd):
    """Logger level is read at runtime; models with different levels share one cfunc."""
    _NBMODEL_CFUNC_CACHE.clear()
    monkeypatch.setattr(numbaModelModule, "getCacheDir", lambda: tmp_path)

    modA = _LoggerCacheModel(); modA.ModelTag = "logCacheA"
    _runWithLoggerLevel(modA, bskLogging.BSK_ERROR)
    suppressed = capfd.readouterr()
    assert "logger cache warning" not in suppressed.out

    modB = _LoggerCacheModel(); modB.ModelTag = "logCacheB"
    _runWithLoggerLevel(modB, bskLogging.BSK_WARNING)
    emitted = capfd.readouterr()
    assert "logger cache warning" in emitted.out
    assert modA._cfunc is modB._cfunc


# ---------------------------------------------------------------------------
# 18d. Shared cfunc, independent per-instance state (allPtrs_ is per-instance)
# ---------------------------------------------------------------------------

def test_cacheSharedCfuncIndependentState():
    """Two instances share one cfunc but each has its own ``allPtrs_`` buffer, so outputs never bleed."""
    modA = _ConstWriterModel(11.0); modA.ModelTag = "indepA"
    modB = _ConstWriterModel(22.0); modB.ModelTag = "indepB"

    _run(modA, dt_ns=5, n_ticks=3)
    _run(modB, dt_ns=5, n_ticks=3)

    assert modA.dataOutMsg.read().dataVector[0] == pytest.approx(11.0)
    assert modB.dataOutMsg.read().dataVector[0] == pytest.approx(22.0)


# ---------------------------------------------------------------------------
# 18d. In-process cache cleared → disk cache produces correct output
# ---------------------------------------------------------------------------

def test_cacheDiskReuseAfterInProcessClear():
    """Clear _NBMODEL_CFUNC_CACHE, re-run same config → disk path gives correct result."""
    _NBMODEL_CFUNC_CACHE.clear()

    modA = _ConstWriterModel(7.0); modA.ModelTag = "diskA"
    _run(modA, dt_ns=5, n_ticks=1)
    assert modA.dataOutMsg.read().dataVector[0] == pytest.approx(7.0)

    # Remember which fp was cached
    fps_before = set(_NBMODEL_CFUNC_CACHE.keys())

    _NBMODEL_CFUNC_CACHE.clear()

    # Second instance, same class+dtype → must use disk cache
    modB = _ConstWriterModel(13.0); modB.ModelTag = "diskB"
    _run(modB, dt_ns=5, n_ticks=1)
    assert modB.dataOutMsg.read().dataVector[0] == pytest.approx(13.0)

    # fp should be back in the in-process cache (disk hit re-populates it)
    fps_after = set(_NBMODEL_CFUNC_CACHE.keys())
    assert fps_before == fps_after, (
        "Disk cache hit must restore the fingerprint to the in-process cache"
    )


# ---------------------------------------------------------------------------
# 18e. Two classes with identical structure + dtypes but different UpdateStateImpl
#      → implIdentity differs → different fingerprints → independent cfuncs
# ---------------------------------------------------------------------------

class _ImplIsoClassA(NumbaModel):
    """Isolation test class whose implementation always writes ``100.0``."""

    def __init__(self):
        """Declare the output message for the implementation-isolation test."""
        super().__init__()
        self.dataOutMsg = messaging.CModuleTemplateMsg()

    @staticmethod
    def UpdateStateImpl(dataOutMsgPayload, memory):
        """Publish the class-A sentinel output value."""
        dataOutMsgPayload.dataVector[0] = 100.0


class _ImplIsoClassB(NumbaModel):
    """Isolation test class whose implementation always writes ``200.0``."""

    def __init__(self):
        """Declare the output message for the implementation-isolation test."""
        super().__init__()
        self.dataOutMsg = messaging.CModuleTemplateMsg()

    @staticmethod
    def UpdateStateImpl(dataOutMsgPayload, memory):
        """Publish the class-B sentinel output value."""
        dataOutMsgPayload.dataVector[0] = 200.0


def test_cacheImplIdentityIsolation():
    """Two classes with same dtype/structure but different UpdateStateImpl must not share cfunc."""
    _NBMODEL_CFUNC_CACHE.clear()

    modA = _ImplIsoClassA(); modA.ModelTag = "isoA"
    modB = _ImplIsoClassB(); modB.ModelTag = "isoB"
    _run(modA, dt_ns=5, n_ticks=1)
    _run(modB, dt_ns=5, n_ticks=1)

    assert modA.dataOutMsg.read().dataVector[0] == pytest.approx(100.0), (
        "Class A impl must return 100.0, not bleed into class B's cfunc"
    )
    assert modB.dataOutMsg.read().dataVector[0] == pytest.approx(200.0), (
        "Class B impl must return 200.0, not be overshadowed by class A's cfunc"
    )
    assert modA._cfunc is not modB._cfunc, (
        "Different classes must have different cfuncs even with identical dtype + structure"
    )


# ---------------------------------------------------------------------------
# 18f. Multiple resets of same model → in-process cache hit on 2nd+ reset
# ---------------------------------------------------------------------------

def test_cacheMultipleResetsReusesCfunc():
    """Re-running InitializeSimulation reuses the in-process cfunc (Tier 2 benefit)."""
    mod = _ConstWriterModel(5.0); mod.ModelTag = "multiReset"

    _NBMODEL_CFUNC_CACHE.clear()
    _run(mod, dt_ns=5, n_ticks=1)
    cfunc_first = mod._cfunc
    cache_size_after_first = len(_NBMODEL_CFUNC_CACHE)

    # Second reset (another InitializeSimulation inside _run)
    _run(mod, dt_ns=5, n_ticks=1)
    cfunc_second = mod._cfunc

    assert cfunc_first is cfunc_second, "Second reset must return the same cfunc object"
    assert len(_NBMODEL_CFUNC_CACHE) == cache_size_after_first, (
        "No new entry should be added on second reset"
    )
    assert mod.dataOutMsg.read().dataVector[0] == pytest.approx(5.0)

# ---------------------------------------------------------------------------
# 18g. Subprocess: disk cache survives process restart, correct output on reload
# ---------------------------------------------------------------------------

_SUBPROCESS_SCRIPT = textwrap.dedent("""\
    import sys, os
    import numpy as np
    from Basilisk.architecture.numbaModel import NumbaModel
    from Basilisk.architecture import messaging
    from Basilisk.utilities import SimulationBaseClass

    class SubprocModel(NumbaModel):
        def __init__(self, val):
            super().__init__()
            self.dataOutMsg = messaging.CModuleTemplateMsg()
            self.memory.val = np.float64(val)

        @staticmethod
        def UpdateStateImpl(dataOutMsgPayload, memory):
            dataOutMsgPayload.dataVector[0] = memory.val

    val = float(sys.argv[1])
    mod = SubprocModel(val); mod.ModelTag = "sub"
    scSim = SimulationBaseClass.SimBaseClass()
    proc  = scSim.CreateNewProcess("proc")
    proc.addTask(scSim.CreateNewTask("task", 5))
    scSim.AddModelToTask("task", mod)
    scSim.InitializeSimulation()
    scSim.ConfigureStopTime(0)
    scSim.ExecuteSimulation()
    print(mod.dataOutMsg.read().dataVector[0])
""")


def _parse_subprocess_float(stdout):
    """Return the numeric result line from subprocess stdout."""
    for line in stdout.splitlines():
        try:
            return float(line.strip())
        except ValueError:
            continue
    raise AssertionError(f"No numeric result found in subprocess stdout:\n{stdout}")


def test_cacheDiskSurvivesProcessRestart(tmp_path):
    """Disk cache written in process 1 is used correctly by process 2."""
    script = tmp_path / "sub_model.py"
    script.write_text(_SUBPROCESS_SCRIPT)

    # Process 1: cold compile, writes disk cache
    r1 = subprocess.run(
        [sys.executable, str(script), "42.0"],
        capture_output=True, text=True, timeout=120
    )
    assert r1.returncode == 0, f"Process 1 failed:\n{r1.stderr}"
    assert _parse_subprocess_float(r1.stdout) == pytest.approx(42.0)

    # Process 2: should load from disk cache (fast path)
    r2 = subprocess.run(
        [sys.executable, str(script), "77.0"],
        capture_output=True, text=True, timeout=120
    )
    assert r2.returncode == 0, f"Process 2 failed:\n{r2.stderr}"
    assert _parse_subprocess_float(r2.stdout) == pytest.approx(77.0), (
        "Disk-cached cfunc from process 1 must not contaminate process 2's output"
    )


# ---------------------------------------------------------------------------
# 18i. Subprocess: edit UpdateStateImpl body on disk → new process picks it up
#
# This is the canonical production staleness scenario:
#   Process 1 → compiles with impl v1 (11.0), writes disk cache
#   File edited → impl v2 (22.0), mtime changes
#   Process 2 → implCodeHash changes → fp changes → new _wrapper file compiled;
#               Numba detects mtime change → recompiles _impl → output 22.0
# ---------------------------------------------------------------------------

_FILE_EDIT_TEMPLATE = textwrap.dedent("""\
    import sys, numpy as np
    from Basilisk.architecture.numbaModel import NumbaModel
    from Basilisk.architecture import messaging
    from Basilisk.utilities import SimulationBaseClass

    class FileEditModel(NumbaModel):
        def __init__(self):
            super().__init__()
            self.dataOutMsg = messaging.CModuleTemplateMsg()

        @staticmethod
        def UpdateStateImpl(dataOutMsgPayload, memory):
            dataOutMsgPayload.dataVector[0] = {value}

    mod = FileEditModel(); mod.ModelTag = "fmod"
    scSim = SimulationBaseClass.SimBaseClass()
    proc  = scSim.CreateNewProcess("p")
    proc.addTask(scSim.CreateNewTask("t", 5))
    scSim.AddModelToTask("t", mod)
    scSim.InitializeSimulation()
    scSim.ConfigureStopTime(0)
    scSim.ExecuteSimulation()
    print(mod.dataOutMsg.read().dataVector[0])
""")


def test_cacheDiskImplBodyChange(tmp_path):
    """Edit ``UpdateStateImpl`` on disk between processes and require the new implementation to run.

    This verifies the full production invalidation chain. The implementation
    body hash changes, the fingerprint changes, a new wrapper file is emitted,
    and Numba recompiles the implementation after its source timestamp changes.
    A stale-cache bug would show up as process 2 still returning ``11.0``.
    """
    script = tmp_path / "file_edit_model.py"

    # Process 1: impl outputs 11.0 → compile + cache
    script.write_text(_FILE_EDIT_TEMPLATE.format(value="11.0"))
    r1 = subprocess.run(
        [sys.executable, str(script)],
        capture_output=True, text=True, timeout=120
    )
    assert r1.returncode == 0, f"Process 1 failed:\n{r1.stderr}"
    assert _parse_subprocess_float(r1.stdout) == pytest.approx(11.0)

    # Edit the file: impl now outputs 22.0 (mtime advances, co_consts changes)
    script.write_text(_FILE_EDIT_TEMPLATE.format(value="22.0"))

    # Process 2: must detect the change and output 22.0, not the cached 11.0
    r2 = subprocess.run(
        [sys.executable, str(script)],
        capture_output=True, text=True, timeout=120
    )
    assert r2.returncode == 0, f"Process 2 failed:\n{r2.stderr}"
    result = _parse_subprocess_float(r2.stdout)
    assert result == pytest.approx(22.0), (
        f"Edited UpdateStateImpl (22.0) must be used; got {r2.stdout.strip()} — "
        "stale disk cache not invalidated after impl body change"
    )
