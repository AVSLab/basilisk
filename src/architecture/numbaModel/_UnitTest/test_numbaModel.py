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
import numpy as np
import pytest

from Basilisk.architecture.numbaModel import NumbaModel
from Basilisk.architecture import messaging, bskLogging
from Basilisk.utilities import SimulationBaseClass


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
    def __init__(self):
        super().__init__()
        self.dataInMsg  = messaging.CModuleTemplateMsgReader()
        self.dataOutMsg = messaging.CModuleTemplateMsg()
        self.memory.step = np.int32(0)

    @staticmethod
    def UpdateStateImpl(dataInMsgPayload, dataInMsgIsLinked,
                        dataOutMsgPayload, CurrentSimNanos, memory):
        vec = np.zeros(3)
        if dataInMsgIsLinked:
            vec[:] = dataInMsgPayload.dataVector
        memory.step += np.int32(1)
        out = vec.copy()
        out[0] += memory.step
        dataOutMsgPayload.dataVector[:] = out


def test_basic_model():
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
    def __init__(self):
        super().__init__()
        self.dataOutMsg = messaging.CModuleTemplateMsg()
        self.memory.kp   = 2.0              # plain float → scalar field
        self.memory.bias = [0.1, 0.2, 0.3]  # list        → 1-D array field

    @staticmethod
    def UpdateStateImpl(dataOutMsgPayload, memory):
        for i in range(3):
            dataOutMsgPayload.dataVector[i] = memory.kp + memory.bias[i]


def test_auto_convert_memory():
    mod = AutoConvertModel()
    _run(mod, dt_ns=5, n_ticks=1)

    assert float(mod.memory.kp)  == pytest.approx(2.0)
    assert mod.memory.bias.shape == (3,)

    out = mod.dataOutMsg.read()
    np.testing.assert_allclose(out.dataVector, [2.1, 2.2, 2.3])


# ===========================================================================
# 3. Post-Reset convenience writes via memory namespace
# ===========================================================================
def test_post_reset_convenience_write():
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
        super().__init__()
        self.dataOutMsg    = messaging.CModuleTemplateMsg()
        self.memory.last_ns    = np.uint64(0)
        self.memory.tick_count = np.int32(0)

    @staticmethod
    def UpdateStateImpl(dataOutMsgPayload, CurrentSimNanos, memory):
        memory.last_ns     = CurrentSimNanos
        memory.tick_count += np.int32(1)
        dataOutMsgPayload.dataVector[0] = float(CurrentSimNanos)


def test_current_sim_nanos():
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
        super().__init__()
        self.dataInMsg  = messaging.CModuleTemplateMsgReader()
        self.dataOutMsg = messaging.CModuleTemplateMsg()
        self.memory.ever_linked = np.int32(0)

    @staticmethod
    def UpdateStateImpl(dataInMsgPayload, dataInMsgIsLinked,
                        dataOutMsgPayload, memory):
        if dataInMsgIsLinked:
            memory.ever_linked = np.int32(1)
            dataOutMsgPayload.dataVector[:] = dataInMsgPayload.dataVector
        else:
            dataOutMsgPayload.dataVector[0] = -1.0


def test_islinked_unlinked():
    """Unlinked reader → IsLinked is False; cfunc takes the else branch."""
    mod = GuardedReaderModel(); mod.ModelTag = "unlinked"
    _run(mod, dt_ns=5, n_ticks=2)
    assert mod.memory.ever_linked == 0
    assert mod.dataOutMsg.read().dataVector[0] == pytest.approx(-1.0)


def test_islinked_linked():
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


def test_islinked_becomes_linked_mid_sim():
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
    def __init__(self):
        super().__init__()
        self.dataOutMsg = messaging.CModuleTemplateMsg()
        self.memory.count = 0.0

    @staticmethod
    def UpdateStateImpl(dataOutMsgPayload, memory):
        memory.count += 1.0
        dataOutMsgPayload.dataVector[0] = memory.count
        dataOutMsgPayload.dataVector[1] = memory.count * 2.0
        dataOutMsgPayload.dataVector[2] = memory.count * 3.0


class ConsumerModel(NumbaModel):
    def __init__(self):
        super().__init__()
        self.dataInMsg  = messaging.CModuleTemplateMsgReader()
        self.dataOutMsg = messaging.CModuleTemplateMsg()

    @staticmethod
    def UpdateStateImpl(dataInMsgPayload, dataInMsgIsLinked, dataOutMsgPayload):
        if dataInMsgIsLinked:
            for i in range(3):
                dataOutMsgPayload.dataVector[i] = dataInMsgPayload.dataVector[i] * 10.0


def test_message_chain():
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
        super().__init__()
        self.dataInMsg  = [messaging.CModuleTemplateMsgReader(),
                           messaging.CModuleTemplateMsgReader()]
        self.dataOutMsg = messaging.CModuleTemplateMsg()

    @staticmethod
    def UpdateStateImpl(dataInMsgPayload, dataInMsgIsLinked, dataOutMsgPayload):
        total = 0.0
        for i in range(2):
            if dataInMsgIsLinked[i]:
                total += dataInMsgPayload[i].dataVector[0]
        dataOutMsgPayload.dataVector[0] = total


def test_list_reader_both_linked():
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


def test_list_reader_partial_link():
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


def test_list_reader_none_linked():
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
        super().__init__()
        self.dataInMsg  = messaging.CModuleTemplateMsgReader()
        self.dataOutMsg = [messaging.CModuleTemplateMsg(),
                           messaging.CModuleTemplateMsg()]

    @staticmethod
    def UpdateStateImpl(dataInMsgPayload, dataInMsgIsLinked, dataOutMsgPayload):
        v = 0.0
        if dataInMsgIsLinked:
            v = dataInMsgPayload.dataVector[0]
        dataOutMsgPayload[0].dataVector[0] = v
        dataOutMsgPayload[1].dataVector[0] = v * 2.0


def test_list_writer():
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
    def __init__(self):
        super().__init__()
        self.dataInMsg = {
            'a': messaging.CModuleTemplateMsgReader(),
            'b': messaging.CModuleTemplateMsgReader(),
        }
        self.dataOutMsg = messaging.CModuleTemplateMsg()

    @staticmethod
    def UpdateStateImpl(dataInMsgPayload, dataOutMsgPayload):
        dataOutMsgPayload.dataVector[0] = dataInMsgPayload['a'].dataVector[0]
        dataOutMsgPayload.dataVector[1] = dataInMsgPayload['b'].dataVector[0]


def test_dict_reader_keys():
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


def test_dict_reader_data():
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


def test_error_unlinked_without_islinked_guard():
    """Declaring *InMsgPayload without *InMsgIsLinked crashes at Reset if reader is unlinked."""
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
        super().__init__()
        self.dataInMsg  = messaging.CModuleTemplateMsgReader()
        self.dataOutMsg = {
            'pos': messaging.CModuleTemplateMsg(),
            'neg': messaging.CModuleTemplateMsg(),
        }

    @staticmethod
    def UpdateStateImpl(dataInMsgPayload, dataInMsgIsLinked, dataOutMsgPayload):
        v = 0.0
        if dataInMsgIsLinked:
            v = dataInMsgPayload.dataVector[0]
        dataOutMsgPayload['pos'].dataVector[0] =  v
        dataOutMsgPayload['neg'].dataVector[0] = -v


def test_dict_writer():
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
        super().__init__()
        self.dataOutMsg  = messaging.CModuleTemplateMsg()
        self.memory.diag = np.diag([1.0, 2.0, 3.0])   # shape (3, 3)

    @staticmethod
    def UpdateStateImpl(dataOutMsgPayload, memory):
        for i in range(3):
            dataOutMsgPayload.dataVector[i] = memory.diag[i, i]


def test_2d_memory_array_read():
    """Cfunc reads 2-D memory field correctly."""
    mod = MatrixMemoryModel(); mod.ModelTag = "matmem"
    _run(mod, dt_ns=5, n_ticks=1)
    np.testing.assert_allclose(mod.dataOutMsg.read().dataVector, [1.0, 2.0, 3.0])


def test_2d_memory_array_python_write():
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
    def __init__(self):
        super().__init__()
        self.dataOutMsg = messaging.CModuleTemplateMsg()
        self.memory.captured_id = np.int64(0)

    @staticmethod
    def UpdateStateImpl(dataOutMsgPayload, memory, moduleID):
        memory.captured_id = moduleID


def test_module_id_parameter():
    mod = ModuleIDModel(); mod.ModelTag = "mid"
    _run(mod, dt_ns=5, n_ticks=1)
    # Python modules receive negative IDs from the sim framework
    assert mod.memory.captured_id != 0
    assert int(mod.memory.captured_id) == mod.moduleID


# ===========================================================================
# 13. Re-wiring: subscribeTo after Reset picks up new payload
# ===========================================================================
def test_rewire_after_reset():
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
def test_multiple_resets():
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


def test_memory_write_persists_through_reset():
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
def test_error_not_overridden():
    class Bad(NumbaModel):
        pass
    with pytest.raises(NotImplementedError, match="must override UpdateStateImpl"):
        Bad().Reset()


def test_error_not_staticmethod():
    class Bad(NumbaModel):
        def UpdateStateImpl(self):
            pass
    with pytest.raises(TypeError, match="@staticmethod"):
        Bad().Reset()


def test_error_missing_in_msg():
    class Bad(NumbaModel):
        def __init__(self):
            super().__init__()
            # forgot to add self.dataInMsg

        @staticmethod
        def UpdateStateImpl(dataInMsgPayload):
            pass

    with pytest.raises(AttributeError, match="dataInMsg"):
        Bad().Reset()


def test_error_islinked_without_payload():
    """An IsLinked flag without a matching Payload param leaves the variable
    undefined in the generated wrapper, which Numba catches as a TypingError."""
    import numba
    class Bad(NumbaModel):
        def __init__(self):
            super().__init__()
            self.dataInMsg  = messaging.CModuleTemplateMsgReader()
            self.dataOutMsg = messaging.CModuleTemplateMsg()

        @staticmethod
        def UpdateStateImpl(dataInMsgIsLinked, dataOutMsgPayload):
            pass   # missing dataInMsgPayload

    with pytest.raises(numba.core.errors.TypingError, match="dataInMsgIsLinked"):
        Bad().Reset()


def test_error_unrecognised_param():
    class Bad(NumbaModel):
        def __init__(self):
            super().__init__()

        @staticmethod
        def UpdateStateImpl(mystery):
            pass

    with pytest.raises(AttributeError, match="does not match any known pattern"):
        Bad().Reset()


def test_empty_list_reader():
    """Empty list attribute is allowed; cfunc receives a zero-element array (never iterated)."""
    class EmptyListInModel(NumbaModel):
        def __init__(self):
            super().__init__()
            self.dataInMsg  = []
            self.dataOutMsg = messaging.CModuleTemplateMsg()
            self.memory.count = np.int32(0)

        @staticmethod
        def UpdateStateImpl(dataInMsgPayload, dataOutMsgPayload, memory):
            memory.count += np.int32(1)
            dataOutMsgPayload.dataVector[0] = float(memory.count)

    mod = EmptyListInModel(); mod.ModelTag = "emptylist"
    _run(mod, dt_ns=5, n_ticks=3)
    assert mod.memory.count == 3
    assert mod.dataOutMsg.read().dataVector[0] == pytest.approx(3.0)


def test_empty_memory():
    """No memory fields defined: 'memory' param still compiles as a valid (ignored) Record."""
    class EmptyMemoryModel(NumbaModel):
        def __init__(self):
            super().__init__()
            self.dataOutMsg = messaging.CModuleTemplateMsg()
            # no self.memory.* fields defined

        @staticmethod
        def UpdateStateImpl(dataOutMsgPayload, memory):
            dataOutMsgPayload.dataVector[0] = 42.0

    mod = EmptyMemoryModel(); mod.ModelTag = "emptymem"
    _run(mod, dt_ns=5, n_ticks=1)
    assert mod.dataOutMsg.read().dataVector[0] == pytest.approx(42.0)


# ===========================================================================
# 16. bskLogger proxy - bskLog / bskLog1 inside UpdateStateImpl
# ===========================================================================
def test_dict_islinked_keyed():
    """Dict reader IsLinked is accessible by key name: dataInMsgIsLinked['a']."""
    class DictIsLinkedModel(NumbaModel):
        def __init__(self):
            super().__init__()
            self.dataInMsg = {
                'a': messaging.CModuleTemplateMsgReader(),
                'b': messaging.CModuleTemplateMsgReader(),
            }
            self.dataOutMsg = messaging.CModuleTemplateMsg()

        @staticmethod
        def UpdateStateImpl(dataInMsgPayload, dataInMsgIsLinked, dataOutMsgPayload):
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
        def __init__(self):
            super().__init__()
            self.dataOutMsg = messaging.CModuleTemplateMsg()
            self.memory.step = np.int32(0)

        @staticmethod
        def UpdateStateImpl(dataOutMsgPayload, bskLogger, memory):
            memory.step += np.int32(1)
            bskLogger.bskLog(bskLogging.BSK_INFORMATION, "step update")
            bskLogger.bskLog1(bskLogging.BSK_WARNING, "step:", memory.step)
            dataOutMsgPayload.dataVector[0] = float(memory.step)

    mod = LoggingModel(); mod.ModelTag = "logsimple"
    _run(mod, dt_ns=5, n_ticks=3)
    assert mod.memory.step == 3
    assert mod.dataOutMsg.read().dataVector[0] == pytest.approx(3.0)
