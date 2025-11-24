from Basilisk.utilities import SimulationBaseClass, macros
from Basilisk.ExternalModules import _custom_cpp
import Basilisk.architecture.sysModel as sysModel


# Wrap plugin instance as a real Basilisk SysModel since im using PyBind11
# currently
class PluginAsSysModel(sysModel.SysModel):
    def __init__(self, plugin_impl):
        super().__init__()
        self.impl = plugin_impl
        self.ModelTag = "CustomCppModuleDemo"

    def Reset(self, t):
        self.impl.Reset(t)

    def UpdateState(self, t):
        self.impl.UpdateState(t)


sim = SimulationBaseClass.SimBaseClass()
proc = sim.CreateNewProcess("proc")
task_name = "task"
proc.addTask(sim.CreateNewTask(task_name, macros.sec2nano(1.0)))

plugin = _custom_cpp.create_factory()()
wrapper = PluginAsSysModel(plugin)

sim.AddModelToTask(task_name, wrapper)
plugin.set_input_payload(_custom_cpp.CustomPluginMsgPayload([4.0, 5.0, 6.0]))

sim.InitializeSimulation()
sim.ConfigureStopTime(macros.sec2nano(1.0))
sim.ExecuteSimulation()

print("reset_called:", plugin.reset_called)
print("update_called:", plugin.update_called)
print("Last input :", list(plugin.last_input.dataVector))
print("Last output:", list(plugin.last_output.dataVector))
