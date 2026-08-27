# MonteCarlo: Brief Guide

For complete examples, see `examples/scenarioMonteCarloAttRW.py` and its test in
`src/tests/test_scenarioMonteCarloAttRW.py`. This guide provides a brief overview of
the main Monte Carlo features.

The Monte Carlo utilities execute a simulation repeatedly while varying random seeds and
selected initial parameters. `Controller` manages the runs, archives retained data, reloads
completed runs, and reproduces selected cases from their saved parameters.

To create a Monte Carlo simulation, import `Controller`, `RetentionPolicy`, and any
dispersion classes needed to vary the initial parameters. Then create and configure a
`Controller` for the run.

## Configure the simulation

```python
import matplotlib.pyplot as plt
import numpy as np

from Basilisk.utilities import (
    SimulationBaseClass,
    macros,
    pythonVariableLogger,
    simHelpers,
)
from Basilisk.utilities.MonteCarlo.Controller import Controller
from Basilisk.utilities.MonteCarlo.Dispersions import UniformEulerAngleMRPDispersion
from Basilisk.utilities.MonteCarlo.RetentionPolicy import RetentionPolicy

monteCarlo = Controller()
```

Every Monte Carlo simulation must define and return the `SimBaseClass` instance to run.

```python
def myCreationFunction():
    sim = SimulationBaseClass.SimBaseClass()
    # configure sim ...
    return sim

monteCarlo.setSimulationFunction(myCreationFunction)
```

Every Monte Carlo simulation must also define a function that executes the created
simulation:

```python
def myExecutionFunction(sim):
    sim.InitializeSimulation()
    sim.ExecuteSimulation()

monteCarlo.setExecutionFunction(myExecutionFunction)
```

Optionally, a configuration function can adjust the simulation after random seeds have
been populated. Non-seed parameter dispersions are applied after this configuration step
and before the execution function runs.

```python
def myConfigureFunction(sim):
    # Configure the sim now that random seeds have been applied ...
    pass

monteCarlo.setConfigureFunction(myConfigureFunction)
```

Statistical dispersions can be applied to initial parameters. These parameters are saved
for reference and to make individual cases reproducible. See `MonteCarlo/Dispersions.py`
for the available dispersion classes.

```python
monteCarlo.addDispersion(
    UniformEulerAngleMRPDispersion(
        "TaskList[0].TaskModels[0].hub.sigma_BNInit"
    )
)
```

Dispersion names can reference nested simulation attributes with dotted names, integer list
indices, and zero-argument accessor methods. For example,
`TaskList[0].TaskModels[0].hub.sigma_BNInit` updates the `sigma_BNInit` attribute
on the first model's hub object, while `get_DynModel().scObject.hub.r_CN_NInit`
resolves the object returned by `get_DynModel()` before applying the dispersion. Repeated
indices can be used for matrix-like attributes, such as
`scObject.hub.IHubPntBc_B[0][1]`.

## Retain simulation data

If data is being retained, an archive directory must be specified. The same directory is
used to reload retained data after the Monte Carlo execution.

```python
monteCarlo.setArchiveDir("dirName")
```

Data is retained from a simulation to a unique file for each run. A `RetentionPolicy`
defines which recorded message fields, module variables, and custom values are saved.
It can also provide a callback for plotting or processing the retained data.

Message recorders must be created by the simulation creation function and stored in
`sim.msgRecList`; their sampling interval is set when the recorder is created. A module
variable is identified in `<ModelTag>.<variableName>` format and must be public or
available through its standard getter. The scheduled module must support Basilisk's
standard `logger()` API, and its model tag must be unique. A model tag may contain periods
because the final period in the identifier separates it from the variable name. `logRate`
sets the variable logger's nonnegative integer minimum recording period in nanoseconds.

The complete scalar, vector, or array is recorded by default. The first column of every
retained message or variable array is simulation time in nanoseconds. Multidimensional
variable samples are flattened in row-major order into the remaining columns.

The legacy `startIndex`, `stopIndex`, and `varType` arguments remain available for
migration until 2027-08-26. Component indices are inclusive and refer to the flattened
variable. Output column zero contains time, and the selected components are stored
consecutively beginning in column one. `varType` is ignored because modern loggers determine
the value type directly. New code should retain the complete variable and select component
columns from the resulting NumPy array. To record nested or computed values over time, create a
`pythonVariableLogger.PythonVariableLogger` during simulation setup, schedule it on the source
model's task after that model, and store it on the simulation object. Use
`addRetentionFunction()` to copy its time and value arrays into the retained `custom` data, and
use `simHelpers.addTimeColumn(logger.times(), logger["valueName"])` when the custom array should
follow the same time-column convention as message and direct-variable data.

```python
# Add a retention policy that logs message fields and a module variable.
# Assume myCreationFunction stores the recorder as sim.msgRecList["spacecraftState"]
# and adds a module whose ModelTag is "bsk-Sat".
retainedRate = macros.sec2nano(10.0)  # [ns]
plotRetentionPolicy = RetentionPolicy()
plotRetentionPolicy.addMessageLog(
    "spacecraftState",
    ["v_BN_N", "r_BN_N"],
)
plotRetentionPolicy.addVariableLog(
    "bsk-Sat.totOrbEnergy",
    logRate=retainedRate,
)

# This callback plots one retained run.
def myDataCallback(data, retentionPolicy):
    r_BN_N = np.asarray(data["messages"]["spacecraftState.r_BN_N"])
    orbitalEnergy = np.asarray(data["variables"]["bsk-Sat.totOrbEnergy"])
    plt.figure()
    plt.plot(r_BN_N[:, 1], r_BN_N[:, 2])
    plt.figure()
    plt.plot(orbitalEnergy[:, 0], orbitalEnergy[:, 1])


plotRetentionPolicy.setDataCallback(myDataCallback)

monteCarlo.addRetentionPolicy(plotRetentionPolicy)
```

## Execute and analyze the runs

Random-seed dispersion is recommended whenever a simulation uses random number generation.
Seeds are archived so an individual run can be reproduced. Configure the number of runs and,
optionally, the worker-process count and console verbosity before execution:

```python
monteCarlo.setShouldDisperseSeeds(True)
monteCarlo.setExecutionCount(100)
monteCarlo.setThreadCount(4)
monteCarlo.setVerbose(False)

failures = monteCarlo.executeSimulations()
```

`executeSimulations()` returns the indices of failed runs. Once execution is complete, the
controller and retained data can be reloaded in the same script or a later analysis script:

```python
monteCarlo = Controller.load("dirName")

# Execute callbacks for every run, or select specific runs and policies.
monteCarlo.executeCallbacks()
# This second form assumes plotRetentionPolicy is available in the analysis script.
monteCarlo.executeCallbacks([4, 6, 7], [plotRetentionPolicy])

plt.show()
```

Retained data from a run is a dictionary containing message, variable, and custom
data, together with the Monte Carlo run index:

```python
{
    "messages": {
        "recorderName.fieldName": [[time, value1, value2]]
    },
    "variables": {
        "modelTag.variableName": [[time, value1, value2]]
    },
    "custom": {
        "customName": [value1, value2]
    },
    "index": 19,
}
```

```python
retainedData = monteCarlo.getRetainedData(19)
retainedData["messages"]["spacecraftState.r_BN_N"]
orbitalEnergy = retainedData["variables"]["bsk-Sat.totOrbEnergy"]

# Column 0 is time; subsequent columns contain the retained variable components.
time = orbitalEnergy[:, 0]
energy = orbitalEnergy[:, 1]
```

Additional methods for loading retained parameters and rerunning cases are documented in
`Controller` and the referenced Monte Carlo example.
