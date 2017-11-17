# MonteCarlo: Brief Guide

*If you plan to use this it is highly recommended to read the documentation in the `MonteCarlo/Controller.py` source file and the example usage in `src/tests/scenarios/test_MonteCarloSimulation.py`. This guide offers only a brief overview of features*

A MonteCarlo simulation can be created using the `MonteCarlo` module. This module is used to execute monte carlo simulations, and access retained data from previously executed MonteCarlo runs.

First, the `Controller` class is used in order to execute a simulation repeatedly, applying unique random seeds to each run, statistically dispersing initial parameters, executing the simulation run, and compressing and retaining data about the run.

Data retained through a MonteCarlo simulation is compressed and saved to disk. The `Controller` class is also used to access this data from each run.  The MonteCarlo `Controller` can reload the directory where data was retained in, and access the retained data, or rerun unusual cases using the same random seeds and initial parameters.

To create a Monte Carlo simulation, import the `Controller`, `RetentionPolicy`, and other objects described later from the `MonteCarlo` module, along with `Dispersion` classes used to disperse initial parameters. Then create a `Controller` and configure that object for the particular monte carlo run.

```
from MonteCarlo.Controller import Controller, RetentionPolicy
monteCarlo = Controller()
```

Every MonteCarlo simulation must define a function that creates the `SimulationBaseClass` to execute and returns it. Within this function, the simulation is created and configured

```
def myCreationFunction():
   sim = SimulationBaseClass()
   # modify sim ...
   return sim

monteCarlo.setSimulationFunction(myCreationFunction)
```


Also, every MonteCarlo simulation must define a function which executes the simulation that was created. It could look like this:

```
def myExecutionFunction(sim):
    sim.InitializeSimulationAndDiscover()
    sim.ExecuteSimulation()

monteCarlo.setExecutionFunction(myExecutionFunction)
```

Optionally, there is a function that can be used to configure the simulation after all dispersions of random seeds and variables have been applied. This may be unused, it is only necessary to access the simulation at this time in some cases.

```
def myConfigureFunction(sim):
  # do something with the sim now that random seeds have been applied
  # and variables are dispersed ...

monteCarlo.setConfigureFunction(myConfigureFunction)
```

Statistical dispersions can be applied to initial parameters using the MonteCarlo module. These initial parameters are saved for reference and in order to re-run cases. Various dispersions have been created, and these are not specified here. To see available dispersions, examine `MonteCarlo/Dispersions.py`

```
monteCarlo.addDispersion(UniformEulerAngleMRPDispersion("taskName.hub.sigma_BNInit"))
```

If data is being retained, a archive directory to store retained data must be specified. This directory is later used to reload the retained data from an executed Monte Carlo simulation.

```
monteCarlo.setArchiveDir("dirName")
```

Data is retained from a simulation to a unique file for each run. A `RetentionPolicy` is used to define what data from the simulation should be retained. A `RetentionPolicy` is a list of messages and variables to log from each simulation run. It also has a callback, used for plotting/processing the retained data. If a user wanted to create a plot of each run of a simulation message, they would create a retention policy defining the message they want to plot, and a callback that uses that message to draw a plot. This plot can be created any time after the initial execution of the monte carlo run, from the retained data.

```
# add retention policy that logs a message and plots it
plotRetentionPolicy = RetentionPolicy()
# log this message
plotRetentionPolicy.addMessageLog("inertial_state_output", [("v_BN_N", range(3)), ("r_BN_N", range(3)], retainedRate)

# this is the plot command (for only one run)
def myDataCallback(data, retentionPolicy):
    v_BN_N = np.array(monteCarloData["messages"]["inertial_state_output.v_BN_N"])
    plt.plot(v_BN_N[:,1], v_BN_N[:,2])
plotRetentionPolicy.setDataCallback(myDataCallback)

monteCarlo.addRetentionPolicy(plotRetentionPolicy)

# now execute the simulations, and the message will be retained
monteCarlo.executeSimulations()

# After the simulation has been executed
# this can be in a different script than the executeSimulations
# or in the same script this line can be skipped
monteCarlo = Controller.load("dirName")

# now we can plot all plots for all runs on the same plot
monteCarlo.executeCallbacks()
# or plot only this one plot with only the data from runs 4, 6, and 27
monteCarlo.executeCallbacks([4,6,7], [plotRetentionPolicy])

plt.show()
```

The simulations can have random seeds of each simulation dispersed randomly. This is recommended to be used when a simulation relies on random number generation. Whether to disperse random seeds on all simulation tasks is controlled via the method `monteCarlo.setShouldDisperseSeeds(True)`. If random seeds are used, the random seeds are saved in case a user wants to rerun a particular run. This is all stored with the initial parameters in an individual json file for each run in the archive directory.

A Monte Carlo simulation must define how many simulation runs to execute for the Monte Carlo using `monteCarlo.setExecutionCount(NUMBER_OF_RUNS)`

Optionally, the number of processes to use for the simulation. If this isn't called use the number of cores on the computer `monteCarlo.setThreadCount(PROCESSES)`

Whether to print more verbose information during the run `monteCarlo.setVerbose(False)`


After the monteCarlo run is configured, it is executed. This method returns the list of jobs that failed.

```
failures = monteCarlo.executeSimulations()
```

Now in another script (or the current one), the data from this simulation can be easily loaded.

```
# Test loading data from runs from disk
monteCarlo = Controller.load(dirName)
```

Then retained data from any run can then be accessed in the form of a dictionary with two sub-dictionaries for messages and variables:

```
  {
      "messages": {
          "messageName": [value1,value2,value3]
      },
      "variables": {
          "variableName": [value1,value2,value3]
      }
  }
```

```
retainedData = monteCarlo.getRetainedData(19)
retainedData["messages"]["inertial_state_output.r_BN_N"]
```

There are various other methods to get retained initial parameters, which are further documented in the `Controller` class and the test script.
