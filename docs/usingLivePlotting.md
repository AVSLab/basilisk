# Live Plotting {#usingLivePlotting}

The framework exists for simulations to be run with an option to plot data continuously. Resulting plots are not plotted with the same precision as the final plots, therefore should be used qualitatively. This document will give an overview of the options, and how to use live plotting in simulations. 

## Templates

Reference implementations / templates for live plotting exist for bskSimScenarios at scenario_BasicOrbit_LivePlot.py and for scenarios at scenarioBasicOrbitLivePlot.py. The same implementation can be used for any scenario within Basilisk.

## Defining Plots

The data structure that contains all information for what to plot live is contained in:
dataRequests = [{"plotID" : 0,
                 "plotFun" : "plot_orbit",
                 "dataReq" : [self.masterSim.get_DynModel().simpleNavObject.outputTransName + ".r_BN_N"]},
                {"plotID" : 1,
                 "plotFun" : "plot_orientation",
                 "dataReq" : [self.masterSim.get_DynModel().simpleNavObject.outputTransName + ".r_BN_N",
                              self.masterSim.get_DynModel().simpleNavObject.outputTransName + ".v_BN_N",
                              self.masterSim.get_DynModel().simpleNavObject.outputAttName + ".sigma_BN"]}]

plotID = unique plot identifier
plotFun = name of plotting function in bskSimScenarios/plotting/BSK_Plotting.py to use
dataReq = data to be pulled from simulation

## Refresh Rate

The scenario contains method live_outputs(self, plotComm, rate). Here, rate is defined in ms and dictates how often the scenario will request data from the simulation to plot. Note: in the case of multiple simultaneous live plots, the rate at which any one plot will be updated will be approximately rate/# of plots.

## Running Without Live Plots

The scenario's member livePlots is a boolean that can be set to False to retain original scenario functionality.

## NOTE

On macOS there is a NSXPCSharedListener warning of invalid connection. This does not effect performance and is under attention. 
