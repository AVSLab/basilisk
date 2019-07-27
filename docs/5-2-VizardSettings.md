# Vizard - BSK Scripting Settings  {#vizardSettings}





## Overview
The [Vizard](@ref vizard) Unity-based visualization can have its settings scripted from a Basilisk python simulation script.  When calling the `enableUnityVisualization` macro method a copy of the vizInterface module is returned.  All scriptable Vizard settings are stored inside the `settings` variable.  For example, to set the Vizard ambient lighting the following code is used:
~~~~~~~~~~~~~~~{.py}
    viz = vizSupport.enableUnityVisualization(scSim, simTaskName, simProcessName, gravBodies=gravFactory, saveFile=fileName)
    viz.settings.ambient = 0.5
~~~~~~~~~~~~~~~


## Listing of all BSK scriptable Vizard settings
The following list contains the optinal Vizard settings that can be specified.  Only the settings used will be applied to Vizard.  If a variable below is not specified, then it is not applied to Vizard and the Vizard default values are used.

### General Settings
Variable      |  Range | Description
------------- | ---------|-----------------
ambient | [0,8]| value to specify the ambient Vizard lighting.
orbitLinesOn | (0,1) | flag to show (1) or hide (0) the orbit trajectory lines
spacecraftCSon | (0,1) | flag to show (1) or hide (0) the spacecraft coordinate axes
planetCSon | (0,1) | flag to show (1) or hide (0) the planet coordinate axes
