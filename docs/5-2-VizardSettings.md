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


### Defining a Pointing Line
Vizard can create a heading line from one object to another.  For example, it might be handy to create a line from the spacecraft pointing towards the sun direction, or from the spacecraft towards Earth to know how the antennas should point.  These pointing lines can be scripted from Basilisk as well using using a helper function `createPointLine()`:
~~~~~~~~~~~~~~~{.py}
    viz = vizSupport.enableUnityVisualization(scSim, simTaskName, simProcessName, gravBodies=gravFactory, saveFile=fileName)
    vizSupport.createPointLine(viz, toBodyName="earth", lineColor=[0, 0, 255, 255])
    vizSupport.createPointLine(viz, toBodyName="sun", lineColor="yellow")
~~~~~~~~~~~~~~~
The `createPointLine` support macro requires the parameters `toBodyName` and `lineColor` to be defined.  The parameter `fromBodyName` is optional.  If it is not specified, then the `viz.spacecraftName` is used as a default origin.  The `lineColor` state can be either a string containing the color name, or a list containing RGBA values.  The support macro converts this into the required set of numerical values.

Each pointing line message contains the three variables listed in the next table.

Variable      |  Range | Required | Description
------------- | ---------|------|-----------
fromBodyName | string| No, sc name default | contains the name of the originating body
toBodyName | string | Yes | contains the name of the body to point towards
lineColor | int(4) | Yes | color name or array on integer values specifying the RGBA values between 0 to 255


### Defining Keep In/Out Cones
Vizard can create cones relative to the spacecraft which illustrated if a body axis is within some angle to the sun (i.e. keep in cone), or if a sensor axis is outside some angle to the sun (i.e. keep out cone).  These cones can be setup in Vizard, but can also be scripted from Basilisk using the helper function `createConeInOut`: 
~~~~~~~~~~~~~~~{.py}
    viz = vizSupport.enableUnityVisualization(scSim, simTaskName, simProcessName, gravBodies=gravFactory, saveFile=fileName)
    vizSupport.createConeInOut(viz, toBodyName="earth", coneColor="teal", normalVector_B=[1, 0, 0], incidenceAngle=30*macros.D2R, isKeepIn=True, coneHeight=5.0, coneName='sensorCone')
    vizSupport.createConeInOut(viz, toBodyName="earth", coneColor="blue", normalVector_B=[0, 1, 0], incidenceAngle=30*macros.D2R, isKeepIn=False, coneHeight=5.0, coneName='comCone')
~~~~~~~~~~~~~~~
The following table illustrate the arguments for the `createConeInOut` method:
Variable      |  Range | Required | Description
------------- | ---------|--------|---------
isKeepIn | bool | Yes | make cone keep in (True) or keep out (False)
fromBodyName | string| No, sc name default | contains the name of the originating body
toBodyName | string | Yes | contains the name of the body to point towards
lineColor | int(4) | Yes | color name or array on integer values specifying the RGBA values between 0 to 255
position_B | float(3) | No, (0,0,0) default | position of the cone vertex
normalVector_B | float(3) | Yes | normal axis of the cone in body frame components
incidenceAngle | float | Yes | angle (rad) of the cone
coneHeight | float | Yes | height of the cone
coneName | string | No | cone label name, if unspecified, viz will autogenerate name

