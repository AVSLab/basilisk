.. toctree::
.. role:: raw-latex(raw)
   :format: latex
..

.. contents::
   :depth: 3
..

.. _vizardSettings:

Vizard - BSK Scripting Settings
===============================

Overview
--------

The `Vizard <@ref%20vizard>`__ Unity-based visualization can have its
settings scripted from a Basilisk python simulation script. This allows
the user to write a BSK simulation script and specify in the same script
what Vizard options should be used. Within Vizard the user can change
these gain if needed. The BSK scripted Vizard settings are only used
once at the beginning of the playback.

When calling the ``enableUnityVisualization`` macro method a copy of the
vizInterface module is returned. All scriptable Vizard settings are
stored inside the ``settings`` variable. For example, to set the Vizard
ambient lighting the following code is used:
~~~~~~~~~~~~~\ [STRIKEOUT:{.py} viz =
vizSupport.enableUnityVisualization(scSim, simTaskName, simProcessName,
gravBodies=gravFactory, saveFile=fileName) viz.settings.ambient =
0.5]\ ~~~~~~~~~~~~~ If a setting is not provided, then the Vizard
defaults are used. This allows the user to specify just a few or a lot
of settings, as is appropriate.

Listing of all BSK scriptable Vizard settings
---------------------------------------------

The following list contains the optional Vizard settings that can be
specified. Only the settings used will be applied to Vizard. If a
variable below is not specified, then it is not applied to Vizard and
the Vizard default values are used.

General Settings
~~~~~~~~~~~~~~~~

+-----------------------+---------------+------------------------------+
| Variable              | Type          | Description                  |
+=======================+===============+==============================+
| ambient               | [0,8]         | value to specify the ambient |
|                       |               | Vizard lighting.             |
+-----------------------+---------------+------------------------------+
| orbitLinesOn          | (0,1)         | flag to show (1) or hide (0) |
|                       |               | the orbit trajectory lines   |
+-----------------------+---------------+------------------------------+
| spacecraftCSon        | (0,1)         | flag to show (1) or hide (0) |
|                       |               | the spacecraft coordinate    |
|                       |               | axes                         |
+-----------------------+---------------+------------------------------+
| planetCSon            | (0,1)         | flag to show (1) or hide (0) |
|                       |               | the planet coordinate axes   |
+-----------------------+---------------+------------------------------+
| skyBox                | String        | Used determine what star     |
|                       |               | background should be shown.  |
|                       |               | The empty string "" provides |
|                       |               | default NASA SVS Starmap,    |
|                       |               | “ESO” shows the ESO Milky    |
|                       |               | Way skybox, “black” provides |
|                       |               | a black background, or the   |
|                       |               | user can provide a filepath  |
|                       |               | to custom background image   |
|                       |               | file.                        |
+-----------------------+---------------+------------------------------+

Setting Actuator GUI Options
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

To specify the actuator GUI settings use the ``setActuatorGuiSetting``
helper method in Python. An example is

::

   vizSupport.setActuatorGuiSetting(viz, viewRWPanel=True, viewRWHUD=True)

The following table includes the keword options for this method.

+-----------------+-------------+---------------+---------------------+
| Variable        | Type        | Required      | Description         |
+=================+=============+===============+=====================+
| ``viewThrusterP | Boolean     | No            | Show the thruster   |
| anel``          |             |               | panel               |
+-----------------+-------------+---------------+---------------------+
| ``viewThrusterH | Boolean     | No            | Show the thruster   |
| UD``            |             |               | particle streams    |
+-----------------+-------------+---------------+---------------------+
| ``viewRWPanel`` | Boolean     | No            | Show the reaction   |
|                 |             |               | wheel panel         |
+-----------------+-------------+---------------+---------------------+
| ``viewRWHUD``   | Boolean     | No            | Show the reaction   |
|                 |             |               | wheel disks         |
|                 |             |               | configuration       |
|                 |             |               | outside the         |
|                 |             |               | spacecraft          |
+-----------------+-------------+---------------+---------------------+
| ``spacecraftNam | String      | No, sc name   | Specify which       |
| e``             |             | default       | spacecraft should   |
|                 |             |               | show actuator       |
|                 |             |               | information. If not |
|                 |             |               | provided then the   |
|                 |             |               | name specified in   |
|                 |             |               | ``viz.spacecraftNam |
|                 |             |               | e``                 |
|                 |             |               | is used.            |
+-----------------+-------------+---------------+---------------------+

Defining a Pointing Line
~~~~~~~~~~~~~~~~~~~~~~~~

Vizard can create a heading line from one object to another. For
example, it might be handy to create a line from the spacecraft pointing
towards the sun direction, or from the spacecraft towards Earth to know
how the antennas should point. These pointing lines can be scripted from
Basilisk as well using using a helper function ``createPointLine()``:
~~~~~~~~~~~~~\ [STRIKEOUT:{.py} viz =
vizSupport.enableUnityVisualization(scSim, simTaskName, simProcessName,
gravBodies=gravFactory, saveFile=fileName)
vizSupport.createPointLine(viz, toBodyName=“earth”, lineColor=[0, 0,
255, 255]) vizSupport.createPointLine(viz, toBodyName=“sun”,
lineColor=“yellow”)]\ ~~~~~~~~~~~~~ The ``createPointLine`` support
macro requires the parameters ``toBodyName`` and ``lineColor`` to be
defined. The parameter ``fromBodyName`` is optional. If it is not
specified, then the ``viz.spacecraftName`` is used as a default origin.
The ``lineColor`` state can be either a string containing the color
name, or a list containing RGBA values. The support macro converts this
into the required set of numerical values.

Each pointing line message contains the three variables listed in the
next table.

+-----------------------+---------------+----------+-------------------+
| Variable              | Type          | Required | Description       |
+=======================+===============+==========+===================+
| fromBodyName          | string        | No, sc   | contains the name |
|                       |               | name     | of the            |
|                       |               | default  | originating body  |
+-----------------------+---------------+----------+-------------------+
| toBodyName            | string        | Yes      | contains the name |
|                       |               |          | of the body to    |
|                       |               |          | point towards     |
+-----------------------+---------------+----------+-------------------+
| lineColor             | int(4)        | Yes      | color name or     |
|                       |               |          | array on integer  |
|                       |               |          | values specifying |
|                       |               |          | the RGBA values   |
|                       |               |          | between 0 to 255  |
+-----------------------+---------------+----------+-------------------+

Defining Keep In/Out Cones
~~~~~~~~~~~~~~~~~~~~~~~~~~

Vizard can create cones relative to the spacecraft which illustrated if
a body axis is within some angle to the sun (i.e. keep in cone), or if a
sensor axis is outside some angle to the sun (i.e. keep out cone). These
cones can be setup in Vizard, but can also be scripted from Basilisk
using the helper function ``createConeInOut``:
~~~~~~~~~~~~~\ [STRIKEOUT:{.py} viz =
vizSupport.enableUnityVisualization(scSim, simTaskName, simProcessName,
gravBodies=gravFactory, saveFile=fileName)
vizSupport.createConeInOut(viz, toBodyName=“earth”, coneColor=“teal”,
normalVector_B=[1, 0, 0], incidenceAngle=30\ macros.D2R, isKeepIn=True,
coneHeight=5.0, coneName=‘sensorCone’) vizSupport.createConeInOut(viz,
toBodyName=“earth”, coneColor=“blue”, normalVector_B=[0, 1, 0],
incidenceAngle=30\ macros.D2R, isKeepIn=False, coneHeight=5.0,
coneName=‘comCone’)]\ ~~~~~~~~~~~~~ The following table illustrates the
arguments for the ``createConeInOut`` method:

+-------------------+---------+---------+--------------+-------------+
| Variable          | Type    | Units   | Required     | Description |
+===================+=========+=========+==============+=============+
| isKeepIn          | bool    |         | Yes          | make cone   |
|                   |         |         |              | keep in     |
|                   |         |         |              | (True) or   |
|                   |         |         |              | keep out    |
|                   |         |         |              | (False)     |
+-------------------+---------+---------+--------------+-------------+
| fromBodyName      | string  |         | No, sc name  | contains    |
|                   |         |         | default      | the name of |
|                   |         |         |              | the         |
|                   |         |         |              | originating |
|                   |         |         |              | body        |
+-------------------+---------+---------+--------------+-------------+
| toBodyName        | string  |         | Yes          | contains    |
|                   |         |         |              | the name of |
|                   |         |         |              | the body to |
|                   |         |         |              | point       |
|                   |         |         |              | towards     |
+-------------------+---------+---------+--------------+-------------+
| lineColor         | int(4)  |         | Yes          | color name  |
|                   |         |         |              | or array on |
|                   |         |         |              | integer     |
|                   |         |         |              | values      |
|                   |         |         |              | specifying  |
|                   |         |         |              | the RGBA    |
|                   |         |         |              | values      |
|                   |         |         |              | between 0   |
|                   |         |         |              | to 255      |
+-------------------+---------+---------+--------------+-------------+
| position_B        | float(3 | m       | No, (0,0,0)  | position of |
|                   | )       |         | default      | the cone    |
|                   |         |         |              | vertex      |
+-------------------+---------+---------+--------------+-------------+
| normalVector_B    | float(3 |         | Yes          | normal axis |
|                   | )       |         |              | of the cone |
|                   |         |         |              | in body     |
|                   |         |         |              | frame       |
|                   |         |         |              | components  |
+-------------------+---------+---------+--------------+-------------+
| incidenceAngle    | float   | rad     | Yes          | angle of    |
|                   |         |         |              | the cone    |
+-------------------+---------+---------+--------------+-------------+
| coneHeight        | float   | m       | Yes          | height of   |
|                   |         |         |              | the cone    |
+-------------------+---------+---------+--------------+-------------+
| coneName          | string  |         | No           | cone label  |
|                   |         |         |              | name, if    |
|                   |         |         |              | unspecified |
|                   |         |         |              | ,           |
|                   |         |         |              | viz will    |
|                   |         |         |              | autogenerat |
|                   |         |         |              | e           |
|                   |         |         |              | name        |
+-------------------+---------+---------+--------------+-------------+

Defining the Vizard Camera View Panels
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Vizard can create two spacecraft relative camera panels (camera One and
Two) and one planet-pointing camera panel. This functionality can be
controlled by using the ‘createCameraViewPanel’ helper method.
~~~~~~~~~~~~~\ [STRIKEOUT:{.py} viz =
vizSupport.enableUnityVisualization(scSim, simTaskName, simProcessName,
gravBodies=gravFactory, saveFile=fileName)
vizSupport.createCameraViewPanel(viz, “One”, viewPanel=True, setView=0)
vizSupport.createCameraViewPanel(viz, “Two”, viewPanel=True, setView=3,
spacecraftVisible=True, fieldOfView=50.\ macros.D2R)
vizSupport.createCameraViewPanel(viz, “Planet”, viewPanel=True,
setView=2, spacecraftVisible=True, fieldOfView=50.\ macros.D2R,
targetBodyName=‘earth’)]\ ~~~~~~~~~~~~~ The following table illustrates
the arguments for the ``createCameraViewPanel`` method if invoking the
spacecraft relative camera headings for cameras ``One`` and ``Two``.

+-------------------+---------+---------+--------------+-------------+
| Variable          | Type    | Units   | Required     | Description |
+===================+=========+=========+==============+=============+
| spacecraftName    | string  |         | No, sc name  | name of the |
|                   |         |         | default      | spacecraft  |
|                   |         |         |              | with        |
|                   |         |         |              | respect to  |
|                   |         |         |              | which the   |
|                   |         |         |              | camera is   |
|                   |         |         |              | shown       |
+-------------------+---------+---------+--------------+-------------+
| viewPanel         | bool    |         | No, default  | flag        |
|                   |         |         | is false     | indicating  |
|                   |         |         |              | if a panel  |
|                   |         |         |              | should be   |
|                   |         |         |              | shown       |
|                   |         |         |              | (true) or   |
|                   |         |         |              | not (false) |
+-------------------+---------+---------+--------------+-------------+
| setView           | int     |         | Yes          | index       |
|                   |         |         |              | specifying  |
|                   |         |         |              | along which |
|                   |         |         |              | axis the    |
|                   |         |         |              | camera is   |
|                   |         |         |              | pointing (0 |
|                   |         |         |              | -> +X, 1 -> |
|                   |         |         |              | -X, 2 ->    |
|                   |         |         |              | +Y, 3 ->    |
|                   |         |         |              | -Y, 4 ->    |
|                   |         |         |              | +Z, 5 ->    |
|                   |         |         |              | -Z)         |
+-------------------+---------+---------+--------------+-------------+
| spacecraftVisible | bool    |         | No, default  | flag        |
|                   |         |         | is false     | indicating  |
|                   |         |         |              | if the      |
|                   |         |         |              | spacecraft  |
|                   |         |         |              | should be   |
|                   |         |         |              | shown in    |
|                   |         |         |              | the camera  |
|                   |         |         |              | view        |
+-------------------+---------+---------+--------------+-------------+
| fieldOfView       | float   | rad     | No, default  | camera      |
|                   |         |         | -1           | field of    |
|                   |         |         |              | view, to    |
|                   |         |         |              | use the     |
|                   |         |         |              | Vizard      |
|                   |         |         |              | default set |
|                   |         |         |              | it to -1    |
+-------------------+---------+---------+--------------+-------------+

The following tale illustrates the arguments for the
``createCameraViewPanel`` method if a planet pointing camera is setup.

+-------------------+---------+---------+--------------+-------------+
| Variable          | Type    | Units   | Required     | Description |
+===================+=========+=========+==============+=============+
| spacecraftName    | string  |         | No, sc name  | name of the |
|                   |         |         | default      | spacecraft  |
|                   |         |         |              | with        |
|                   |         |         |              | respect to  |
|                   |         |         |              | which the   |
|                   |         |         |              | camera is   |
|                   |         |         |              | shown       |
+-------------------+---------+---------+--------------+-------------+
| viewPanel         | bool    |         | No, default  | flag        |
|                   |         |         | is false     | indicating  |
|                   |         |         |              | if a panel  |
|                   |         |         |              | should be   |
|                   |         |         |              | shown       |
|                   |         |         |              | (true) or   |
|                   |         |         |              | not (false) |
+-------------------+---------+---------+--------------+-------------+
| setView           | int     |         | Yes          | index       |
|                   |         |         |              | specifying  |
|                   |         |         |              | along which |
|                   |         |         |              | orbit axis  |
|                   |         |         |              | the camera  |
|                   |         |         |              | is pointing |
|                   |         |         |              | (0 ->       |
|                   |         |         |              | Nadir, 1 -> |
|                   |         |         |              | Orbit       |
|                   |         |         |              | Normal, 2   |
|                   |         |         |              | -> Along    |
|                   |         |         |              | Track)      |
+-------------------+---------+---------+--------------+-------------+
| spacecraftVisible | bool    |         | No, default  | flag        |
|                   |         |         | is false     | indicating  |
|                   |         |         |              | if the      |
|                   |         |         |              | spacecraft  |
|                   |         |         |              | should be   |
|                   |         |         |              | shown in    |
|                   |         |         |              | the camera  |
|                   |         |         |              | view        |
+-------------------+---------+---------+--------------+-------------+
| fieldOfView       | float   | rad     | No, default  | camera      |
|                   |         |         | -1           | field of    |
|                   |         |         |              | view, to    |
|                   |         |         |              | use the     |
|                   |         |         |              | Vizard      |
|                   |         |         |              | default set |
|                   |         |         |              | it to -1    |
+-------------------+---------+---------+--------------+-------------+
| targetBodyName    | string  |         | Yes          | name of the |
|                   |         |         |              | planet to   |
|                   |         |         |              | point at    |
+-------------------+---------+---------+--------------+-------------+

:raw-latex:`\image `html Images/doc/vizard-ImgCustomCam.png
“Illustration of custom camera panel Panel” width=400px

It is also possible to create a custom camera view which points in an
arbitrary direction as illustrate in the image above. The following
helper method is an example of how such a custom camera view can be
created:

::

   vizSupport.createCameraConfigMsg(viz, cameraID=1, fieldOfView=10 * macros.D2R,
                                        resolution=[1024, 1024], renderRate=int(1e9 / 10), sensorSize=[0.2, 0.2],
                                        cameraPos_B=[0.2, 0.1, 0.3], sigma_CB=[-1./3., 1./3., -1./3.])

The following tale illustrates the arguments for the
``createCameraConfigMsg`` method.

+-------------------+---------+---------+--------------+-------------+
| Variable          | Type    | Units   | Required     | Description |
+===================+=========+=========+==============+=============+
| cameraID          | Int     |         | Yes          | ID of the   |
|                   |         |         |              | Vizard      |
|                   |         |         |              | camera      |
+-------------------+---------+---------+--------------+-------------+
| parentName        | string  |         | No, sc name  | name of the |
|                   |         |         | default      | spacecraft  |
|                   |         |         |              | with        |
|                   |         |         |              | respect to  |
|                   |         |         |              | which the   |
|                   |         |         |              | camera is   |
|                   |         |         |              | shown       |
+-------------------+---------+---------+--------------+-------------+
| fieldOfView       | Float   | rad     | Yes          | field of    |
|                   |         |         |              | view        |
+-------------------+---------+---------+--------------+-------------+
| resolution        | Int(2)  |         | Yes          | image       |
|                   |         |         |              | sensor      |
|                   |         |         |              | pixels      |
+-------------------+---------+---------+--------------+-------------+
| renderRate        | Int     | ns      | Yes          | time        |
|                   |         |         |              | between     |
|                   |         |         |              | image grabs |
+-------------------+---------+---------+--------------+-------------+
| sensorSize        | Float(2 | m       | Yes          | sensor      |
|                   | )       |         |              | dimensions  |
+-------------------+---------+---------+--------------+-------------+
| cameraPos_B       | Float(3 | m       | Yes          | camera      |
|                   | )       |         |              | location    |
|                   |         |         |              | relative to |
|                   |         |         |              | body frame  |
|                   |         |         |              | in B frame  |
|                   |         |         |              | components  |
+-------------------+---------+---------+--------------+-------------+
| sigma_CB          | Float(3 |         | Yes          | camera      |
|                   | )       |         |              | orientation |
|                   |         |         |              | relative to |
|                   |         |         |              | teh body    |
|                   |         |         |              | frame in    |
|                   |         |         |              | MRPs        |
+-------------------+---------+---------+--------------+-------------+
| skyBox            | String  |         | No           | Used        |
|                   |         |         |              | determine   |
|                   |         |         |              | what star   |
|                   |         |         |              | background  |
|                   |         |         |              | should be   |
|                   |         |         |              | shown. The  |
|                   |         |         |              | empty       |
|                   |         |         |              | string ""   |
|                   |         |         |              | provides    |
|                   |         |         |              | default     |
|                   |         |         |              | NASA SVS    |
|                   |         |         |              | Starmap,    |
|                   |         |         |              | “ESO” shows |
|                   |         |         |              | the ESO     |
|                   |         |         |              | Milky Way   |
|                   |         |         |              | skybox,     |
|                   |         |         |              | “black”     |
|                   |         |         |              | provides a  |
|                   |         |         |              | black       |
|                   |         |         |              | background, |
|                   |         |         |              | or the user |
|                   |         |         |              | can provide |
|                   |         |         |              | a filepath  |
|                   |         |         |              | to custom   |
|                   |         |         |              | background  |
|                   |         |         |              | image file. |
+-------------------+---------+---------+--------------+-------------+
