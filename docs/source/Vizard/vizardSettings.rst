
.. _vizardSettings:

BSK Scripting Settings
======================

Overview
--------

The :ref:`vizard` Unity-based visualization can have its
settings scripted from a Basilisk python simulation script. This allows
the user to write a BSK simulation script and specify in the same script
what Vizard options should be used. Within Vizard the user can change
these gain if needed. The BSK scripted Vizard settings are only used
once at the beginning of the playback.

When calling the ``enableUnityVisualization`` macro method a copy of the
vizInterface module is returned. All scriptable Vizard settings are
stored inside the ``settings`` variable. For example, to set the Vizard
ambient lighting the following code is used:

.. code-block:: python

	viz = vizSupport.enableUnityVisualization(scSim, simTaskName, simProcessName, gravBodies=gravFactory, saveFile=fileName) 
	viz.settings.ambient = [0.5] 

If a setting is not provided, then the Vizard
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
The following settings can be set directly using::

    viz.settings.variableName = value

.. table:: Vizard Simulation Parameters
    :widths: 10 10 100

    +---------------------------+---------------+------------------------------+
    | Variable                  | Type          | Description                  |
    +===========================+===============+==============================+
    | ambient                   | [0,8]         | value to specify the ambient |
    |                           |               | Vizard lighting.             |
    +---------------------------+---------------+------------------------------+
    | orbitLinesOn              | (0,1)         | flag to show (1) or hide (0) |
    |                           |               | the orbit trajectory lines   |
    +---------------------------+---------------+------------------------------+
    | spacecraftCSon            | (0,1)         | flag to show (1) or hide (0) |
    |                           |               | the spacecraft coordinate    |
    |                           |               | axes                         |
    +---------------------------+---------------+------------------------------+
    | planetCSon                | (0,1)         | flag to show (1) or hide (0) |
    |                           |               | the planet coordinate axes   |
    +---------------------------+---------------+------------------------------+
    | skyBox                    | String        | Used determine what star     |
    |                           |               | background should be shown.  |
    |                           |               | The empty string "" provides |
    |                           |               | default NASA SVS Starmap,    |
    |                           |               | “ESO” shows the ESO Milky    |
    |                           |               | Way skybox, “black” provides |
    |                           |               | a black background, or the   |
    |                           |               | user can provide a filepath  |
    |                           |               | to custom background image   |
    |                           |               | file.                        |
    +---------------------------+---------------+------------------------------+
    | viewCameraBoresightHUD    | (0,1)         | flag to show (1) or hide (0) |
    |                           |               | the camera boresight line    |
    +---------------------------+---------------+------------------------------+
    | viewCameraConeHUD         | (0,1)         | flag to show (1) or hide (0) |
    |                           |               | the camera cone              |
    +---------------------------+---------------+------------------------------+
    | showCSLabels              | (0,1)         | flag to show (1) or hide (0) |
    |                           |               | the coordinate system labels |
    +---------------------------+---------------+------------------------------+
    | showCelestialBodyLabels   | (0,1)         | flag to show (1) or hide (0) |
    |                           |               | the celestial body labels    |
    +---------------------------+---------------+------------------------------+
    | showSpacecraftLabels      | (0,1)         | flag to show (1) or hide (0) |
    |                           |               | the spacecraft labels        |
    +---------------------------+---------------+------------------------------+

Setting Actuator GUI Options
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

To specify the actuator GUI settings use the ``setActuatorGuiSetting``
helper method in Python. An example is::

   vizSupport.setActuatorGuiSetting(viz, viewRWPanel=True, viewRWHUD=True)

The following table includes the keyword options for this method.

.. table:: GUI Parameter Options
    :widths: 10 10 20 100

    +---------------------------+-------------+---------------+-----------------------+
    | Variable                  | Type        | Required      | Description           |
    +===========================+=============+===============+=======================+
    | ``viewThrusterPanel``     | Boolean     | No            | Show the thruster     |
    |                           |             |               | panel                 |
    +---------------------------+-------------+---------------+-----------------------+
    | ``viewThrusterHUD``       | Boolean     | No            | Show the thruster     |
    |                           |             |               | particle streams      |
    +---------------------------+-------------+---------------+-----------------------+
    | ``showThrusterLabels``    | Boolean     | No            | Show the thruster     |
    |                           |             |               | labels                |
    +---------------------------+-------------+---------------+-----------------------+
    | ``viewRWPanel``           | Boolean     | No            | Show the reaction     |
    |                           |             |               | wheel panel           |
    +---------------------------+-------------+---------------+-----------------------+
    | ``viewRWHUD``             | Boolean     | No            | Show the reaction     |
    |                           |             |               | wheel disks           |
    |                           |             |               | configuration         |
    |                           |             |               | outside the           |
    |                           |             |               | spacecraft            |
    +---------------------------+-------------+---------------+-----------------------+
    | ``showRWLabels``          | Boolean     | No            | Show the reaction     |
    |                           |             |               | wheel labels          |
    +---------------------------+-------------+---------------+-----------------------+
    | ``spacecraftName``        | String      | No, sc name   | Specify which         |
    |                           |             | default       | spacecraft should     |
    |                           |             |               | show actuator         |
    |                           |             |               | information. If not   |
    |                           |             |               | provided then the     |
    |                           |             |               | ``viz.spacecraftName``|
    |                           |             |               | is used.              |
    +---------------------------+-------------+---------------+-----------------------+

Defining a Pointing Line
~~~~~~~~~~~~~~~~~~~~~~~~

Vizard can create a heading line from one object to another. For
example, it might be handy to create a line from the spacecraft pointing
towards the sun direction, or from the spacecraft towards Earth to know
how the antennas should point. These pointing lines can be scripted from
Basilisk as well using using a helper function ``createPointLine()``:

.. code-block::

    viz = vizSupport.enableUnityVisualization(scSim, simTaskName, simProcessName, gravBodies=gravFactory, saveFile=fileName)
    vizSupport.createPointLine(viz, toBodyName='earth', lineColor=[0, 0, 255, 255]) vizSupport.createPointLine(viz, toBodyName=“sun”, lineColor=“yellow”)]

The ``createPointLine`` support macro requires the parameters ``toBodyName`` and ``lineColor`` to be
defined. The parameter ``fromBodyName`` is optional. If it is not
specified, then the ``viz.spacecraftName`` is used as a default origin.
The ``lineColor`` state can be either a string containing the color
name, or a list containing RGBA values. The support macro converts this
into the required set of numerical values.

Each pointing line message contains the three variables listed in the
next table.

.. table:: Pointing Line Parameter Options
    :widths: 10 10 10 100

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

.. code-block::
	
	viz = vizSupport.enableUnityVisualization(scSim, simTaskName, simProcessName, gravBodies=gravFactory, saveFile=fileName)
	vizSupport.createConeInOut(viz, toBodyName='earth', coneColor='teal', normalVector_B=[1, 0, 0], incidenceAngle=30\ macros.D2R, isKeepIn=True, coneHeight=5.0, coneName=‘sensorCone’)
	vizSupport.createConeInOut(viz,toBodyName='earth', coneColor='blue', normalVector_B=[0, 1, 0], incidenceAngle=30\ macros.D2R, isKeepIn=False, coneHeight=5.0, coneName=‘comCone’)]
	
The following table illustrates the
arguments for the ``createConeInOut`` method:

.. table:: Keep In/Out Cones Parameter Options
    :widths: 20 10 10 10 100

    +-------------------+----------+---------+--------------+-------------+
    | Variable          | Type     | Units   | Required     | Description |
    +===================+==========+=========+==============+=============+
    | isKeepIn          | bool     |         | Yes          | make cone   |
    |                   |          |         |              | keep in     |
    |                   |          |         |              | (True) or   |
    |                   |          |         |              | keep out    |
    |                   |          |         |              | (False)     |
    +-------------------+----------+---------+--------------+-------------+
    | fromBodyName      | string   |         | No, sc name  | contains    |
    |                   |          |         | default      | the name of |
    |                   |          |         |              | the         |
    |                   |          |         |              | originating |
    |                   |          |         |              | body        |
    +-------------------+----------+---------+--------------+-------------+
    | toBodyName        | string   |         | Yes          | contains    |
    |                   |          |         |              | the name of |
    |                   |          |         |              | the body to |
    |                   |          |         |              | point       |
    |                   |          |         |              | towards     |
    +-------------------+----------+---------+--------------+-------------+
    | lineColor         | int(4)   |         | Yes          | color name  |
    |                   |          |         |              | or array on |
    |                   |          |         |              | integer     |
    |                   |          |         |              | values      |
    |                   |          |         |              | specifying  |
    |                   |          |         |              | the RGBA    |
    |                   |          |         |              | values      |
    |                   |          |         |              | between 0   |
    |                   |          |         |              | to 255      |
    +-------------------+----------+---------+--------------+-------------+
    | position_B        | float(3) | m       | No, (0,0,0)  | position of |
    |                   |          |         | default      | the cone    |
    |                   |          |         |              | vertex      |
    +-------------------+----------+---------+--------------+-------------+
    | normalVector_B    | float(3) |         | Yes          | normal axis |
    |                   |          |         |              | of the cone |
    |                   |          |         |              | in body     |
    |                   |          |         |              | frame       |
    |                   |          |         |              | components  |
    +-------------------+----------+---------+--------------+-------------+
    | incidenceAngle    | float    | rad     | Yes          | angle of    |
    |                   |          |         |              | the cone    |
    +-------------------+----------+---------+--------------+-------------+
    | coneHeight        | float    | m       | Yes          | height of   |
    |                   |          |         |              | the cone    |
    +-------------------+----------+---------+--------------+-------------+
    | coneName          | string   |         | No           | cone label  |
    |                   |          |         |              | name, if    |
    |                   |          |         |              | unspecified |
    |                   |          |         |              | ,           |
    |                   |          |         |              | viz will    |
    |                   |          |         |              | autogenerate|
    |                   |          |         |              | name        |
    +-------------------+----------+---------+--------------+-------------+

Defining the Vizard Camera View Panels
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Vizard can create a spacecraft relative camera panel. This functionality can be
controlled by using the ``createStandardCamera`` helper method.  The camera can
point in a body-fixed direction (``setMode=1``), or be aimed at a celestial target
(``setMode=0``).  Multiple camera panels can be setup at the same time, and
they can be attached to different spacecraft through the ``spacecraftName`` argument.

.. code-block:: python

	viz = vizSupport.enableUnityVisualization(scSim, simTaskName, simProcessName,
	gravBodies=gravFactory, saveFile=fileName)
	vizSupport.createStandardCamera(viz, setMode=0, bodyTarget='earth', setView=0)
	vizSupport.createStandardCamera(viz, setMode=1, fieldOfView=60.*macros.D2R, pointingVector_B=[0.0, -1.0, 0.0])


The following table illustrates
the arguments for the ``createStandardCamera`` method.

.. table:: Standard Camera View Panel Parameter Options
    :widths: 15 10 10 15 50

    +-------------------+---------+---------+--------------+--------------------------------------------+
    | Variable          | Type    | Units   | Required     | Description                                |
    +===================+=========+=========+==============+============================================+
    | spacecraftName    | string  |         | No, sc name  | name of the spacecraft                     |
    |                   |         |         | default      | with respect to which the camera is shown  |
    +-------------------+---------+---------+--------------+--------------------------------------------+
    | setMode           | int     |         | No, default  | 0 -> body targeting, 1 -> pointing vector  |
    |                   |         |         | is 1         |                                            |
    +-------------------+---------+---------+--------------+--------------------------------------------+
    | setView           | int     |         | No, default  | 0 -> Nadir, 1 -> Orbit Normal, 2 ->        |
    |                   |         |         | is 0         | Along Track (default to nadir). This       |
    |                   |         |         |              | is a setting for body targeting mode.      |
    +-------------------+---------+---------+--------------+--------------------------------------------+
    | bodyTarget        | string  |         | No, default  | Name of body camera should point to. This  |
    |                   |         |         | to first     | is a setting for body targeting mode.      |
    |                   |         |         | celestial    |                                            |
    |                   |         |         | body in      |                                            |
    |                   |         |         | messages     |                                            |
    +-------------------+---------+---------+--------------+--------------------------------------------+
    | fieldOfView       | float   | rad     | No, default  | camera field of view, to use the Vizard    |
    |                   |         |         | -1           | default set it to -1                       |
    +-------------------+---------+---------+--------------+--------------------------------------------+
    | pointingVector_B  | float(3)|         | No, default  | Body relative unit vector. This is a       |
    |                   |         |         | is           | setting for pointing vector mode           |
    |                   |         |         | (1, 0, 0)    |                                            |
    +-------------------+---------+---------+--------------+--------------------------------------------+
    | position_B        | float(3)|         | No, default  | If populated,                              |
    |                   |         |         | is           | sets camera  position relative             |
    |                   |         |         | (0, 0, 0)    | to parent body coordinate frame            |
    |                   |         |         | for auto     | in meters using B frame components.        |
    |                   |         |         | placement    | If unpopulated camera is positioned        |
    |                   |         |         |              | automatically along camera view direction  |
    |                   |         |         |              | outside of parent body's mesh to prevent   |
    |                   |         |         |              | obstruction of view.                       |
    +-------------------+---------+---------+--------------+--------------------------------------------+

.. image:: /_images/static/vizard-ImgCustomCam.jpg
   :align: center
   :width: 90 %

It is also possible to create a custom instrument camera view for opNav mode which points in an
arbitrary direction as illustrate in the image above. The following
helper method is an example of how such an instrument camera view can be
created::

   vizSupport.createCameraConfigMsg(viz, cameraID=1, fieldOfView=10 * macros.D2R,
                                        resolution=[1024, 1024], renderRate=0.1, sensorSize=[0.2, 0.2],
                                        cameraPos_B=[0.2, 0.1, 0.3], sigma_CB=[-1./3., 1./3., -1./3.])

Note that with this instrument camera Vizard will save off images the the user home folder at the rate
specified in ``renderRate``.  To avoid saving off images just make ``renderRate`` zero.

The camera frame is illustrated in the following image.  It uses classical image image coordinates where ``x`` points
to the right, ``y`` point downwards and ``z`` points outward.  More information is availabe in section 2.4.1 of
Dr. Teil's `dissertation <http://hanspeterschaub.info/Papers/grads/ThibaudTeil.pdf>`_.

.. image:: /_images/static/imageFrame.jpg
   :align: center
   :width: 600px

The following tale illustrates the arguments for the
``createCameraConfigMsg`` method.

.. table:: ``createCameraConfigMsg`` Parameter Options
    :widths: 15 10 10 15 100

    +-------------------+---------+---------+--------------+------------------------+
    | Variable          | Type    | Units   | Required     | Description            |
    +===================+=========+=========+==============+========================+
    | cameraID          | Int     |         | Yes          | ID of the Vizard       |
    |                   |         |         |              | camera                 |
    +-------------------+---------+---------+--------------+------------------------+
    | parentName        | string  |         | No, sc name  | name of the spacecraft |
    |                   |         |         | default      | with respect to which  |
    |                   |         |         |              | the camera is shown    |
    +-------------------+---------+---------+--------------+------------------------+
    | fieldOfView       | Float   | rad     | Yes          | field of view          |
    +-------------------+---------+---------+--------------+------------------------+
    | resolution        | Int(2)  |         | Yes          | image sensor pixels    |
    +-------------------+---------+---------+--------------+------------------------+
    | renderRate        | Float   | s       | Yes          | time between image     |
    |                   |         |         |              | grabs. 0 turns this    |
    |                   |         |         |              | off (default).         |
    +-------------------+---------+---------+--------------+------------------------+
    | sensorSize        | Float(2)| m       | Yes          | sensor dimensions      |
    +-------------------+---------+---------+--------------+------------------------+
    | cameraPos_B       | Float(3)| m       | Yes          | camera  location       |
    |                   |         |         |              | relative to body frame |
    |                   |         |         |              | in B frame components  |
    +-------------------+---------+---------+--------------+------------------------+
    | sigma_CB          | Float(3)|         | Yes          | camera orientation     |
    |                   |         |         |              | relative to the body   |
    |                   |         |         |              | frame in MRPs          |
    +-------------------+---------+---------+--------------+------------------------+
    | skyBox            | String  |         | No           | Used to determine      |
    |                   |         |         |              | what star background   |
    |                   |         |         |              | should be shown. The   |
    |                   |         |         |              | empty string ""        |
    |                   |         |         |              | provides default NASA  |
    |                   |         |         |              | SVS Starmap, “ESO”     |
    |                   |         |         |              | shows the ESO Milky Way|
    |                   |         |         |              | skybox, “black”        |
    |                   |         |         |              | provides a black       |
    |                   |         |         |              | background, or the user|
    |                   |         |         |              | can provide a filepath |
    |                   |         |         |              | to custom  background  |
    |                   |         |         |              | image file.            |
    +-------------------+---------+---------+--------------+------------------------+


Defining the Custom Spacecraft Shape model
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

You can specify a custom OBJ model to be used with Vizard spacecraft representation.
An sample is shown in the following screen capture.

.. image:: /_images/static/vizard-ImgCustomCAD.jpg
   :align: center
   :scale: 80 %

This functionality can be controlled by using the ‘createCustomModel’ helper method.

.. code-block::

	viz = vizSupport.enableUnityVisualization(scSim, simTaskName, simProcessName,
	gravBodies=gravFactory, saveFile=fileName)
	vizSupport.createCustomModel(viz,
	                            modelPath="/Users/hp/Downloads/Topex-Posidon/Topex-Posidon-composite.obj",
	                            scale=[2, 2, 10])


The following table illustrates the arguments for the ``createCustomModel`` method.

.. table:: Custom Space Object OBJ Import Parameter Options
    :widths: 15 10 10 15 50

    +-------------------+---------+---------+--------------+------------------------------+
    | Variable          | Type    | Units   | Required     | Description                  |
    +===================+=========+=========+==============+==============================+
    | modelPath         | string  |         | Yes          | Path to model obj -OR-       |
    |                   |         |         |              | "CUBE", "CYLINDER", or       |
    |                   |         |         |              | "SPHERE" to use a primitive  |
    |                   |         |         |              | shape                        |
    +-------------------+---------+---------+--------------+------------------------------+
    | simBodiesToModify | string  |         | No, default  | Which bodies in scene to     |
    |                   |         |         | is `bsk-Sat` | replace with this model, use |
    |                   |         |         |              | "ALL_SPACECRAFT" to apply    |
    |                   |         |         |              | custom model to all          |
    |                   |         |         |              | spacecraft in simulation     |
    +-------------------+---------+---------+--------------+------------------------------+
    | offset            | float(3)|  m      | No, default  | offset to use to draw the    |
    |                   |         |         | is (0,0,0)   | model                        |
    +-------------------+---------+---------+--------------+------------------------------+
    | rotation          | float(3)|  rad    | No, default  | 3-2-1 Euler angles to rotate |
    |                   |         |         | is (0,0,0)   | CAD about z, y, x axes       |
    +-------------------+---------+---------+--------------+------------------------------+
    | scale             | float(3)|         | No, default  | desired model scale in       |
    |                   |         |         | is (1,1,1)   | x, y, z in spacecraft CS     |
    +-------------------+---------+---------+--------------+------------------------------+
    | customTexturePath | string  |         | No           | Path to texture to apply to  |
    |                   |         |         |              | model (note that a custom    |
    |                   |         |         |              | model's .mtl will be         |
    |                   |         |         |              | automatically imported with  |
    |                   |         |         |              | its textures during custom   |
    |                   |         |         |              | model import)                |
    +-------------------+---------+---------+--------------+------------------------------+
    | normalMapPath     | string  |         | No           | Path to the normal map for   |
    |                   |         |         |              | the customTexture            |
    +-------------------+---------+---------+--------------+------------------------------+
    | shader            | int     |         | No, default  | Value of -1 to use viz       |
    |                   |         |         | is -1        | default, 0 for Unity Specular|
    |                   |         |         |              | Standard Shader, 1 for Unity |
    |                   |         |         |              | Standard Shader              |
    +-------------------+---------+---------+--------------+------------------------------+
