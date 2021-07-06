
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

	viz = vizSupport.enableUnityVisualization(scSim, simTaskName, scObject, saveFile=fileName)
	viz.settings.ambient = 0.5

Here ``scObject`` is a :ref:`spacecraft` instance.  This can also be a list of spacecraft objects
for a multi-satellite simulation.
The spacecraft names are pulled from `scObject.ModelTag`.
If a setting is not provided, then the Vizard
defaults are used. This allows the user to specify just a few or a lot
of settings, as is appropriate.

Listing of all BSK Scriptable Vizard Settings
---------------------------------------------

The following list contains the optional Vizard settings that can be
specified. Only the settings used will be applied to Vizard. If a
variable below is not specified, then it is not applied to Vizard and
the Vizard default values are used.

General Settings
~~~~~~~~~~~~~~~~
The following settings can be set directly using::

    viz.settings.variableName = value

Note that for setting flags 1 means turn on, -1 means turn off, and a setting of 0 tells Vizard to use its own
default setting for that behavior.

.. list-table:: Vizard Simulation Parameters
    :widths: 10 10 80
    :header-rows: 1

    * - Variable
      - Type
      - Description
    * - ``ambient``
      - [0,1]
      - float value to specify the ambient Vizard lighting.
    * - ``orbitLinesOn``
      - (-1,1)
      - flag to show (1) or hide (-1) the orbit trajectory lines
    * - ``spacecraftCSon``
      - (-1,1)
      - flag to show (1) or hide (-1) the spacecraft coordinate axes
    * - ``planetCSon``
      - (-1,1)
      - flag to show (1) or hide (-1) the planet coordinate axes
    * - ``skyBox``
      - String
      - Used determine what star background should be shown. The empty string "" provides default NASA SVS Starmap,
        “ESO” shows the ESO Milky Way skybox, “black” provides a black background, or the user can provide a
        filepath to custom background image file.
    * - ``viewCameraBoresightHUD``
      - (-1,1)
      - flag to show (1) or hide (-1) the camera boresight line
    * - ``viewCameraConeHUD``
      - (-1,1)
      - flag to show (1) or hide (-1) the camera cone
    * - ``showCSLabels``
      - (-1,1)
      - flag to show (1) or hide (-) the coordinate system labels
    * - ``showCelestialBodyLabels``
      - (-1,1)
      - flag to show (1) or hide (-1) the celestial body labels
    * - ``showSpacecraftLabels``
      - (-1,1)
      - flag to show (1) or hide (-1) the spacecraft labels
    * - ``showCameraLabels``
      - (-1,1)
      - flag to show (1) or hide (-1) the camera labels
    * - ``customGUIScale``
      - pos. double
      - GUI scaling factor, default is -1 which uses Vizard default.
    * - ``defaultSpacecraftSprite``
      - string
      - Set sprite for ALL spacecraft through shape name and optional int RGB color values [0,255].
        Possible settings: ``CIRCLE``, ``SQUARE``, ``STAR``, ``TRIANGLE`` or ``bskSat`` for a 2D spacecraft
        sprite of the bskSat shape.  Default value is empty yielding a white ``CIRCLE``.
        To set this in python, use the helper function ``vizSupport.setSprite("STAR", color="red")``
    * - ``showSpacecraftAsSprites``
      - (-1,1)
      - Flag to show spacecraft as sprites if their visual size gets too small
    * - ``showCelestialBodiesAsSprites``
      - (-1,1)
      - Flag to show celestial bodies as sprites if their visual size gets too small
    * - ``show24hrClock``
      - (-1,1)
      - Flag to make mission date/time use a 24h clock instead of a 12h clock with am/pm
    * - ``showDataRateDisplay``
      - (-1,1)
      - Flag to show the data frame rate
    * - ``keyboardAngularRate``
      - pos. double
      - [rad/sec] controls the angular rate at which the camera rotates with keyboard hot-keys.
    * - ``keyboardZoomRate``
      - pos. double
      - Non-dimensional speed at which the camera zooms in and out with hot-keys.
    * - ``defaultThrusterColor``
      - int(4)
      - RGBA color values between (0,255).  Default values of -1 makes Vizard use the default thruster plume color
        You can use ``vizSupport.toRGBA255("red")`` to convert common color names to RGBA values.
    * - ``defaultThrusterPlumeLifeScalar``
      - double
      - Value of 1.0 or 0.0 to use viz default, values between 0 and 1 will decrease the length of all thruster plumes,
        >1 will increase lengths of all thruster plumes
    * - ``orbitLineSegments``
      - int
      - Number of line segments to use when drawing an osculating trajectory. Value of 0 (protobuffer default)
        to use viz default or any value greater than or equal to 4
    * - ``relativeOrbitRange``
      - int
      - +/- angular range in degrees of the osculating trajectory to show.  Value of 0 (protobuffer default) to use
        viz default or any value greater than or equal to 1
    * - ``showHillFrame``
      - int
      - flag to show the orbit Hill frame of the spacecraft camera target. Value of 0 (protobuffer default)
        to use viz default, -1 for false, 1 for true
    * - ``showVelocityFrame``
      - int
      - flag to show the orbit velocity frame of the spacecraft camera target. Value of 0 (protobuffer default)
        to use viz default, -1 for false, 1 for true
    * - ``relativeOrbitFrame``
      - int
      - flag to set with respect to which frame the relative orbit trajectory is drawn.
        Value of 0 (protobuffer default) or 1 to use Hill Frame, 2 to use Velocity Frame
    * - ``relativeOrbitFrame``
      - string
      - If valid spacecraft name provided, the relative orbit chief spacecraft will be set to that
        spacecraft object. Setting the string to "AUTO" or leaving this field empty will select the camera
        target spacecraft as the chief.
    * - ``spacecraftShadowBrightness``
      - double
      - Control the ambient light specific to spacecraft objects, value between 0 and 1, use negative value
        to use viz default
    * - ``spacecraftSizeMultiplier``
      - double
      - Control the display size of spacecraft in the Planet and Solar System Views, values greater than 0,
        use negative value to use viz default
    * - ``showLocationCommLines``
      - int
      - Value of 0 (protobuffer default) to use viz default, -1 for false, 1 for true
    * - ``showLocationCones``
      - int
      - Value of 0 (protobuffer default) to use viz default, -1 for false, 1 for true
    * - ``showLocationLabels``
      - int
      - Value of 0 (protobuffer default) to use viz default, -1 for false, 1 for true


Setting Actuator GUI Options
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

To specify the actuator GUI settings use the ``setActuatorGuiSetting``
helper method in Python. An example is::

   vizSupport.setActuatorGuiSetting(viz, viewRWPanel=True, viewRWHUD=True)

The following table includes the keyword options for this method.

.. list-table:: GUI Parameter Options
    :widths: 10 10 20 100
    :header-rows: 1

    * - Variable
      - Type
      - Required
      - Description
    * - ``viewThrusterPanel``
      - Boolean
      - No
      - Show the thruster panel
    * - ``viewThrusterHUD``
      - Boolean
      - No
      - Show the thruster particle streams
    * - ``showThrusterLabels``
      - Boolean
      - No
      - Show the thruster labels
    * - ``viewRWPanel``
      - Boolean
      - No
      - Show the reaction wheel panel
    * - ``viewRWHUD``
      - Boolean
      - No
      - Show the reaction wheel disks configuration outside the spacecraft
    * - ``showRWLabels``
      - Boolean
      - No
      - Show the reaction wheel labels
    * - ``spacecraftName``
      - string
      - No, sc name default
      - Specify which spacecraft should show actuator information. If not provided then
        the ``viz.spacecraftName`` is used.

Setting Instrument GUI Options
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

To specify the instrument GUI settings use the ``setInstrumentGuiSetting``
helper method in Python. An example is::

   vizSupport.setInstrumentGuiSetting(viz, viewCSSPanel=True, viewCSSCoverage=True)

The following table includes the keyword options for this method.

.. list-table:: GUI Parameter Options
    :widths: 10 10 20 100
    :header-rows: 1

    * - Variable
      - Type
      - Required
      - Description
    * - ``viewCSSPanel``
      - Boolean
      - No
      - Show the CSS panel
    * - ``viewCSSCoverage``
      - Boolean
      - No
      - Show the CSS coverage spheres
    * - ``viewCSSBoresight``
      - Boolean
      - No
      - Show the CSS boresight axes
    * - ``showCSSLabels``
      - Boolean
      - No
      - Show the CSS labels
    * - ``spacecraftName``
      - string
      - No, sc name default
      - Specify which spacecraft should show actuator information. If not provided then
        the ``viz.spacecraftName`` is used.


Defining a Pointing Line
~~~~~~~~~~~~~~~~~~~~~~~~

Vizard can create a heading line from one object to another. For
example, it might be handy to create a line from the spacecraft pointing
towards the sun direction, or from the spacecraft towards Earth to know
how the antennas should point. These pointing lines can be scripted from
Basilisk as well using using a helper function ``createPointLine()``:

.. code-block::

    viz = vizSupport.enableUnityVisualization(scSim, simTaskName, scObject, saveFile=fileName)
    vizSupport.createPointLine(viz, toBodyName='earth_planet_data', lineColor=[0, 0, 255, 255])
    vizSupport.createPointLine(viz, toBodyName=“sun_planet_data”, lineColor=“yellow”)]

The ``createPointLine`` support macro requires the parameters ``toBodyName`` and ``lineColor`` to be
defined. The parameter ``fromBodyName`` is optional. If it is not
specified, then the ``viz.spacecraftName`` is used as a default origin.
The ``lineColor`` state can be either a string containing the color
name, or a list containing RGBA values. The support macro converts this
into the required set of numerical values.

Each pointing line message contains the three variables listed in the
next table.

.. list-table:: Pointing Line Parameter Options
    :widths: 10 10 10 100
    :header-rows: 1

    * - Variable
      - Type
      - Required
      - Description
    * - ``fromBodyName``
      - string
      - No, sc name default
      - contains the name of the originating body
    * - ``toBodyName``
      - string
      - Yes
      - contains the name of the body to point towards
    * - ``lineColor``
      - int(4)
      - Yes
      - color name or array on integer values specifying the RGBA values between 0 to 255


Defining Keep In/Out Cones
~~~~~~~~~~~~~~~~~~~~~~~~~~

Vizard can create cones relative to the spacecraft which illustrated if
a body axis is within some angle to the sun (i.e. keep in cone), or if a
sensor axis is outside some angle to the sun (i.e. keep out cone). These
cones can be setup in Vizard, but can also be scripted from Basilisk
using the helper function ``createConeInOut``:

.. code-block::
	
	viz = vizSupport.enableUnityVisualization(scSim, simTaskName, scObject, saveFile=fileName)
	vizSupport.createConeInOut(viz, toBodyName='earth', coneColor='teal',
                               normalVector_B=[1, 0, 0], incidenceAngle=30\ macros.D2R, isKeepIn=True,
                               coneHeight=5.0, coneName=‘sensorCone’)
	vizSupport.createConeInOut(viz,toBodyName='earth', coneColor='blue', normalVector_B=[0, 1, 0],
                               incidenceAngle=30\ macros.D2R, isKeepIn=False, coneHeight=5.0, coneName=‘comCone’)]
	
The following table illustrates the
arguments for the ``createConeInOut`` method:

.. list-table:: Keep In/Out Cones Parameter Options
    :widths: 20 10 10 10 100
    :header-rows: 1

    * - Variable
      - Type
      - Units
      - Required
      - Description
    * - ``isKeepIn``
      - bool
      -
      - Yes
      - make cone keep in (True) or keep out (False)
    * - ``fromBodyName``
      - string
      -
      - No, sc name default
      - contains the name of the originating body
    * - ``toBodyName``
      - string
      -
      - Yes
      - contains the name of the body to point towards
    * - ``lineColor``
      - int(4)
      -
      - Yes
      - color name or array on integer values specifying the RGBA  values between 0 to 255
    * - ``position_B``
      - float(3)
      - m
      - No, (0,0,0) default
      - position of the cone vertex
    * - ``normalVector_B``
      - float(3)
      - m
      - Yes
      - normal axis of the cone in body frame components
    * - ``incidenceAngle``
      - float
      - rad
      - Yes
      - angle of the cone
    * - ``incidenceAngle``
      - float
      - rad
      - Yes
      - height of the cone
    * - ``coneName``
      - string
      -
      - No
      - cone label name, if unspecified viz will autogenerate name


Defining the Vizard Camera View Panels
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Vizard can create a spacecraft relative camera panel. This functionality can be
controlled by using the ``createStandardCamera`` helper method.  The camera can
point in a body-fixed direction (``setMode=1``), or be aimed at a celestial target
(``setMode=0``).  Multiple camera panels can be setup at the same time, and
they can be attached to different spacecraft through the ``spacecraftName`` argument.

.. code-block:: python

	viz = vizSupport.enableUnityVisualization(scSim, simTaskName, scObject, saveFile=fileName)
	vizSupport.createStandardCamera(viz, setMode=0, bodyTarget='earth', setView=0)
	vizSupport.createStandardCamera(viz, setMode=1, fieldOfView=60.*macros.D2R, pointingVector_B=[0.0, -1.0, 0.0])


The following table illustrates
the arguments for the ``createStandardCamera`` method.

.. list-table:: Standard Camera View Panel Parameter Options
    :widths: 15 10 10 15 50
    :header-rows: 1

    * - Variable
      - Type
      - Units
      - Required
      - Description
    * - ``spacecraftName``
      - string
      -
      - No, sc name default
      - name of the spacecraft with respect to which the camera is shown
    * - ``setMode``
      - int
      -
      - No, default is 1
      - 0 -> body targeting, 1 -> pointing vector
    * - ``setView``
      - int
      -
      - No, default is 0
      - 0 -> Nadir, 1 -> Orbit Normal, 2 -> Along Track (default to nadir). This is a setting for body targeting mode.
    * - ``bodyTarget``
      - string
      -
      - No, default to first celestial body in messages
      - Name of body camera should point to. This is a setting for body targeting mode.
    * - ``fieldOfView``
      - float
      - rad
      - No, default is -1
      - camera edge-to-edge field of view in the camera vertical ``y`` axis, to use the Vizard default set it to -1
    * - ``pointingVector_B``
      - float(3)
      -
      - No, default is (0,0,0) for auto placement
      - Name of body camera should point to. This is a setting for pointing vector mode
    * - ``position_B``
      - float(3)
      - m
      - No, default is (0,0,0) for auto placement
      - If populated, ets camera  position relative to parent body coordinate frame in meters using B frame
        components.  If unpopulated camera is positioned automatically along camera view direction outside
        of parent body's mesh to prevent obstruction of view.

.. image:: /_images/static/vizard-ImgCustomCam.jpg
   :align: center
   :width: 90 %

It is also possible to create a custom instrument camera view for opNav mode which points in an
arbitrary direction as illustrate in the image above. The following
helper method is an example of how such an instrument camera view can be
created::

   vizSupport.createCameraConfigMsg(viz, cameraID=1, fieldOfView=10 * macros.D2R,
                                        resolution=[1024, 1024], renderRate=0.1,
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

.. list-table:: ``createCameraConfigMsg`` Parameter Options
    :widths: 15 10 10 15 100
    :header-rows: 1

    * - Variable
      - Type
      - Units
      - Required
      - Description
    * - ``cameraID``
      - int
      -
      - Yes
      - ID of the Vizard camera
    * - ``parentName``
      - string
      -
      - No, sc name default
      - name of the spacecraft with respect to which the camera is shown
    * - ``fieldOfView``
      - float
      - rad
      - yes
      - edge-to-edge field of view in the camera vertical ``y`` axis
    * - ``resolution``
      - int(2)
      -
      - yes
      - image sensor pixels
    * - ``renderRate``
      - float
      -
      - yes
      - time between image grabs. 0 turns this off (default).
    * - ``cameraPos_B``
      - float(3)
      - m
      - yes
      - camera  location relative to body frame in B frame components
    * - ``sigma_CB``
      - float(3)
      -
      - yes
      - camera orientation relative to the body frame in MRPs
    * - ``skyBox``
      - string
      -
      - No
      - Used to determine what star background should be shown. The empty string "" provides default NASA
        SVS Starmap, “ESO”  shows the ESO Milky Way skybox, “black” provides a black background, or the
        user can provide a filepath to custom  background image file.


Defining the Custom Spacecraft Shape model
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

You can specify a custom OBJ model to be used with Vizard spacecraft representation.
An sample is shown in the following screen capture.

.. image:: /_images/static/vizard-ImgCustomCAD.jpg
   :align: center
   :scale: 80 %

This functionality can be controlled by using the ‘createCustomModel’ helper method.

.. code-block::

	viz = vizSupport.enableUnityVisualization(scSim, simTaskName, scObject, saveFile=fileName)
	vizSupport.createCustomModel(viz,
	                            modelPath="/Users/hp/Downloads/Topex-Posidon/Topex-Posidon-composite.obj",
	                            scale=[2, 2, 10])


The following table illustrates the arguments for the ``createCustomModel`` method.

.. list-table:: Custom Space Object OBJ Import Parameter Options
    :widths: 15 10 10 15 50
    :header-rows: 1

    * - Variable
      - Type
      - Units
      - Required
      - Description
    * - ``modelPath``
      - string
      -
      - Yes
      - Path to model obj -OR- "CUBE", "CYLINDER", or "SPHERE" to use a primitive shape
    * - ``simBodiesToModify``
      - string
      -
      - No, default is `bsk-Sat`
      - Which bodies in scene to replace with this model, use "ALL_SPACECRAFT" to apply custom model to
        all spacecraft in simulation
    * - ``offset``
      - float(3)
      - m
      - No, default is (0,0,0)
      - offset to use to draw the model
    * - ``rotation``
      - float(3)
      - rad
      - No, default is (0,0,0)
      - 3-2-1 Euler angles to rotate CAD about z, y, x axes
    * - ``scale``
      - float(3)
      -
      - No, default is (1,1,1)
      - desired model scale in x, y, z in spacecraft CS
    * - ``customTexturePath``
      - String
      -
      - No
      - Path to texture to apply to model (note that a custom model's .mtl will be automatically imported with
        its textures during custom model import).  The image file types supported are: jpg, bmp, exr,
        gif, hdr, iff, pict, png, psd, tga, and tiff. The maximum image dimensions supported for runtime
        import are 16384 pixels by 16384 pixels. The image does not have to be square.
    * - ``normalMapPath``
      - string
      -
      - No
      - Path to the normal map for the customTexture
    * - ``shader``
      - int
      -
      - No, default is -1
      - Value of -1 to use viz default, 0 for Unity Specular Standard Shader, 1 for Unity Standard Shader


Specifying the Spacecraft Sprite Representation
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
In the spacecraft centric view a 3D model is rendered of the spacecraft.  However, in planet and heliocentric views
the spacecraft is automatically represented as a 2D sprite (circle, triangle, etc.) if more than one
spacecraft is being simulated.  The default sprite shape for all spacecraft can be set through the
``defaultSpacecraftSprite`` value discussed above.  To specify a specific sprite shape, and optional color, for a
specific spacecraft this can be done by setting the string variable ``spacecraftSprite`` inside the
spacecraft data structure.

The example scenario :ref:`scenarioFormationBasic` illustrates how to simulate multiple spacecraft.  To make
a spacecraft use a specific sprite representation use::

    scData.spacecraftSprite = vizSupport.setSprite("STAR")



Specifying the Simulation Epoch Date and Time Information
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Vizard can show the both the simulation time that has elapsed, or the mission time.  If now epoch message has been
set then Basilisk assumes a default January 1, 2019, 00:00:00 epoch time and date.  The simulation time elapsed is
thus the time since epoch.  To specify a different simulation epoch data and time the :ref:`EpochMsgPayload` can be
setup as discussed in :ref:`scenarioMagneticFieldWMM`.  To tell :ref:`vizInterface` what epoch message to read use::

	viz.epochInMsg.subscribe(epochMsg)

An example of the use of this epoch message is shown in :ref:`scenarioMagneticFieldWMM`.


Specifying Reaction Wheel (RW) Information
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
The simplest method to include the RW states of a one more spacecraft in the Vizard data file is to
call ``vizSupport.enableUnityVisualization()`` with the additional argument::

    rwEffectorList=rwStateEffector

Here ``rwStateEffector`` is an instance of a single :ref:`ReactionWheelStateEffector` which already has all
the spacecraft's RW devices added to it.  If you have multiple spacecraft, then use a list of RW effectors,
one effector per spacecraft::

    rwEffectorList=[rwStateEffector1, rwStateEffector2]

This method is illustrated in the :ref:`scenarioAttitudeFeedbackRW` script.  Note that this list must contain
one entry per spacecraft.  If a spacecraft has no RW devices, then add ``None`` instead of an effector instance.

If custom RW state output messages are used, then the ``scData.rwInMsgs`` can be specified directly.  This case
is employed in the test script :ref:`test_dataFileToViz`.

Specifying Thruster Information
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
The simplest method to include the clusters of thrusters of a one more spacecraft in the Vizard data file is to
call ``vizSupport.enableUnityVisualization()`` with the additional argument::

    thrEffectorList=thrusterSet

Here ``thrusterSet`` is an instance of a single :ref:`ThrusterDynamicEffector` which already has all
the spacecraft's THR devices added to this one THR cluster.  If you have multiple spacecraft, or a spacecraft
has multiple clusters of THR devices such as ACS and DV thrusters, then use a double list of THR effectors.
The outer list has one entry per spacecraft, and the inner list has one entry per spacecraft THR cluster::

    thrEffectorList=[[thrusterSet1Sc1, thrusterSet2Sc1], [thrusterSet1Sc2]]

The outer list must have one THR cluster list per spacecraft.  If a spacecraft has no THR devices, then
add ``None`` instead of this cluster list.
The illustration of thrusters is shown in the example script :ref:`scenarioAttitudeFeedback2T_TH`.

Note that if the maximum force of a thruster is less than 0.01N (i.e. a micro-thruster),
then the plume length is held the same as with a 0.01N thruster.
Otherwise the micro-thruster plumes would not be visible.

If you want to change the thruster plume illustration color, then you can use the optional argument::

    thrColors=vizSupport.toRGBA255("red")

This example is for a single spacecraft.  If you have multiple spacecraft this must again be wrapped in a list
of lists as above.  The inner list is the color you want to for each cluster.  Thus, its dimension must match the
``thrEffectorList`` double list dimension.  If you want to keep the default color for a spacecraft then
add ``None`` as the cluster color.

The thruster information for each spacecraft can also be set directly by specifying ``sc.thrInMsgs`` and
``sc.thrInfo`` directly as demonstrated in :ref:`test_dataFileToViz`.

Adding Location or Communication Stations
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
The :ref:`groundLocation` is able to simulate a location on a celestial body like Earth.
The location can also be fixed to a satellite.  Vizard will show a line between a satellite
and this location including if the satellite is within the
field of view of this location.  Vizard can illustrate this ground location using the
``addLocation()`` method, such as::

    vizSupport.addLocation(viz, stationName="Boulder Station"
                           , parentBodyName='earth'
                           , r_GP_P=groundStation.r_LP_P_Init
                           , fieldOfView=np.radians(160.)
                           , color='pink'
                           , range=1000.0
                           )

The following table lists all required and optional arguments that can be provided to ``addLocation``:

.. list-table:: Location Parameter Options
    :widths: 20 10 10 10 100
    :header-rows: 1

    * - Variable
      - Type
      - Units
      - Required
      - Description
    * - ``stationName``
      - string
      -
      - Yes
      - Label of the ground location
    * - ``parentBodyName``
      - string
      -
      - Yes
      - name of the planet object
    * - ``r_GP_P``
      - float(3)
      - m
      - Yes
      - position vector of the location G relatiave to parent body (planet or spacecraft) frame P in P frame components
    * - ``gHat_P``
      - float(3)
      -
      - No
      - normal vector of the location station boresight, default is unit vector of ``r_GP_P``
    * - ``fieldOfView``
      - float
      - rad
      - No
      - edge-to-edge location station field of view, default is :math:`\pi`
    * - ``color``
      - int(4)
      -
      - No
      - specify the location station color using RGBA value of 0-255
    * - ``range``
      - double
      - m
      - No
      - range of the location station, use 0 or negative value (protobuffer default) to use viz default
