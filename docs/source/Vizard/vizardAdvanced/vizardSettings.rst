
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
for a multi-satellite simulation:

.. code-block:: python

	viz = vizSupport.enableUnityVisualization(scSim, simTaskName, [scObject1, scObject2, scObject3]
	                                          , saveFile=fileName)

The spacecraft names are pulled from ``scObject.ModelTag``.
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

.. list-table:: Vizard Simulation Parameters Read at Start Up
    :widths: 10 10 80
    :header-rows: 1

    * - Variable
      - Type
      - Description
    * - ``ambient``
      - [0,1]
      - float value to specify the ambient Vizard lighting.
    * - ``orbitLinesOn``
      - (-1,0,1,2)
      - Toggle to show osculating orbit lines, Value of 0 (protobuffer default) to use viz default,
        -1 for false, 1 for relative to parent body, 2 for relative to chief spacecraft body
    * - ``trueTrajectoryLinesOn``
      - (-1,0,1,2)
      - Toggle to show true orbit lines, Value of 0 (protobuffer default) to use viz default,
        -1 for false, 1 to use inertial positions, 2 for relative to chief spacecraft body
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
    * - ``mainCameraTarget``
      - string
      - If valid spacecraft or celestial body name is provided, the main camera will be targeted at
        that body at start
    * - ``spacecraftShadowBrightness``
      - double
      - Control the ambient light specific to spacecraft objects, value between 0 and 1, use negative value
        to use viz default
    * - ``spacecraftSizeMultiplier``
      - double
      - Control the display size of spacecraft in the Planet and Solar System Views, values greater than 0,
        use negative value to use viz default
    * - ``spacecraftHelioViewSizeMultiplier``
      - double
      - Control the display size of spacecraft in the Solar System View, values greater than 0, use negative
        value to use viz default
    * - ``forceStartAtSpacecraftLocalView``
      - int
      - Require Vizard to start up in spacecraft-view on start up
    * - ``showLocationCommLines``
      - int
      - Value of 0 (protobuffer default) to use viz default, -1 for false, 1 for true
    * - ``showLocationCones``
      - int
      - Value of 0 (protobuffer default) to use viz default, -1 for false, 1 for true
    * - ``showLocationLabels``
      - int
      - Value of 0 (protobuffer default) to use viz default, -1 for false, 1 for true
    * - ``useSimpleLocationMarkers``
      - int
      - Value of 0 (protobuffer default) to use simplified Location markers when number
        of locations is greater than 100, -1 to force use of full-featured Location, 1 to
        force use of simplified Location (no cones, range, or communication lines)
    * - ``atmospheresOff``
      - int
      - Toggle to disable the atmosphere effect on celestial bodies, Value of 0 (protobuffer default to use
        viz default, -1 for false, 1 for true.
    * - ``scViewToPlanetViewBoundaryMultiplier``
      - int
      - Multiplier x 1000m to set the boundary at which the spacecraft local view transitions to planet view.
        Valid range from 1 to 10 or 0 to use viz default.
    * - ``planetViewToHelioViewBoundaryMultiplier``
      - int
      - Multiplier x (10000 * current planet local scale) at which the planet view transitions to the solar
        system view. Valid range from 1 to 10 or 0 to use viz default.
    * - ``sunIntensity``
      - double
      - Multiplier for the intensity of the light being used as the main light source or sun, value of 0 to use
        viz default
    * - ``attenuateSunLightWithDistance``
      - int
      - Toggle to reduce brightness of sun lighting with the square of the distance from the sun.
        Value of 0 (protobuffer default) to use viz default, -1 for false, 1 for true.
    * - ``showLightLabels``
      - int
      - Toggle to label spacecraft light elements, Value of 0 (protobuffer default) to use viz
        default, -1 for false, 1 for true
    * - ``celestialBodyHelioViewSizeMultiplier``
      - double
      - Control the display size of celestial bodies in the Solar System View,
        values greater than 0, use negative value to use viz default.
        Default value is -1 to use Vizard default value.
    * - ``showMissionTime``
      - int
      - flag to show the mission time instead of the simulation time. Value of 0 (protobuffer default)
        to use viz default, -1 for false, 1 for true
    * - ``keyboardLiveInput``
      - string
      - string of alphanumeric key inputs to listen for during 2-way communication    * - ``messageBufferSize``
    * - ``messageBufferSize``
      - int
      - [bytes] Maximum size of vizMessages to be loaded into memory at one time,
        -1 to force loading of entire file into memory, 0 to use viz default


While the prior settings are only read once during start up, the following settings are checked
with every message being sent.  The following live settings can be set directly using::

    viz.liveSettings.variableName = value

.. list-table:: Vizard Simulation Parameters Read Live
    :widths: 10 10 80
    :header-rows: 1

    * - Variable
      - Type
      - Description
    * - ``targetLineList``
      - ``std::vector<PointLine>``
      - vector of lines between 2 scenario targets.  This list is redrawn on each update step,
        thus the line properties can change with time.
    * - ``relativeOrbitChief``
      - string
      - If valid spacecraft name provided, the relative orbit chief spacecraft will be set to that spacecraft
        object. Setting the string to ``AUTO`` or leaving this field empty will select the camera target
        spacecraft as the chief.

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
    * - ``showGenericSensorLabels``
      - Boolean
      - No
      - Value of 0 (protobuffer default) to use viz default, -1 for false, 1 for true
    * - ``showTransceiverLabels``
      - Boolean
      - No
      - Value of 0 (protobuffer default) to use viz default, -1 for false, 1 for true
    * - ``showTransceiverFrustrum``
      - Boolean
      - No
      - Value of 0 (protobuffer default) to use viz default, -1 for false, 1 for true
    * - ``showGenericStoragePanel``
      - Boolean
      - No
      - Value of 0 (protobuffer default) to use viz default, -1 for false, 1 for true
    * - ``showMultiSphereLabels``
      - Boolean
      - No
      - Value of 0 (protobuffer default) to use viz default, -1 for false, 1 for true


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
    * - ``displayName``
      - string
      -
      - No, Default is ``Standard Camera``
      - (optional) name that is used to label the camera window

.. image:: /_images/static/vizard-ImgCustomCam.jpg
   :align: center
   :width: 90 %

It is also possible to create one or more custom instrument camera view for ``opNav`` mode which points in an
arbitrary direction as illustrate in the image above. Such a camera can be created first creating the camera
configuration message ``camMsg`` of type :ref:`CameraConfigMsgPayload` and then adding this to :ref:`vizInterface`
through the command::

    viz = vizSupport.enableUnityVisualization(scSim, simTaskName, scObject)
    viz.addCamMsgToModule(camMsg)

If ``addCamMsgToModule()`` is called multiple times then multiple Vizard instrument cameras are created.

The following helper method is an example of how such an instrument camera message can be
created directly and added to :ref:`vizInterface` directly::

   vizSupport.createCameraConfigMsg(viz, cameraID=1, fieldOfView=10 * macros.D2R,
                                        resolution=[1024, 1024], renderRate=macros.sec2nano(10),
                                        cameraPos_B=[0.2, 0.1, 0.3], sigma_CB=[-1./3., 1./3., -1./3.])

Note that with this instrument camera Vizard will save off images at the user home folder at the rate
specified in ``renderRate``.  To avoid saving off images just make ``renderRate`` zero.

The camera frame is illustrated in the following image.  It uses classical image image coordinates where ``x`` points
to the right, ``y`` point downwards and ``z`` points outward.  More information is availabe in section 2.4.1 of
Dr. Teil's `dissertation <http://hanspeterschaub.info/Papers/grads/ThibaudTeil.pdf>`_.

.. image:: /_images/static/imageFrame.jpg
   :align: center
   :width: 600px

The following table illustrates the possible variables for the
``createCameraConfigMsg()`` method.

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
      - [ns]
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
    * - ``postProcessingOn``
      - int
      -
      - needed for any ``ppXXX`` parameters to work
      - flag to turn on any post-processing features of the camera.  Values are 0 (default) or 1.
    * - ``ppFocusDistance``
      - double
      -
      - No
      - Distance to the point of focus
    * - ``ppAperture``
      - double
      -
      - No
      - Ratio of the aperture (known as f-stop or f-number). The smaller the value is, the shallower the depth of field is.
    * - ``ppFocalLength``
      - double
      - m
      - No
      - Valid setting range: 0.001m to 0.300m.
    * - ``ppMaxBlurSize``
      - int
      -
      - No
      - Convolution kernel size of the bokeh filter, which determines the maximum radius of bokeh.
    * - ``updateCameraParameters``
      - int
      -
      - No
      - If true, commands camera to update Instrument Camera to current message's parameters
    * - ``renderMode``
      - int
      -
      - No
      - Value of 0 to render visual image (default), value of 1 to render depth buffer to image
    * - ``depthMapClippingPlanes``
      - double(2)
      - m
      - No
      - Set the bounds of rendered depth map by setting the near and far clipping planes when
        in renderMode=1 (depthMap mode). Default values of 0.1 and 100.


If the ``renderMode`` is set to 1 the camera outputs a depth map.
Depth maps rendered by an Instrument Camera utilize Unity’s ``Linear01Depth`` shader helper macro inside
Vizard’s DepthMap shader. The macro linearizes the non-linear internal depth texture whose precision
is configuration and platform dependent to return a value between 0 and 1 where 1 is the maximum depth.
Vizard’s DepthMap shader takes the value returned and encodes it as an RGB color. The far clipping plane
of the Instrument Camera determines the maximum depth of the rendered texture and can be set as part of
the camera configuration.

.. warning::

    The internal depth texture values are more accurate for objects closer to the camera. Error in
    the calculated depth increases with distance from the camera.


To decode the depth for a specific pixel, sample its RGB color values :math:`(r,g,b)` and calculate the depth as as:

.. math::

    depth = (farClippingPlane)(\frac{\frac{\frac{b} {256} +g} {256} + r} {256})

If the depth is equal to or greater than the far clipping plane of the instrument camera,
the pixel color will be white (255, 255, 255).





Defining the Custom Spacecraft Shape model
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

You can specify a custom OBJ model to be used with Vizard spacecraft representation.
An sample is shown in the following screen capture.

.. image:: /_images/static/vizard-ImgCustomCAD.jpg
   :align: center
   :scale: 80 %

This functionality can be controlled by using the ``createCustomModel()`` helper method.

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
    * - ``color``
      - int(4)
      -
      - No
      - RGBA as values between 0 and 255, default is gray, and will be applied to the albedo color setting


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

    viz = vizSupport.enableUnityVisualization(scSim, simTaskName, scObject
                                              , spriteList=vizSupport.setSprite("STAR", color="red")
                                              , saveFile=fileName,
                                              )

If you are using multiple spacecraft, then the sprite information list must have the same length as
the number of spacecraft::

    viz = vizSupport.enableUnityVisualization(scSim, simTaskName, [scObject, scObject2, scObject3]
                                              , spriteList=[None, vizSupport.setSprite("STAR", color="red"), None]
                                              , saveFile=fileName,
                                              )


Specifying the Simulation Epoch Date and Time Information
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Vizard can show the both the simulation time that has elapsed, or the mission time.  If now epoch message has been
set then Basilisk assumes a default January 1, 2019, 00:00:00 epoch time and date.  The simulation time elapsed is
thus the time since epoch.  To specify a different simulation epoch data and time the :ref:`EpochMsgPayload` can be
setup as discussed in :ref:`scenarioMagneticFieldWMM`.  To tell :ref:`vizInterface` what epoch message to read use::

	viz.epochInMsg.subscribeTo(epochMsg)

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

Specifying CSS Information
~~~~~~~~~~~~~~~~~~~~~~~~~~
To include a cluster clusters of CSS sensors to the spacecraft,
call ``vizSupport.enableUnityVisualization()`` with the additional argument::

    cssList=[cssDeviceList]

Here ``cssDeviceList`` is a list of :ref:`CoarseSunSensor` objects.  The length of ``cssDeviceList``
must match the number of spacecraft being modeled.  If a spacecraft has no CSS devices, then use
the ``None`` label.  See :ref:`scenarioCSS` for an example of CSS devices being visualized in Vizard.


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

If you used the gravity factor to generate the planet states, you can pull the planet name
by using ``earth.displayName`` as shown in :ref:`scenarioAttLocPoint`.

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


Adding Generic Sensor Visualization
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Vizard can illustrate generic sensors in the 3d environments which have either a conical or rectangular field of
view.  For example, these sensors could be a camera, a star tracker or a fine sun sensor.   Instead of making a
specific visualization of such sensors, the generic sensor message allows a series of messages to be tied
to a spacecraft and be configured to look like either sensor type.  Further, an optional :ref:`DeviceCmdMsgPayload`
message can be provided for each sensor such that the sensor state (active, inactive, etc.) can be
visualized as well.

.. image:: /_images/static/vizard-ImgGenSensor.jpg
   :align: center
   :scale: 80 %

First, let's discuss how to setup a generic sensor.  The associated sensor structure and the required
parameters are set using::

    genericSensor = vizInterface.GenericSensor()
    genericSensor.r_SB_B = [1., 1.0, 1.0]
    genericSensor.fieldOfView = [20.0 * macros.D2R, -1]
    genericSensor.normalVector = [0., 0., 1.]

.. caution::
    As a pointer to the ``GenericSensor`` structure is connected to :ref:`vizInterface`, it is
    important that the python structure is retained in memory.  If the python structure instance
    is created in a manner where this is not the case, use ``genericSensor.this.disown()``
    to ensure the python structure remains intact throughout the simulation.

The sensor location relative to the spacecraft B frame is given by ``r_SB_B``.  The sensor view axis is
set through ``normalVector``.  The ``fieldOfView`` is a vector with up to 2 floats.  If a single positive value is provided,
then the sensor shape is a cone with this edge-to-edge field of view.  If 2 positive floats are provided
then the sensor shape is a rectangle.
The full list of required and optional generic sensor parameters are provided in the following table.

.. list-table:: Generic Sensor Configuration Options
    :widths: 20 10 10 10 100
    :header-rows: 1

    * - Variable
      - Type
      - Units
      - Required
      - Description
    * - ``r_SB_B``
      - double[3]
      - m
      - Yes
      - sensor location relative to body frame in body frame components
    * - ``normalVector``
      - double[3]
      -
      - Yes
      - sensor view axis
    * - ``fieldOfView``
      - vector<float>
      - rad
      - Yes
      - edge-to-edge field of view of cone (single positive float) or rectangle (two positive floats)
    * - ``isHidden``
      - bool
      -
      - No
      - argument to hide the sensor visualization.  Default value is ``False``
    * - ``size``
      - double
      - m
      - No
      - Value of 0 (protobuffer default) to show HUD at viz default size
    * - ``label``
      - string
      -
      - No
      - string to display on sensor label
    * - ``color``
      - vector<int>
      -
      - No
      - Send desired RGBA as values between 0 and 255, multiple colors can be populated in this
        field and will be assigned to the additional mode (Modes 0 and 1 will use the 0th color, Mode 2
        will use the color indexed to 1, etc.  If the mode number exceeds the number of colors provided
        then the default color is used again.
    * - ``genericSensorCmd``
      - int
      -
      - No
      - set the sensor command state from python.  Note that this value is replaced with the value from
        the sensor cmd input message if such an input message is provided.
    * - ``genericSensorCmdInMsg``
      - ReadFunctor<:ref:`DeviceCmdMsgPayload`>
      -
      - No
      - sensor cmd input message

Thus, to setup a sensor that uses red to display the location, orientation and status, you could use::

    genericSensor = vizInterface.GenericSensor()
    genericSensor.r_SB_B = [1., 1.0, 1.0]
    genericSensor.fieldOfView.push_back(20.0 * macros.D2R)
    genericSensor.fieldOfView.push_back(25.0 * macros.D2R)
    genericSensor.normalVector = [0., 0., 1.]
    genericSensor.color = vizInterface.IntVector(vizSupport.toRGBA255("red"))
    genericSensor.label = "genSen1"

Note that here a rectangular 20x25 degree field of view is specified.  To add a conical 20 degree field of view,
then a single angle should be provided.

Next, each sensor can be connected to the optional device status message of type :ref:`DeviceCmdMsgPayload`::

    cmdInMsg = messaging.DeviceCmdMsgReader()
    cmdInMsg.subscribeTo(simpleInsControlConfig.deviceCmdOutMsg)
    genericSensor.genericSensorCmdInMsg = cmdInMsg

The sensor command state can also be set directly from python using::

    genericSensor.genericSensorCmd = 1

However, if the input message is specified then this value is replaced with the content of the input message.

Multiple generic sensors can be created for each spacecraft, and multiple spacecraft are supported.  Using
the ``vizSupport.py`` file, the sensors are sent to :ref:`vizInterface` using they keyword ``genericSensorList``::

    viz = vizSupport.enableUnityVisualization(scSim, simTaskName, scObject
                                              , saveFile=fileName
                                              , genericSensorList=genericSensor
                                              )

Note that here a single sensor and spacecraft is setup.  If you have multiple sensors, or multiple spacecraft,
then lists of lists are required::

    viz = vizSupport.enableUnityVisualization(scSim, simTaskName, [scObject, scObject2]
                                              , saveFile=fileName
                                              , genericSensorList=[ [genericSensor1], [genericSensor2, genericSensor3] ]
                                              )

If the sensor has multiple activity types, such as taking a red, green, and blue color image, the :ref:`DeviceCmdMsgPayload`
message can have several positive command states.  These distinct activity states can be visualized using multiple colors.
For example, to use ``red`` for state 1, ``green`` for state 2, you could use::

    genericSensor.color = vizInterface.IntVector(vizSupport.toRGBA255("red") + vizSupport.toRGBA255("green"))

See :ref:`scenarioGroundLocationImaging` for an example of using the generic sensor visualization.



Adding Transceiver Visualization
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Vizard can illustrate the state of an antenna in the 3d environments.  The state can be either off (default),
transmitting, receiving, or transmitting and receiving. The antenna communication state is dynamically
set through an optional :ref:`DataNodeUsageMsgPayload`
message.  Note that this message contains a baud rate variable which dictactes if the module is transmitting
data (negative baud rate) or receiving data (positive baud rate).  Thus, a single transceiver HUD element
can connect to a vector of :ref:`DataNodeUsageMsgPayload` input messages.  These messages are scaned if they are
transmitting, receiving or doing mix, and the transceiver state is set accordingly.

.. image:: /_images/static/vizard-ImgTransceiver.jpg
   :align: center
   :scale: 80 %

First, let's discuss how to setup a transceiver HUD element.  The associated structure and the required
parameters are set using::

    transceiverHUD = vizInterface.Transceiver()
    transceiverHUD.r_SB_B = [0., 0., 1.]
    transceiverHUD.fieldOfView = 40.0 * macros.D2R
    transceiverHUD.normalVector = [0., 0., 1.]

.. caution::
    As a pointer to the ``Transceiver`` structure is connected to :ref:`vizInterface`, it is
    important that the python structure is retained in memory.  If the python structure instance
    is created in a manner where this is not the case, use ``transceiverHUD.this.disown()``
    to ensure the python structure remains intact throughout the simulation.

The transceiver location relative to the spacecraft B frame is given by ``r_SB_B``.  The transceiver
bore sight axis is
set through ``normalVector``.  The ``fieldOfView`` sets the edge-to-edge field of view of this antenna communication
process.
The full list of required and optional transceiver parameters are provided in the following table.

.. list-table:: Transceiver Configuration Options
    :widths: 20 10 10 10 100
    :header-rows: 1

    * - Variable
      - Type
      - Units
      - Required
      - Description
    * - ``r_SB_B``
      - double[3]
      - m
      - Yes
      - transceiver location relative to body frame in body frame components
    * - ``normalVector``
      - double[3]
      -
      - Yes
      - transceiver center axis
    * - ``fieldOfView``
      - float
      - rad
      - Yes
      - edge-to-edge field of the antenna communication access cone
    * - ``isHidden``
      - bool
      -
      - No
      - argument to hide the transceiver visualization.  Default value is ``False``
    * - ``animationSpeed``
      - int
      -
      - No
      - Set transmission animation speed to a value between 1(slowest) to 10 (fastest), or 0 to use viz default
    * - ``label``
      - string
      -
      - No
      - string to display on transceiver label
    * - ``color``
      - vector<int>
      -
      - No
      - Send desired RGBA as values between 0 and 255.
    * - ``transceiverState``
      - int
      -
      - No
      - set the transceiver state from python.  This can be 0 (off), 1 (sending), 2 (receiving) and
        3 (sending and receiving).  Note that this value is replaced with the value from
        the transceiver state input message if such an input message is provided.
    * - ``transceiverStateInMsgs``
      - vector<ReadFunctor<:ref:`DataNodeUsageMsgPayload`>>
      -
      - No
      - vector of transceiver communication state message(s)

Thus, to setup a sensor that uses yellow to display the location, orientation and status, you could use::

    transceiverHUD = vizInterface.Transceiver()
    transceiverHUD.r_SB_B = [0., 0., 1.]
    transceiverHUD.fieldOfView = 40.0 * macros.D2R
    transceiverHUD.normalVector = [0., 0., 1.]
    transceiverHUD.color = vizInterface.IntVector(vizSupport.toRGBA255("yellow", alpha=0.5))
    transceiverHUD.label = "antenna"

Next, each sensor can be connected to the optional device status message of type :ref:`DataNodeUsageMsgPayload`::

    trInMsg = messaging.DataNodeUsageMsgReader()
    trInMsg.subscribeTo(transmitter.nodeDataOutMsg)
    transceiverHUD.transceiverStateInMsgs.push_back(trInMsg)

The transceiver state can also be set directly from python using::

    transceiverHUD.transceiverState = 1

However, if the input message is specified then this value is replaced with the content of the input message.

Multiple transceiver HUD elements can be setup for each spacecraft, and multiple spacecraft are supported.  Using
the ``vizSupport.py`` file, the sensors are sent to :ref:`vizInterface` using they keyword ``genericSensorList``::

    viz = vizSupport.enableUnityVisualization(scSim, simTaskName, scObject
                                              , saveFile=fileName
                                              , transceiverList=transceiverHUD
                                              )

Note that here a single transceiver and spacecraft is setup.  If you have multiple sensors, or multiple spacecraft,
then lists of lists are required::

    viz = vizSupport.enableUnityVisualization(scSim, simTaskName, [scObject, scObject2]
                                              , saveFile=fileName
                                              , genericSensorList=[ None, [genericSensor2, genericSensor3] ]
                                              )

Here the first spacecraft has no transceiver, and the 2nd spacecraft has 2 transceivers.

See :ref:`scenarioGroundLocationImaging` for an example of using the generic sensor visualization.



Adding Storage Device Panel
~~~~~~~~~~~~~~~~~~~~~~~~~~~
Vizard can illustrate the state of storage devices (battery, data storage, fuel tank) in a 2D panel.  A
panel is create for each spacecraft, and the storage devices types are ordered with the panel.  Each
storage device state is illustrated on a horizontal bar chart.  Hovering over the bar yields a
popup with the current number.  Thus, it is possible to have a spacecraft with multiple data storage devices,
a large and small battery, as well as a single tank.

First, let's discuss how to setup a generic storage element.  The associated structure and the required
parameters are set using::

    hdDevicePanel = vizInterface.GenericStorage()
    hdDevicePanel.label = "Main Disk"

The full list of required and optional generic storage parameters are provided in the following table.

.. list-table:: Transceiver Configuration Options
    :widths: 20 10 10 10 100
    :header-rows: 1

    * - Variable
      - Type
      - Units
      - Required
      - Description
    * - ``label``
      - string
      -
      - Yes
      - Name of storage device
    * - ``currentValue``
      - float
      - variable
      - No
      - Current value of the storage device.  If this is not set, then a storage status message must
        be connected to set this value.
    * - ``maxValue``
      - float
      -
      - No
      - maximum absolute value of the storage device.  If this is not set, then a storage status message must
        be connected to set this value.
    * - ``units``
      - string
      -
      - No
      - Units of stored quantity, i.e. "bytes", "TB", "kg", etc.
    * - ``color``
      - vector<int>
      -
      - No
      - Send desired RGBA as values between 0 and 255, multiple colors can be populated in this field
        and will be used to color the bar graph between thresholds (i.e. the first color will be used
        between values of 0 and threshold 1, the second color will be used between threshold 1 and 2,...,
        the last color will be used between threshold n and the maxValue
    * - ``thresholds``
      - vector<int>
      -
      - No
      - set the transceiver state from python.  This can be 0 (off), 1 (sending), 2 (receiving) and
        3 (sending and receiving).  Note that this value is replaced with the value from
        the transceiver state input message if such an input message is provided.
    * - ``batteryStateInMsg``
      - ReadFunctor<:ref:`PowerStorageStatusMsgPayload`>
      -
      - No
      - incoming battery state msg, only connect one input message
    * - ``dataStorageStateInMsg``
      - ReadFunctor<:ref:`DataStorageStatusMsgPayload`>
      -
      - No
      - incoming data storage state msg, only connect one input message
    * - ``fuelTankStateInMsg``
      - ReadFunctor<:ref:`FuelTankMsgPayload`>
      -
      - No
      - incoming fuel tank state msg, only connect one input message


Thus, to setup a data storage device that uses blue if the storage state is less than 80%, and
orange if the storage is more than 80% full, you could use::

    hdDevicePanel = vizInterface.GenericStorage()
    hdDevicePanel.label = "Main Disk"
    hdDevicePanel.units = "bytes"
    hdDevicePanel.color = vizInterface.IntVector(vizSupport.toRGBA255("blue") + vizSupport.toRGBA255("orange"))
    hdDevicePanel.thresholds = vizInterface.IntVector([80])
    hdInMsg = messaging.DataStorageStatusMsgReader()
    hdInMsg.subscribeTo(dataMonitor.storageUnitDataOutMsg)
    hdDevicePanel.dataStorageStateInMsg = hdInMsg

.. caution::
    As a pointer to the ``GenericStorage`` structure is connected to :ref:`vizInterface`, it is
    important that the python structure is retained in memory.  If the python structure instance
    is created in a manner where this is not the case, use ``hdDevicePanel.this.disown()``
    to ensure the python structure remains intact throughout the simulation.

Multiple storage panel elements can be setup for each spacecraft, and multiple spacecraft are supported.  Using
the ``vizSupport.py`` file, the generic storage structures list is sent to :ref:`vizInterface` using they keyword ``genericStorageList``::

    viz = vizSupport.enableUnityVisualization(scSim, simTaskName, scObject
                                              , saveFile=fileName
                                              , genericStorageList=hdDevicePanel
                                              )

Note that here a single storage device and spacecraft is setup.  If you have multiple storage devices,
or multiple spacecraft, then lists of lists are required::

    viz = vizSupport.enableUnityVisualization(scSim, simTaskName, [scObject, scObject2]
                                              , saveFile=fileName
                                              , genericStorageList=[ None, [hdDevicePanel2, hdDevicePanel3] ]
                                              )

Here the first spacecraft has no transceiver, and the 2nd spacecraft has 2 transceivers.

See :ref:`scenarioGroundLocationImaging` for an example of using a data storage visualization.
The example :ref:`scenario_BasicOrbitMultiSat` illustrates how to show a battery or fuel tank storage device.



Adding Spacecraft Light Devices
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Vizard can add light emitting devices, i.e. spotlights, to a spacecraft object. First, let's discuss how to
setup a light device.  The associated light structure and the required
parameters are set using::

    scLight = vizInterface.Light()
    scLight.label = "Main Light"
    scLight.position = [0.2, -1.0, 1.01]
    scLight.fieldOfView = 3.0 * macros.D2R
    scLight.normalVector = [0, 0, 1]
    scLight.range = 50.0
    scLight.intensity = 6.0
    scLight.markerDiameter = 0.02
    scLight.color = vizInterface.IntVector(vizSupport.toRGBA255("red"))
    scLight.showLensFlare = 1
    scLight.lensFlareFadeSpeed = 2.0
    scLight.lensFlareBrightness = 0.5

.. caution::
    As a pointer to the ``Light`` structure is connected to :ref:`vizInterface`, it is
    important that the python structure is retained in memory.  If the python structure instance
    is created in a manner where this is not the case, use ``scLight.this.disown()``
    to ensure the python structure remains intact throughout the simulation.

The light location relative to the spacecraft B frame is given by ``position``.  The light normalaxis is
set through ``normalVector``.  The edge-to-edge ``fieldOfView`` is set in radians.
The full list of required and optional generic sensor parameters are provided in the following table.

.. list-table:: Light Object Configuration Options
    :widths: 20 10 10 10 100
    :header-rows: 1

    * - Variable
      - Type
      - Units
      - Required
      - Description
    * - ``label``
      - string
      -
      - No
      - Label to use to identify light
    * - ``position``
      - double[3]
      - m
      - Yes
      - position of the light in body frame
    * - ``normalVector``
      - double[3]
      -
      - Yes
      - normal vector of the light in the body frame
    * - ``fieldOfView``
      - double
      - rad
      - Yes
      - edge-to-edge light cone angle
    * - ``lightOn``
      - double
      -
      - No
      - Turn the light element on or off.  Value of 0 (protobuffer default) to use viz default,
        -1 for false, 1 for true
    * - ``onOffCmdInMsg``
      - ReadFunctor<:ref:`DeviceCmdMsgPayload`>
      -
      - No
      - incoming light on/off cmd state msg.  If this input message is connected, then the ``lightOn``
        variable is overwritten with the value from this input message.
    * - ``range``
      - double
      - m
      - Yes
      - Distance light will act over
    * - ``intensity``
      - double
      -
      - No
      - Intensity of light at light origin, default is 1.0
    * - ``showLightMarker``
      - int
      -
      - No
      - flag to turn on the light marker,  Value of 0 (protobuffer default) to use viz default,
        -1 for false, 1 for true
    * - ``markerDiameter``
      - double
      - m
      - No
      - Size to draw the visible lens of the light, default is 0.01 m
    * - ``color``
      - vector<int>
      -
      - No
      - Send desired RGBA as values between 0 and 255, default is pure white
    * - ``showLensFlare``
      - double
      -
      - No
      - Value of 0 (protobuffer default) to use viz default, -1 for false, 1 for true
    * - ``lensFlareBrightness``
      - double
      -
      - No
      - Simulates refraction of light in camera lens, this value controls the size and brightness
        of the lens flare, default is 0.3
    * - ``lensFlareFadeSpeed``
      - double
      -
      - No
      - Speed with which the lens flare fades, default is 4.0


Multiple light devices can be created for each spacecraft, and multiple spacecraft are supported.  Using
the ``vizSupport.py`` file, the sensors are sent to :ref:`vizInterface` using they keyword ``lightList``::

    viz = vizSupport.enableUnityVisualization(scSim, simTaskName, scObject
                                              , saveFile=fileName
                                              , lightList=scLight
                                              )

Note that here a single light and spacecraft is setup.  If you have multiple sensors, or multiple spacecraft,
then lists of lists are required::

    viz = vizSupport.enableUnityVisualization(scSim, simTaskName, [scObject, scObject2]
                                              , saveFile=fileName
                                              , lightList=[ [scLight], None ]
                                              )

Next, each light can be connected to the optional device status message of type :ref:`DeviceCmdMsgPayload`::

        lightCmdMsgData = messaging.DeviceCmdMsgPayload()
        lightCmdMsgData.deviceCmd = 1
        lightCmdMsg = messaging.DeviceCmdMsg().write(lightCmdMsgData)

        cmdInMsg = messaging.DeviceCmdMsgReader()
        cmdInMsg.subscribeTo(lightCmdMsg)
        scLight.onOffCmdInMsg = cmdInMsg

The light command state can also be set directly from python using::

    scLight.onLight = 1

However, if the input message is specified then this value is replaced with the content of the input message.


.. _specifySpacecraftCAD_label:

Specifying the Spacecraft CAD Model to use
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
The spacecraft Vizard data supports the use of ``modelDictionaryKey`` to override the default spacecraft shape
and is selected by the name, and specify a CAD model to use.  Assume a Vizard spacecraft CAD model is
labeled with ``cadString``, then you use::

    viz = vizSupport.enableUnityVisualization(scSim, simTaskName, scObject
                                              , modelDictionaryKeyList="cadString")

If you have multiple spacecraft, then this argument must be a list with the length being the number of
spacecraft::

    viz = vizSupport.enableUnityVisualization(scSim, simTaskName, [scObject, scObject2]
                                              , modelDictionaryKeyList=["cadString", None])

The argument None is used to specify the Vizard default shape to be used.

The following table provides the keywords for the built-in spacecraft shape models.

.. list-table:: Built-In Spacecraft Models
    :widths: 25 75
    :header-rows: 1

    * - Key Name
      - Description
    * - ``bskSat``
      - Default hexagonal spacecraft model with 3 solar panels.
    * - ``3USat``
      - 3U cube-sat model
    * - ``6USat``
      - 6U cube-sat model

If you want to customize the log on the built-in spacecraft models, this can be done using
``logoTexture`` spacecraft structure string::

    viz = vizSupport.enableUnityVisualization(scSim, simTaskName, [scObject, scObject2]
                                              , logoTextureList=["pathToTexture", None])

In this example the first spacecraft logo is overwritten from the default logo, while the 2nd
spacecraft model retains the default logo.  If just a single spacecraft is simulated then the
user can also just provide a simple string path value instead of a list of string path values.

The image can be a ``jpg`` or ``png`` image and should have square dimensions, such as being 256x256
in size.

.. _specifyCelestialCAD_label:

Specifying the Celestial Object CAD Model to Use
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
The gravity body data structure
contains a ``modelDictionaryKey`` string which can specify what CAD model to use.  By default Vizard uses
the ``planetName`` variable to determine if the celestial object matches an available model.  If ``displayName``
is specified then this overrules the ``planetName`` info.  Lastly, the ``modelDictionaryKey`` overrules the prior
two names.  This is done to provide fine control over what ``planetName`` is used, such as a
Spice name, the ``displayName`` could be ``TestAsteroid`` if desired, and the object shown could be a custom shape
called ``custom_test_asteroid``.

The ``createCustomGravObject()`` method in the gravity factory class has an optional ``modelDictionaryKey``
argument to specify this string if desired.  By default Vizard will read in the CAD model assuming the dimensions are
in kilometers unless ``radEquator`` is specified.
To scale the CAD model differently, specify the ``radEquator`` argument.  For a celestial object
with a general shape, Vizard finds the largest dimension along the x, y and z axes and scales the body
uniformly for this largest dimension to be ``radEquator``.  Note that the ``createCustomGravObject()`` method
requires ``radEquator`` to be given in meters.


Specifying the Osculating or True Orbit Line Colors
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
The spacecraft Vizard data supports the use of ``oscOrbitColorList`` to override the default spacecraft osculating
orbit line color and specify a custom color.  This is done using::

    viz = vizSupport.enableUnityVisualization(scSim, simTaskName, scObject
                                              , oscOrbitColorList=vizSupport.toRGB255("red"))

If you have multiple spacecraft, then this argument must be a list with the length being the number of
spacecraft::

    viz = vizSupport.enableUnityVisualization(scSim, simTaskName, [scObject, scObject2]
                                              , oscOrbitColorList=[vizSupport.toRGB255("red"), None])

The argument None is used to specify the Vizard default shape to be used.

Similarly, to set the actual or true trajectory color, use the keyword ``trueOrbitColorList`` with the same behavior
as ``oscOrbitColorList``.  Note that if the color is set through this variable it remains the same
throughout the simulation.  By reading in the line color through an input message it is possible
to change the color of the local true orbit line segment to a new color.  This is useful to denote
during what parts of the orbit an ion engine is active, or we are in sun pointing mode, etc.  To connect
a color message of type :ref:`ColorMsgPayload`, you use the argument ``trueOrbitColorInMsgList``
and provide it the color message.  This could be the output of a BSK module, or a stand alone message.
Here is sample code using a stand-alone message::

    colorMsgContent = messaging.ColorMsgPayload()
    colorMsgContent.colorRGBA = vizSupport.toRGBA255("Yellow")
    colorMsg = messaging.ColorMsg().write(colorMsgContent)

    viz = vizSupport.enableUnityVisualization(scSim, simTaskName, scObject
                                              , trueOrbitColorInMsgList=colorMsg.addSubscriber()
                                              , saveFile=__file__
                                              )

See :ref:`scenarioHelioTransSpice` for an example where the true trajectory line color is
changed during the simulation.


Adding Ellipsoid Objects to a Spacecraft Location
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Vizard can illustrate generic ellipsoid shapes in the 3d environments relative to a spacecraft location.
These could indicate keep-out zones, position uncertainties, etc.  Multiple ellipsoids can be
attached to a spacecraft, each with unique features and colors.

First, let's discuss how to setup an ellipsoid.  The associated ellipsoid structure and the required
parameters are set using::

    gncEllipsoid = vizInterface.Ellipsoid()
    gncEllipsoid.isOn = 1
    gncEllipsoid.useBodyFrame = 0
    gncEllipsoid.position = [0, 0, 0]
    gncEllipsoid.semiMajorAxes = [10, 20, 10]
    gncEllipsoid.color = vizInterface.IntVector(vizSupport.toRGBA255("yellow", alpha=0.5))

.. caution::
    As a pointer to the ``Ellipsoid`` structure is connected to :ref:`vizInterface`, it is
    important that the python structure is retained in memory.  If the python structure instance
    is created in a manner where this is not the case, use ``gncEllipsoid.this.disown()``
    to ensure the python structure remains intact throughout the simulation.

The ellipsoid location relative to the spacecraft B frame is given by ``position`` in B-frame coordinates.
The ``semiMajorAxes`` provide the ellipsoid semi-major axes in meters.  If the flag ``useBodyFrame``
is set to 1, then the ellipsoid is drawn relative to the body frame, not the orbit or hill frame.

The full list of required and optional generic sensor parameters are provided in the following table.

.. list-table:: Ellipsoid Configuration Options
    :widths: 20 10 10 10 100
    :header-rows: 1

    * - Variable
      - Type
      - Units
      - Required
      - Description
    * - ``isOn``
      - int
      -
      - No
      - Flag indicating if the ellipsoid is on (1) or off (-1).  0 is the default value with the object shown.
    * - ``useBodyFrame``
      - int
      -
      - No
      - Flag indicating if the ellipsoid should be drawn in the body frame (1) or Hill/Orbit frame (0)
    * - ``position``
      - double[3]
      - m
      - No
      - Center of the ellipsoid location in either body or Hill frame depending on ``useBodyFrame``
    * - ``semiMajorAxes``
      - double[3]
      - m
      - Yes
      - The three semi-major axes of the ellipsoid object
    * - ``color``
      - vector<int>
      -
      - No
      - Desired ellipsoid RGBA values, default is translucent gold
    * - ``showGridLines``
      - int
      -
      - No
      - Show Gridlines on ellipsoid


Multiple ellipsoid objects can be created for each spacecraft, and multiple spacecraft are supported.  Using
the ``vizSupport.py`` file, the sensors are sent to :ref:`vizInterface` using they keyword ``ellipsoidList``::

    viz = vizSupport.enableUnityVisualization(scSim, simTaskName, scObject
                                              , saveFile=fileName
                                              , ellipsoidList=gncEllipsoid
                                              )

Note that here a single ellipsoid and spacecraft is setup.  If you have multiple ellipsoids, or multiple spacecraft,
then lists of lists are required::

    viz = vizSupport.enableUnityVisualization(scSim, simTaskName, [scObject, scObject2]
                                              , saveFile=fileName
                                              , genericSensorList=[ None, [gncEllipsoid] ]
                                              )

Displaying Time Varying Components of a Spacecraft
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
The spacecraft may have rigid body components attached whose position and orientation varies with time.  One
example would be :ref:`hingedRigidBodyStateEffector` where a rigid body is hinged about a body fixed axis.
These effectors output a spacecraft state message containing its inertial position and orientation
information.  This allows Vizard to show this rigid body as a separate spacecraft object.

To show these time-varying body components such as :ref:`hingedRigidBodyStateEffector`,
:ref:`spinningBodyOneDOFStateEffector` or :ref:`dualHingedRigidBodyStateEffector` , the BSK modules can be
added to the ``enableUnityVisualization`` spacecraft list.  The parent spacecraft object
should be listed first, followed by the spacecraft effector objects as a list of
the effector name and effector state output message.  Let ``panel1`` and ``panel2``
be instances of :ref:`hingedRigidBodyStateEffector` which are attached to spacecraft ``scObject``,
all three components can be visualized using::

    viz = vizSupport.enableUnityVisualization(scSim, simTaskName, [scObject
                                                                    , [panel1.ModelTag, panel1.hingedRigidBodyConfigLogOutMsg]
                                                                    , [panel2.ModelTag, panel2.hingedRigidBodyConfigLogOutMsg]
                                                                   ]
                                              , saveFile=fileName
                                              )

Each effector is treated like a Vizard spacecraft object.  This means it is possible to add CSS,
lights etc. to the panel objects as you do with the primary spacecraft.
Note that the device list length must match that of the rigid body objects being
sent to Vizard.  In the above example, this means Vizard is seeing 3 objects and the ``lightList``, for
example, would need to contain three entrees.

By default each component will be given the default ``bsk-Sat`` shape.  It is recommended that a
custom model is assigned to each component.  See the scenario :ref:`scenarioDeployingPanel` for
an example on how to illustrate deploying panels in Vizard.



Displaying Multi-Sphere-Model (MSM) Charging Information
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
In the paper entitled "Multi-Sphere Method
for Modeling Electrostatic Forces and Torques", see http://dx.doi.org/10.1016/j.asr.2012.08.014,
a method is introduced to
approximate the electrostatic charging on a spacecraft through a series of spheres.  Each
sphere has a spacecraft-fixed location and radius.  Vizard can display such spheres
and color them using the current sphere charge value.  See :ref:`scenarioDebrisReorbitET`
for an example of this visualization tool being used.

The MSM information is added using the ``enableUnityVisualization()`` method with the
keyword ``msmInfoList``.  This information must be provided for each spacecraft in the
simulation as a list of ``vizInterface.MultiSphereInfo()`` structures.
Each craft can use a different MSM setup.  If the spacecraft has no MSM model then
use ``None`` in the list for that spacecraft.

The ``MultiSphereInfo`` structure contains the input message ``msmChargeInMsg``
where the spacecraft sphere charge values are read in from.  It also
contains a vector of MSM configuration information structures of
type ``MultiSphere``.  The following list show all ``MultiSphere`` structure
variables.

.. list-table:: ``MultiSphere`` variables
    :widths: 20 10 10 10 100
    :header-rows: 1

    * - Variable
      - Type
      - Units
      - Required
      - Description
    * - ``isOn``
      - int
      -
      - No
      - Flag indicating if the MSM is on (1) or off (-1).  0 is the default value
        with the object shown.
    * - ``position``
      - double(3)
      - meters
      - Yes
      - MSM sphere position in the body frame
    * - ``radius``
      - double
      - meter
      - Yes
      - radius of the sphere
    * - ``currentValue``
      - double
      - Coulomb
      - No
      - (optional) Current charge value of the MSM sphere.  If the ``msmChargeInMsg``
        is connected, then the content of the message overrides the value set
        in python.
    * - ``maxValue``
      - double
      - Coulomb
      - Yes
      - maximum sphere charge value
    * - ``positiveColor``
      - int(4)
      -
      - No
      - desired RGBA as values between 0 and 255, default is green
    * - ``negativeColor``
      - int(4)
      -
      - No
      - desired RGBA as values between 0 and 255, default is red
    * - ``neutralOpacity``
      - int
      -
      - No
      - desired opacity value between 0 and 255 for when charge is neutral.  Default
        is -1 which yield the Vizard default opacity value.


The following code illustrates how to add support for visualizing
MSM charge values in Vizard.  Here a single spacecraft is simulated with
an :ref:`msmForceTorque` to emulate the charging.  The charge values are read in from an input
message.  The MSM body-fixed positions are stored in the list ``spPosListDebris``, while
the MSM radii are stored in the list ``rListDebris``.  The sample code is::

    msmInfoDebris = vizInterface.MultiSphereInfo()
    msmInfoDebris.msmChargeInMsg.subscribeTo(MSMmodule.chargeMsmOutMsgs[0])
    msmDebrisList = []
    for (pos, rad) in zip(spPosListDebris, rListDebris):
        msmDebris = vizInterface.MultiSphere()
        msmDebris.position = pos
        msmDebris.radius = rad
        msmDebris.maxValue = 30e-6  # Coulomb
        msmDebrisList.append(msmDebris)
    msmInfoDebris.msmList = vizInterface.MultiSphereVector(msmDebrisList)

    viz = vizSupport.enableUnityVisualization(scSim, dynTaskName, [scObjectDebris]
                                              , saveFile=fileName
                                              , msmInfoList=[msmInfoDebris]
                                              )


