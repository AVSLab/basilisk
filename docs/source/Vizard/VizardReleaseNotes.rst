
.. _vizardReleaseNotes:

Release Notes
=============


.. sidebar:: In Progress Features

    - Adding star tracker configuration visualization
    - general GUI enhancements
    - dynamic texture rendering
    - articulating CAD models
    - fuel tank visualization
    - show the battery and power state
    - show the hard drive state
    - Add the rate gyro visualization
    - using Unity HD Rendering pipeline

**Version 1.8.4.1**

- main camera was not transforming the camera up-axis for the changing Hill Frame resulting in drift that
  was very noticeable at large time steps or rapidly changing orbits, added recalculation of up-axis for
  each change in Hill Frame to resolve this issue for most cases
- known issue:  long mouse drags when running through messages at frame rate at large time steps can
  result in the main camera spinning about target. This problem does not manifest when using the
  keyboard camera controls.

**Version 1.8.4**

- reflective solar panels(!) on the default spacecraft model
- removed dedicated scene for OpNav mode and added support in Main scene for OpNav mode functionality.
  This change will improve maintainability of the code base going forward
- changed names of directComm mode toggles to ``LiveDisplay`` and ``NoDisplay``. ``NoDisplay`` mode will result in
  the OpNav functionality wherein no image is rendered to the window and the camera is only refreshed when a
  new image is requested by the sim, ``LiveDisplay`` will take the user to the interactive screen where the
  messages will be rendered to the screen as they come in or at the user specified rate
- added command line argument support for ``-noDisplay`` to launch into OpNav functionality and clearly link
  the startup screen options with the command line arguments. The old arguments ``-opNav`` or ``-opNavMode`` will
  continue to be supported to launch into OpNav function.
- added console message to inform user the file path they are currently playing back or the tcp
  address they are connected using
- added a ``LiveSettings`` sub message to allow user to continuously update the supported fields.
- added ``TargetLines`` to the ``LiveSettings`` message. All PointLines in this field can be
  updated with every message, added, modified, or removed, and only the current message’s array of lines
  will be drawn to the screen.
- added support to the VizMessage Logging panel to show the contents of the LiveSettings sub message
- increased the opacity of the menu bar and menu panels to improve user experience


**Version 1.8.3**

- When a spacecraft is the target of the Main Camera, the Main Camera will track the spacecraft in that
  spacecraft’s Hill Frame provided that there is a parent body in the sim. If there is only a spacecraft
  in the messages or if the camera target is a celestial body, the default inertial frame tracking and
  offset of the target by the Main Camera will occur.
- added ability to show FPS under ``Time`` menu
- added command line argument ``-saveMetrics`` to record the rendering times to the file
  ``~/VizardData/opNavMetrics.txt``
- the Apple M1 version of Vizard is now able to cast shadows like the other versions of Vizard did


**Version 1.8.2**

- added ability to show true orbital trajectory.  This works both for trajectory relative to planet or another
  spacecraft
- added ability to show local osculating orbit in spacecraft-centric view
- added toggle to ``View`` menu to specify if osculating or true orbit should be show relative to
  planet or another spacecraft
- improved the custom shape import panel
- on macOS the Vizard binary is now a Universal binary supporting both the Intel and Apple M1 processors
- new ``Display Console Log`` under ``View`` menu to show any Vizard error or warning messages

**Version 1.8.1**

- Keep Out cone bug fix to restore cone detection of Sun impingement (even when the Sun is named “sun_planet_data”)
- Improved utility of the Adjust Model panel:

    - "Create Custom Texture" button renamed “Preview Custom Texture”
    - when user selects the Apply button, the custom texture and normal map will be automatically loaded into a texture, even if the user has not pressed the Preview button

**Version 1.8.0**

- user can create bodies that are not in the internal viz support dictionary. Bodies included in the internal dictionary: Sun, Mercury, Venus, Earth, Moon, Mars, Phobos, Deimos, Jupiter, Saturn, Uranus, Neptune, and Pluto
- added mu, equatorial radius, and radiusRatio to CelestialBody sub-message to support creation of custom bodies and also allow modification to those properties for previously supported bodies
- added support for intermediate sized bodies (like asteroids or small moons)
- improved the sphere of influence calculations used to choose the parent body for a spacecraft or custom celestial body (parent body needed when calculating osculating orbit lines)
- added support for custom models for celestial bodies to OpNav mode
- fixed bugs with sprite mode that would cause spacecraft to be incorrectly determined to be in contact
- fixed bugs with custom location range and added a hemisphere to range cone to better indicate the location range when custom range is enabled
- improved the custom shape importing GUI to allow an object to return to the default shape and texture


**Version 1.7.1**

- extended GroundLocation from celestial bodies to spacecraft and changed the name to Location to reflect
  new capability. Now a Location can be added to any celestial body or spacecraft and used to
  detect line-of-sight with other spacecraft or locations.
- added setting to the Save Message panel (under File) to opt in to saving a copy of all messages
  to a file on Vizard exit
- added command line argument -saveMsgFile to allow user to opt in to saving message file during launch of Vizard
- changed how opNav stores its received messages: unless the -saveMsgFile argument is used on launch,
  the message dictionary will retain only the most recent 20 messages. This was done to prevent unnecessary
  memory usage by Vizard during long opNav runs.
- brought Phobos and Deimos prefabs to current standard Celestial Body prefab configuration.
- Add was changed to Edit for the three items under the View menu
- Added a script for protection for 2 finger scrolling to all scroll bars in the Viz:
  If the mouse cursor is over a scroll window, the zoom function of the main camera is disabled
- added a new ``View/Display VizMessageLog`` option which brings up a panel to see the raw
  protobuffer messages being displayed.

**Version 1.7.0**

- added a protobuffer setting and a GUI setting to allow users to set the Spacecraft scale size in Planet
  View or Solar System View.
- changed the standard camera positioning in Planet View such that the cameras will be repositioned
  outside of the scaled up spacecraft mesh when the view changes from Spacecraft Local View
- added Ground Location objects to Vizard:

    - Ground Locations can be created in the GUI under ``View>Add Ground Location``
    - Ground Locations can be scripted as a sub message of the VizMessage protobuffer message
    - Ground locations are drawn with a small sphere on their parent body
    - Field of view of ground locations can be visualized by showing the Field of View Cone.
      These cones can be toggled in the Ground Location panel.
    - If a spacecraft passes within the field of view of a Ground Location, a line indicating communication
      will be drawn between the Ground Location and Spacecraft. If another body (planet, moon, or spacecraft)
      occludes the spacecraft, the line will not be drawn. These lines can be toggled on the
      Ground Location panel and are on by default.
    - Added labels for Ground Locations, a toggle in the Labels panel, and a protobuffer setting show/hide the labels



**Version 1.6.1**

- organized the Settings panel into tabs to improve usability and future expansion
- user can choose the chief spacecraft for the relative orbits to be other than the current main camera target under the View menu. This setting can also be scripted as a vizMessage setting
- spacecraft shadows can be brightened using a setting in the General tab of the Settings panel. This setting can also be scripted as a vizMessage setting.
- added the ability to save off all or a portion of the messages in the current run to a new file. This new function is available as “Save Messages to File” under the File menu. This feature works in both file playback mode and live-streaming mode. The data is safed into a sub-folder ``VizardData`` in the user's home directory.


**Version 1.6.0**

- Heads Up Displays of Coarse Sun Sensor coverage and boresight vectors
- Panel Display of Coarse Sun Sensors measurements
- updates to the vizMessage.proto to support Coarse Sun Sensors messages and settings
- Standard Camera Panel settings fields can now be hidden by clicking a button on the panel providing a more compact view
- changed the Unity player setting for the resolution quality panel to “Hidden By Default” to hopefully prevent it from popping up on every Windows app launch
- inertial origin coordinate frame that is visible when only spacecraft messages are present (no celestial body messages) can now be hidden by toggling off the All Planet/Moon CS under the View menu


**Version 1.5.1**

- spacecraft relative orbit lines can now be calculated using the chief spacecraft’s velocity frame instead
  of the Hill frame.
- added Setting panel toggle and vizMessage setting field to allow user selection of relative orbit frame
- added velocity frame coordinate system that can be toggled on under the View menu and a vizMessage setting field
  to show the axes
- the settings fields on the Standard Camera panels can now be hidden by the user so that only the camera
  image portion of the panel remains visible

**Version 1.5.0**

- Added the ability to visualize the osculating relative trajectory with respect to a target spacecraft.
  This works for circular, elliptical and hyperbolic trajectories!
- Added scripting options to support the relative trajectory settings
- Make the instrument camera show other spacecraft within the field of view
- Enhanced the look of the default bskSat CAD model
- Added Hill Frame Coordinate Axes display that can be toggled on under the View Menu or through scripting



**Version 1.4.1**

- The vizMessage thruster default and individual color settings are supported. You can also change the
  default color setting in Settings panel and scale the length of the thruster plumes (make them half
  as long, double, etc.).
- The thruster panel now properly labels the thruster groups
- The size of the thrust plumes is scaled to maxThrust until the maxThrust value is equal to or less
  than 0.01N. All micro-thrusters below 0.01N in size are visualized as a very small thrust puff/plume.
- This update also contains the ellipticity for all the celestial bodies we currently support.
- Fixed a small bug in the Keep Out/In Cones. If you modified an existing cone and changed it’s type
  (from Out to In or In to Out) the coneViolation flag was not reset so you could end up seeing erroneous
  results until the state update was triggered. Now whenever an existing cone is modified that flag is reset.


**Version 1.4.0**

- keyboard camera rate controls now with hot-keys for zooming in and out (``[`` or ``]``),
  pan left and right (``←`` or ``→``),
  tilt up and down (``↑`` and ``↓``), roll left and right (``<`` or ``>``).  Pressing these keys
  multiple times increases or decreases the camera rate
- hot-key ``s`` to stop all camera rates toggled with hot-keys
- hot-key help panel (press ``h`` to show or use button under view menu)
- removed time display button under Time menu because we now use hot keys
- switched to 3 quality levels for viz app (Fast, Good, and Beautiful) and confirmed that shadows are showing up in Beautiful even on the AVS model.  The Beautiful mode requires a good graphics card to yield a good frame rate.
- added flashlight to camera, still toggled by ``L``, useful to illuminate spacecraft when in shadow of a planet
- fixed sun threshold bug that caused mesh not to show up from some angles
- new timeline slider bar that live updates the rendered view
- new ``File/Settings`` option to bring up a settings panel to change system default values
- very cool new ray-traced shadows.  You need the medium or highest graphics setting to see these. It is even
  possible to cast shadows onto nearby spacecraft.

**Version 1.3.0**

- added option for playback of messages in real time. Real time mode references the system clock and advances
  messages as needed to keep pace with the current sim elapsed time of the message. Real time playback can be
  increased or decreased from 1x with the playback speed controls.
- Real Time or Frame Rate playback options can be selected under the new Time menu
- playback speed display modified to show the current playback speed relative to real time rather than the old
  frame rate speed
- added a data rate display to allow user to see for how many Unity frames a vizMessage is displayed. Data rate
  display can be toggled under the Time menu or by pressing ‘d’ on the keyboard.
- added epoch submessage to the vizMessage. If user omits epoch message, a default epoch of January 1, 2019 0h 0m 0s
  is used
- epoch message is used to calculate Mission Time display which can be toggled on from Time menu or by pressing
  ’t’ on the keyboard
- added vizMessage user setting to show mission time as 24 hr clock. This setting can also be toggled from the Time menu.
- added vizMessage user setting to show the data rate display
- updated the playback control sprites and slider for a clean look
- fixed a bug in the handling of custom model user settings where the Standard shader was not applied during
  custom model import when specified by user
- identified issue in custom model import: obj importer will not correctly import materials that were given a
  numeric name (i.e. “1”) Current workaround is to rename materials in .mtl and .obj files to use non-numeric strings.


**Version 1.2.0**

- added 2D sprite representation of spacecraft and celestial bodies to support spacecraft constellation modeling and easier visualization of distant objects
- added Sprite settings panel under view menu to allow customization of displayed sprites’ shape, size, and color
- panel includes toggles to turn on/off sprite visualization for distant spacecraft or celestial bodies
- added fields to protobuffer messages to allow setting of those toggles
- added fields to protobuffer messages to allow user to specify a default spacecraft sprite  as well as sprites for individual spacecraft
- added code to disable HUD when their parent spacecraft is in sprite mode
- other issues addressed in this release:

    - fixed bug in thruster HUD where thrusters with a minimum thrust of 0.5 N or less would not display a thruster plume by adding a floor to the particle life setting
    - changed the protobuffer message default value for boolean flags to use 0 for viz default, -1 for OFF and 1 for ON. This matches the default value of 0 sent in all protobuffer messages for int32 fields.
    - labels that belong to occluded bodies will now disappear until their body is visible again

**Version 1.1.0**

- added floating labels for the following:

   - spacecraft names
   - sun, planets, moons names
   - coordinate axes
   - thruster HUD components
   - reaction wheel HUD components
   - standard and instrument camera names

- labels can be toggled on and off from Labels Panel accessed through the View menu
- label font size can be increased or decreased through the Labels panel
- labels can also be toggled by type by using newly added fields in the Protobuffers vizMessage
- revamped startup screen and file browser appearance to follow the general Vizard application design themes

**Version 1.0.0** 🍾🍾🍾🍾

- Support for various screen sizes, including retina support on macOS
- Added support to show the boresight of a camera model (both instrument and standard cameras)
- Added support to draw a camera frustum that illustrates the camera orientation,
  field of view, camera pixel sensor aspect ratio.  If the camera position is provided then the frustum is draw at
  that location as well.
- Support for the user changing the GUI scaling on the fly from within the ``View`` menu
- Improved 2-way camera models that interface with Basilisk v 1.7.0.  The custom cameras are now called instrument cameras
- Standard camera panels are now drawn at a more compact size by default.  This makes their sizing and positioning more flexible.
- Various under the hood improvements to support a BSK simulation with a simulated camera
- Improved full screen support
- Added support for all the new Vizard features to be scriptable from a python BSK simulation


**Version 0.9.0**

- added an option to Standard Camera GUI panel and vizMessage to supply a custom camera position
- eliminated camera jitter in body view when pointing at a nearby spacecraft
- improved reaction wheel panels and HUD to better support multiple spacecraft by tracking the max speed and torque for each spacecraft’s reaction wheels
- added fields to the vizMessage reaction wheel sub message to allow user to set the max torque and max speed
- fixed broken link between Main scene manager and direct comm controller to restore direct comm ``liveStream`` as illustrated in :ref:`scenarioBasicOrbitStream`
- improved support for Unity’s physical camera properties, focal length and sensor size, when setting up Custom Cameras

**Version 0.8.1**

- trigger colliders now resize to fit the spacecraft mesh being used (improves the user experience when double-clicking to change camera targets)
- fixed a bug that prevented multiple custom models being loaded back-to-back
- rebased on Unity2019.2.16f1

**Version 0.8.0**

- The camera view panel screen shot button now stores the PNG image in the user's home folder
- Changed how standard cameras work.  The user can invoke readily 2 standard cameras and specify for which spacecraft these are attached.  This scales much better with lots of spacecraft where before we attached 3 standard cameras to each spacecraft by default
- made it possible to launch vizard in Black Lion live streaming mode from command line
- Added option under File menu to compress simulation data
- usability improvements to custom CAD model inventory and tuning GUI panels

**Version 0.7.0**

- added ability to load in a custom CAD obj file to replace the default spacecraft model
- added the ability to replace any simulation object with a custom object or a default shape like sphere, cone, cylinder, etc.
- added vizMessage user settings support for custom models to allow automatic import at runtime

**Version 0.6.0**

- scriptable vizMessage user settings allow users to customize the start-up configuration of vizard. Users can now toggle spacecraft and planet coordinate systems, orbit lines, actuator Heads Up Displays, actuator panels.
- users can specify a custom skybox by providing a file path to the desired texture, one of the default skybox textures, or a plain black background with the skybox user setting
- Spacecraft camera vizMessages can be configured to user specified headings or targets and panels can be automatically visible on start-up.
- configuration messages specifying multiple pointing vectors and/or Keep Out or Keep In cones can be added to generate these items automatically during Vizard initialization

**Version 0.5.0**

- added a lightweight opNav mode that can livestream camera images to the Basilisk simulation over the Direct Comm connection on demand
- improved main camera panning
- added support for reaction wheel spin sub message
- scriptable user setting message for Ambient Brightness

**Version 0.4.0**

- New option to set the ambient brightness
- New Camera menu option to select the target object
- General code fixes and improvements
- Added option to toggle off/on orbit illustration
- Added keyboard support to quit the application

**Version 0.3.0**

- Initial public release of the new Unity based Basilisk visualization tool.
- This tool is able to illustrate spacecraft translational position, trajectory, orientation and primary celestial bodies.
- Currently this public Vizard copy support saving Basilisk simulation data to file to be then viewed in Vizard.
- In development feature is being able to live stream Basilisk simulation data to Vizard directly
- The Visualization can show a spacecraft centric view (default), a planet centric view (enabled by double clicking on planet or zooming out even further), and a heliocentric view (by zoom out even further)
- Spacecraft and planet axes can be toggled on or off
- Screen size can by dynamically changed
- The menu bar at the top is dynamic in that it only shows device options if such devices are actually modeled and sent as messages to Vizard.
- Heads-up visualization of the thrusters is possible
- Device state panels can be enables for Reaction Wheels or Thrusters
- Separate camera views can be invoked to get perspectives from the spacecraft along particular body-fixed directions
- Direction vectors can be added from the spacecraft to other object to always illustrate the heading to the sun, the Earth, etc.
- Keep-out and keep-in zones can be set within Vizard to visualize if a celestial object is visible within a body-fixed cone.  This enables checking if a solar panel axis is within some degrees of the sun, or checking that a sensor axis is outside a cone relative to the sun.