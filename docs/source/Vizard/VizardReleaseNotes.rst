
.. _vizardReleaseNotes:

Release Notes
=============


.. sidebar:: In Progress Features

    - Adding star tracker configuration visualization
    - Adding coarse sun sensor configuration visualization
    - Add ability to place the location of the standard camera relative to spacecraft
    - Visualizing the standard camera within the 3D window
    - Better automatic simulation data usage monitoring
    - general GUI enhancements
    - dynamic texture rendering
    - articulating CAD models
    - fuel tank visualization
    - better support for visualizing multiple spacecraft
    - add labels to spacecraft and sensor visualization

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

**Version 1.0.0**🍾🍾🍾🍾

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