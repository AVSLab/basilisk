.. toctree::
.. role:: raw-latex(raw)
   :format: latex
..

.. contents::
   :depth: 3
..

@page vizard Vizard - Unity Based Basilisk Visualization
@tableofcontents

:raw-latex:`\image `html Images/doc/vizard-img1.png “Basic Vizard
Interface Illustration” width=576px

@section vizOverview Overview The Vizard Unity-based Basilisk
visualization is able to display in a three-dimensional view the
Basilisk simulation data. The intent of the visualization is to show
what states are being simulated. If a planet is being modeled as a
gravitational simulation element then it is visible in the simulation.
If the planet or moon is not modeled then it is not shown. Similarly for
the sun. If there is a sun being modeled then the light comes from this
sun direction and the sun is visible. However, if no sun is being
modeled then a default lighting from a fixed direction is modeled, but
no sun is visible in the visualization. Vizard can also show some panels
illustrating spacecraft device states, or show the spacecraft device in
a heads-up display (HUD) mode. If not device or sensor is being modeled,
then none will show up as options in the Vizard menu bar.

In the above Vizard window interface illustration, the slider on the
lower-left allows the user to skim forwards and backwards through the
simulation data. The Play/Pause button at the bottom-center allows for
the Visualization to be paused and resumed. The +/- buttons on the
lower-right allow the simulation to speed and and slow down. Note that
with 1x the visualization is moving through the data with 1 frame per
simulation time step. To see a
`video <https://hanspeterschaub.info/Movies/Vizard-Basic-Features.mp4>`__
of the interface, click on the image above.

@section vizInstall Installing Vizard The application can be downloaded
as a complete binary file from the
`Overview <http://hanspeterschaub.info/bskMain.html>`__ page.

@section vizStartup Vizard Startup When starting up the Vizard software
the user is presented with a resolution and graphics setting option
panel as shown above. There is an option on the lower portion of this
panel to turn off this plane on start-up and only show it if the program
is started while pressing the option key. Note that the Vizard screen
size can be dynamically changed after startup as well.
:raw-latex:`\image `html Images/doc/vizard-img0.png “Vizard Startup
Panel” width=300px

Next Vizard presents a panel where the user can select which simulation
to visualize. To play back a previously recorded BSK simulation press
the ``Select`` button and navigate to the binary BSK recording file.
After a file has been selected press the ``Start Visualization`` button.
:raw-latex:`\image `html Images/doc/vizard-img2.png “Vizard Simulation
Selection Panel” width=300px

@section vizViewModes View Modes To engage with the visualization, the
view point can be rotated and the user can zoom in and out. There are
three view modes available:

-  **Spacecraft-Centric View Mode** (default): Here the spacecraft is
   drawn 1:1 while showing other celestial objects about it. When
   rotating the center of the spacecraft is the center of rotation. The
   spacecraft trajectory is not shown in this view. You can zoom in and
   out locally, but if you zoom out too far then the view mode switched
   to a planet-centric view mode.
-  **Planet-Centric View Mode**: Here a planet-wide view is presented.
   When rotating the view point this is about with the center of the
   planet as the center of rotation. The spacecraft trajectory is shown.
   The spacecraft is drawn at an exaggerated size so it is visible as a
   3D object in this view. To return to a spacecraft-centric view mode
   double click on the spacecraft. If you zoom out far enough then the
   mode switches to a heliocentric view.
-  **Heliocentric View Mode**: Here a solar system wide view is shown.
   The planets are drawn enlarged to make them visible, and the planet
   trajectories are shown as well. If the spacecraft is orbiting a
   planet it is not visible in this view. If the spacecraft is on a
   heliocentric trajectory it is shown, also enlarged, in this view.
   Double clicking on a planet returns the user to a planet-centric
   view.

@section vizStates Space Vehicle States The following sections describe
the basic user interface elements of Vizard. Some settings can be set
via a Basilisk script as discribed in the `scripting support
page <@ref%20vizardSettings>`__.

Basic Position and Orientation
==============================

Vizard is able to show the position and orientation of the spacecraft
being simulated. If one or more planets are being modeled, then the
spacecraft is show relative to the nearest planet.

Reaction Wheel States
=====================

If Reaction Wheels or RWs are modeled, then a RW panel can be opened
from within the ``Actuator`` menu bar item. Here the RW wheel speeds and
motor torques are shown. :raw-latex:`\image `html
Images/doc/vizard-ImgRW.png “Illustration of RW Panel” width=400px

Thruster States
===============

| If thrusters are being simulated then a range of visualizations can be
  enables within the ``Actuator`` menu item. The options include to open
  a Thruster Panel which shows the thruster firings as bar charts. The
  thruster HUD uses a particle engine to illustrate if a thruster is
  firing. Here the length and density of the particles is related to the
  strength and duty cycle of the thruster. The thruster geometry option
  draws small cones where the thrusters are modeled to be. This is
  useful when debugging that a thruster configuration is being properly
  modeled. Finally, the thruster normals option illustrates the thrust
  axes being modeled.
| :raw-latex:`\image `html Images/doc/vizard-ImgTHR.png “Illustration of
  Thruster Panel and HUD” width=400px

@section vizConfigurations Vizard Configuration Options

``View`` Menu Item
==================

| The ``View`` menu tab contains a range of Vizard options. A range of
  coordinate frames can be toggled on or off.
| :raw-latex:`\image `html Images/doc/vizard-imgAxes.png “Illustration
  of spacecraft and planet Coordinate frame Axes” width=400px

Add Pointing Vector
===================

This allows a line to be drawn from the spacecraft aimed at another
celestial body such as the sun, a planet, etc. The spacecraft location
is referred to as “Inertial”. The purpose of these lines is to have a
quick visual reference in what direction another body is located. The
lines can be hidden or removed as needed. Some celestial bodies come
with default colors such as yellow for sun heading, or red for Mars
heading, etc. However, each line color can be customized as needed.
:raw-latex:`\image `html Images/doc/vizard-ImgPointing.png “Illustration
of Pointing Vectors to Mars and the Sun” width=400px

Add Keep Out/In Cone
====================

This feature allows for a cone to be added relative to the spacecraft
which indicates if a cone about a particular body-fixed axis intersects
with a celestial object. For example, this can be used to add a cone to
validate that the sensor axis doesn’t get too close to the sun (keep out
cone), or if the solar panel normal axis stays within some cone to the
sun (keep in cone). If the cone in/out condition is not triggered, then
the cone is opaque. If the in/out condition is triggered, then the cone
becomes solid. :raw-latex:`\image `html Images/doc/vizard-ImgCones.png
“Illustration of Pointing Vectors to Mars and the Sun” width=400px

``Camera`` Menu Item
====================

The Camera menu item allows for custom camera views to be created into
the visualization.

Inertial Planet Camera
----------------------

This is a camera whose view always points relative to a particular
celestial body. The user can set the field of view value, as well as
grab a screen shot if needed. The user can select relative to which
planet the camera should point, and if the camera should point along
orbit axis, along track or orbit normal. :raw-latex:`\image `html
Images/doc/vizard-imgCamera1.png “Illustration of Inertial Planet Camera
Window” width=400px

Inertial Camera
---------------

Up to two custom views can be generated that look out of the spacecraft
+/- x-, y- and z-axis. Again the field of view can be configured, and a
screen grab button is present. :raw-latex:`\image `html
Images/doc/vizard-imgCamera2.png “Illustration of Inertial Camera
Window” width=400px

``Skybox`` Menu Item
====================

| The default star field is a realistic NASA star field. The alternate
  option is an ESO Milky Way star field that is more visually pleasing,
  but less realistic.
| :raw-latex:`\image `html Images/doc/vizard-img3.png “Illustration of
  Skybox Option with Milky Way Star Field” width=400px
