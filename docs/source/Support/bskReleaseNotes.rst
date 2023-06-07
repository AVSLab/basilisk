.. _bskReleaseNotes:

Basilisk Release Notes
======================

.. Note::

    We are very excited by the great capabilities that this software already has, and plan to be updating this
    software regularly.  There is some documentation in terms of Sphinx generated HTML documentation, but also
    documentation within the code, and several Basilisk modules that are math heavy have LaTeX'd documentation
    folders as well.  Best place to start is to run the integrated tutorial scripts inside the ``basilisk/examples``
    folder, described in :ref:`examples`.  To learn how to use and program Basilisk, see :ref:`learningBasilisk`.

.. Danger::

   This next generation of Basilisk 2.0+ introduces a new messaging system and file architecture.  As a result
   using BSK2 requires upgrading existing Basilisk 1.x simulation scripts (see :ref:`migratingToBsk2`) and C/C++ modules
   (see :ref:`migratingModuleToBsk2`) to be used with 2.x and onwards.  All unit test and example scenario scripts
   are updated and form a good source for examples on how to use the new software framework.

.. sidebar:: In Progress Features

    - GPU based methods to evaluate solar radiation pressure forces and torques
    - new tutorial example scripts
    - landing dynamics force/torque effector that computes the interaction between a CAD spacecraft model and a
      CAD asteroid or lunar surface terrain.
    - spacecraft charging related modules
    - ability to integrate dynamics of multiple spacecraft simultaneously
    - support a way to do thread-safe messaging
    - ability to integrate Python Basilisk modules in the same task and process as C/C++ modules
    - automated documentation build system when code is pushed to the repo



Version |release|
-----------------
- Created new way to define Python modules by inheriting from ``Basilisk.architecture.sysModel.SysModel``.
  See :ref:`pyModules` for details.
- Added the ability to integrate the ODE's of two or more Basilisk modules that are ``DynamicObject`` class
  member at the same time.  See :ref:`bskPrinciples-9`
- updated ZMQ version to 4.5.0.  For 2-way communication with ``opNav`` modules talking to Vizard
  then Vizard 2.1.5 or newer should be used.  This also removes the need for the legacy bincrafters code repo.
  Delete ``~/.conan`` folder if you run into ``conan`` issues.
- The Basilisk project C++ version is advanced from C++11 to C++17
- Disabled the following build options in the conan included OpenCV dependency; with_ffmpeg video frame encoding lib,
  with_ade graph manipulations framework, with_tiff generate image in TIFF format, with_openexr generate image in EXR
  format, with_quirc QR code lib. Users that have Basilisk control the build of these modules through the External
  Modules CMake integration will need to manual toggle these OpenCV build options.
- Updated :ref:`SmallBodyNavEKF` with several bug fixes. Removed spacecraft attitude estimation component.
- Bug fix made to :ref:`eclipse`: Saturn, Jupiter, Uranus, and Neptune radii were incorrectly being assigned the 
  radius of Mars. 
- Added custom planet name to :ref:`eclipse` in case the user wants to use a body not contained within the module.
- Removed all instances of using ``unitTestSupport.np2EigenVectorXd()``, as this function is now unneeded.
- Created a :ref:`facetSRPDynamicEffector` dynamics module to calculate the B frame SRP force and torque acting on a static spacecraft.
- fixed ``PCI2PCPF()`` and ``PCPF2PCI`` methods in :ref:`geodeticConversion` to use the correct DCM
- updated :ref:`geodeticConversion` to be able to account for planet ellipsoidal shape if polar radius is provided
- Google Test C/C++ testing framework added
- Created a :ref:`prescribedRot2DOF` fsw module to profile a prescribed 2 DOF rotational maneuver for a secondary rigid
  body connected to the spacecraft hub. To simulate the maneuver, this module must be connected to the
  :ref:`prescribedMotionStateEffector` dynamics module.
- Corrected default value of ``accuracyNanos`` in :ref:`simSynch` to be 0.01 seconds.
- Added a deprecation system for Basilisk. For developers, see :ref:`deprecatingCode`.
- Changed the units of plasma flux in :ref:`dentonFluxModel` and :ref:`PlasmaFluxMsgPayload` from
  [cm^-2 s^-1 sr^-2 eV^-1] to [m^-2 s^-1 sr^-2 eV^-1], because m^-2 is used more frequently in computations
- Fixed a bug in eclipse that caused potentially occluding bodies to be skipped if a prior body was closer to the sun than
  the spacecraft
- fixed the time evaluation in :ref:`msisAtmosphere`
- Added an optional ``controllerStatus`` variable and ``deviceStatusInMsg`` message to the :ref:`simpleInstrumentController` to 
  match the functionality of the corresponding data and power modules
- Corrected tasks priorities in several scenarios and added checks in two modules to ensure that C MSG read errors are not thrown


Version 2.1.7 (March 24, 2023)
------------------------------
- Fixed ``CMake/conan`` case sensitivty issue when compiling Basilisk with ``opNav`` flag set to ``True`` on Linux platforms
- Created fsw :ref:`hingedRigidBodyPIDMotor` to compute the commanded torque to :ref:`spinningBodyOneDOFStateEffector` using a proportional-integral-derivative controller.
- Added :ref:`torqueScheduler` to combine two :ref:`ArrayMotorTorqueMsgPayload` into one and implement effector locking logic.
- Refactored how ``Custom.cmake`` files are included and how they are to be constructed. ``Custom.cmake`` files
  should no longer include an include guard (e.g. ``if(BUILD_OPNAV) ... endif(BUILD_OPNAV)`` ). Rather, to add
  optionally compile a module, its directory name should be added to a list in
  ``src/cmake/bskTargetExcludeBuildOptions.cmake``. Most importantly, the build target is now accessible within the
  a ``Custom.cmake`` file as ``${TARGET_NAME}``. This enables appropriate modularization of build target specific
  includes, dependencies, and compiler flags. For an example of the implications of this refactor review the before
  and after of the ``src/cmake/usingOpenCV.cmake`` file.
- updated :ref:`unitTestSupport` to create the file path in a platform agnostic manner
- Created a :ref:`sensorThermal` module to model the temperature of a sensor using radiative heat transfer
- Created a :ref:`tempMeasurement` module to add sensor noise/bias and fault capabilities to temperature readings
- Added a ``terminal`` flag to the event handlers that cause the simulation to terminate when triggered; demonstrated
  use of flag in update to :ref:`scenarioDragDeorbit`.
- Created a :ref:`prescribedMotionStateEffector` dynamics module for appending rigid bodies with prescribed motion
  to the spacecraft hub.
- Created a :ref:`prescribedRot1DOF` fsw module to profile a prescribed rotational maneuver for a secondary rigid body
  connected to the spacecraft hub. To simulate the maneuver, this module must be connected to the
  :ref:`prescribedMotionStateEffector` dynamics module.
- Created a :ref:`prescribedTrans` fsw module to profile a prescribed translational maneuver for a secondary rigid body
  connected to the spacecraft hub. To simulate the maneuver, this module must be connected to the
  :ref:`prescribedMotionStateEffector` dynamics module.
- Added :ref:`solarArrayReference` to compute the reference angle and angle rate for a rotating solar array.
- Update python dependency documentation and check to not use ``conan`` version 2.0.0 for now
- Changed the ``SpinningBodyStateEffector`` module name to :ref:`spinningBodyOneDOFStateEffector`.
- Added the ability to lock the axis on the :ref:`spinningBodyOneDOFStateEffector` module.
- Added two new unit tests to :ref:`spinningBodyOneDOFStateEffector`.
- Updated :ref:`magneticFieldWMM` to use the latest WMM coefficient file and evaluation software
- Added a :ref:`spinningBodyTwoDOFStateEffector` module that simulates a two-axis rotating rigid component.
- Created :ref:`oneAxisSolarArrayPoint` to generate the reference attitude for a spacecraft that needs to point a body-fixed
  axis along an inertial direction while ensuring maximum power generation on the solar arrays
- Added a maximum power parameter ``maxPower`` to :ref:`reactionWheelStateEffector` for limiting supplied
  power, independent of the modules in simulation/power.
- Added :ref:`thrusterPlatformReference` to align the dual-gimballed thruster with the system's center of mass, or at an offset thereof to perform momentum dumping.
- Improved reliability of opNav scenario communication between :ref:`vizInterface` and Vizard
- provide support or Vizard 2.1.4 features


Version 2.1.6 (Jan. 21, 2023)
-----------------------------
- Refactored :ref:`keplerianOrbit` to not depend on the ``gravityEffector`` class
- Updated Basilisk install documentation to discuss accessing source code from GitHub.com
- Fixed an issue where attaching a thruster to a body different than the hub when using ``zeroBase`` would yield very large offsets.
- Added documentation in :ref:`bskPrinciples-4` on how to read the current message values
- Highlighted the challege of setting up a ``recorder`` on a re-directed message in :ref:`bskPrinciples-7`
- added the ability to add a ``recorder()`` to a C-wrapped module input message
- Fix an issue in in :ref:`magneticFieldWMM` where a fixed width array holding a file path would result in a cutoff
  path when basilisk is located in a directory path of greater than 100 characters.
- Updated the build system to use newer versions of ``eigen``, ``protobuf``, ``cppzmq``
  and ``opencv``.  This corrects some build issues with new compilers.
- The ``linearAlgebra``, ``rigidBodyKinematics``, ``orbitalMotion`` were mistakenly exposed as part of the
  :ref:`sim_model` module's API. They have been removed and the functions they provided are still found in
  ``Basilisk.utilities.orbitalMotion``, ``Basilisk.architecture.linearAlgebra``, and
  ``Basilisk.architecture.rigidBodyKinematics``.
- Fixed an issued recording the ``timeWritten`` information of a C-wrapped message
  with a ``recorder()`` module.
- Updated :ref:`pullCloneBSK` to ask the user to first install ``lfs`` before pulling a copy
  of the Basilisk repo due to some large files being stored in the GitHub large file storage
  system.
- Updated :ref:`scenarioGroundLocationImaging` to properly save off the ground location
  information for Vizard
- Added a new helper function to convert C arrays to ``Eigen::MRPd`` and vice-versa inside ``avsEigenSupport``.
- Updated ``SpinningBodyStateEffector`` to use the :ref:`HingedRigidBodyMsgPayload` output message type for compatibility with other modules
- Added the ability to set an inertial heading in the :ref:`boreAngCalc` module. Changed the internal module logic to use ``Eigen`` library variables and functions instead of C-style arrays and methods.
- Added support for Vizard v2.1.3
- Updated :ref:`simpleInstrumentController` to provide the option to consider the angular velocity tracking error norm
  when considering to take an image.


Version 2.1.5 (Dec. 13, 2022)
-----------------------------
- Made the external module custom message definitions work again with the
  latest build system.
- Fixed the custom RW in :ref:`simIncludeRW` to store the information regarding ``u_min`` in the RW class.
- provide support for the swig 4.1 software
- Added the ability in both :ref:`thrusterDynamicEffector` and :ref:`thrusterStateEffector` to connect a thruster to a moving body different than the hub.
- The thrusters now have an additional variable called ``MaxSwirlTorque``. Useful for ion thrusters, it adds a torque about the thrust axis proportional to the current thrust factor.
- Added a torsional spring and damper to the ``SpinningBodyStateEffector`` module.
- Added support for having multiple Vizard instrument cameras setup in :ref:`vizInterface`

.. warning::

    The support for having multiple Vizard instrument cameras requires a change in :ref:`vizInterface`,
    and thus breaks existing code that using direct Vizard communication.  The
    image output message is now a vector of output messages, and the ``cameraConfigBuffer`` class variable
    can no longer be set directly.  Rather, the camera configuration message should be added
    using the ``viz.addCamMsgToModule()``, or the message can be created and added using the convenience method
    ``vizSupport.createCameraConfigMsg()``.

- Updated :ref:`hingedRigidBodyStateEffector` and :ref:`dualHingedRigidBodyStateEffector` such that
  the effector inertial states are relative to the inertial frame of the simulation, not the
  central body frame.
- Added ``color`` keyword support to the ``vizSupport.createCustomModel()`` method
- Updated :ref:`cppModules-4` to explain how now the swig interface to vectors of input/output messages
  are now auto-generated when making Basilisk project
- added documentation about creating and importing custom Unity addressable assets
  in :ref:`vizardCustomUnityModels`.
- fixed :ref:`scenarioAttLocPoint` to display the Earth location and the associated cone
  again in Vizard.  Updated :ref:`vizardSettings` description of ``addLocation()`` method.
- Added experimental support to build Basilisk on Linux with a computer using an ARM processor.
- Updated :ref:`CameraConfigMsgPayload` to support the Vizard flag ``updateCameraParameters`` which
  allows the camera parameters to be updated live.
- Updated documentation to discuss downloading Basilisk from GitHub


Version 2.1.4 (Oct. 1, 2022)
----------------------------

- revised how the build system swig's all the message objects.  This leads to compile time improvements across all
  platforms.  In Linux in particular we are seeing a 2x reduction in compile time.  These changes also reduce
  the memory requirements when compiling. Note: The ``basilisk.architecture.cMsgCInterfacePy`` content is
  now included in ``basilisk.architecture.messaging`` package.  Prior scripts using ``cMsgCInterfacePy``
  still run as a link has been created.  However, the use of ``cMsgCInterfacePy`` is depreciated and code
  should be updated to using ``messaging`` instead.
- added new :ref:`hingedRigidBodyMotorSensor` for adding noise, bias, and discretization to panel state message
- added new :ref:`simpleVoltEstimator` to provide simulated voltage measurements of a neighboring space object
- added the ability to have a RW motor torque break slow down the wheel speed if saturated.  The motor torque
  is set to zero if it is trying to increase the wheel speed in saturation conditions.
- updated Windows install instructions about setting path variables
- enhanced Windows install instructions to highlight adding ``cmake`` binary to the command line path
- added new training videos to :ref:`scenario_BasicOrbit`, :ref:`scenario_FeedbackRW` to discuss how to
  create class based Basilisk simulations
- added new :ref:`groundMapping` for mapping points on the surface of a spherical body.
- added new :ref:`mappingInstrument` to pass along access from a vector of map points to a storage unit.
- updated :ref:`locationPointing` to allow for spacecraft targeting as well
- added new :ref:`scenarioGroundMapping` scenario to demonstrate the new mapping capabilities.
- added new :ref:`scenarioRendezVous` scenario illustrating a servicer approaching
  a debris object and engage several flight modes.
- added new scenario :ref:`scenarioDragDeorbit`
- added new scenario :ref:`scenarioLagrangePointOrbit`
- added Vizard 2.1.1 support for spacecraft ellipsoid shapes, Unity camera parameters
- added support for the polynomial gravity model to :ref:`gravityEffector`
- updated the ``conanFile.py`` to fix configuration and building issues with the latest
  version of python
- fixed issue computing latitude angle in ``PCPF2LLA()`` in :ref:`geodeticConversion`.  This was used
  in the helper method ``specifyLocationPCPF()`` method inside :ref:`groundLocation`, as well as
  :ref:`msisAtmosphere` and :ref:`albedo`.
- fixed an issue in the RKF45 variable time step integrator where one of the constants had the wrong sign.
- added new :ref:`scenarioMomentumDumping` to illustrate how to perform momentum dumping using thrusters.
- updated :ref:`hingedRigidBodyStateEffector` to allow for an optional panel reference state input message
- added new :ref:`scenarioDeployingPanel` to demonstrate panel deployment using panel reference message
- added new :ref:`thrusterStateEffector` which is compatible with a variable time step integrator.  Here
  the thrust on-off command is passed through a first order low-pass filter to provide smooth on- and
  off-ramping.
- added new attitude pointing scenario :ref:`scenarioAttitudeFeedback2T_stateEffTH` that uses
  the new :ref:`thrusterStateEffector`
- added ability to simulate faults within :ref:`coarseSunSensor` module
- created a 1-DoF rotating rigid body class ``SpinningBodyStateEffector``. It is built in a general way to simulate 
  any effector with a single spinning axis.


Version 2.1.3 (May 25, 2022)
----------------------------
- corrected how :ref:`planetEphemeris` computes the celestial body orientation
- corrected issue in Monte Carlo controller class where if a single run is called that fails,
  this was not reported
- updated Basilisk documentation CSS to work with the latest version of ``sphinx`` and ``breathe``
- added new :ref:`tabularAtmosphere` to calculate atmospheric density using atmosphere tables
- created new :ref:`smallBodyNavUKF` to make an UKF filter for small body navigation
- created new example script :ref:`scenarioSmallBodyNavUKF` to demonstrate the use of :ref:`smallBodyNavUKF`
- added a function titled ``SpherePlot()`` that allows for plotting of charged spheres based
  on the MSM model :ref:`msmForceTorque`
- created new :ref:`smallBodyWaypointFeedback` module for waypoint-to-waypoint control about a small body
- created new example script :ref:`scenarioSmallBodyFeedbackControl` to demonstrate the new module
- added :ref:`scenario_AddRWFault` to show how to use event handlers to add faults
- added :ref:`constrainedAttitudeManeuver` with MRP-cartesian-distance- and effort-based A* graph search algorithms
- added :ref:`scenarioAttitudeConstrainedManeuver` to illustrate how to use :ref:`constrainedAttitudeManeuver`
- added ``specifyLocationPCPF()`` method to :ref:`groundLocation` for specifying ground locations in planet-centered,
  planet-fixed coordinates
- updated :ref:`spacecraftLocation` to handle cases where the closed approach point to
  the planet is outside the spacecraft-spacecraft interval
- added new :ref:`scenarioAerocapture` which simulates an aerocapture scenario
- added new :ref:`hingedBodyLinearProfiler` to provide a panel deployment angular profile
- added new :ref:`hingedRigidBodyMotor` to provide panel motor torque control
- added new training videos to :ref:`configureBuild`, :ref:`installOptionalPackages`, :ref:`scenarioBasicOrbit`,
  :ref:`scenarioOrbitManeuver`, :ref:`scenarioOrbitMultiBody`, :ref:`scenarioCustomGravBody`
- added support for Vizard 2.1 scripting


Version 2.1.2 (March 12, 2022)
------------------------------
- enhanced :ref:`spiceInterface` to allow Spice spacecraft names to be setup to pull their
  trajectory and attitude states from a spice kernel
- added :ref:`scenarioSpiceSpacecraft` to illustrate using Spice to specify the trajectory of a
  spacecraft while leaving the attitude dynamics unprescribed.
- fixed a bug where using the generator flag to build on windows would skip a line in the conanfile.py that is crucial for building opNav.
- added :ref:`dentonFluxModel` to compute electron and ion fluxes for the GEO regime.
- fixed build issue with ``conan`` version 1.44 or newer
- fixed an issue doing a clean build of ``opNav`` mode where conan failed to install ``opencv/4.1.1``
  with the ``jasper`` dependency.
- enhanced ability to set the planet Spice frame in the gravity factory class
- new ability to set the Vizard celestial body name to be different from the Spice planet body name
- added support for ``pytest`` version 7.0.0 and higher
- updated how ``pytest`` is run to generate a resulting HTML report
- modified :ref:`msmForceTorque` to create an output message with the MSM charge values for each spacecraft
- added new :ref:`scenarioInertialSpiral` example scenario
- improved robustness of Basilisk installation script
- provide support for Vizard 2.0.4 feature scripting
- added a new heliocentric mission simulation example using custom Spice spacecraft
  trajectory file :ref:`scenarioHelioTransSpice`
- added a new planetary fly-by mission example using a custom Spice translational file and
  attitude pointing modes :ref:`scenarioFlybySpice`
- added a new asteroid arrival mission example with attitude pointing modes :ref:`scenarioAsteroidArrival`
- added a new scenario :ref:`scenarioTwoChargedSC` illustrating how to apply the MSM spacecraft
  charging model to a relative motion simulation


Version 2.1.1 (Dec. 15, 2021)
-----------------------------
- Updated ``OpNav`` mode dependency ``gettext`` to version 0.21 to allow BSK to be build on Windows
  with ``OpNav`` support.
- created two new messages that contain the information regarding scheduled burns for orbit reconfiguration in
  formation flying scenarios. See :ref:`ReconfigBurnInfoMsgPayload` and :ref:`ReconfigBurnArrayInfoMsgPayload`.
- the module :ref:`spacecraftReconfig` now outputs a message of type :ref:`ReconfigBurnArrayInfoMsgPayload`.
  All internal calculation are also done using a buffer of this message type.
- Added the time standard library to include statements in atmosphereBase.h to fix a build issue found on windows.
- updated :ref:`spacecraft` to include an optional translational reference message to specify the trajectory
- Added a swig array-type ``ARRAYINTASLIST`` that fixes a double to int conversion error when building the
  ``FSWdeviceAvailability`` message on windows.
- Updated dispersions.py to support functionality that was deprecated in python3.10. This change supports
  python versions >=3.3.
- Updated the Windows build process to fix a static runtime library issue with ``vizInterface`` found
  in older versions of visual studio.
- Added scripting support for Vizard 2.0.3

Version 2.1.0 (Nov. 13, 2021)
-----------------------------
- added BSpline function to ``utilities`` and related UnitTest. 
- added kinematic relations between angular accelerations and second derivative of MRP set to
  :ref:`rigidBodyKinematicsutilities` library
- updated the installation script to function with the latest ``conan`` program and the recent
  ``conan`` repo changes.  Note, you will have to delete the ``.conan`` folder in your home
  directory to create a fresh copy of the software dependencies.
- added a Developer support page :ref:`debugging`
- fixed a memory leak with the Swig layer where an object was not released properly.  Thanks go to
  Stephen Ritter and Toney for tracking down this issue.
- added a new orbit maneuver example :ref:`scenarioJupiterArrival`
- made SWIG interface to the MRP derivative variable
- added two new variable time step integrators. See :ref:`svIntegratorRKF45` and :ref:`svIntegratorRKF78`.
- updated the state effector base class to also provide the current integration time step in addition to the
  current time
- added new scenario :ref:`scenarioVariableTimeStepIntegrators`
- updated :ref:`scenarioIntegrators` to include the ``rkf45`` and ``rkf78`` options
- changed the way :ref:`spacecraftReconfig` gets the deputy's mass properties. It now receives that information
  through a message of the type ``VehicleConfigMsgPayload`` instead of an internal variable. Relevant example 
  scripts have been updated.
- new tutorial example scenario script :ref:`scenarioTAMcomparison`
- new mass sensor that converts a ``simulation`` mass properties message to a ``FSW`` vehicle configuration message :ref:`simpleMassProps`
- added scripting support for Vizard 2.0.1 and 2.0.2
- This release provides a new ability to run a single Basilisk simulation in a multi-threaded manner.
  The BSK processes can be spread across multiple threads.  See :ref:`scenario_BasicOrbitMultiSat_MT`
  for an example of how to use this.

.. warning::

    The BSK v2.1 multi-threading assumes all processes assigned to a thread can run independently
    from processes in another thread.  Further, cross thread message communication is not yet
    thread safe!


Version 2.0.7
-------------
- new :ref:`forceTorqueThrForceMapping` to map commanded forces and torques to a set of thrusters
- updated Vizard documentation on the setting flags ``orbitLinesOn`` and ``trueTrajectoryLinesOn``
- added power and fuel tank modules to the :ref:`BSK_MultiSatDynamics` class.
- improved the DV calculation of the spacecraft state output message by integrating the gravitational acceleration
  using the current integration scheme rather than using a first order approximation.
- updated install script to be able to have ``conan`` install ``opencv`` again.  Something changed with the ``conan``
  repo that broke this.
- updated BSK install instructions on the M1 Apple Silicon platform as Basiliks can now run natively


Version 2.0.6
-------------
- updated :ref:`vizInterface` to support Vizard 1.9.1 and the ability to visualize generic sensor types and
  antenna communication status
- updated :ref:`ephemerisConverter` to also convert the planet orientation states, not just the
  translational states
- added a :ref:`planetNav` module that adds noisy to planet ephemeris, similar to simpleNav.
- created a new device command status message :ref:`DeviceCmdMsgPayload` and updated :ref:`simpleInstrumentController`,
  :ref:`simpleInstrument` and :ref:`spaceToGroundTransmitter` to make use of it.
- added :ref:`attRefCorrection` to adjust the reference attitude by a fixed rotation
- added :ref:`scenarioAttitudePrescribed` to illustrate how to prescribe the spacecraft orientation
- added new modules :ref:`mtbFeedforward`, :ref:`mtbMomentumManagementSimple`, :ref:`dipoleMapping` and
  :ref:`torque2Dipole` which are using in combination to achieve MTB based RW momentum dumping.
- added a new magnetic RW momentum dumping example in :ref:`scenarioMtbMomentumManagementSimple` which illustrates
  using the above new MTB related modules to change the momentum, as well as drive the nominal momentum to
  a desired value using :ref:`rwNullSpace`.
- created a new architecture based on ``BskSim`` called ``MultiSatBskSim``. It exploits the new messaging system to create a simulation
  with any number of spacecraft in a highly modular way. It allows for the addition of homogeneous or heterogeneous satellites without 
  having to hard code their properties into a single dynamics or FSW script. It will be a foundation to test the upcoming multithreading
  capabilities of Basilisk.
- added three example scenarios that showcase this new architecture. See :ref:`scenario_BasicOrbitMultiSat`, :ref:`scenario_AttGuidMultiSat` 
  and :ref:`scenario_StationKeepingMultiSat`.
- added a new FSW module :ref:`formationBarycenter`. It computes the barycenter's position and velocity of a swarm of satellites. This barycenter
  can be either computed with cartesian coordinates (usual mass-weighted average), or using orbital elements weighted average. Will be useful 
  for spacecraft formations defined around the barycenter of the swarm and not a chief spacecraft.
- enhanced :ref:`locationPointing` to support the target input msg being either a location message or an
  ephemeris message
- updated install notes to ensure Linux python3 developer libraries are installed, and to ensure that ``wheel``
  package is installed along with ``conan``
- created :ref:`smallBodyNavEKF` to simulate autonomous navigation in proximity of a small body
- added a :ref:`AttRefMsgPayload` output message to the :ref:`locationPointing` module.
- added :ref:`cppModules-5` to the section on learning how to create BSK modules
- updated :ref:`locationPointing` to support 3D rate damping as an option


Version 2.0.5
-------------
- fixed issue in :ref:`waypointReference` to interpolate between waypoint ``n`` and shadow set of
  waypoint ``n+1`` when these are described by opposite MRP sets. Updated documentation and corrected
  typos in :ref:`scenarioAttitudeConstraintViolation`.
- Added :ref:`hillStateConverter` and :ref:`hillToAttRef` modules for formation flight navigation and attitude-driven differential drag
- Added representative scenario :ref:`scenarioDragRendezvous` demonstrating attitude-driven differential drag formation flight
- Added new scenario :ref:`scenarioDragSensitivity` showing how to do a differential drag
  spacecraft control sensitivity analysis
- updated :ref:`celestialTwoBodyPoint` to account for a case where the celestial objects are in opposite directions
- replaced ``acos()`` and ``asin()`` with ``safeAcos()`` and ``safeAsin()`` which ensure that arguments are
  clipped to be within and including -1 and 1
- updated :ref:`dataFileToViz` to allow the ``Reset()`` method to be called multiple times.  If a data file
  was already opened, then it is closed before the next data file is opened.
- updated :ref:`groundLocation` to also output SEZ coordinates, as well as range, azimuth, elevation, south, east
  and zenith coordinate rates.  These coordinates are always computed regardless if a spacecraft is visible to the
  target.  Check the ``hasAccess`` message variable to see if the spacecraft is visible.
- updated the OpNav examples script to set a black sky background in the Vizard camera images
- added a new Python method ``isSubscribedTo()`` to query if the input and output messages between
  two modules are connected
- updated :ref:`gravityEffector` documentation to properly pull in the RST documentation and link to the
  PDF describing the gravity models
- updated ``setAllButCurrentEventActivity`` method in :ref:`SimulationBaseClass` to work with multiple satellites. We can now add an index at the 
  end of each event name that guarantees only events with the same index are affected. The ``useIndex`` flag must be set to ``True``.
- added new magnetic torque bar effector in :ref:`MtbEffector`
- added new FSW module to control the RW momentum using MTBs in :ref:`mtbMomentumManagement`
- new tutorial example script :ref:`scenarioMtbMomentumManagement`
- updated :ref:`rwNullSpace` to have an optional input message of desired RW speeds.  These desired values default to
  zero so the module retains the earlier behavior if this optional input message is not connected.
- added two lines in :ref:`waypointReference` to normalize the attitude quaternion that is read from file.

Version 2.0.4
-------------
- updated :ref:`spacecraft` ``Reset()`` method to write all spacecraft and effector state output messages
  with their initial values.  This way these output messages are correct as already as calling the
  ``InitializeSimulation()`` method.
- fixed an issue that could prevent ``.subscribeTo`` from a C++ to C wrapped message object to not function
  properly.
- new :ref:`simpleInstrumentController` that sends an imaging command to a :ref:`simpleInstrument` if the attitude error
  and access to a :ref:`groundLocation` module are within requirements.
- new :ref:`scenarioGroundLocationImaging` example script that demonstrates the aforementioned module integrated into a
  full on-board data system.
- new :ref:`etSphericalControl` module that controls the relative motion of the Electrostatic Tug
- new :ref:`scenarioDebrisReorbitET` example script that demonstrates using the Electrostatic Tug and the
  Multi-Sphere Method
- updated :ref:`groundLocation` to always compute the elevation, range and azimuth information, even if
  the satellite does not have access.  The output message variable ``hasAccess`` provides access information.
- added scripting support for Vizard 1.8.4
- updated :ref:`scenarioGroundLocationImaging` to demonstrate the use of the
  new ``vizSupport.createTargetLine()`` method



Version 2.0.3
-------------
- new integrated scenario in :ref:`scenarioAttitudeConstraintViolation`. Shows how to use the :ref:`boreAngCalc` to display keep-in and keep-out constraint violations while
  performing slew maneuvers.
- new :ref:`locationPointing` module to do 2-axis attitude control which aligns a body-fixed vector to a
  desired inertial location
- new :ref:`scenarioAttLocPoint` example script how to point a spacecraft body axis towards Boulder
- new integrated scenario in :ref:`scenarioAttitudeConstraintViolation`. Shows how to use the :ref:`boreAngCalc` to
  display keep-in and keep-out constraint violations while performing slew maneuvers.
- updated :ref:`inertial3DSpin` to make the attitude input message optional, updated documentation to be RST only
  and more descriptive of the associated math, and changed the module variable ``omega_spin`` to ``omega_RR0_R0``
- enables the message ``recorder()`` module to function if the message structure contains structures itself.
- make the build system compatible with Python 3.8 and higher on Windows
- fixed custom RW support method in ``simIncludeRW.py``
- fixed new C++20 related compiler warnings

Version 2.0.2
-------------
- new waypoint reference module in :ref:`waypointReference`. It can be used to read an attitude maneuver from a set of waypoints on a text file, likely generated outside Basilisk.
- updated :ref:`gravityEffector` to allow the planet message module (``spiceInterface`` or ``planetEphemeris``) to
  be called either before or after the ``spacecraft`` module update is called
- Fix a range of long-standing HTML Documentation build warnings and issues
- Renamed the messages ``CirclesOpNavMsgPayload`` to ``OpNavCirclesMsgPayload`` and
  ``OpNavLimbMsgPayload`` to ``OpNavLimbMsgPayload`` to avoid sphinx naming conflicts
- unified the identical ``ukfUtilities.c/h`` files in ``attDetermination`` and ``opticalNavigation`` folders
  into ``architecture/utilities``
- added a new RW encoder simulation module :ref:`encoder`
- Fixed a bug in the onboardDataHandling module that allowed for data that did not exist to be downlinked
- changed default behavior of ``python3 conanfile.py`` to automatically compile the Basilisk project.  This was
  a common stumbling point for new users.  The build flag ``--buildProject`` can be used to enable automatic
  compiling or not.  For developers making new code this should likely be set to ``False`` when configuring
  the project.
- Fixed a bug in :ref:`SimulationBaseClass` that prevented creating an event with multiple conditions
- added ``ShowExecutionOrder()`` method to :ref:`SimulationBaseClass` to print to the terminal the order that the
  process, tasks and modules are executed.
- added ``ShowExecutionFigure()`` method to :ref:`SimulationBaseClass` to create a figure illustration the
  execution order.
- added a new :ref:`bskPrinciples-2b` web page on how to visualize the BSK process, task and module execution
- added new ``bskSim`` example scenario showing how to alternate between flight modes in :ref:`scenario_AttModes`
- provide scripting support for Vizard 1.8.2 release


Version 2.0.1
-------------
- Added the ability to clear the data of a message recorder using ``.clear()``
- Fixed a rare issue where RW data didn't stick
- Fixed an issue subscribing to a C++ wrapped message object from python
- Cleaned up documentation on using datashaders and bokeh to interactively plot large simulation data sets.
  The script :ref:`scenarioAnalyzeMonteCarlo` is updated to discuss the particular challenges in running this
  datashader example of plotting data.
- enable Monte Carlo ``pytest`` test scripts to run on macOS if Python 3.9 or higher is used
- enable opNav scenario ``pytest`` test scripts to be tested by ``pytest`` if the build flag ``--opNav``
  is set to true and the path to :ref:`Vizard <vizard>` application is set in :ref:`BSK_OpNav`.
- fixed an issue that prevented subscribing to a C++ msg from python
- moved :ref:`cModuleTemplate` and :ref:`cppModuleTemplate` to a common folder ``src/moduleTemplates``.  The
  associated HTML documentation now appears inside the ``Documentation`` tab under ``moduleTemplates``.
- added the ``src/utilities/makeDraftModule.py`` script that is able to create a draft module template given

    - module name
    - module description
    - module location
    - list of module input or output messages containing

      - message variable name
      - message payload definition
      - message description
      - message type (ie. ``C`` or ``C++``)

  The script then generates either a C or C++ module folder that contains the elemental ``*.c/cpp``, ``*.h``, ``*.i``
  code which compiles into a functioning prototype module.  Also included are the module ``*.rst`` file which provides
  the basic description and message table (including hyperlinks to message payload type and message description),
  as well as a functioning python unit test that loads the module, connects zero'd input messages and sets up
  output message recorders.  The coder can then take this draft module code and modify to achieve the desired
  functionality.  The page :ref:`Folder_moduleTemplates` discusses how to use it and provides to 2 sample
  auto-generated modules that get created inside ``src/moduleTemplates`` with ``python conanfile.py``.
- new thermal motor module in :ref:`motorThermal`.  It it be used to simulate the temperature of a RW motor.


Version 2.0.0
-------------
- New message system with strong type checking.  You now get a much simpler method to create message objects,
  how to connect them within python, create stand-alone messages in python, etc.  If you engage with a message
  of the wrong type you get immediate compiler warnings.
- New C++ based message recording system that is much faster than the older python based message logging
- New messaging recording now stores the message data separately from the time a message was recorded
  and the time the message was written
- Removed the arbitrary distinction between ``FSW``, ``SIM`` and ``INT`` messages.  All messages are now
  available to all modules
- Both C and C++ based message interfaces are now auto-generated when running ``python3 conanfile.py`` command
- New ability to create zero'd message structures in the modules
- Seamless message subscribing in Python across all modules types (C, C++ or Python)
- New generic RW device type in :ref:`simIncludeRW` and updated the support library to work with BSK2
- Updated :ref:`simIncludeGravBody` to work with BSK2.  If needed the :ref:`spiceInterface` and
  :ref:`EpochMsgPayload` message is created within the gravity factory class.
- Updated :ref:`simIncludeThruster` to work with BSK2
- Updated :ref:`fswSetupRW` to work with BSK2
- Updated :ref:`fswSetupThrusters` to work with BSK2
- Update Basilisk module documentation that shows all input and output message variables, their
  type and explanation
- Cleaned up the Basilisk `src` folder layout by moving all Basilisk architecture support files
  to `src/architecture`.  This impacts some include statements
- Made the C/C++ ``#include`` statements all relative to `src` to make it easier to find the associated
  files in the source code
- Updated message names to now all comply with the Basilisk message naming convention.  See
  :ref:`migratingToBsk2` for a table of how some message names have changed
- Updated :ref:`vizSupport` to work with BSK2.  It is now much easier to include RW, thruster and CSS devices.
  Further, the simulation gravity bodies don't have to be explicitly provided to the
  ``vizSupport.enableUnityVisualization()`` method.  Rather, these are pulled from the spacecraft object
  directly.
- :ref:`reactionWheelStateEffector` is updated where the list of RW configuration parameters are now linked
  from python, not copied.  As a result it is now possible to stop the simulation and change RW parameters on
  the fly, emulating a failure with a physical change in the RW mechanics.
- changed the output message type of :ref:`magnetometer` to be compatible with :ref:`tamComm`
- Created several instructional pages in the Quick-Start documentation folder.  The examples folder
  has moved to the Quick-Start guide as well.  The new quick start guide now discusses

  - how to write Basilisk python simulation scripts
  - how to write C++, C and Python modules

- Added installation instructions to run Basilisk on a computer with the Apple M1 processor
- added :ref:`spacecraftLocation` module to allow checking for satellite to satellite line-of-sight access
- made ``maximumRange`` an optional variable in :ref:`groundLocation`
- renamed ``spacecraftDynamics`` to :ref:`spacecraftSystem`, and renamed the associated ``spacecraft`` to ``spacecraftUnit()``.
- renamed ``spacecraftPlus()`` to be now simply :ref:`spacecraft`
- renamed the `spacecraftPlus` associated messages to :ref:`SCStatesMsgPayload` and :ref:`SCMassPropsMsgPayload`
- renamed ``fswModuleTemplate()`` to be :ref:`cModuleTemplate`.  This makes this naming consistent with the new :ref:`cppModuleTemplate`.
- renamed `rwMotorVoltageInterface` to :ref:`motorVoltageInterface`.  This motor model can be used for both RW and hinged panel devices.
- added support to creating custom gravity bodies to :ref:`simIncludeGravBody`.  Including support to have custom gravity bodies shown in :ref:`Vizard <vizard>` as well.  The example script :ref:`scenarioCustomGravBody` provides an illustration of this functionality.




**Version 1.8.10**

- Added support and expanded installation instructions making use of virtual environments

**Version 1.8.9**

- Added support for ``Location`` scripting in Vizard 1.7.1
- Added a new documentation page discussing how to launch Vizard from the command line
  and what optional arguments are available

**Version 1.8.8**

- The protobuffer interface files are now automatically created from
  ``src/utilities/vizProtobuffer/vizMessage.proto`` without having to manually run the
  ``protoc`` command each time the protobuffer message definitions changed.
- centerRadiusCNN is now supported on all the platforms
- Support Terminal Progress bar while running a Basilisk simulation
- Improved the build system to re-swig the module if the dependencies have changed.
  This avoids having to do a clean build or manually deleting the swing python files from within ``dist3/Basilisk``.
- All unit test cases are compatible with windows platform
- Added scripting support for Vizard 1.7.0

**Version 1.8.7**

- Updated ``orbitalMotion`` python and C libraries to include the new methods ``hillFrame()``, ``hill2rv()`` and ``rv2hill()``
- Updated :ref:`dualHingedRigidBodyStateEffector` to support an output message of the panel angular states, an output message of the panel inertial position and attitude states, as well as upgrading the module to support ``spacecraftDynamics``.
- Updated :ref:`vizInterface` to support scripting of new Vizard 1.6.1 features

**Version 1.8.6**

- Fixed an issue where some Sim-FSW interface messages could not be written to from the Python layer
- Fixed an issue that prevented the ``opNav`` build mode to compile the OpenCV related libraries
  on macOS with Xcode 12 installed
- renamed ``RWArraytorqueIntMsg`` to ``arrayMotorTorqueIntMsg``
- updated :ref:`hingedRigidBodyStateEffector` to

    - write the panel angle and angle rate output message
    - write the panel inertial and position states as an output message
    - updated document to make use of RST format and specify module input and output messages
- updated ``avsEigenSupport.h`` to add new methods ``eigenMRPd2Vector3d()`` and ``eigenC2MRP()``
- updated ``spacecraftPlus`` to allow the attitude motion to be prescribed through
  an optional input message of type ``attRefMsg``.
- fixed sign issue in :ref:`simpleSolarPanel`
- support Vizard 1.6.0 scripting  



**Version 1.8.5**

- Provide support of Vizard 1.5.1 scripting
- Updated conan to 1.29.2 to address issues building with opNav and support xcode 12
- Disable freetype for windows because of opencv build issues.

**Version 1.8.4**

- update the macOS dependency to use either ``conan~=1.24`` or ``conan>=1.28``.  The later resolves the linking issues
  that ``conan`` had on macOS.  Other platforms can use ``conan>=1.24.0``.
- updated ``vizInterface`` to support the latest features of Vizard 1.5, including the ability to show
  relative trajectories
- updated :ref:`scenarioFormationBasic` example script to show more general orbits and the use
  of the scientific camera sensor scripting
- On Windows the new build system now builds :ref:`vizInterface`


**Version 1.8.3**

- Removed old ``CMakeLists.txt`` files that are no longer needed
- Improved the build process for Linux such that ``vizInterface`` and ``opNav`` related modules are available
  again in Basilisk python scripts.  Thus Linux users can use 1.8.x onwards and still use these enhanced features.
  The similar issue on the Windows platorm is not resolved yet.
- Updated setup instructions to remind the user to delete ``.conan`` folder if upgrading from a BSK version
  prior to 1.8.0
- Added support for Vizard 1.4.1 that allows setting default and thruster group plume colors.  The built-in
  thruster pluming length can be now be custo scaled as well.
- Added a video gallery page to the Vizard documentation section

**Version 1.8.2**

- Updated :ref:`dataFileToViz` to include the ability to read thruster force values.  The spacecraft can have
  multiple thruster sets, and this works for multiple spacecraft as well.  See :ref:`test_dataFileToViz` for an
  example on how to set this up.
- Updated :ref:`dataFileToViz` to include support for reaction wheel data.
- Updated documentation and ``CMakeLists.txt`` to required 3.14 or higher
- Updated how ``openCV`` is included to avoid false Xcode warnings about the library not being installed
- Added :ref:`centerRadiusCNN` for doing CNN-based image processing as well as a pre-trained model
  (read by the module) that allows to extract center and apparent diameter from Mars
  images.  Note that for now this module is only built
  on macOS systems.  As we are able to test on other platforms we will include it there too.
- Added :ref:`scenario_CNNAttOD` to illustrate the use of the CNN-based image processing
- Added support for Vizard v1.4.0 scripting

**Version 1.8.1**

- Added a new folder ``externalTools/fswAuto`` that contains :ref:`Folder_externalTools` to migrate BSK simulations and modules to C-code
- Added a new :ref:`albedo` which can simulate the average or data driven albedo of a single planet.  This works
  also if multiple celestial bodies are setup.
- New :ref:`scenarioAlbedo` to illustrate the use of :ref:`albedo`
- Made the RST HTML document creation work on Windows as well (see :ref:`createHtmlDocumentation`)
- Fixed the conan issues where the IDE only saw the Debug path of the Eigen library, not the Release path.
  This gets rid of false warnings in Xcode that ``<Eigen/Dense>`` could not be found.
- updated the installer script to automatically set the ``conan`` repo information.  This removes one more step
  from the installation process.

**Version 1.8.0**

- updated :ref:`imuSensor` to initialize all class variables in the constructor
- fixed a data frame issue in :ref:`groundLocation`
- first iteration of the CMake refactor completed. The refactor updates the project CMakeList to

    1) conform with more modern CMake practices,
    2) allow developers to include custom dependencies on the module level with Custom.cmake files,
    3) refactors existing SWIG interface files to generate significantly smaller _wrap.c(xx) files,
    4) generates single libraries for GeneralModuleFiles rather than re-including, re-wraping, and
       recompiling those files at the module level. The latter two changes provide significant
       improvements in build time.

- The need for folder module ``__init__.py`` files has been removed.  If local python support files should be
  included in the swig'd module, they can be included in the module ``*.i`` file using something like
  ``%pythoncode "parseSRPLookup.py"``.
- The support files in ``_GeneralModuleFiles`` are now compiled into a library with the parent folder name.  Thus,
  the ``src/simulation/dynamics/_GeneralModuleFiles`` support files yield a swig'd library ``dynamicsLib``.
  Similarly, ``src/simulation/environment/_GeneralModuleFiles`` yields ``environmentLib``.
- Cleaned up small RST documentation issues
- Updated the install process to check automatically for required python packages.  They are not available,
  then the user is prompted to install for user, for the system or cancel.
- Updated the install process to allow a user selectable checking of all optional python packages
  through ``allOptBsk`` flag
- fixed memory issue in the :ref:`camera`
- Updated the HTML documentation process to provide tools to clean out the auto-generated documentation,
  as well as to open the HTML output from the command line

**Version 1.7.5**

- Added the ability to shift the HSV or BGR colors of :ref:`camera`
- Updated :ref:`vizInterface` to allow the user to set the Vizard direct communication protocol, host name and port
  number.
- fixed an issues in :ref:`simIncludeGravBody` where the method ``unloadSpiceKernels`` had the order of the spice package name and the spice path reversed 😟
- New :ref:`dataFileToViz` that reads in spacecraft simulation states from a text file and converts them into
  BSK messages.  For example, this allows :ref:`vizInterface` store the simulation data into a Vizard compatible manner.
- Updated :ref:`spiceInterface` to allow for optional overriding the IAU planet frame with custom values
- Updated :ref:`vizInterface` to allow setting ``show24hrClock`` and ``showDataRateDisplay`` flags for Vizard files
  supported in Vizard v1.3.0 

Version 1.7.4

- hot-fix of an issue compiling Basilisk on Windows.  A ``#define _USE_MATH_DEFINES`` was missing that
  Windows expected, but Unix systems didn't need

**Version 1.7.3**

- updated :ref:`scenarioFormationMeanOEFeedback` and :ref:`scenarioFormationReconfig` to increase
  the orbit altitude to not hit the Earth. Also, added code that can be enabled to record the
  simulation parameters for Vizard.
- updated :ref:`vizInterface` to support the latest Vizard v1.2.0 features.  You can script that the spacecraft
  and/or celestial objects are shown as sprites if they become very small.  This makes it easier to see where
  satellites are in a constellation or formation, as well as where Earth is if orbiting about Mars
- automated how the release number is pulled from a single txt file now


**Version 1.7.2**

- new spacecraft formation flying control :ref:`meanOEFeedback` that implements a mean orbit element feedback
  control law
- new relative orbit control tutorial example :ref:`scenarioFormationMeanOEFeedback` that uses :ref:`meanOEFeedback`
- updated documentation of :ref:`cModuleTemplate` to show how to make much simpler lists of module messages
  using the ``list-table`` RST command
- new spaceraft relative motion control :ref:`spacecraftReconfig` that implements an orbit element based
  impulsive feedback control strategy.  The control is implemented with a thruster model and an
  attitude guidance message is used to point the spacecraft in the correct direction.
- new example scenario :ref:`scenarioFormationReconfig` illustrating the use of the new impulsive relative motion
  control module

**Version 1.7.1**

- Added the ability to detect if a satellite is visible to a ground location in the new :ref:`groundLocation`
- Added support to script Vizard to specify spacecraft, planet and actuator labels
- Added :ref:`spaceToGroundTransmitter` which simulates transmitting data from space to an antenna at a ground location.
- Added a nice new integrated scenario :ref:`scenarioGroundDownlink` that shows how to use :ref:`groundLocation` and :ref:`spaceToGroundTransmitter`
- Updated the definition of the variable noiseMatrix in ``gaussMarkov.h``, and PMatrix in ``simple_nav.h``,
  ``imu_sensor.h`` and ``star_tracker.h``

**Version 1.7.0**

- Fixed a transformation issue in ``avsEigenSupport.cpp`` where ``cArray2EigenMatrix3d()`` has to deal with
  both column and row dominant matrix formulations.  This only got used in :ref:`scenarioCSS` and the issue was offset
  by an issue in ``setUnitDirectionVectorWithPerturbation()`` that compensated.  Now, all is as it should be.
- Removed unneeded instances of using ``unitTestSupport.np2EigenVectorXd()`` when setting the spacecraft states
- Many new Basilisk scenarios illustration interfacing with :ref:`Vizard <Vizard>` to simulate opNav cases:

    - scenario_DoubleOpNavOD uses the two OpNav methods at once
    - :ref:`scenario_faultDetOpNav` implements two OpNav methods and employs a fault detection
    - :ref:`scenario_OpNavAttOD` uses the OpNav FSW stack to perform both pointing towards the target planet
    - :ref:`scenario_OpNavAttODLimb` uses a Canny transform to extract limb points
    - :ref:`scenario_OpNavHeading` point the spacecraft visually towards a target
    - :ref:`scenario_OpNavOD` only performs the orbit determination component
    - :ref:`scenario_OpNavODLimb` only performs the orbit determination component using the Limb based method
    - :ref:`scenario_OpNavPoint` only performs the pointing component
    - :ref:`scenario_OpNavPointLimb` only performs the pointing component using the Limb based method
    - :ref:`scenario_LimbAttOD` performs a longer simulation using the limb based method
    - :ref:`scenario_OpNavAttOD` performs a longer simulation using the Hough transform method

- make :ref:`scenarioVizPoint` work with the latest :ref:`Vizard <Vizard>` scripting methods

    - Add scripting support for the `customGUIScale` parameter
    - All instrument cameras are now specified through `fieldOfView`, not sensor size and focal length
    - Added scripting support to turn on camera boresight line or HUD frustum
    - Made instrument cameras not render images to the home folder by default by setting `renderRate` to zero by default



**Version 1.6.0**

- Fixed the long-standing issue of not being able to run ``pytest`` on Windows from ``src``, but it only ran from
  within sub-folders of ``src``.  Still recommended to run on Windows multi-threaded ``pytest -n XXX``
  using ``pytest-xdist``.
- temporary fix for opencv not finding conan gflags for opencv sfm lib on windows.  See the discussion
  at `<https://github.com/conan-community/community/issues/210>`_
- Updated :ref:`cModuleTemplate` to include a message I/O figure and move it's message definition to ``simMessages``
- Updated the documentation of :ref:`Folder_mrpPD` to the RST format
- Updated the documentation of :ref:`Folder_mrpSteering` to the RST format
- At long last, 🍾, created :ref:`GravityGradientEffector`  which can simulate the gravity gradient torque acting on a
  spacecraft due to the gravitational influence from one or more planets.
- Create a new example script :ref:`scenarioAttitudeGG` that illustrates the use of the gravity gradient effector
- Enhanced the ``GravBodyData`` class to now register the planet position, velocity, orientation and attitude
  rate states.  This allows other effectors, such as the gravity gradient effector, to have access to the current
  planet states at any time step.
- added :ref:`ReactionWheelPower` which can compute the electrical power consumed by a reaction wheel device
- added new example script :ref:`scenarioAttitudeFeedbackRWPower` that illustrates doing a RW-based attitude
  maneuver and tracking the RW power and net battery capacity left.
- added ``BCT_RWP015`` RW model template to the ``simIncludeRW.py`` support file


**Version 1.5.1**

- Fixed an issue running :ref:`test_reactionWheelStateEffector_integrated` using Python 2
- fixed a ``cmake`` issue where the module renaming from ``viz_interface`` to ``vizInterface`` was applied

**Version 1.5.0**

- Updated documentation for :ref:`eclipse` module with new RST format
- Updated :ref:`cModuleTemplate` documentation to show how to add equation numbers, cite equations, do bold math variables and cite a figure caption.
- Updated :ref:`reactionWheelStateEffector` and :ref:`vscmgStateEffector` such that max speed and max torque are consistently initialized to -1.  A negative value was supposed to turn of speed and torque saturation, but this wasn't consistenly applied.
- Updated :ref:`reactionWheelStateEffector` such that the RW state output message was not hard-coded and un-changeable.  Otherwise a BSK process could never have multiple spacecraft being simulated.  Now, the rw effector ``ModelTag`` is added to the beginning of the output message.  This auto-generate method of message output names is avoided if the user sets the vector of output names from Python during the simulation setup.  **Note:** Any prior BSK script that was logging the old auto-generated RW state messages will need to update the msg name now to work again.  See :ref:`bskKnownIssues` for more information.
- Major enhancement to :ref:`vizInterface` where now multiple spacecraft can be added.  You can create a list of spacecraft where :ref:`vizInterface` relies on common naming rules to find the right messages, or specify the messages for each spacecraft directly.  This is demonstrated in :ref:`scenarioFormationBasic`.  For now multiple craft with RW actuators are supported.  Multi craft with thrusters will need to be added later.
- New spacecraft formation flying scenario :ref:`scenarioFormationBasic` where 3 satellites are flying 10m apart in a lead-follower configuration.  Each has a different number of RWs.  This scenario is a nice script to demonstrate the new multi-spacecraft support in :ref:`vizard`.

**Version 1.4.2**

- added link to Basilisk facebook page to Sphinx-based documentation
- made the html documentation compatible with dark mode on macOS, iOS and iPad OS browsers.  If the user sets the system interface to dark mode, then the dark version of the web site is shown automatically.
- added a fix to cmake to get around a ``lipsodium`` and ``conan`` issue we are seeing on a Linux system

**Version 1.4.1**

- added :ref:`Vizard scripting <vizardSettings>` abilities to control the new spacecraft camera view panel behaviors
- added :ref:`Vizard scripting <vizardSettings>` abilities to specify custom CAD OBJ models to replace the default satellite shape
- added  :ref:`Folder_onboardDataHandling` modules for simulating data generated, downlinked, and stored by instruments, transmitters, and storage units onboard a spacecraft. See :ref:`scenarioDataDemo` for a demo.
- updated :ref:`sunlineSuKF` with some general improvements
- tweak to ``cmake`` file to make BSK be portable across Linux systems
- changed the :ref:`bskLogging` level names to make them unique.  This avoids potential variable name conflicts, especially on Windows.

**Version 1.4.0**

- updates to the Monte Carlo controller and plotting algorithms to make use of better use of Pandas and Datashader
- Added a message to the heading estimator in order to perform OpNav pointing
- added a general message to the Sphinx HTML documentation landing page
- updated the :ref:`bskModuleCheckoutList` with updated information and expectations
- Added a fault detection module for optical navigation
- Added camera module to own the message and to add corruptions to images
- Added a new support document :ref:`makingNewBskModule` on getting started writing BSK modules
- Added a new support document :ref:`addSphinxDoc`
- Updated the :ref:`aboutBSK` page to include Basilisk highlights
- Made sure the Monte Carlo unit tests didn't leave any temporary data files behind
- Added new helper functions to the RW and Thruster factory classes to return the equivalent FSW configuration message.  Updated :ref:`scenarioAttitudeFeedbackRW` simulation script to illustrate how to use such a helper function.
- Added a new Basilisk logging system called :ref:`bskLogging`.  This allows modules to print information with a variable verbosity level
- Include a new example scenario :ref:`scenarioBskLog` to illustrate how to use variable verbosity BSK notices

**Version 1.3.2**

- added the ability to include the unit test python files, along with their documentation, within the sphinx html documentation
- updated Vizard live streaming documentation
- updated unit test templates to have better formatting of the html validation report obtained with ``pytest --report``
- exclude some un-needed files from the html documenation
- general sphinx documentation related fixed and enhancements

**Version 1.3.1**

- small fixes to the new HTML documentation
- correct the path includes in Monte Carlo Integrated tests
- updated the ``MRP_Steering`` module documentation to include plots of all test cases

**Version 1.3.0**

- Update template illustrating how the validation accuracy can be recording in the ``pytest`` parameters.
- Created a new method in ``SimulationBaseClass`` called ``pullMultiMessageLogData``  This is much faster in pulling the data log from multiple messages at once.
- It is no longer necessary to call sim.TotalSim.terminateSimulation() at the beginning of Basilisk scripts. This call has been moved to the SimBaseClass constructor and removed from scripts in the repository.
- A new module in the environments directory, SolarFlux, provides the solar flux value at a spacecraft location including (optionally) eclipse effects
- New module in the navigation directory, PlanetHeading, provides the heading to a planet in the spacecraft body frame. There is a corresponding new message type BodyHeadingSimMsg.
- New Sphinx/Breathe based BSK documentation system!  All documentation is still stored in the ``basilisk/docs`` folder.  The new system provides much better directory structure to access the BSK modules, and has a cleaner way to list the tutorial examples.

**Version 1.2.1**

- fixed an issued with the magnetometer module tests not passing on all platforms. The tolerances are now adjusted to pass everywhere.
- various improvements to the ``OpNav`` modules and ``vizInterface``

**Version 1.2.0**

- Making the Python 3 compile flag be turned on by default.  To compile with Python 2 the ``cmake`` flag ``-DUSE_PYTHON3`` can still be set to ``OFF``
- Revised the FSW template module to use the updated in-line module documentation style which adds the description to the module ``*.h`` doxygen description, and adds the validation discussion as a doc-string to the ``test_xxx.py`` test file.
- make sure ``mrpRotation`` is non-singular for any general referene rotation.
- Created a Three-Axis-Magnetometer (TAM) sensor simulation model
- Created a TAM FSW communication model
- Changed the BSK ``ReadMessage()`` method to automatically zero the message memory space before reading in the data
- Added a base classes for battery energy storage and power consumption/provider nodes
- Added a simple power node module
- Added a simpler battery module
- Added a simple solar panel power module


**Version 1.1.0**

- The circle finding module using openCV has been cleaned up and the noise is now dynamically measured given the image
- A new dispersion was added for Monte Carlo analysis which allows for per-axis control on an initial MRP value
- Cleaned up opNav messages to be consistent with other messages, and simplified the limbFinding code. Only functionality change is Gaussian Blur.
- Add new OpNav module using a planet limb. Algorithm developed by J. Christian
- Added support for OpenCV v 4.1.1 and Eigen library 3.3.7
- fixed issue with Windows having trouble compiling due to use of ``uint``
- added instructions on how to use the new Xcode 11 on macOS.  This requires installing 2 more tools.  Updated the install and macOS FAW pages.
- added the ability to ``pytest`` to use the ``--report`` flag to generate a comprehensive html test and validation document.  All future modules should use this method to discuss the module validation.  Legacy modules will be converted over time.
- Corrected an issue with some some BSK modules in a low memory computer environment



**Version 1.0.0 🍾🍾🍾🍾🍾**


.. raw:: html

   <ul>

.. raw:: html

   <li>

Added the ability to plot select BSK simulation data live as teh
simulation is running. See the new tutorials examples and the new FAQ
response page (under Support tab) on how to do this.

.. raw:: html

   </li>

.. raw:: html

   <li>

Lots of code clean up to remove compiler warnings about implicit
signedness conversions, print types, etc.

.. raw:: html

   </li>

.. raw:: html

   <li>

Updated ``scenarioMagneticFieldWMM.py`` scenario to store images into
the correct doxygen folder.

.. raw:: html

   </li>

.. raw:: html

   <li>

[Bugfix] NRLMSISE-00 now defaults to kg/m^3 output, to be consistent
with other atmospheric density models.

.. raw:: html

   </li>

.. raw:: html

   <li>

Added the ability to live stream the Basilisk simulation data to Vizard!
This functions now in addition to saving BSK data to file and playing it
back later on.

.. raw:: html

   </li>

.. raw:: html

   </ul>

**Version 0.9.1**


.. raw:: html

   <ul>

.. raw:: html

   <li>

Created a new attitude guidance module for OpNav: opNavPoint. Similar to
sunSafePoint, it matches a target heading with the OpNav heading for a
simple and robust solution.

.. raw:: html

   </li>

-  added new tutorial on calling Python Spice functions within a Monte Carlo BSK simulation
-  Added Keplerian Orbit utility class which is swig'd. This first implementation takes in elliptical orbit elements and can produce a range of related outputs like position, velocity, orbital period, etc.  This makes it easier to create Keplerian orbits within python.
-  Added a LimbFinding module for OpNav: limbFinding. This module performs a Canny transform to find the end of the planet and saves away the non-zero pixels for pose-estimation. 
- made BSK compatible with both swig version 3 and 4

.. raw:: html

   </ul>

**Version 0.9.0**


.. raw:: html

   <ul>

.. raw:: html

   <li>

Updated the MD help file on how to compile from the command line
environment using a custom configuration of Python.

.. raw:: html

   </li>

.. raw:: html

   <li>

Created new optical navigation filter that estimates bias in the
measurements. This filter takes in pixel and line data directly.

.. raw:: html

   </li>

.. raw:: html

   <li>

Added the ability to specify Vizard settings from Basilisk
``vizInterface`` module settings. This way Basilisk simulations can set
the desired Vizard settings from within the simulation script.

.. raw:: html

   </li>

.. raw:: html

   <li>

Added a new MD help file to discuss the helper methods that setup Vizard
features

.. raw:: html

   </li>

.. raw:: html

   <li>

Added a python helper function to setup cameraConfigMsg message and
create a custom camera view.

.. raw:: html

   </li>

.. raw:: html

   <li>

Added the ability to script what starfield Vizard should use.

.. raw:: html

   </li>

.. raw:: html

   <li>

Made the Vizard helper check that correct keywords are being used.

.. raw:: html

   </li>

.. raw:: html

   <li>

The cmake file now turns ON by default the ``USE_PROTOBUFFERS`` and
``USE_ZMQ`` build flag options. This enables out of the box support for
saving BSK data to Vizard binary files.

.. raw:: html

   </li>

.. raw:: html

   </ul>

**Version 0.8.1**


.. raw:: html

   <ul>

.. raw:: html

   <li>

Added a new kind of dispersion for Monte Carlos which disperses the
orbit with classic orbital elements instead of cartesian postion and
velocity.

.. raw:: html

   </li>

.. raw:: html

   <li>

Added a new module that provides the Earth atmospheric neutral density
using the MSIS model.

.. raw:: html

   </li>

.. raw:: html

   <li>

Updated the Doxygen HTML documentation layout

.. raw:: html

   </li>

.. raw:: html

   </ul>

**Version 0.8.0**


.. raw:: html

   <ul>

.. raw:: html

   <li>

ADDED PYTHON 3 SUPPORT! This is a major step for Basilisk. Python 2
remains suppored, but is now treated as a depreciated capability. It is
possible to compile BSK for P3 into a ``dist3`` folder, and for P2 into
a ``dist`` folder at the same time.

.. raw:: html

   </li>

.. raw:: html

   <li>

Updated the BSK installation notes to reflect a default installation
using Python 3

.. raw:: html

   </li>

.. raw:: html

   <li>

Updated all unit test BSK scripts to work in both Python 2 and 3

.. raw:: html

   </li>

.. raw:: html

   <li>

Updated all tutorial scripts to work in both Python 3 and 2. Default
instructions are now for Python 3

.. raw:: html

   </li>

.. raw:: html

   <li>

Added a new support file with tips on migrating a Python 2 BSK script to
function in both Python 3 and 2. This is called Migrating BSK Scripts to
Python 3.

.. raw:: html

   </li>

.. raw:: html

   </ul>

**Version 0.7.2**


.. raw:: html

   <ul>

.. raw:: html

   <li>

Added a new Earth magnetic field model based on the World Magnetic Model
(WMM). The module has PDF documetnation, and extensive unit test within
the source code folder, as well as a tutorial script demonstrating how
to run this.

.. raw:: html

   </li>

.. raw:: html

   <li>

Updated the ``spice_interface`` module to be able to read in an epoch
message

.. raw:: html

   </li>

.. raw:: html

   <li>

Updated scenarios to use the epoch message

.. raw:: html

   </li>

.. raw:: html

   <li>

Created a new support macro to convert a general date and time string
into an epoch message

.. raw:: html

   </li>

.. raw:: html

   <li>

updated the ``VizInterface`` module to now provide the reaction wheel
and thruster states to Vizard

.. raw:: html

   </li>

.. raw:: html

   <li>

Cleaned up ``VizInterface`` to only subscribe to BSK messages that are
already created

.. raw:: html

   </li>

.. raw:: html

   <li>

Adjust ``simpleNav`` to only subscribe to the sun message it is already
created

.. raw:: html

   </li>

.. raw:: html

   <li>

Update all the tutorial scenario and bskSim simulations to use the
updated ``vizSupport.enableUnityVisualization`` method

.. raw:: html

   </li>

.. raw:: html

   <li>

Fixed and cleaned up bugs in heading and opnav UKFs, pixelLineConverter,
houghCircles, and vizInterface

.. raw:: html

   </li>

.. raw:: html

   <li>

Added validity falg to OpNav messages in order to exclude potential
measurements

.. raw:: html

   </li>

.. raw:: html

   <li>

Fixed camera orientation given the Unity camera frame definition

.. raw:: html

   </li>

.. raw:: html

   <li>

Updated BSK installation instructions to warn about not using swig v4

.. raw:: html

   </li>

.. raw:: html

   </ul>

**Version 0.7.1**


.. raw:: html

   <ul>

.. raw:: html

   <li>

Added a new plotting utility library to support interactive plotting
using datashaders with Python3.

.. raw:: html

   </li>

.. raw:: html

   <li>

Fixed a garbage collecting leak in the monte carlo controller to
minimize impact on computer memory.

.. raw:: html

   </li>

.. raw:: html

   </ul>

**Version 0.7.0**


.. raw:: html

   <ul>

.. raw:: html

   <li>

Added the enableViz method to the bskSim scnearios.

.. raw:: html

   </li>

.. raw:: html

   <li>

Added dvGuidance PDF module description

.. raw:: html

   </li>

.. raw:: html

   <li>

Added new orbital simulation tutorial on a transfer orbit from Earth to
Jupiter using a patched-conic Delta_v

.. raw:: html

   </li>

.. raw:: html

   <li>

Added the first image processing FSW module using OpenCV’s HoughCirlces.

.. raw:: html

   </li>

.. raw:: html

   <li>

Added the a module to convert pixel/line and apparent diameter data from
circle-finding algorithm to a OpNav message with relative position and
covariance.

.. raw:: html

   </li>

.. raw:: html

   <li>

New faceted model for atmospheric drag evaluation

.. raw:: html

   </li>

.. raw:: html

   <li>

Updated RW and Thruster Simulation factory classes to use ordered
dictionary lists. This ensures that the devices are used in the order
they are added.

.. raw:: html

   </li>

.. raw:: html

   <li>

Fixed issue where the Viz would show a custom camera window on startup
if playing back a data file from bskSim scenarios.

.. raw:: html

   </li>

.. raw:: html

   <li>

Added relative Orbit Determination filter (relativeODuKF) in
fswAlgorithms/opticalNavigation. This filter reads measurements treated
by the image processing block to estimate spacecraft position and
velocity

.. raw:: html

   </li>

.. raw:: html

   <li>

Changed the C++ message ID to consitently be of type int64_t, not
uint64_t

.. raw:: html

   </li>

.. raw:: html

   <li>

Rearchitected how data is retained in BSK monte carlo runs using Pandas.
The python pandas package is now required to run MC runs.

.. raw:: html

   </li>

.. raw:: html

   <li>

Updated the CMake to handle both Microsoft Visual Studio 2017 and 2019

.. raw:: html

   </li>

.. raw:: html

   <li>

Added a new attitude control scenario that uses a cluster of thrusters
to produce the required ADCS control torque.

.. raw:: html

   </li>

.. raw:: html

   </ul>

**Version 0.6.2**


.. raw:: html

   <ul>

.. raw:: html

   <li>

hot fix that adds back a missing method in sim_model.c/h that causes the
``enableViz`` support method to not work.

.. raw:: html

   </li>

.. raw:: html

   <li>

updated Viz_Interface module with opNavMode flag. This triggers logic to
link Basilisk and Vizard with a TCP connection. This is ground work for
closed loop visual navigation capabilities.

.. raw:: html

   </li>

.. raw:: html

   <li>

updated enableUnityViz python function in utilities/vizSupport. It now
takes in key word arguments to simplify the user interface. It also
reliably saves Vizard files for play back in the same directory as the
scenario that calls it.

.. raw:: html

   </li>

.. raw:: html

   </ul>

**Version 0.6.1**


.. raw:: html

   <ul>

.. raw:: html

   <li>

Created a user guide MD file that is included in the BSK Doxygen HTML
documentation.

.. raw:: html

   </li>

.. raw:: html

   <li>

Removed the BOOST library from Basilisk as it is no longer needed. This
makes the BSK repository much leaner. Note that this removes the
capability to communicate with the old Qt-based Visualization that is
now defunct and replaced with the new Vizard Visualization.

.. raw:: html

   </li>

.. raw:: html

   <li>

Updated switch unscented kalman filter for sunline estimation with code
cleanup and documentation updates.

.. raw:: html

   </li>

.. raw:: html

   <li>

updated ``pytest`` environment to have markers registered

.. raw:: html

   </li>

.. raw:: html

   <li>

added a PPTX support file that explains the core Basilisk architecture.
HTML documentation is updated to link to this.

.. raw:: html

   </li>

.. raw:: html

   <li>

Creates new simulation module called ``planetEphemeris`` which creates a
planet Spice ephemeris message given a set of classical orbit elements.

.. raw:: html

   </li>

.. raw:: html

   <li>

updated the ``thrMomentumDumping`` module to read in the
``thrMomentumManagement`` module output message to determine if a new
momentum dumping sequence is required.

.. raw:: html

   </li>

.. raw:: html

   <li>

updated the hillPoint and velocityPoint scenarios on how to connect a
planet ephemeris message.

.. raw:: html

   </li>

.. raw:: html

   <li>

updated ``hillPoint`` and ``velocityPoint`` to meet BSK coding
guidelines

.. raw:: html

   </li>

.. raw:: html

   <li>

updated BSK_PRINT macro to automatically now add a new line symbol at
the end of the message

.. raw:: html

   </li>

.. raw:: html

   </ul>

**Version 0.6.0**


.. raw:: html

   <ul>

.. raw:: html

   <li>

Added a new ``vizInterface`` module. This version is able to record a
BSK simulation which can then be played back in the BSK Vizard
visualization program. Vizard must be downloaded separately. To enable
this capabilty, see the scenario tutorial files.
``scenariosBasicOrbit.py`` discusses how to enable this. The python
support macro ``vizSupport.enableUnityVisualization()`` is commented out
by default. Further, to compile ``vizInterface`` the CMake flags
``USE_PROTOBUFFERS`` and ``USE_ZEROMQ`` must be turned on. A new MD FAQ
support file discusses the Cmake options.

.. raw:: html

   </li>

.. raw:: html

   <li>

Updated ``inertialUKF`` module documentation and unit tests.

.. raw:: html

   </li>

.. raw:: html

   <li>

Updated unit test and documentation of ``dvAccumulation``.

.. raw:: html

   </li>

.. raw:: html

   <li>

added a small include change to fix BSK compiling on Windows

.. raw:: html

   </li>

.. raw:: html

   <li>

updated unit test and documentation of ``sunlineEphem()``

.. raw:: html

   </li>

.. raw:: html

   <li>

updated cmake files to set the policy for CMP0086 required by Cmake
3.14.x and higher

.. raw:: html

   </li>

.. raw:: html

   <li>

updated ``thrForceMapping`` module after code review with new expansive
unit tests and updated PDF documentation

.. raw:: html

   </li>

.. raw:: html

   </ul>

**Version 0.5.1**


.. raw:: html

   <ul>

.. raw:: html

   <li>

updated the ``orbitalMotion.c/h`` support library to have more robust
``rv2elem()`` and ``elem2rv()`` functions. They now also handle
retrograde orbits. The manner in covering parabolic cases has changed
slightly.

.. raw:: html

   </li>

.. raw:: html

   <li>

This module implements and tests a Switch Unscented Kalman Filter in
order to estimate the sunline direction.

.. raw:: html

   </li>

.. raw:: html

   <li>

Added documentation to the ``dvAccumulation`` module and included proper
time info in the output message.

.. raw:: html

   </li>

.. raw:: html

   <li>

Providing new support functions to enable the upcoming Vizard Basilisk
Visualization.

.. raw:: html

   </li>

.. raw:: html

   <li>

updated the ‘oeStateEphem()’ module to fit radius at periapses instead
of SMA, and have the option to fit true versus mean anomaly angles.

.. raw:: html

   </li>

.. raw:: html

   <li>

updated
’sunlineSuKF\ ``module which provides a switch Sunline UKF estimation filter.  New documentation and unit tests.     </li>     <li>         updated 'MRP_Steering' module documentation and unit tests     </li>     <li>         updated orbital motion library functions``\ rv2elem()\ ``and elem2rv()``

.. raw:: html

   </li>

.. raw:: html

   <li>

updated ``rateServoFullNonlinear`` module documentation and unit tests.

.. raw:: html

   </li>

.. raw:: html

   </ul>

**Version 0.5.0**


.. raw:: html

   <ul>

.. raw:: html

   <li>

``attTrackingError`` has updated documentation and unit tests.

.. raw:: html

   </li>

.. raw:: html

   <li>

navAggregate module has new documentation and unit tests.

.. raw:: html

   </li>

.. raw:: html

   <li>

small FSW algorithm enhancements to ensure we never divide by zero

.. raw:: html

   </li>

.. raw:: html

   <li>

new unit test for RW-config data

.. raw:: html

   </li>

.. raw:: html

   <li>

included a new environment abstract class that creates a common
interface to space environment modules like atmospheric density, or
magnetic fields in the future. This currently implements the exponential
model, but will include other models in the future. NOTE: this change
breaks earlier simulation that used atmospheric drag. The old
``exponentialAtmosphere`` model usage must be updated. See the
integrated and unit tests for details, as well as the module
documentation.

.. raw:: html

   </li>

.. raw:: html

   <li>

added new documentation on using the new atmosphere module to simulate
the atmospheric density and temperature information for a series of
spacecraft locations about a planet.

.. raw:: html

   <li>

updated documentation and unit tests of ``celestialTwoBodyPoint``

.. raw:: html

   </li>

.. raw:: html

   <li>

added a new planetary magnetic field module. Currently it provides
centered dipole models for Mercury, Earth, Jupiter, Saturn, Uranus and
Neptune. This will be expanded to provide convenient access to other
magnetic field models in the future.

.. raw:: html

   </li>

.. raw:: html

   <li>

updated ``eulerRotation()`` to remove optional output message and did
general code clean-up

.. raw:: html

   </li>

.. raw:: html

   <li>

updated ``mrpRotation()``, new PDF documentation, did code cleanup,
updated unit tests, removed optional module output that is not needed

.. raw:: html

   </li>

.. raw:: html

   <li>

updated ``MRP_Feedback()``, new PDF documentation, did code cleanup,
updated unit tests to cover all code branches.

.. raw:: html

   </li>

.. raw:: html

   <li>

Added a new tutorial on using the magnetic field model.

.. raw:: html

   </li>

.. raw:: html

   <li>

Updated ``mrpMotorTorque()`` with code cleanup, updated doxygen
comments, PDF documentation and comprehensive unit test.

.. raw:: html

   </li>

.. raw:: html

   <li>

Added documentation to ``thrFiringRemainder`` module

.. raw:: html

   </li>

.. raw:: html

   <li>

Added documentation to ``thrFiringSchmitt`` module

.. raw:: html

   </li>

.. raw:: html

   <li>

Updated documentation of ``thrMomentumManagement`` module

.. raw:: html

   </li>

.. raw:: html

   <li>

Updated documentation of ``thrMomentumDumping`` module

.. raw:: html

   </li>

.. raw:: html

   <li>

Added documentation of ``MRP_PD`` module

.. raw:: html

   </li>

.. raw:: html

   <li>

added a new tutorial on how to use the planetary magnetic field model.

.. raw:: html

   </li>

.. raw:: html

   </ul>

**Version 0.4.1**


.. raw:: html

   <ul>

.. raw:: html

   <li>

cssComm has updated documentation and unit tests.

.. raw:: html

   </li>

.. raw:: html

   <li>

updated Documentation on ``rwNullSpace`` FSW module

.. raw:: html

   </li>

.. raw:: html

   <li>

updated how the FSW and Simulation modules are displayed with the
DOxygen HTML documenation, as well as how the messages are shown. Now
the use can click on the “Modules” tab in the web page to find a cleaner
listing of all BSK modules, messages, utilities and architecture
documentation.

.. raw:: html

   </li>

.. raw:: html

   <li>

modified the ``cmake`` file to allow the build type to be passed in from
the command line

.. raw:: html

   </li>

.. raw:: html

   <li>

updated Doxygen documentation on ``cssWlsEst()``

.. raw:: html

   </li>

.. raw:: html

   <li>

updated documentation and unit tests of ``cssComm()`` module

.. raw:: html

   </li>

.. raw:: html

   </ul>

**Version 0.4.0**


.. raw:: html

   <uL>

.. raw:: html

   <li>

Integrated the ``conan`` package management system. This requires conan
to be installed and configured. See the updated Basilisk installation
instructions. It is simple to add this to a current install. Further,
the CMake GUI application can’t be used directly with this
implementation if the app is double-clicked. Either the GUI is launched
form a terminal (see macOS installation instructions), or ``cmake`` is
run from the command line (again see your platform specific installation
instructions). Using ``conan`` now enables BSK to be compiled with
specific support packages, and will allow us to integrate other packages
like OpenCV, Protobuffers, etc. into the near future in a flexible
manner.

.. raw:: html

   </li>

.. raw:: html

   <li>

updated install instructions to allow for pytest version 4.0.0 or newer

.. raw:: html

   </li>

.. raw:: html

   <li>

updated code to remove some depreciated python function call warnings

.. raw:: html

   </li>

.. raw:: html

   <li>

Added a new sun heading module computed exclusively from ephemeris data
and spacecraft attitude (sunlineEphem). Documentation and a unit test
are included.

.. raw:: html

   </li>

.. raw:: html

   <li>

Added a new scenario that shows how to simulate multiple spacecraft in
one simulation instance.

.. raw:: html

   </li>

.. raw:: html

   <li>

Added a spacecraftPointing module that allows a deputy spacecraft to
point at a chief spacecraft. Besides that, added a scenario that
demonstrates the use of this new module.

.. raw:: html

   </li>

.. raw:: html

   <li>

added the ability to the thrForceMapping FSW module to handle thruster
saturation better by scaling the resulting force solution set.

.. raw:: html

   </li>

.. raw:: html

   <li>

Added lots of new unit tests to BSK modules

.. raw:: html

   </li>

.. raw:: html

   <li>

rwNullSpace() module now sets ups module states in reset() instead of
crossInit(), and includes new documentation and unit tests

.. raw:: html

   </li>

.. raw:: html

   </ul>

**Version 0.3.3**


.. raw:: html

   <ul>

.. raw:: html

   <li>

Added a new message output with the CSS fit residuals. This is optional.
If the output message is not set, then this information is not computed.

.. raw:: html

   </li>

.. raw:: html

   <li>

Updated ``sunSafePoint()`` to allow for a nominal spin rate to be
commanded about the sun heading vector. The unit tests and module
documentation is updated accordingly.

.. raw:: html

   </li>

.. raw:: html

   <li>

Added a new scenario ``scenarioAttitudeFeedbackNoEarth.py`` which
illustrates how to do an attitude only simulation without any gravity
bodies present.

.. raw:: html

   </li>

.. raw:: html

   <li>

Updated the macOS Basilisk installation instructions to make them easier
to follow, and illustrate how to use the macOS provided Python along
with all the Python packages installed in the user Library directory.
This provides for a cleaner and easier to maintain Basilisk
installation.

.. raw:: html

   </li>

.. raw:: html

   <li>

Created new switched CSS sun heading estimation algorithms called
``Sunline_SuKF`` and ``Sunline_SEKF``. These switch between two body
frames to avoid singularities, but with direct body rate estimation.
Previous filters ``Sunline_UKF``, ``Sunline_EKF``, and ``OKeefe_EKF``
either subtract unobservability or difference sunheading estimate for a
rate approximation.

.. raw:: html

   </li>

.. raw:: html

   <li>

Updated the Windows specific install instructions to include explicit
steps for setting up and installing Basilisk on machine with a fresh
copy of Windows 10.

.. raw:: html

   </li>

.. raw:: html

   <li>

Added policy statements to the CMake files. This now silences the
warnings that were showing up in CMake 3.12 and 3.13

.. raw:: html

   </li>

.. raw:: html

   <li>

Modified CMake to silence the excessive warnings in XCode that
``register`` class is no depreciated in C++

.. raw:: html

   </li>

.. raw:: html

   </ul>

**Version 0.3.2**


.. raw:: html

   <ul>

.. raw:: html

   <li>

Fixed an issue with the eclipse unit test.

.. raw:: html

   </li>

.. raw:: html

   <li>

updated the installation instructions to warn about an incompatibility
between the latest version of ``pytest`` (version 3.7.1). Users should
use a version of ``pytest`` that is 3.6.1 or older for now until this
issue is resolved.

.. raw:: html

   </li>

.. raw:: html

   <li>

Updated the ``.gitignore`` file to exclude the ``.pytest_cache`` folder
that pytest generates with the newer versions of this program

.. raw:: html

   </li>

.. raw:: html

   </ul>

**Version 0.3.1**


.. raw:: html

   <ul>

.. raw:: html

   <li>

Tutorials added for BSK_Sim architecture. Added the ability to customize
the frequency for FSW and/or dynamics modules.

.. raw:: html

   </li>

.. raw:: html

   <li>

Updated the dynamics thruster factor classes. This streamlines how
thrusters can be added to the dynamics. Also, a new blank thruster
object is included in this factory class to allow the user to specify
all the desired values.

.. raw:: html

   </li>

.. raw:: html

   <li>

bskSim now adds 8 thrusters to the spacecraft. These are not used yet,
but will be in future bskSim scenarios.

.. raw:: html

   </li>

.. raw:: html

   <li>

Modified how bskSim now includes CSS sensors in the spacecraft dynamics
setup

.. raw:: html

   </li>

.. raw:: html

   <li>

Modified the FSW ``sunSafePoint()`` guidance module to read in the body
angular velocity information from standard ``NavAttIntMsg``. This will
break any earlier simulation that uses ``sunSafePoint()``.

.. raw:: html

   <ul>

.. raw:: html

   <li>

FIX: update the ``sunSafePoint()`` input connection to use the current
message format.

.. raw:: html

   </li>

.. raw:: html

   </ul>

.. raw:: html

   </li>

.. raw:: html

   <li>

Fixed an issue with energy not conserving if the fully coupled VSCMG
imbalance model is used. This imbalanced gimbal and wheel version now
conserves momentum and energy!

.. raw:: html

   </li>

.. raw:: html

   <li>

Added initial draft of VSCMG module documentation

.. raw:: html

   </li>

.. raw:: html

   <li>

Added documentation to all the bskSim scenarios inside
``src/test/bskSimScenarios``. The documentation now outlines how the
bskSim class can get setup and used to create complex spacecraft
behaviors with little code.

.. raw:: html

   </li>

.. raw:: html

   </ul>

**Version 0.3.0**


.. raw:: html

   <ul>

.. raw:: html

   <li>

Updated cssWlsEst() module to also compute a partial angular velocity
vector.

.. raw:: html

   </li>

.. raw:: html

   <li>

New FSW Guidance module ``mrpRotation()`` to perform a constant body
rate rotation. The initial attitude is specified through a MRP set.

.. raw:: html

   </li>

.. raw:: html

   <li>

Enhanced Linux installation instructions

.. raw:: html

   </li>

.. raw:: html

   <li>

Updated the simIncludeThruster to use the same factor class as the RW
factory class. This will break old scripts that use the old method of
setting up Thrusters with this helper function.

.. raw:: html

   <ul>

.. raw:: html

   <li>

FIX: Update the script to use the new factory class. Examples are seen
in
``src/simulation/dynamics/Thrusters/_UnitTest/test_thruster_integrated.py``.

.. raw:: html

   </li>

.. raw:: html

   </ul>

.. raw:: html

   </li>

.. raw:: html

   <li>

Updated bskSim to use the RW factory class to setup the simulation RW
devices, as well as to use fsw helper functions to setup the RW FSW
config messages

.. raw:: html

   </li>

.. raw:: html

   <li>

At supportData/EphermerisData, updated the leap second kernel version to
from 0011 to 0012.

.. raw:: html

   </li>

.. raw:: html

   <li>

Added a force and torque calculation method in the stateEffector
abstract class, and provided the necessary method calls in
``spacecraft``. This allows for stateEffectors to calculate the force
and torque that they are imparting on the rigid body hub. The
hingedRigidBodyStateEffector and the linearSpringMassDamper classes
provide their implementation of these calculations.

.. raw:: html

   </li>

.. raw:: html

   <li>

Fixed an issue with ``extForceTorque`` effector where the flag about
having a good input message was not being initialized properly. This
caused a rare failure in the unit test.

.. raw:: html

   </li>

.. raw:: html

   <li>

Reaction wheel state effector has an updated friction model that allows
the user to implement coulomb, viscous, and static friction.

.. raw:: html

   </li>

.. raw:: html

   <li>

Reaction wheel state effector now has max torque saturation logic in
which the wheels can only implement a maximum wheel torque and max wheel
speed saturation logic in which if the wheel speed goes over the maximum
wheel speed, then the wheel torque is set to zero.

.. raw:: html

   </li>

.. raw:: html

   <li>

A new method called writeOutputStateMessages was added to the
stateEffector abstract class which allows for stateEffectors to write
their states as messages in the system and the states will always be
written out to the system after integration. This fixed an issue with
reaction wheels where the commanded torque information needs to be
tasked before the spacecraft but the reaction wheel state messages need
to be written out after integration.

.. raw:: html

   </li>

.. raw:: html

   <li>

A new dynamics class called ``spacecraftDynamics`` has been created.
This allow multiple complex spacecraft systems to be either rigidly
connected or free-flying. This allow for example a mother craft to house
a daughter craft which has its own RWs, etc, and then release the
daughter craft at a specified time.

.. raw:: html

   </li>

.. raw:: html

   <li>

Cleaned up the gravity effector class variable names, and streamlined
the evaluation logic. The gravity effector documentation has been
updated to include information on the the multi-body gravity
acceleration is evaluated.

.. raw:: html

   </li>

.. raw:: html

   <li>

Updated the FSW modules ``MRP_Feedback``,\ ``MRP_Steering``,
``dvAccumulation`` and ``oeStateEphem`` to zero out the output message
first in the ``Update()`` routine.

.. raw:: html

   </li>

.. raw:: html

   <li>

Fixed an issue with the RW factory class and the Stribeck friction model
not being turned off by default.

.. raw:: html

   </li>

.. raw:: html

   <li>

added a new bskSim based tutorial scenario that illustrates a
sun-pointing control while the spacecraft goes through a planets shadow.

.. raw:: html

   </li>

.. raw:: html

   </ul>

**Version 0.2.3 (June 12, 2018)**


.. raw:: html

   <ul>

.. raw:: html

   <li>

Improved how the ``fuelSloshSpringMassDamper`` effector class works. It
is now renamed to ``LinearSpringMassDamper``. It can be used to simulate
both fuel sloshing, but also structural modes. If the
``LinearSpringMassDamper`` is connected to a fuel tank, then it’s mass
depends on the amount of fuel left. The associated unit test illustrated
how to setup this last capability. The module also contains
documentation on the associated math.

.. raw:: html

   </li>

.. raw:: html

   <li>

A new ``SphericalPendulum`` effector class has been added. For rotations
a spherical pendulum is a better approximation rotational fuel slosh.
This effector can model rotational fuel slosh if connected to a tank
(see unit test again), or it can model a torsional structural mode if
not connected to a tank. Associated math documentation is included with
the class.

.. raw:: html

   </li>

.. raw:: html

   <li>

The booleans useTranslation and useRotation have been removed from the
``HubEffector()`` class. The defaults in hubEffector for mass
properties: ``mHub = 1``, ``IHubPntBc_B = diag``\ (1), and
``r_BcB_B = zeros(3)``, enable us to evaluate the same code no matter if
the desire is only to have translational states, only rotational states,
or both. This allows for less logic in hubEffector and removes
possibility of fringe cases that result in unexpected results from a
developer standpoint. The fix for if your python script is not working
related to this change:

.. raw:: html

   <ul>

.. raw:: html

   <li>

FIX: Remove any instances of useTranslation or useRotation defined in
the hubEffector class.

.. raw:: html

   </li>

.. raw:: html

   </ul>

.. raw:: html

   <li>

Changed name of the method ``computeBodyForceTorque`` to
``computeForceTorque`` in the ``dynamicEffector`` abstract class and any
inheriting classes. This avoids the confusion of thinking that only body
frame relative forces can be defined, but in reality this class gives
the ability to define both external forces defined in the body frame and
the inertial frame.

.. raw:: html

   </li>

.. raw:: html

   <li>

Fixed an issue in ``RadiationPressure`` where the cannonball model was
not computed in the proper frame. An integrated test has been added, and
the unit test is updated. Note that the ``RadiationPressure`` model
specification has changes slightly. The default model is still the
cannonball model. To specify another model, the python methods
``setUseCannonballModel()`` or ``setUseFacetedCPUModel()`` are used.
Note that these take no argument anymore.

.. raw:: html

   <ul>

.. raw:: html

   <li>

FIX: remove the argument from ``setUseCannonballModel(true)`` and use
the methods ``setUseCannonballModel()`` or ``setUseFacetedCPUModel()``
without any arguments instead.

.. raw:: html

   </li>

.. raw:: html

   </ul>

.. raw:: html

   </li>

.. raw:: html

   </ul>

**Version 0.2.2 (May 14, 2018)**


.. raw:: html

   <ul>

.. raw:: html

   <li>

Fixed a build issues on the Windows platform is Visual Studio 2017 or
later is used.

.. raw:: html

   </li>

.. raw:: html

   <li>

Unified the Coarse Sun Sensor (CSS) sun heading filtering modules to use
the same I/O messages. All used messages are now in the fswMessage
folder.

.. raw:: html

   </li>

.. raw:: html

   <li>

Made the CSS sun heading filter messages consistently use the CBias
value. This allows particular sensors to have an individual (known)
scaling correction factor. For example, if the return of one sensor is
10% stronger then that of the other sensors, then CBias is set to 1.10.
Default value is 1.0 assuming all CSS units have the same gain.

.. raw:: html

   </li>

.. raw:: html

   <li>

The ``src\tests\bskSimScenarios`` folder now functions properly with the
``bskSim`` spacecraft class.

.. raw:: html

   </li>

.. raw:: html

   <li>

The tutorial scripts in ``src\tests\scenarios`` are now simplified to
pull out the unit testing functionality. The unit testing is now down
with the ``test_XXX.py`` scripts inside the ``src\tests\testScripts``
folder.

.. raw:: html

   </li>

.. raw:: html

   <li>

The ``bskSim`` tutorial files are now tested through pytest as well. The
file ``testScripts\bskTestScript.py`` calls all the ``bskSim`` tutorial
fails and ensures they run without error.

.. raw:: html

   </li>

.. raw:: html

   </ul>

**Version 0.2.1**


.. raw:: html

   <ul>

.. raw:: html

   <li>

Added messages for current fuel tank mass, fuel tank mDot, and thruster
force and torque

.. raw:: html

   </li>

.. raw:: html

   <li>

Changed the linearAlgebra.c/h support library to avoid using any dynamic
memory allocation.

.. raw:: html

   </li>

.. raw:: html

   <li>

Added some new function to linearAlgebra.c/h while making the library
use the new BSK_PRINT() function.

.. raw:: html

   </li>

.. raw:: html

   <li>

Added ability to simulate noise to the RW devices.

.. raw:: html

   </li>

.. raw:: html

   <li>

Created a more complete spacecraft python simulation class called
BSKsim, and recreated some BSK tutorial scripts to use BSKsim instead of
the more manual spacecraft setup in the earlier scripts.

.. raw:: html

   </li>

.. raw:: html

   <li>

Developed general functions to add saturation, discretization and Gauss
Markov processes to signals.

.. raw:: html

   </li>

.. raw:: html

   <li>

Created a new BSK_PRINT() function. Here the coder can tag a message as
an ERROR, WARNING, DEBUG or INFORMATION status. The printout can be set
to selectively show these print statements.

.. raw:: html

   </li>

.. raw:: html

   </ul>

**Version 0.2.0 (First public beta)**


.. raw:: html

   <ul>

.. raw:: html

   <li>

First open beta release of Basilisk

.. raw:: html

   </li>

.. raw:: html

   <li>

Moved to a new file architecture. This means older BSK python scripts
need to be updated as the method to import BSK has changed.

.. raw:: html

   </li>

.. raw:: html

   <li>

The source an now be forked from Bitbucket

.. raw:: html

   </li>

.. raw:: html

   <li>

Precompiled binaries are provided through a python pip install wheel
file.

.. raw:: html

   </li>

.. raw:: html

   <li>

The Doxygen documentation now pulls in the BSK module description PDF
file and makes it available via the class definition html page.

.. raw:: html

   </li>

.. raw:: html

   <li>

The tutorial python scripts are now moved to ``src/test/scenarios``

.. raw:: html

   </li>

.. raw:: html

   <li>

The ``pytest`` common should now be run within the ``src`` sub-directory

.. raw:: html

   </li>

.. raw:: html

   <li>

Updated fuel slosh model documentation

.. raw:: html

   </li>

.. raw:: html

   <li>

Updated fuel tank documentation

.. raw:: html

   </li>

.. raw:: html

   <li>

Adding noise and corruptions using a new utility to the BSK modules (in
progress)

.. raw:: html

   </li>

.. raw:: html

   <li>

New N-panel hinged rigid body module

.. raw:: html

   </li>

.. raw:: html

   <li>

New 2-panel hinged rigid body module

.. raw:: html

   </li>

.. raw:: html

   <li>

Added CSS sun-heading estimation tutorial script

.. raw:: html

   </li>

.. raw:: html

   <li>

Added O’Keefe CSS sun-heading estimation module

.. raw:: html

   </li>

.. raw:: html

   </ul>

**Version 0.1.7**


.. raw:: html

   <ul>

.. raw:: html

   <li>

New Monte-Carlo capability that uses multiple cores and hyperthreading
to accelerate the MC evaluations. Data is retained and stored for each
MC run for robustness. See ``test_scenarioMonteCarloAttRW.py`` for an
example.

.. raw:: html

   </li>

.. raw:: html

   <li>

Coarse Sun Sensor (CSS) modules can now scale the sensor output with the
distance from the sun.

.. raw:: html

   </li>

.. raw:: html

   <li>

CSS now have updated documentation that includes validation results.

.. raw:: html

   </li>

.. raw:: html

   <li>

CSS, IMU have updated means to apply sensor corruptions.

.. raw:: html

   </li>

.. raw:: html

   <li>

IMU, simple_nav and star tracker modules have been updated to use now
internally Eigen vectors rather than C-Arrays. NOTE: if you have
simulation scripts that use these modules you may have to update the
script to set sensor states as Eigen vectors from python.

.. raw:: html

   </li>

.. raw:: html

   <li>

All the dynamics, thruster and sensor simulation modules have expanded
documentation and valdiation unit and integrated tests. The validation
results are automatically included in the module TeX documentation.

.. raw:: html

   </li>

.. raw:: html

   </ul>

**Version 0.1.6**


.. raw:: html

   <ul>

.. raw:: html

   <li>

new unit tests to validate the multi-body gravity simulation code in
``SimCode/dynamics/gravityEffector/_UnitTest/test_gavityDynEffector.py``

.. raw:: html

   </li>

.. raw:: html

   <li>

new hinged rigid body tutorial script in
``SimScenarios/test_scenarioAttGuideHyperbolic.py``

.. raw:: html

   </li>

.. raw:: html

   <li>

new tutorial to do velicity frame pointing on a hyperbolic orbit in
``SimScenarios/test_scenarioHingedRigidBody.py``

.. raw:: html

   </li>

.. raw:: html

   <li>

fixed various unit test issues that came up on the non-macOS builds

.. raw:: html

   </li>

.. raw:: html

   <li>

added reaction wheel effector documentation

.. raw:: html

   </li>

.. raw:: html

   <li>

added ``orb_elem_convert`` documentation

.. raw:: html

   </li>

.. raw:: html

   <li>

added ``boreAngCalc`` documentation

.. raw:: html

   </li>

.. raw:: html

   </ul>

**Version 0.1.5**


.. raw:: html

   <ul>

.. raw:: html

   <li>

Lots of new module documentation which includes a discussion of what is
being modeled, the validation tests, as well as a user guide to the
module. The new documentation includes:

.. raw:: html

   <ul>

.. raw:: html

   <li>

Thruster ``DynEffector`` module

.. raw:: html

   </li>

.. raw:: html

   <li>

ephemeris conversion module

.. raw:: html

   </li>

.. raw:: html

   <li>

Coarse Sun Sensor module

.. raw:: html

   </li>

.. raw:: html

   <li>

Updated BSK module template documentation

.. raw:: html

   </li>

.. raw:: html

   <li>

Updated documentation for IMU Sensor module

.. raw:: html

   </li>

.. raw:: html

   <li>

Gravity Effector module

.. raw:: html

   </li>

.. raw:: html

   <li>

SimpleNav Sensor module

.. raw:: html

   </li>

.. raw:: html

   <li>

Hinged Panel ``StateEffector`` module

.. raw:: html

   </li>

.. raw:: html

   </ul>

.. raw:: html

   </li>

.. raw:: html

   <li>

New tutorial scripts on

.. raw:: html

   <ul>

.. raw:: html

   <li>

using CSS modules

.. raw:: html

   </li>

.. raw:: html

   <li>

using fuel tank module and the fuel slosh particle ``StateEffector``

.. raw:: html

   </li>

.. raw:: html

   <li>

How to use ``MRP_Steering()`` along with the rate tracking sub-servo
module

.. raw:: html

   </li>

.. raw:: html

   </ul>

.. raw:: html

   </li>

.. raw:: html

   <li>

The CSS modules now use the planetary shadow message information to
simulated being in a planet’s shadow

.. raw:: html

   </li>

.. raw:: html

   <li>

SRP DynEffector modules now simulates the impact of being in a planets
shadow

.. raw:: html

   </li>

.. raw:: html

   <li>

Included a method to validate all the AVS C-Function libraries like
``rigidBodyKinematics``, ``linearAlgebra`` and ``orbitalMotion`` when
the Basilisk ``pytest`` command is called. There is also some
documentation on using these libraries in
``/SimCode/utilitiesSelfCheck/_Documentation``

.. raw:: html

   </li>

.. raw:: html

   <li>

Updated the RW and gravitational body (i.e. adding Earth, sun, etc. to
the simulation) to use new factory classes. If you did use the older
``simIncludeRW.py`` or ``simIncludeGravity.py`` libraries, you’ll need
to update your python code to work with the new factory classes.

.. raw:: html

   </li>

.. raw:: html

   </ul>

**Version 0.1.4**


.. raw:: html

   <ul>

.. raw:: html

   <li>

A planetary eclipse model has been added. This allows for the shadow of
one or multiple planets to be taken into account, including the penumbra
region. This module writes an output message indicating if the
spacecraft is in full sun light, partial shadow, or full shadow of the
sun.

.. raw:: html

   </li>

.. raw:: html

   <li>

The body-fixed spacecraft structure frame has now been removed from the
simulation and flight algorithm codes. All spacecraft vectors and
tensors are now set directly in the body frame B. If the spacecraft
parameters are given in terms of an alternate structure frame, these
vectors and tensor must be transformed into the body frame first before
being set in BSK.

.. raw:: html

   </li>

.. raw:: html

   <li>

The integrated tutorial test for using a Python based BSK module now has
some documentation.

.. raw:: html

   </li>

.. raw:: html

   <li>

Created a method to compute the orbital potential and angular momentum
energy. This allows for the kinetic energy and angular momentum checks
to flat-line even if the satellite is in orbit. The spherical harmonics
of the planet are taken into account as well.

.. raw:: html

   </li>

.. raw:: html

   <li>

Included a new Extended Kalman Filter module that determines the
body-relative sun heading using the CSS signals.

.. raw:: html

   </li>

.. raw:: html

   </ul>

**Version 0.1.3**


.. raw:: html

   <ul>

.. raw:: html

   <li>

There is a new capability to now write BSK modules in Python, and
integrated them directly with the C and C++ BSK modules. Documentation
is still in progress, but a sample is found in
SimScenarios/test_scenarioAttitudePythonPD.py.

.. raw:: html

   </li>

.. raw:: html

   <li>

A new Variable Speed Control Moment Gyroscope (VSCMG) state effector
module has been created. This module provides a torque-level VSCMG
simulation which also includes the gyro frame or wheel being imbalanced.
If the latter modes are engaged, the simulation does slow down
noticeably, but you get the full physics.

.. raw:: html

   </li>

.. raw:: html

   <li>

In the simulation the initial spacecraft position and velocity states
are now specified now using the spacecraft center of mass location C,
not the body fixed point B. This greatly simplifies the simulation
setup. Upon initialization, the sim determines what the true center of
mass of the spacecraft is using all time varying mass components, and
sets the proper B point position and velocity vectors.

.. raw:: html

   </li>

.. raw:: html

   <li>

Specifying the initial spacecraft position and velocity states can now
be done anywhere before the BSK initialization. The user sets init
versions of the position and velocity vectors. The setState() method on
the state engine thus doesn’t have to be used.

.. raw:: html

   </li>

.. raw:: html

   <li>

There is a new initializeSimulationAndDiscover method to init the BSK
simulation that automatically checks if messages are shared across
multiple simulation threads. See the modified
SimScenarios/test_scenarioAttitudeFeedback2T.py file for how this
simplifies the dual-threaded setup.

.. raw:: html

   </li>

.. raw:: html

   <li>

The MRP_Steering and PRV_Steering FSW modules have been broken up into a
separate kinematic steering command (commanded desired angular velocity
vector) and an associated angular velocity servo module name
rateServoFullNonlinear. This will break any existing code that used
either of these two attitude steering modules. The Python simulation
code must be updated to to account for these new modules as done in the
MRP_Steering integrated test test_MRP_steeringInt.py.

.. raw:: html

   </li>

.. raw:: html

   </ul>

**Version 0.1.2**


.. raw:: html

   <ul>

.. raw:: html

   <li>

All unit and integrated tests now pass on Linux. The root issue was a
variable length string variable in an output message. These strings have
now been removed as they are no longer needed.

.. raw:: html

   </li>

.. raw:: html

   <li>

The position and velocity of the center of mass of the spacecraft was
added to the messaging system, so now the spacecraft’s translational
states can be logged by the center of mass of the spacecraft (r_CN_N and
v_CN_N) or the origin of the body frame which is fixed to the hub
(r_BN_N and v_BN_N). Additionally, the mass properties of the spacecraft
was organized into an updateSCMassProps method that incapsulates the
calculations of mass property calculations.

.. raw:: html

   </li>

.. raw:: html

   <li>

Updated UKF FSW module to be able to run on gryo only information when
the star tracker is not available.

.. raw:: html

   </li>

.. raw:: html

   </ul>

**Version 0.1.1**

- On Linux, simplified the processing running BSK modules that require
  boost. This makes the Viz related communication modules working again.
- Added boost libs built on Ubuntu against gcc 5.4.0 20160609.
- Added RPATH settings to allow for build directory to be placed outside
  source directory
- Major addition with new depleatable mass dynamic modeling, including
  some fuel tank dynamic models.
- minor fix for Monte Carlo dispersions


**Version 0.1.0**


**Simulation modules include:**

.. raw:: html

   <ul>

.. raw:: html

   <li>

    Flexible integration structure with fixed time step RK1, RK2 and RK4
    included</li>

    <li>Rigid spacecraft simulated through <code>spacecraftPlus()</code> module.  The spacecraft object makes it simple to add external disturbances through <code>dynEffectors</code> and state depended actuation through <code>stateEffectors</code>.
      <ul>
          <li>Dynamics Effectors (actuation methods which do not have their own states to integrate)</li>
              <ul>
                  <li>External force or torque module</li>
                  <li>Solar radiation pressure module</li>
                  <li>Thruster module</li>
              </ul>
          <li>State Effectors (actuation methods which have states to integrate)</li>
              <ul>
                  <li>Fuel Tank model with fuel slosh particles</li>
                  <li>Hinged panel model to simulate flexing structures such as solar panels</li>
                  <li>Reaction wheel module with 3 modes (perfectly balanced, simple jitter with the disturbance modeled as an external force and torque, fully coupled imbalanced RW model)
              </ul>
       </ul>
       <li>RW voltage interface module that mapes an input voltage to a RW motor torque</li>
       <li>integrate Spice ephemeris information</li>
       <li>simple navigation module that produces the position and attitude measurement states</li>
       <li>IMU sensor</li>
       <li>Star Tracker module</li>
       <li>Coarse Sun Sensor (CSS) module</li>
       <li>Added the ability to simulate the gravity from multiple celestial objects, as well as include spherical harmonic expansion of a particular celestial body.</li>

.. raw:: html

   </ul>

**The AVS Lab Flight Algorithm folder contains:**

- FSW template module
- CSS based sun heading estimation module
- UKF filter to determine inertial attitude
- UKF filter to determine CSS based body-relative sun heading
- Attitude Guidance modules:

    - Pointing towards two celestial objects
    - Inertial Pointing
    - Hill Frame Pointing
    - Euler rotation sequence to add dynamics capabilities to the attitude reference generation
    - Spinning about an inertially fixed axis
    - A raster manager module that can change the guidance module states
    - Velocity frame pointing
    - attitude tracking error evaluation module
    - Deadband module for attitude tracking error
- DV guidance module
- Effector Interfaces:

    - mapping of control torque onto RW motor torques
    - Converting RW motor torques to voltages
    - RW null motion module to equalize the wheel speeds continuously
    - Thruster (THR) firing logic using a Schmitt trigger
    - THR firing logic using a remainder calculation
    - mapping of a command torque onto a set of THR devices
    - module to evaluate the net momentum to dump with thrusters
