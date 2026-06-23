
.. _bskKnownIssues:

Basilisk Known Issues
=====================

.. caution::

    The use of ``cMsgCInterfacePy`` is deprecated.  Use ``messaging`` instead.

Version |release| (July 7, 2026)
--------------------------------
- SWIG 4.4.0 caused Basilisk build failures in some Python 3.13+ source-build configurations.
  Basilisk now requires SWIG 4.4.1 or a newer supported 4.x release, which provides SWIG ABI 5 support
  for Basilisk and compatible extensions. If source builds fail with SWIG 4.4.0 or emit
  ``builtin type swigvarlink has no __module__ attribute`` warnings, upgrade to SWIG 4.4.1 or a newer
  supported 4.x release.
- ``python conanfile.py --clean`` now removes Basilisk Numba cache artifacts in addition to ``dist3``,
  preventing clean source builds from reusing stale compiled Numba objects after API changes.
- The :ref:`sphericalPendulum` fuel-slosh effector applied its viscous damping force without the pendulum
  moment arm, so an isotropic damping matrix ``D`` dissipated no rotational energy and an anisotropic ``D``
  could add energy to the spacecraft. This is fixed in the current version.
- BSK-676: A stand-alone message that went out of scope in Python could be garbage collected while a
  C++ module input message or recorder still pointed into it, causing the reader to silently access
  freed memory. Subscribing a C++ ``ReadFunctor`` input message, calling ``Message.addSubscriber()``,
  or calling ``Message.recorder()`` now keeps the source alive for the life of the reader or recorder,
  so messages created inside Python setup or helper functions no longer need to be manually retained.
  This is fixed in the current version. Note: C-module input messages (``Msg_C`` readers, e.g. a C
  module's ``dataInMsg``) are not yet covered by this automatic keep-alive and still require retaining
  the source message in a persistent scope (see BSK-1107); this case is tracked in pull request
  #1442.


Version 2.11.0 (July 7, 2026)
-----------------------------
- BSK-422: Several SWIG-wrapped C++ members in simulation and FSW modules could emit
  ``swig/python detected a memory leak`` warnings when read from Python because their wrappers lacked
  visible destructors. Internal-only members are now private or hidden from Python, and public value-type
  members now include the required Eigen, STL, enum, and BSpline wrapper support. This is fixed in the
  current version.
- BSK-469: The :ref:`hingedRigidBodyStateEffector` and :ref:`nHingedRigidBodyStateEffector` constructors
  called ``Eigen``'s static ``Identity()`` factory as a statement and discarded the result, so the default
  ``dcm_HB`` (and ``IPntS_S`` for the single hinged effector) held uninitialized memory instead of identity.
  Configurations that set these members explicitly (all shipped examples and tests do) were unaffected.
- BSK-469: The spacecraft hub properties were not validated, so a zero hub mass or a singular hub
  inertia tensor silently produced ``NaN`` states, and a negative hub mass silently reversed the
  translational response to applied forces. :ref:`spacecraft` and :ref:`spacecraftSystem` now verify
  on reset that ``mHub`` is strictly positive and ``IHubPntBc_B`` is symmetric positive definite.
  This is fixed in the current version.
- Extension-generated custom messages using ``bsk_generate_messages(GENERATE_C_INTERFACE)`` could fail to subscribe
  Python readers to C-interface output messages because generated bindings imported the built-in
  ``Basilisk.architecture.messaging`` package. Generated bindings now resolve peer message classes from their own
  module. This is fixed in the current version.
- BSK-1446: Several BSK modules deallocated output message objects created with C++ ``new`` by calling
  ``free()``, and retained image buffers in :ref:`camera` and :ref:`vizInterface` were not released during
  module destruction. This could cause invalid deallocation behavior or memory leaks when these modules were
  destroyed. These cleanup paths now use C++ ``delete`` for the message objects and release the retained image
  buffers. This is fixed in the current version.
- BSK-788: Message recorders stored their recorded-message and time history in ``std::vector``, so the
  recorder's ``UpdateState`` incurred periodic reallocation-and-copy timing spikes at power-of-two record
  counts that grew with recording length, disrupting soft real-time and hardware-in-the-loop simulations.
  Recorders now store this history in ``std::deque``. This is fixed in the current version.
- BSK-2026-001, BSK-2026-002, BSK-2026-003, and related fixed-buffer string handling and format-string
  logging issues are fixed in the current version.
- Additional build-helper command execution, temporary file cleanup, remote example download, and image buffer
  validation hardening is included in the current version.
- A Vizard ``noDisplay`` image request regression that caused OpNav scenarios to stop with a
  ``Vizard image request acknowledgement was not received`` error is fixed in the current version.
- The camera module now passes PNG encoding options as OpenCV key-value pairs, fixing OpNav image
  processing failures with OpenCV 4.13.
- The camera module filename input was only used to test filters and never published to imageOut. This is fixed
  so images loaded from filename now follow the same processing and publishing pipeline as images from imageInMsg.
- The MuJoCo dynamics wrapper now uses MuJoCo 3.7 element-name APIs, fixing builds after the MuJoCo
  3.7 upgrade.
- Monte Carlo now applies nested dispersion paths and randomized ``RNGSeed`` modifications to the actual
  simulation objects.
- The :ref:`linearTranslationNDOFStateEffector` getter assertion no longer references the spinning-body-only
  degree-of-freedom counter, fixing source builds where that assertion is compiled. The related
  :ref:`spinningBodyNDOFStateEffector` getter assertion now rejects the first out-of-range index.
- SWIG 4.4.0 caused Basilisk build failures in some Python 3.13+ source-build configurations.
  Basilisk now requires SWIG 4.4.1 or a newer supported 4.x release, which provides SWIG ABI 5 support
  for Basilisk and compatible extensions. If source builds fail with SWIG 4.4.0 or emit
  ``builtin type swigvarlink has no __module__ attribute`` warnings, upgrade to SWIG 4.4.1 or a newer
  supported 4.x release.
- BSK-1387: :ref:`linkBudget` could compute excessive pointing loss for non-identity antenna attitudes
  because the inertial line-of-sight vector was transformed using the antenna-to-inertial DCM. This is
  fixed in the current version.
- BSK-1416 : :ref:`jointThrAllocation` was using a static value for the vector between the hub center of
  mass (CoM) and system CoM. This caused the resulting thruster mapping to be incorrect for a given
  set of joint angles. This is fixed in the current version.
- BSK-1395: :ref:`scenarioAttitudeConstrainedManeuver` and :ref:`scenarioAttitudeConstraintViolation`
  pre-converted the reaction wheel ``maxSpeed`` to rad/s before passing it to ``rwFactory.create()``,
  which expects ``Omega_max`` in RPM and applies the RPM-to-rad/s conversion internally. The resulting
  double conversion set the wheel saturation speed about an order of magnitude too low, and since the
  derived wheel inertia is ``maxMomentum / Omega_max`` it came out about an order of magnitude too high.
  This is fixed in the current version.
- BSK-968: :ref:`spacecraft` reported ``NaN`` for ``nonConservativeAccelpntB_B`` on the first integration
  step because the accumulated velocity change was divided by a zero time step. The value is now set to
  zero when the time step is zero, matching the existing ``omegaDot_BN_B`` guard. This is fixed in the
  current version.
- BSK-1107: A standalone message created inside a Python setup or helper function (for example a
  reference or vehicle-configuration message) can be garbage collected when that function returns,
  before the simulation runs. Consumers then read zeros or unwritten data. Retain such messages on the
  simulation object or another persistent scope so they survive for the life of the simulation. This
  affected :ref:`scenarioRoboticArm` and :ref:`scenarioFlexiblePanel` and is fixed in the current version.
- BSK-1410: user-facing utilities such as :ref:`simIncludeGravBody` and :ref:`vizSupport` imported the
  test-support module :ref:`unitTestSupport`, which imports the test-only ``pytest`` package at module
  load time. Scripts run against the Basilisk wheels (which do not ship ``pytest``) therefore failed to
  import these utilities. The few general-purpose helpers those utilities needed have been moved into a
  new :ref:`simHelpers` module, so user-facing code no longer depends on ``unitTestSupport`` (or ``pytest``)
  at all. This is fixed in the current version.
- BSK-548: :ref:`simHelpers` ``timeStringToGregorianUTCMsg`` leaked the SWIG-allocated ``doubleArray``
  scratch buffer used to receive the ``str2et_c`` result (about 32 bytes per call). The buffer is now
  freed with ``delete_doubleArray``. This is fixed in the current version.
- BSK-1353: :ref:`sunlineSEKF` and :ref:`okeefeEKF` computed the post-fit residual ``measMat * x`` using the
  full ``SKF_N_STATES`` width rather than each filter's reduced state width, striding the measurement matrix
  across the wrong row width and over-reading the state-error vector. This corrupted the reported post-fit
  residuals during the convergence transient (the state and covariance estimates were unaffected). This is
  fixed in the current version.
- BSK-643: several simulation modules own dynamically-allocated output messages that are freed in their
  destructors but inherited the compiler-generated copy operations, so copying such a module shallow-copied
  the raw message pointers and both instances would free the same messages (double free). ``SysModel``-derived
  modules are now non-copyable by default, while message recorders keep explicit copy-construction support.
  This is fixed in the current version.


Version 2.10.0 (April 2, 2026)
------------------------------
- When building from source on Python 3.13 using SWIG 4.4.0, a build failure may occur
  if ``pyLimitedAPI`` is set to an ABI lower than Python 3.13 (e.g., ``0x03080000``).
  SWIG 4.4.0 introduces a new C-API codepath for Python 3.13 that expects newer
  definition macros which are not present when targeting older ``abi3`` compatibility. As such, when building
  Basilisk with Python 3.13 or above, we automatically default to using the newer cp313 ABI.
- The :ref:`simpleAntenna` module raises a warning for each of the following messages not being connected: ``sunInMsg``, ``planetInMsg``, ``sunEclipseInMsg``, even for ground based antennas.
  These messages are not relevant for ground based antennas and the warning should therefore not be risen for an antenna on ground.
- If you install Basilisk via ``pip``, note that the ``basilisk/supportData`` folder does not
  exist in your folder.  Simulation script importing directly using a local file path to ``supportData``
  will not work.  Rather, ``pooch`` is being used to manage data which puts the data files into a
  cache folder on your system.  See :ref:`bskPrinciples-6a` on how to get a file data path and import it.
- The method ``loadGravFromFile()`` was hard-coded to load a 2nd order gravity field.  This is fixed in current
  version.
- In :ref:`reactionWheelStateEffector`, mixed balanced and jitter wheel-model configurations could log ``theta`` to the wrong wheel output index. This is fixed in current version.
- In :ref:`thrJointCompensation`, the sign convention for the computed compensation torque was incorrect. This is fixed in current version.

Version 2.9.0
-------------
- The denton flux model API has changed. The module now uses the new data fetching
  API and thus relies on users passing in the correct support data location via
  ``configureDentonFiles``. Previously, the module automatically searched for the
  data files in the ``supportData`` folder.
- When building from source on Python 3.13 using SWIG 4.4.0, a build failure may occur
  if ``pyLimitedAPI`` is set to an ABI lower than Python 3.13 (e.g., ``0x03080000``).
  SWIG 4.4.0 introduces a new C-API codepath for Python 3.13 that expects newer
  definition macros which are not present when targeting older ``abi3`` compatibility. As such, when building
  Basilisk with Python 3.13 or above, we automatically default to using the newer cp313 ABI.
- :ref:`gravityEffector` had a typo where the total gravity potential contribution of the celestial bodies
  was not being computed properly. Fixed now.
- :ref:`MJSite` has an issue where the angular velocity that was being written into the stateOutMsg
  was in the inertial frame components not the body frame components.  This is now fixed.
- :ref:`locationPointing` was calculating the value of ``omega_RN_B`` but not saving it to the
  ``attGuidOutMsg``. This value is now being saved properly.
- The way body-fixed locations are added to Vizard data is changed.  Now Vizard retains a copy of the
  list of locations and only incremental changes have to be sent using the ``vizSupport.changeLocation()``
  method.  If the script was directly manipulating the :ref:`vizSupport` list that functionality no longer works.
  If only ``addLocation()`` was being used and the location information was not live updated, no changes in the
  simulation script are required.  To do live updates to location it is recommended to use
  ``vizSupport.changeLocation()``.
- The support file :ref:`vizSupport` is updated to only have ``enableUnityVisualization()`` check
  if :ref:`vizInterface` has been built or not.  It is recommended that user scripts safe-guard
  their simulation usage of :ref:`vizSupport` function by checking that ``vizSupport.vizFound`` is true.
- In :ref:`vizSupport`, in the method ``setInstrumentGuiSetting()``, corrected the spelling of
  ``showTransceiverFrustrum`` to be ``showTransceiverFrustum``.  The prior argument name has
  been deprecated and will be removed after October 11, 2026.
- For :ref:`vizInterface`.settings, changed the name of ``viewCameraConeHUD`` to ``viewCameraFrustrumHUD``.
  Using the old settings name results in the setting not being applied.
- When building the Basilisk documentation with doxygen 1.15 or newer, warnings associated with
  unclosed quotes are treated as errors and cause the build to fail. Quotes are fixed now.
- :ref:`spinningBodyOneDOFStateEffector` and :ref:`spinningBodyNDOFStateEffector` both registered
  their state variables under the same names, resulting in one overwriting the other when both are
  added to the same simulation and producing a ``BSK_ERROR``. State names are now made unique.


Version 2.8.0
-------------
- pip-based installation in editable mode using ``pip install -e .`` is not currently supported.
  Developers and users alike should continue to use ``python conanfile.py`` installation.
- The ``Reset()`` function in :ref:`forceTorqueThrForceMapping` was not working properly. This has been addressed in the current release.
- The reaction wheel configuration message was moved from C++ messages to the dynamics folder and renamed to :ref:`RWConfigPayload`.
  The reaction wheel factory was changed accordingly. Users that created the message on their own should now call ``reactionWheelStateEffector.RWConfigPayload`` instead of ``messaging``.
- Simulations that previously logged to ``BSK_ERROR`` and expected to keep running successfully
  will now raise a ``BasiliskError`` exception and stop the simulation immediately.
- The output format when querying non-numeric types in message recorders has changed. The output used to
  be a numpy array of dictionaries with flattened data. Now, it's a numpy array of the objects as if they
  were queried from the payload directly. Users relying on the legacy output format might experience
  code failures at runtime. The recommended approach is to adapt to the new format. If this is impossible
  at the time, Basilisk can be compiled with ``python conanfile.py --recorderPropertyRollback True`` to
  recover the legacy format (although a deprecation message will be raised). This build flag and the legacy
  output format are slated for complete removal in 2026/07.

Version 2.7.0
-------------
- pip-based installation in editable mode using ``pip install -e .`` is not currently supported.
  Developers and users alike should continue to use ``python conanfile.py`` installation.
- When using C++ wrapped sensor objects (CSS, thrusters, reaction wheels), Python references
  must be explicitly retained to prevent premature garbage collection. The recommended approach
  is to store these objects directly on your simulation object. For example:

  .. code-block:: python
     :linenos:

     # Create and configure a CSS device
     cssDevice = coarseSunSensor.CoarseSunSensor()
     cssDevice.ModelTag = "css1"

     # Store it on the simulation object to keep it alive
     scSim.cssDevice = cssDevice  # Prevents garbage collection

  Alternatively, for multiple devices, you can use a list or registry:

  .. code-block:: python
     :linenos:

     # Store multiple devices
     scSim.css_devices = []
     for i in range(4):
         cssDevice = coarseSunSensor.CoarseSunSensor()
         cssDevice.ModelTag = f"css{i}"
         scSim.css_devices.append(cssDevice)

  See specific documentation for:

  - :ref:`coarsesunsensor`
  - :ref:`reactionWheelStateEffector`
  - :ref:`thrusterDynamicEffector`

  For working examples, refer to these scenarios:

  - :ref:`scenarioAttitudeFeedback`
  - :ref:`scenarioCSS`

  Failure to retain references will cause segmentation faults when accessing collected objects.


Version 2.6.0
-------------
- pip-based installation in editable mode using ``pip install -e .`` is not currently supported.
  Developers and users alike should continue to use ``python conanfile.py`` installation.
- When using `senNoiseStd()` to set the sensor noise standard deviations
  in :ref:`magnetometer` and :ref:`coarsesunsensor`
  the value was being multiplied by 1.5 when creating the diagonal noise matrix.
  This 1.5x multiplier has now been removed. This is corrected in current release.
- SWIG wrapper does not fully support all array types in message payloads. This affects custom message
  payloads that use these types for array members. Workaround is to add them to ``swig_conly_data.i``.
- A bug was fixed in the :ref:`facetSRPDynamicEffector` module. A transpose was required to be added to a dcm
  in order to correctly express rotated facet normals in the spacecraft body frame.
- The ``MtbEffector.py`` module was not being imported correctly in Python due to lack of ``swig_eigen.i``
  include file in ``MtbEffector.i``. This is fixed in the current release, however it remains unknown why
  the dynamics engine is re-swigged for every individual effector/dynamics related class.
- This release uses ``conan`` version 2.x which creates a new folder ``.conan2`` in
  the home folder.  Thus, the first time Basilisk is build the project dependencies will
  be downloaded again into ``.conan2``
- If configuring and building Basilisk directly with ``conan install`` and ``build`` commands,
  the ``-if dist3/conan`` argument is no longer needed.  The Basilisk install location is
  setup with ``conan 2`` arguments inside ``conanfile.py``.
- :ref:`simIncludeGravBody` set the moon radius in km, not meters, and was thus 1000x too small when visualized.
- In the python library :ref:`RigidBodyKinematics` the ``subMRP()`` routine didn't compute the expected
  result if the denominator was small.  This is now corrected.
- :ref:`groundLocation` was not respecting the case where ``maximumRange == -1.0`` in the method ``checkInstrumentFOV``.
- Sensor noise models were not being initialized correctly in sensor models such as
  :ref:`magnetometer` and :ref:`simpleVoltEstimator` modules. This is now fixed in the current release.
- Propagation matrices were private in the :ref:`simpleVoltEstimator` and :ref:`starTracker` modules.
  This is now fixed in the current release by the addition of public methods to set and get the propagation matrices.


Version 2.5.0
-------------
- pip-based installation in editable mode using ``pip install -e .`` is not currently supported.
  Developers and users alike should continue to use ``python conanfile.py`` installation.
- If the :ref:`simIncludeRW` python tool was provided a specific ``Js`` value, it was being falsely converted
  before being assigned.  This is now corrected.
- If ``supportData/EphemerisData/de430.bsp`` is not present the current build system will download the file
  from JPL server.  However, if the download is interrupted, then the next build will find the file and
  not attempt to re-download it.  This is now fixed in the current version where the file is only
  stored in the ``supportData`` folder if the download was complete.
- :ref:`vizInterface` was not able to save to a binary data file.
  This is now fixed in the current release.
- :ref:`vizInterface` was not saving the Vizard settings to the binary file.  Fixed now.
- Installing Basilisk 2.4.0 while ``packaging<22`` is installed can lead to an incompatibility and raise a
  "TypeError: ``canonicalize_version()`` got an unexpected keyword argument ``strip_trailing_zero``" error.
  Newer versions of Basilisk now upgrade ``packaging>=22`` to solve this issue.
- :ref:`simIncludeRW` didn't allow ``fCoulomb``, ``fStatic`` and ``cViscous`` to be
  specified even if a prebuilt RW data set is used. This is corrected in the current release.

Version 2.4.0
-------------
- The fuel tank models have become classes and python simulation code using tank modules need to be
  updated.  See :ref:`fueltank` or :ref:`scenarioFuelSlosh` for further documentation.
- The CI test builds starting failing running the `gtest` unit test suite with the error
  ``CMake Error: Unknown argument: --gtest_output``.  The current release fixes this issue.
- There was an issue with the :ref:`thrusterStateEffector` such that its mass depletion rate was
  hard-coded to 100% for all firings. This is corrected in the current release.
- pip-based installation in editable mode using `pip install -e .` is not currently supported.
  Developers and users alike should continue to use `python conanfile.py` installation.
- The CI test builds starting failing running the `gtest` unit test suite with the error
  ``CMake Error: Unknown argument: --gtest_output``.  The current release fixes this issue.
- If the :ref:`simIncludeRW` python tool was provided a specific ``Js`` value, it was being falsely converted
  before being assigned.


Version 2.3.0
-------------
- A bug was introduced at 2.2.1 (2dc0a35) to the :ref:`SimulationBaseClass` `AddModelToTask` function when it was
  refactored to use the updated module variable logging. The bug manifests as no data being logged for a variable when
  there are more than one task, a module in each task, and the variable being logged is from a module assigned to a
  task added to a process after the first task has been added to a process.
- Doing a clean build on Windows appeared to complete, but when running python simulation scripts,
  errors came up about not finding Basilisk packages.  The python version number checking on Windows
  had an issue that is now corrected in the current build.
- ``swig`` 4.2 was causing run-time errors with Basilisk.  The latest version of Basilisk now added
  support for this version of swig.
- Basilisk no longer builds on Windows with the ``opNav`` flag turned on.  The ``opencv`` related
  ``conan`` settings are updated in the current release to address this.


Version 2.2.1
-------------
- There was an issue with :ref:`thrusterStateEffector` where if there are multiple instances of the
  thruster state effector then the last effector will over-write all the state of the earlier thrusters.
  This is corrected in the current release.
- Doing a clean Basilisk build with `opNav` flag on fails building openCV.  The conan
  install script is updated this is corrected in the current release.
- We found a slow memory leak if messages with arrays or vectors were accessed from python.  The ``swig``
  issue has now been fixed in the current release.
- The :ref:`facetSRPDynamicEffector` module was double counting a cosine term in the SRP force calculation. This is
  corrected in the current release.
- The :ref:`facetDragDynamicEffector` module was missing a negative sign in the drag torque calculation. This is
  corrected in the current release.

Version 2.2.0
-------------
- VizInterface has been updated to use 4.5.0 version of ZMQ library.  Be sure to use Vizard 2.1.5 or newer.

Version 2.1.7
-------------
- The way to include effector bodies in Vizard has changed in this release.
- The streaming between ``vizInterface`` and Vizard was not reliable because of a ZMQ compatibility
  issue between the two code bases.  BSK is reverting to using ZMQ 4.3.0 for now to avoid this issue.
- Building Basilisk with ``opNav`` mode was no longer working as a conan package dependency issue came up.
  This has been corrected in the new version by specifying ``xz_utils/5.4.0`` in ``conanfile.py``.  Note
  that building with ``opNav`` now also appears to require ``conan>=1.59.0``, but less than 2.0.0.

Version 2.1.6
-------------
- in :ref:`boreAngCalc`, the variable ``boreVecPoint`` is now called ``boreVec_Po``.

Version 2.1.5
-------------
- In Xcode, when editing ``vizInterface.c/h`` files, the protobuffer library is not properly found
  when opNav is included.
  The code compiles, but auto-completion etc. doesn't work in that module.
- :ref:`hingedRigidBodyStateEffector` and :ref:`dualHingedRigidBodyStateEffector` module inertial state outputs are relative to the central gravity body,
  not the inertial frame.  This is now corrected.
- Adding an instrument camera to :ref:`vizInterface` has changed.  See :ref:`vizardSettings` on how
  to use the new method ``addCamMsgToModule()``.

Version 2.1.4
-------------
- In Xcode, when editing ``vizInterface.c/h`` files, the protobuffer library is not properly found
  when opNav is included.
  The code compiles, but auto-completion etc. doesn't work in that module.
- prior version had a bug in computer the latitude in ``PCPF2LLA()`` inside :ref:`geodeticConversion`.  This is used
  in the ``specifyLocationPCPF()`` method inside :ref:`groundLocation`, and in
  :ref:`msisAtmosphere` and :ref:`albedo`.
- :ref:`coarsesunsensor` now receives in ``sensorList`` a list of CSS configuration state pointers, not
  a copy to the configuration structures.  This allows these values to be changed on the fly from
  within python.  However, the simulation code must ensure that the CSS configuration structures
  are retained in memory or a segmentation fault will ensue.
- How stand alone C-wrapped message objects are created in python has moved from ``cMsgCInterfacePy``
  to ``messaging``.  Old scripts still using ``cMsgCInterfacePy`` still work as a link has been
  created to ``messaging``.  But, the use of ``cMsgCInterfacePy`` is no depreciated and code should
  be updated to using ``messaging`` instead.
- The use of custom message in the external modules folders is broke with the new build
  modification in this release.  This is fixed in the latest release.

Version 2.1.3
-------------
- In Xcode, when editing ``vizInterface.c/h`` files, the protobuffer library is not properly found
  when opNav is included.
  The code compiles, but auto-completion etc. doesn't work in that module.
- :ref:`planetEphemeris` was not computing the correct orientation, this is fixed in current release
- updated ``examples/OpNavScenarios/BSK_OpNav.py`` path to the Vizard binary. If you are calling
  Vizard 2.1+ from the python script, the Vizard binary name changed from ``Basilisk Vizard` to
  simply ``Vizard``.
- using newer versions of Python, such as 3.9.13 or 3.10.5, running ``conanfile.py`` runs into errors
  and fails.  This is now addressed in the next release.  Upgrading ``pip`` to the newest version
  manually can resolve this issue as well.

Version 2.1.2
-------------
- In Xcode, when editing ``vizInterface.c/h`` files, the protobuffer library is not properly found
  when opNav is included.
  The code compiles, but auto-completion etc. doesn't work in that module.
- ``conan`` version 1.44 is causing build issues because the way the source and build folders are set has
  changed.  This is fixed in current version of Basilisk.
- building Basilisk from scratch with ``opNav`` turned on was failing because ``conan`` was not able to
  install ``opencv/4.1.1``.  In particular, the package ``jasper`` was now failing as a required argument
  is no longer supported.  This is fixed in the current release by turning off this ``jasper`` requirement.

Version 2.1.1
-------------
- In Xcode, when editing ``vizInterface.c/h`` files, the protobuffer library is not properly found when opNav is included.
  The code compiles, but auto-completion etc. doesn't work in that module.
- On windows when building with ``opNav`` mode on,
  the ``gettext`` library was not included with the ``conan`` software unless the version is 0.21,
  not 0.20.1.  This dependency has been upgraded.  This might require deleting the ``.conan`` folder in your
  home directory and re-installing all dependencies.


Version 2.1.0
-------------
- In Xcode, when editing ``vizInterface.c/h`` files, the protobuffer library is not properly found when opNav is included.
  The code compiles, but auto-completion etc. doesn't work in that module.
- ``conan`` changed the default repos in version 1.40 onwards which broke the Basilisk installation.
  This is now corrected in the latest release.  After pulling the latest code release, you'll need to
  delete the ``.conan`` folder in your home directory to create a fresh copy.  This only has to be done once.
- :ref:`spacecraftReconfig` has been changed to read in the spacecraft mass from an input message, not be set as
  a module variable


Version 2.0.7
-------------
- In Xcode, when editing ``vizInterface.c/h`` files, the protobuffer library is not properly found when opNav is included.
  The code compiles, but auto-completion etc. doesn't work in that module.
- Over the summer of 2020 the ``conan`` repo wasn't able to install ``opencv`` anymore.  The current build
  fixes this issue and lets Basilisk be compiled with the ``opNav`` flag again.

Version 2.0.6
-------------
- In Xcode, when editing ``vizInterface.c/h`` files, the protobuffer library is not properly found when opNav is included.
  The code compiles, but auto-completion etc. doesn't work in that module.

Version 2.0.5
-------------
- In Xcode, when editing ``vizInterface.c/h`` files, the protobuffer library is not properly found when opNav is included.
  The code compiles, but auto-completion etc. doesn't work in that module.

Version 2.0.4
-------------
- In Xcode, when editing ``vizInterface.c/h`` files, the protobuffer library is not properly found when opNav is included.
  The code compiles, but auto-completion etc. doesn't work in that module.

Version 2.0.3
-------------
- In Xcode, when editing ``vizInterface.c/h`` files, the protobuffer library is not properly found when opNav is included.
  The code compiles, but auto-completion etc. doesn't work in that module.

Version 2.0.2
-------------
- In Xcode, when editing ``vizInterface.c/h`` files, the protobuffer library is not properly found when opNav is included.
  The code compiles, but auto-completion etc. doesn't work in that module.
- On Windows, using Python 3.8 or higher resulted in Basilisk paths not being recognized due to a chance with how
  Python resolves paths on Windows.  This is fixed in the next release.

Version 2.0.1
-------------
- In Xcode, when editing ``vizInterface.c/h`` files, the protobuffer library is not properly found when opNav is included.
  The code compiles, but auto-completion etc. doesn't work in that module.

Version 2.0.0
-------------
- In Xcode, when editing ``vizInterface.c/h`` files, the protobuffer library is not properly found when opNav is included.
  The code compiles, but auto-completion etc. doesn't work in that module.

Version 1.8.10
--------------
- In Xcode, when editing ``vizInterface.c/h`` files, the protobuffer library is not properly found when opNav is included.
  The code compiles, but auto-completion etc. doesn't work in that module.

Version 1.8.9

- In Xcode, when editing ``vizInterface.c/h`` files, the protobuffer library is not properly found when opNav is included.
  The code compiles, but auto-completion etc. doesn't work in that module.

Version 1.8.8

- In Xcode, when editing ``vizInterface.c/h`` files, the protobuffer library is not properly found when opNav is included.
  The code compiles, but auto-completion etc. doesn't work in that module.

Version 1.8.7

- In Xcode, when editing ``vizInterface.c/h`` files, the protobuffer library is not properly found when opNav is included.
  The code compiles, but auto-completion etc. doesn't work in that module.

Version 1.8.6

- In Xcode, when editing ``vizInterface.c/h`` files, the protobuffer library is not properly found when opNav is included.
  The code compiles, but auto-completion etc. doesn't work in that module.

Version 1.8.5

- In Xcode, when editing ``vizInterface.c/h`` files, the protobuffer library is not properly found when opNav is included.
  The code compiles, but auto-completion etc. doesn't work in that module.

Version 1.8.4

- In Xcode, when editing ``vizInterface.c/h`` files, the protobuffer library is not properly found.
  The code compiles, but auto-completion etc. doesn't work in that module.

**Version 1.8.3**

- On Windows the ``vizInterface`` and all ``opNav`` related modules is not properly linking.
  Thus, all associated modules, including saving to Vizard binaries, is not working in this version.
- In Xcode, when editing ``vizInterface.c/h`` files, the protobuffer library is not properly found.
  The code compiles, but auto-completion etc. doesn't work in that module.

**Version 1.8.2**

- On Linux and Windows the ``vizInterface`` and all ``opNav`` related modules is not properly linking.
  Thus, all associated modules, including saving to Vizard binaries, is not working in this version.
- In Xcode, when editing ``vizInterface.c/h`` files, the protobuffer library is not properly found.
  The code compiles, but auto-completion etc. doesn't work in that module.

**Version 1.8.1**

- When deleting ``.conan`` and doing a build with ``opNav`` set to true, the required dependencies can't be found
  on the repo on the first install run.  Running it again makes it work.  This is fixed in the next release to run
  properly on the first try.
- If ``openCV`` is conan installed for Release only the Xcode would give false error messages that it can't
  find the library.  This is now fixed in the current release.
- In Xcode, when editing ``vizInterface.c/h`` files, the protobuffer library is not properly found.
  The code compiles, but auto-completion etc. doesn't work in that module.
- On Linux and Windows the ``vizInterface`` and all ``opNav`` related modules is not properly linking.
  Thus, all associated modules, including saving to Vizard binaries, is not working in this version.

**Version 1.8.0**

- The new conan based built system might need the conan cache folder ``.conan`` to be deleted and reset.  This is
  typically in the user's home folder.  After this you need to re-run the conan setup commands::

    $ conan remote add conan-community https://api.bintray.com/conan/conan-community/conan
    $ conan remote add bincrafters https://api.bintray.com/conan/bincrafters/public-conan

- If running Windows the path to the Basilisk library destination folder must be set, see :ref:`installWindows`.
- On Linux and Windows the ``vizInterface`` and all ``opNav`` related modules is not properly linking.
  Thus, all associated modules, including saving to Vizard binaries, is not working in this version.
- If running Windows, be sure to use ``pip install conan`` to get conan, and don't download the binary installer,
  see :ref:`installWindows`.   The binary installer causes several issues with this new build system in that
  it contains its own copy of Python, and thus checking for required python packages does work.
- The new build system provides many speed improvements in doing a clean or partial build, but some small changes are
  required to update BSK python simulation scripts to be compatible with the new build system.
  These changes include:

  - In BSK python simulation scripts, BSK modules should be included using the indirect method.  Thus::

        from Basilisk.fswAlgorithms.cModuleTemplate import cModuleTemplate

    becomes::

        from Basilisk.fswAlgorithms import cModuleTemplate

  - The ``pyswice`` package is now imported from ``topLevelModule``.  Thus::

        from Basilisk import pyswice

    becomes::

        from Basilisk.topLevelModules import pyswice

  - The support file ``pyswice_ck_utilities.py`` has become a regular suppoort file in ``src/utiliites``.  Thus,
    it is imported using::

        import Basilisk.pyswice.pyswice_ck_utilities

  - Similarly, ``pyswice_spk_utilities.py`` has moved to the utilities folder. To include ``spkRead`` function replace::

        from Basilisk.pyswice.pyswice_spk_utilities import spkRead

    with::

        from Basilisk.utilities.pyswice_spk_utilities import spkRead

  - To include ``loadGravFromFileToList`` function replace::

        from Basilisk.simulation.gravityEffector.gravCoeffOps import loadGravFromFileToList

    with::

        from Basilisk.simulation.gravityEffector import loadGravFromFileToList

- If you have written custom BSK modules outside of the BSK distribution, the swig ``*.i`` files and some code files
  will need to be adjusted as follows:

  - To include the ``swig_common_model.i`` file, replace::

        %include "swig_common_model.i"

    with::

        %pythoncode %{
        from Basilisk.simulation.swig_common_model import *
        %}

  - If Eigen variables are being swig'd, then import::

        %include "swig_eigen.i"

  - To swig C arrays of variables, then import::

        %include "swig_conly_data.i"

  - To provide support of C++ ``std::string`` `types <http://www.swig.org/Doc1.3/Library.html#Library_nn14>`__, then import::

        %include "std_string.i"

  - To provide support of C++ ``std::vector`` `class <http://www.swig.org/Doc1.3/Library.html#Library_nn15>`__, then import::

        %include "std_vector.i"

- The files in ``_GeneralModuleFiles`` folders are now built into a separate library with the parent folders name
  plus ``Lib``.
  This means in the IDE like Xcode and Visual Studio the code in ``_GeneralModuleFiles`` is shown in a folder with
  this library name.  Thus, for example, code in ``src/simulation/environment/_GeneralModuleFiles``
  are shown in the IDE folder ``environmentLib`` within the ``environment`` parent folder.  This keeps the BSK
  folders cleaner and with less duplicated code being displayed.
- A new python package dependency is ``Pillow``.  This is needed for the test scripts for :ref:`camera` to run.
- In Xcode the build will complain that it can't find the ``<Eigen/Dense>`` library.  The code compiles ok.  The work
  around this conan issue is to run the build twice, once for Debug and once for Release.  At that point it can
  be run just once.

**Version 1.7.5**

- :ref:`groundLocation` was not converting between the planet and inertial frame correctly.  This is now fixed in
  the later releases.

**Version 1.7.4**

- None

Version 1.7.3

- On Windows Basilisk didn't compile due to missing math ``#define`` delaration in ``geodeticConversion.cpp/h``.
  This is fixed in the latest release.

**Version 1.7.2**

- None

**Version 1.7.1**

- None

**Version 1.7.0**

- None

**Version 1.6.0**

- None

**Version 1.5.1**

- WINDOWS ONLY: Windows users cannot currently run pytest directly on Basilisk ``src/`` directory (there will be non-resolved python path issues that will result in erroneous ImportErrors). Instead, to verify proper installation of Basilisk, windows users must enter the specific subdirectory they are attempting to test, only then to run pytest. This should result in appropriate behavior.  Right now there is no known solution to this issue.

**Version 1.5.0**

- WINDOWS ONLY: Windows users cannot currently run pytest directly on Basilisk ``src/`` directory (there will be non-resolved python path issues that will result in erroneous ImportErrors). Instead, to verify proper installation of Basilisk, windows users must enter the specific subdirectory they are attempting to test, only then to run pytest. This should result in appropriate behavior.  Right now there is no known solution to this issue.
- Here the reaction wheel dynamics have been modified such that the RW state output message is no longer hard-coded to ``rw_config_0_data``, etc.  Rather, now the ``ModelTag`` string is pre-pended to make this output msg name unique with.  Any scripts that is logging this RW state message will have to be updated.  The reason for this change is to allow multiple spacecraft to have RW devices and unique RW state messages.
- There was an issue doing a clean compile using Python 2 which is addressed in the next version
- :ref:`test_reactionWheelStateEffector_integrated` didn't run on Python 2, this is fixed in the next version.

**Version 1.4.2**

- WINDOWS ONLY: Windows users cannot currently run pytest directly on Basilisk ``src/`` directory (there will be non-resolved python path issues that will result in erroneous ImportErrors). Instead, to verify proper installation of Basilisk, windows users must enter the specific subdirectory they are attempting to test, only then to run pytest. This should result in appropriate behavior.  Right now there is no known solution to this issue.

**Version 1.4.1**

- WINDOWS ONLY: Windows users cannot currently run pytest directly on Basilisk ``src/`` directory (there will be non-resolved python path issues that will result in erroneous ImportErrors). Instead, to verify proper installation of Basilisk, windows users must enter the specific subdirectory they are attempting to test, only then to run pytest. This should result in appropriate behavior.  Right now there is no known solution to this issue.
- We ran into issues compiling on Linux where ``libsodium`` and ``conan`` were not compiling properly  This is fixed in the next point release.

**Version 1.4.0**

- WINDOWS ONLY: Windows users cannot currently run pytest directly on Basilisk ``src/`` directory (there will be non-resolved python path issues that will result in erroneous ImportErrors). Instead, to verify proper installation of Basilisk, windows users must enter the specific subdirectory they are attempting to test, only then to run pytest. This should result in appropriate behavior.  Right now there is no known solution to this issue.
- ``BSK_PRINT`` has been replaced within Basilisk modules using :ref:`bskLogging` (for C++) and ``_bskLog`` (for C).
- WINDOWS ONLY: there appears to be an issue compiling ``vizInterface`` with the new bskLog method on Windows.  We are working a point release that will fix this.

**Version 1.3.2**

- WINDOWS ONLY: Windows users cannot currently run pytest directly on Basilisk ``src/`` directory (there will be non-resolved python path issues that will result in erroneous ImportErrors). Instead, to verify proper installation of Basilisk, windows users must enter the specific subdirectory they are attempting to test, only then to run pytest. This should result in appropriate behavior.  Right now there is no known solution to this issue.

**Version 1.3.1**

- WINDOWS ONLY: Windows users cannot currently run pytest directly on Basilisk ``src/`` directory (there will be non-resolved python path issues that will result in erroneous ImportErrors). Instead, to verify proper installation of Basilisk, windows users must enter the specific subdirectory they are attempting to test, only then to run pytest. This should result in appropriate behavior.  Right now there is no known solution to this issue.

**Version 1.3.0**

- WINDOWS ONLY: Windows users cannot currently run pytest directly on Basilisk ``src/`` directory (there will be non-resolved python path issues that will result in erroneous ImportErrors). Instead, to verify proper installation of Basilisk, windows users must enter the specific subdirectory they are attempting to test, only then to run pytest. This should result in appropriate behavior.  Right now there is no known solution to this issue.

**Version 1.2.1**

- WINDOWS ONLY: Windows users cannot currently run pytest directly on Basilisk ``src/`` directory (there will be non-resolved python path issues that will result in erroneous ImportErrors). Instead, to verify proper installation of Basilisk, windows users must enter the specific subdirectory they are attempting to test, only then to run pytest. This should result in appropriate behavior.  Right now there is no known solution to this issue.


**Version 1.2.0**

- WINDOWS ONLY: Windows users cannot currently run pytest directly on Basilisk ``src/`` directory (there will be non-resolved python path issues that will result in erroneous ImportErrors). Instead, to verify proper installation of Basilisk, windows users must enter the specific subdirectory they are attempting to test, only then to run pytest. This should result in appropriate behavior.  Right now there is no known solution to this issue.
- The magnetometer unit tests don't pass on all platforms. This is corrected in the next release.

**Version 1.1.0**

- WINDOWS ONLY: Windows users cannot currently run pytest directly on Basilisk ``src/`` directory (there will be non-resolved python path issues that will result in erroneous ImportErrors). Instead, to verify proper installation of Basilisk, windows users must enter the specific subdirectory they are attempting to test, only then to run pytest. This should result in appropriate behavior.  Right now there is no known solution to this issue.
- the unit tests of the magnetometer module don't pass on all operating systems as the test tolerances are too tight.  This is resolved in the next release.

**Version 1.0.0**

.. raw:: html

   <ul>

.. raw:: html

   <li>

WINDOWS ONLY: Windows users cannot currently run pytest directly on
Basilisk ``src/`` directory (there will be non-resolved python path
issues that will result in erroneous ImportErrors). Instead, to verify
proper installation of Basilisk, windows users must enter the specific
subdirectory they are attempting to test, only then to run pytest. This
should result in appropriate behavior. Right now there is no known
solution to this issue.

.. raw:: html

   </li>

.. raw:: html

   <li>

Swig version 4 was released over the summer. This version is not
compatible with our current Basilisk software. Be sure to install swig
version 3.0.12.

.. raw:: html

   </li>

.. raw:: html

   </ul>

**Version 0.9.0**

.. raw:: html

   <ul>

.. raw:: html

   <li>

WINDOWS ONLY: Windows users cannot currently run pytest directly on
Basilisk ``src/`` directory (there will be non-resolved python path
issues that will result in erroneous ImportErrors). Instead, to verify
proper installation of Basilisk, windows users must enter the specific
subdirectory they are attempting to test, only then to run pytest. This
should result in appropriate behavior. Right now there is no known
solution to this issue.

.. raw:: html

   </li>

.. raw:: html

   <li>

Swig version 4 was released over the summer. This version is not
compatible with our current Basilisk software. Be sure to install swig
version 3.0.12.

.. raw:: html

   </li>

.. raw:: html

   </ul>

**Version 0.8.1**

.. raw:: html

   <ul>

.. raw:: html

   <li>

WINDOWS ONLY: Windows users cannot currently run pytest directly on
Basilisk ``src/`` directory (there will be non-resolved python path
issues that will result in erroneous ImportErrors). Instead, to verify
proper installation of Basilisk, windows users must enter the specific
subdirectory they are attempting to test, only then to run pytest. This
should result in appropriate behavior. Right now there is no known
solution to this issue.

.. raw:: html

   </li>

.. raw:: html

   <li>

Swig version 4 was released over the summer. This version is not
compatible with our current Basilisk software. Be sure to install swig
version 3.0.12.

.. raw:: html

   </li>

.. raw:: html

   </ul>

**Version 0.8.0**

.. raw:: html

   <ul>

.. raw:: html

   <li>

WINDOWS ONLY: Windows users cannot currently run pytest directly on
Basilisk ``src/`` directory (there will be non-resolved python path
issues that will result in erroneous ImportErrors). Instead, to verify
proper installation of Basilisk, windows users must enter the specific
subdirectory they are attempting to test, only then to run pytest. This
should result in appropriate behavior. Right now there is no known
solution to this issue.

.. raw:: html

   </li>

.. raw:: html

   <li>

Swig version 4 was released over the summer. This version is not
compatible with our current Basilisk software. Be sure to install swig
version 3.0.12.

.. raw:: html

   </li>

.. raw:: html

   </ul>

**Version 0.7.2**

.. raw:: html

   <ul>

.. raw:: html

   <li>

WINDOWS ONLY: Windows users cannot currently run pytest directly on
Basilisk ``src/`` directory (there will be non-resolved python path
issues that will result in erroneous ImportErrors). Instead, to verify
proper installation of Basilisk, windows users must enter the specific
subdirectory they are attempting to test, only then to run pytest. This
should result in appropriate behavior. Right now there is no known
solution to this issue.

.. raw:: html

   </li>

.. raw:: html

   <li>

The python pandas package is now required to run BSK. The installation
instructions have been updated to reflect this.

.. raw:: html

   </li>

.. raw:: html

   <li>

Swig version 4 was released over the summer. This version is not
compatible with our current Basilisk software. Be sure to install swig
version 3.0.12.

.. raw:: html

   </li>

.. raw:: html

   </ul>

**Version 0.7.1**

.. raw:: html

   <ul>

.. raw:: html

   <li>

WINDOWS ONLY: Windows users cannot currently run pytest directly on
Basilisk ``src/`` directory (there will be non-resolved python path
issues that will result in erroneous ImportErrors). Instead, to verify
proper installation of Basilisk, windows users must enter the specific
subdirectory they are attempting to test, only then to run pytest. This
should result in appropriate behavior. Right now there is no known
solution to this issue.

.. raw:: html

   </li>

.. raw:: html

   <li>

The python pandas package is now required to run BSK. The installation
instructions have been updated to reflect this.

.. raw:: html

   </li>

.. raw:: html

   </ul>

**Version 0.7.0**

.. raw:: html

   <ul>

.. raw:: html

   <li>

WINDOWS ONLY: Windows users cannot currently run pytest directly on
Basilisk ``src/`` directory (there will be non-resolved python path
issues that will result in erroneous ImportErrors). Instead, to verify
proper installation of Basilisk, windows users must enter the specific
subdirectory they are attempting to test, only then to run pytest. This
should result in appropriate behavior. Right now there is no known
solution to this issue.

.. raw:: html

   </li>

.. raw:: html

   <li>

The python pandas package is now required to run BSK. The installation
instructions have been updated to reflect this.

.. raw:: html

   </li>

.. raw:: html

   </ul>

**Version 0.6.2**

.. raw:: html

   <ul>

.. raw:: html

   <li>

WINDOWS ONLY: Windows users cannot currently run pytest directly on
Basilisk ``src/`` directory (there will be non-resolved python path
issues that will result in erroneous ImportErrors). Instead, to verify
proper installation of Basilisk, windows users must enter the specific
subdirectory they are attempting to test, only then to run pytest. This
should result in appropriate behavior. Right now there is no known
solution to this issue.

.. raw:: html

   </li>

.. raw:: html

   <li>

The enableUnityViz python function how has different inputs. Earlier
python scripts must be updated. See the scenarios for examples. The
arguments are now provided as optional keywords.

.. raw:: html

   </li>

.. raw:: html

   </ul>

**Version 0.6.1**

.. raw:: html

   <ul>

.. raw:: html

   <li>

WINDOWS ONLY: Windows users cannot currently run pytest directly on
Basilisk ``src/`` directory (there will be non-resolved python path
issues that will result in erroneous ImportErrors). Instead, to verify
proper installation of Basilisk, windows users must enter the specific
subdirectory they are attempting to test, only then to run pytest. This
should result in appropriate behavior. Right now there is no known
solution to this issue.

.. raw:: html

   </li>

.. raw:: html

   <li>

This version of Basilisk no longer support the ASIO module that
communicated with the Qt-based visualization as the BOOST library has
been removed. This visualization has been replaced with the new Vizard
visualization.

.. raw:: html

   </li>

.. raw:: html

   <li>

The ``thrMomentumDumping`` now reads in a 2nd required input message to
determine if a new dumping sequence is requested.

.. raw:: html

   </li>

.. raw:: html

   </ul>

**Version 0.6.0**

.. raw:: html

   <ul>

.. raw:: html

   <li>

WINDOWS ONLY: Windows users cannot currently run pytest directly on
Basilisk ``src/`` directory (there will be non-resolved python path
issues that will result in erroneous ImportErrors). Instead, to verify
proper installation of Basilisk, windows users must enter the specific
subdirectory they are attempting to test, only then to run pytest. This
should result in appropriate behavior. Right now there is no known
solution to this issue.

.. raw:: html

   </li>

.. raw:: html

   </ul>

**Version 0.5.1**

.. raw:: html

   <ul>

.. raw:: html

   <li>

WINDOWS ONLY: Windows users cannot currently run pytest directly on
Basilisk ``src/`` directory (there will be non-resolved python path
issues that will result in erroneous ImportErrors). Instead, to verify
proper installation of Basilisk, windows users must enter the specific
subdirectory they are attempting to test, only then to run pytest. This
should result in appropriate behavior. Right now there is no known
solution to this issue.

.. raw:: html

   </li>

.. raw:: html

   </ul>

**Version 0.5.0**

.. raw:: html

   <ul>

.. raw:: html

   <li>

WINDOWS ONLY: Windows users cannot currently run pytest directly on
Basilisk ``src/`` directory (there will be non-resolved python path
issues that will result in erroneous ImportErrors). Instead, to verify
proper installation of Basilisk, windows users must enter the specific
subdirectory they are attempting to test, only then to run pytest. This
should result in appropriate behavior. Right now there is no known
solution to this issue.

.. raw:: html

   </li>

.. raw:: html

   <li>

the ``exponentialAtmosphere`` module has been replaced with a module
based on the new atmospheric density base class. BSK simulations that
used the older module must update to use the new module. The module unit
test scripts illustrate how to use this module, and the module PDF
documentation discusses this as well. The ``dragEffector`` integrated
test is also updated to make use of the new module

.. raw:: html

   </li>

.. raw:: html

   <li>

The ``MRP_Feedback()`` has the control vector ``domega0`` removed and
keeps this term now as a permanent zero vector. Any code that was
setting this needs to be updated to not set this parameter anymore.

.. raw:: html

   </li>

.. raw:: html

   </ul>

**Version 0.4.1**

.. raw:: html

   <ul>

.. raw:: html

   <li>

WINDOWS ONLY: Windows users cannot currently run pytest directly on
Basilisk ``src/`` directory (there will be non-resolved python path
issues that will result in erroneous ImportErrors). Instead, to verify
proper installation of Basilisk, windows users must enter the specific
subdirectory they are attempting to test, only then to run pytest. This
should result in appropriate behavior. Right now there is no known
solution to this issue.

.. raw:: html

   </li>

.. raw:: html

   <li>

The ``numpy`` python package can't be the current version 1.16.x as this
causes some incompatibilities and massive amounts of depreciated
warnings. These warnings are not related to BSK python code, but other
support code. Thus, for now be sure to install version 1.15.14 of
``numpy``.

.. raw:: html

   </li>

.. raw:: html

   </ul>

**Version 0.4.0**

.. raw:: html

   <ul>

.. raw:: html

   <li>

WINDOWS ONLY: Windows users cannot currently run pytest directly on
Basilisk ``src/`` directory (there will be non-resolved python path
issues that will result in erroneous ImportErrors). Instead, to verify
proper installation of Basilisk, windows users must enter the specific
subdirectory they are attempting to test, only then to run pytest. This
should result in appropriate behavior. Right now there is no known
solution to this issue.

.. raw:: html

   </li>

.. raw:: html

   <li>

Version 4.x.x and higher of pytest works again with Basilisk. You are
free to install the latest version of pytest.

.. raw:: html

   </li>

.. raw:: html

   <li>

As we are now using the conan package management system, you can't
double the the Cmake GUI application. Instead, you must either launch
the Cmake GUI application from the command line, or run CMake from the
command line directly. See the platform specific Basilisk installation
instructions.

.. raw:: html

   </li>

.. raw:: html

   <li>

The ``numpy`` python package can't be the current version 1.16.x as this
causes some incompatibilities and massive amounts of depreciated
warnings. These warnings are not related to BSK python code, but other
support code. Thus, for now be sure to install version 1.15.14 of
``numpy``.

.. raw:: html

   </li>

.. raw:: html

   </ul>

**Version 0.3.3**

.. raw:: html

   <ul>

.. raw:: html

   <li>

WINDOWS ONLY: Windows users cannot currently run pytest directly on
Basilisk ``src/`` directory (there will be non-resolved python path
issues that will result in erroneous ImportErrors). Instead, to verify
proper installation of Basilisk, windows users must enter the specific
subdirectory they are attempting to test, only then to run pytest. This
should result in appropriate behavior. Right now there is no known
solution to this issue.

.. raw:: html

   </li>

.. raw:: html

   <li>

The latest version of pytest (version 3.7.1) has a conflict with the
RadiationPressure module unit test. We are still investigating. In the
meantime, using pytest version 3.6.1 is working correctly.

.. raw:: html

   </li>

.. raw:: html

   </ul>
