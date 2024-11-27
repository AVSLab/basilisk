.. _examples:


Integrated Example Scripts
==========================

To learn how to use Basilisk it is often convenient to study sample scripts and see how they were implemented.  Below are a series of example Basilisk scripts grouped by topics and complexity.  In each script the primary simulation is executed by the ``run`` method.  The reader can learn from this implementation and the numerous associated in-line comments.  The first sections illustrate how to setup simple spacecraft simulations and then the following sections begin to add more complexity.

The example script source code can be viewed directly within the browser by clicking on the Source link next to the ``run`` method name as shown in the image below:

.. image:: /_images/static/scenarioSourceLink.png
   :width: 1316px
   :height: 106px
   :scale: 50%
   :alt: scenario python source link
   :align: center

The python code contains additional comments to explain what is being setup and tested here.  The example instructions are all stored as extensive comments within the python script.  Read the script source to study how this example functions.


Orbital Simulations
-------------------

.. toctree::
   :maxdepth: 1

   Basic Orbit Simulations <scenarioBasicOrbit>
   Delta_v Orbit Maneuvers <scenarioOrbitManeuver>
   Hyperbolic Jupiter Arrival Orbit <scenarioJupiterArrival>
   Multiple Gravitational Bodies <scenarioOrbitMultiBody>
   Lagrange Point Orbits <scenarioLagrangePointOrbit>
   Defining Motion Relative to Planet <scenarioCentralBody>
   Simulating Trajectory about Multiple Celestial Bodies <scenarioPatchedConics>
   Including Custom Gravitational Bodies <scenarioCustomGravBody>
   Near-Halo Orbit Simulation <scenarioHaloOrbit>


Attitude Simulations
--------------------

Attitude Regulation Control
^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. toctree::
   :maxdepth: 1

   Inertial Attitude Pointing <scenarioAttitudeFeedback>
   Using Separate Task Group for Control <scenarioAttitudeFeedback2T>
   Basic Attitude Pointing in Deep Space <scenarioAttitudePointing>
   Complex Attitude Pointing in Deep Space <scenarioAttitudeFeedbackNoEarth>
   Sun-Pointing Constraint Violation in Space <scenarioAttitudeConstraintViolation>
   Inertial Pointing with Spice prescribed translational motion <scenarioSpiceSpacecraft>
   Basic attitude pointing flight mode with Hohmann transfer <scenarioHohmann>
   Thrusted Hohmann Transfer Maneuver <scenarioOrbitManeuverTH>


Attitude Guidance
^^^^^^^^^^^^^^^^^

.. toctree::
   :maxdepth: 1

   Hill Frame Pointing on Elliptic Orbit <scenarioAttitudeGuidance>
   Velocity Frame Pointing on Hyperbolic Orbit <scenarioAttGuideHyperbolic>
   Pointing at Earth Location <scenarioAttLocPoint>
   Prescribing the spacecraft orientation <scenarioAttitudePrescribed>
   Layered spiral attitude guidance <scenarioInertialSpiral>
   Constrained Attitude Maneuver Guidance <scenarioAttitudeConstrainedManeuver>
   Performing Sweeping Maneuvers <scenarioSweepingSpacecraft>



Attitude Control with Actuators
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. toctree::
   :maxdepth: 1

   Pointing with Reaction Wheels <scenarioAttitudeFeedbackRW>
   Pointing with Attitude State Effector Thrusters <scenarioAttitudeFeedback2T_stateEffTH>
   Pointing with Attitude Dynamic Effector Thrusters <scenarioAttitudeFeedback2T_TH>
   Reaction Wheel Momentum Dumping using Thrusters <scenarioMomentumDumping>
   Continuous Momentum Management using Dual-Gimbaled Electric Thruster <scenarioSepMomentumManagement>


Attitude Steering Control
^^^^^^^^^^^^^^^^^^^^^^^^^

.. toctree::
   :maxdepth: 1

   MRP Steering Law <scenarioAttitudeSteering>

Orbit Control
-------------
.. toctree::
   :maxdepth: 1

   Small Body Waypoint-to-Waypoint Control <scenarioSmallBodyFeedbackControl>
   Lambert Solver Delta-V Maneuver <scenarioLambertSolver>



Planetary Environments
----------------------

Magnetic Field Models
^^^^^^^^^^^^^^^^^^^^^

.. toctree::
   :maxdepth: 1

   Centered Dipole Model <scenarioMagneticFieldCenteredDipole>
   World Magnetic Model WMM <scenarioMagneticFieldWMM>

Gravity Gradient Torque
^^^^^^^^^^^^^^^^^^^^^^^
.. toctree::
   :maxdepth: 1

   Gravity Gradient Perturbed Hill Pointing <scenarioAttitudeGG>

Atmospheric Drag
^^^^^^^^^^^^^^^^^^^^^^^
.. toctree::
   :maxdepth: 1

   Satellite Drag Deorbit about Earth <scenarioDragDeorbit>


Access to Communication Locations
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
.. toctree::
   :maxdepth: 1

   Satellite communicating to Earth ground station <scenarioGroundDownlink>
   Satellite checking communication access to another satellite <scenarioSpacecraftLocation>

Planet Albedo
^^^^^^^^^^^^^
.. toctree::
   :maxdepth: 1

   Albedo CSS simulation about Multiple Celestial Objects <scenarioAlbedo>


Spacecraft Sensors
------------------

Coarse Sun Sensors
^^^^^^^^^^^^^^^^^^

.. toctree::
   :maxdepth: 1

   Adding CSS Sensors <scenarioCSS>
   Estimating Sun Heading with CSS <scenarioCSSFilters>


Three-Axis Magnetometers
^^^^^^^^^^^^^^^^^^^^^^^^

.. toctree::
   :maxdepth: 1

   Adding a Three-Axis Magnetometer (TAM)  <scenarioTAM>
   RW Momentum Management using TAMs and MTBs <scenarioMtbMomentumManagementSimple>
   RW Momentum Management using TAMs and MTBs Using desired Nominal RW Speed <scenarioMtbMomentumManagement>
   TAM Comparison <scenarioTAMcomparison>

Spacecraft Sub-Systems
----------------------

Power Sub-System
^^^^^^^^^^^^^^^^

.. toctree::
   :maxdepth: 1

   Basic Power Usage and Tracking  <scenarioPowerDemo>
   Power Usage with RW-Based ADCS  <scenarioAttitudeFeedbackRWPower>

Data Sub-System
^^^^^^^^^^^^^^^^

.. toctree::
  :maxdepth: 1

  Basic Data Generation and Transmission  <scenarioDataDemo>
  Ground Target Imaging and Downlink <scenarioGroundLocationImaging>
  Mapping A Body <scenarioGroundMapping>

Thermal Sub-System
^^^^^^^^^^^^^^^^^^

.. toctree::
  :maxdepth: 1

  Thermal Modeling of a Sensor  <scenarioSensorThermal>
  Temperature Measurement with Random Seed Noise <scenarioTempMeasurementAttitude>

Complex Spacecraft Dynamics Simulations
---------------------------------------

.. toctree::
   :maxdepth: 1

   IMU Gyro Random Walk <scenarioGaussMarkovRandomWalk>
   Fuel Slosh <scenarioFuelSlosh>
   Flexible (Hinged) Panels <scenarioHingedRigidBody>
   Bending and Torsional Flexible Panels <scenarioFlexiblePanel>
   Sensors Attached to a Rotating Panel <scenarioRotatingPanel>
   Hinged Panel Deployment <scenarioDeployingPanel>
   MSM Simulation of Charged Spacecraft <scenarioTwoChargedSC>
   Spacecraft with 1- or 2-DOF Panel using single effector <scenarioSpinningBodiesTwoDOF>
   Prescribed Motion Rotational Solar Array Deployment <scenarioDeployingSolarArrays>
   Robotic Arm Effector with Profiler <scenarioRoboticArm>
   Two Spacecraft Connected Using Holonomic Constraints <scenarioConstrainedDynamics>
   Spacecraft with an multi-link extending component <scenarioExtendingBoom>

Mission Simulations
---------------------------------------

.. toctree::
   :maxdepth: 1

   Heliocentric Translation Using Custom Spice Files <scenarioHelioTransSpice>
   Planetary Flybys Using Custom Spice File <scenarioFlybySpice>
   Asteroid Arrival <scenarioAsteroidArrival>
   Aerocapture Scenario <scenarioAerocapture>



Spacecraft Formation Flying
---------------------------

.. toctree::
   :maxdepth: 1

   Basic Servicer/Debris Simulation <scenarioFormationBasic>
   Mean orbit element based relative motion control <scenarioFormationMeanOEFeedback>
   Impulsive feedback control of relative motion <scenarioFormationReconfig>
   Electrostatic Tractor Debris Reorbiting <scenarioDebrisReorbitET>
   Attitude-Driven differential drag control <scenarioDragRendezvous>
   Servicer approaching a debris object with 3 flight modes <scenarioRendezVous>
   Walker-Delta Satellite Constellation <scenarioSatelliteConstellation>

Small Body Navigation Simulations
---------------------------------

.. toctree::
   :maxdepth: 1

    Proximity Operations Hybrid EKF <scenarioSmallBodyNav>
    Non-Keplerian Acceleration Estimation using UKF <scenarioSmallBodyNavUKF>
    Landmarks-based Navigation <scenarioSmallBodyLandmarks>


``bskSim()`` Architecture
-------------------------

Single Satellite
^^^^^^^^^^^^^^^^

.. toctree::
   :maxdepth: 1
   :numbered:

   Basic Orbital Simulation  <BskSim/scenarios/scenario_BasicOrbit>
   Attitude Detumble Control with RW  <BskSim/scenarios/scenario_FeedbackRW>
   Hill Pointing Attitude Control  <BskSim/scenarios/scenario_AttGuidance>
   Velocity Frame Pointing Control  <BskSim/scenarios/scenario_AttGuidHyperbolic>
   MRP Steering Attitude Control  <BskSim/scenarios/scenario_AttSteering>
   Sun Pointing Mode Include Eclipse Evaluation  <BskSim/scenarios/scenario_AttEclipse>
   Alternating FSW Attitude Pointing Modes <BskSim/scenarios/scenario_AttModes>
   Reaction Wheel Fault Scenario Simulation <BskSim/scenarios/scenario_AddRWFault>
   Lambert Guidance Scenario <BskSim/scenarios/scenario_LambertGuidance>
   Attitude Control <BskSim/scenarios/scenario_AttFeedback>

Satellite Formation
^^^^^^^^^^^^^^^^^^^

.. toctree::
   :maxdepth: 1
   :numbered:

   Two-Spacecraft Formation using BskSim  <BskSim/scenarios/scenario_BasicOrbitFormation>
   Relative Pointing Control  <BskSim/scenarios/scenario_RelativePointingFormation>


Support Files
"""""""""""""

.. toctree::
   :maxdepth: 1

   Master File <BskSim/BSK_masters>
   Models Folder <BskSim/models/index>
   Plotting Functions <BskSim/plotting/index>




``MultiSatBskSim()`` Architecture
---------------------------------

Scenarios
^^^^^^^^^

.. toctree::
   :maxdepth: 1
   :numbered:

   Three-Spacecraft Formation using MultiSat architecture  <MultiSatBskSim/scenariosMultiSat/scenario_BasicOrbitMultiSat>
   Attitude Guidance Modes Scheduling  <MultiSatBskSim/scenariosMultiSat/scenario_AttGuidMultiSat>
   Formation Flying Control  <MultiSatBskSim/scenariosMultiSat/scenario_StationKeepingMultiSat>

Support Files
"""""""""""""

.. toctree::
   :maxdepth: 1

   Master file <MultiSatBskSim/BSK_MultiSatMasters>
   Models Folder <MultiSatBskSim/modelsMultiSat/index>
   Plotting Functions <MultiSatBskSim/plottingMultiSat/index>


Optical Navigation Simulations
------------------------------

Scenarios
^^^^^^^^^

.. toctree::
   :maxdepth: 1
   :numbered:

   BSK OpNav Sim Master File  <OpNavScenarios/BSK_OpNav>
   Hough Circles for Pointing and Orbit Determination  <OpNavScenarios/scenariosOpNav/scenario_OpNavAttOD>
   Limb-based method for Pointing and Orbit Determination  <OpNavScenarios/scenariosOpNav/scenario_OpNavAttODLimb>
   CNN for Pointing and Orbit Determination   <OpNavScenarios/scenariosOpNav/scenario_CNNAttOD>
   Perform fault detection with two OpNav methods  <OpNavScenarios/scenariosOpNav/scenario_faultDetOpNav>
   Orbit Determination with Hough Circles  <OpNavScenarios/scenariosOpNav/scenario_OpNavOD>
   Orbit Determination with Limb-based method   <OpNavScenarios/scenariosOpNav/scenario_OpNavODLimb>
   Pointing with Hough Circles  <OpNavScenarios/scenariosOpNav/scenario_OpNavPoint>
   Pointing with Limb-based method   <OpNavScenarios/scenariosOpNav/scenario_OpNavPointLimb>
   Filter Heading measurements  <OpNavScenarios/scenariosOpNav/scenario_OpNavHeading>

Support Files
"""""""""""""

.. toctree::
   :maxdepth: 1

   Models Folder <OpNavScenarios/modelsOpNav/index>
   Plotting Functions <OpNavScenarios/plottingOpNav/index>

CNN Image Generation
^^^^^^^^^^^^^^^^^^^^
.. toctree::
   :maxdepth: 1

   Monte-Carlo Script to Create Image Set  <OpNavScenarios/scenariosOpNav/CNN_ImageGen/OpNavMonteCarlo>
   Scenario called by MC Image Generation Script  <OpNavScenarios/scenariosOpNav/CNN_ImageGen/scenario_CNNImages>

CNN Image Generation
^^^^^^^^^^^^^^^^^^^^
.. toctree::
   :maxdepth: 1

   Monte-Carlo Simulations using a OpNav Scenario  <OpNavScenarios/scenariosOpNav/OpNavMC/MonteCarlo>
   Optical Limb Tracking Scenario  <OpNavScenarios/scenariosOpNav/OpNavMC/scenario_LimbAttOD>
   Optical Navitation Scenario  <OpNavScenarios/scenariosOpNav/OpNavMC/scenario_OpNavAttODMC>


Monte Carlo Simulations
-----------------------

.. toctree::
   :maxdepth: 1

   MC run with RW control  <scenarioMonteCarloAttRW>
   MC run using Python Spice setup  <scenarioMonteCarloSpice>
   MC bskSim example folder with bokeh visualization <MonteCarloExamples/index>
   Sensitivity Analysis of a differential drag spacecraft control <scenarioDragSensitivity>





Advanced Simulation Options
---------------------------

.. toctree::
   :maxdepth: 1

   Setting Type of Integrator <scenarioIntegrators>
   Using a Variable Time Step Integrator <scenarioVariableTimeStepIntegrators>
   Comparison of different integrators <scenarioIntegratorsComparison>
   Using a Python BSK Module Inherited from SysModel Class <scenarioAttitudePointingPy>
   Changing the bskLog Verbosity from Python <scenarioBskLog>

Multi-Threading Basilisk Simulations
------------------------------------

.. toctree::
   :maxdepth: 1

   Simulation of Multiple independent spacecraft <MultiSatBskSim/scenariosMultiSat/scenario_BasicOrbitMultiSat_MT>


Advanced Visualizations
-----------------------

Interfacing with Vizard
^^^^^^^^^^^^^^^^^^^^^^^

.. toctree::
   :maxdepth: 1

   Live Streaming to Vizard  <scenarioBasicOrbitStream>
   Pointing a Vizard Camera  <scenarioVizPoint>
   Convert Simulation Data File to Vizard File <scenarioDataToViz>
