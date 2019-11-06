.. _examples:


Example Scripts
===============

To learn how to use Basilisk it is often convenient to study sample scripts and see how they were implemented.  Below are a series of example Basilisk scripts grouped by topics and complexity.  The first sections illustrate how to setup simple spacecraft simulations and then the following sections begin to add more complexity.

The example script source code can be viewed directly within the browser by clicking on the Source link next to the python method name.  The python code contains additional comments to explain what is being setup and tested here.  The example instructions are all stored as extensive comments within the python script.  Read the script source to study how this example functions.


Orbital Simulations
-------------------
This section contains scripts that illustrate how to setup a simple spacecraft orbital simulations.

.. toctree::

   Basic Orbit Simulations <OrbitalSimulations/scenarioBasicOrbit>
   Delta_v Orbit Maneuvers <OrbitalSimulations/scenarioOrbitManeuver>
   Multiple Gravitational Bodies <OrbitalSimulations/scenarioOrbitMultiBody>
   Defining Motion Relative to Planet <OrbitalSimulations/scenarioCentralBody>
   Simulating Trajectory about Multiple Celestial Bodies <OrbitalSimulations/scenarioPatchedConics>

.. toctree::

    OrbitalSimulations/


Attitude Simulations
--------------------

Attitude Regulation Control
^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. toctree::

   Inertial Attitude Pointing <AttitudeSimulations/scenarioAttitudeFeedback>
   Using Separate TAsk Group for Control <AttitudeSimulations/scenarioAttitudeFeedback2T>
   Inertial Pointing with Python Module <AttitudeSimulations/scenarioAttitudePythonPD>
   Basic Attitude Pointing in Deep Space <AttitudeSimulations/scenarioAttitudePointing>
   Complex Attitude Pointing in Deep Space <AttitudeSimulations/scenarioAttitudeFeedbackNoEarth>


Attitude Guidance
^^^^^^^^^^^^^^^^^

.. toctree::

   Hill Frame Pointing <AttitudeSimulations/scenarioAttitudeGuidance>
   Velocity Frame Pointing <AttitudeSimulations/scenarioAttGuideHyperbolic>


Attitude Control with Actuators
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. toctree::

   Pointing with Reaction Wheels <AttitudeSimulations/scenarioAttitudeFeedbackRW>
   Pointing with Attitude Thrusters <AttitudeSimulations/scenarioAttitudeFeedback2T_TH>


Attitude Steering Control
^^^^^^^^^^^^^^^^^^^^^^^^^

.. toctree::

   MRP Steering Law <AttitudeSimulations/scenarioAttitudeSteering>

Complex Spacecraft Dynamics Simulations
---------------------------------------

.. toctree::

   Fuel Slosh <AdvDynamicsSimulations/scenarioFuelSlosh>
   Flexible (Hinged) Panels <AdvDynamicsSimulations/scenarioHingedRigidBody>


Planetary Environments
----------------------

Magnetic Field Models
^^^^^^^^^^^^^^^^^^^^^

.. toctree::

   Centered Dipole Model <Environments/scenarioMagneticFieldCenteredDipole>
   World Magnetic Model WMM <Environments/scenarioMagneticFieldWMM>



Spacecraft Sensors
------------------

Coarse Sun Sensors
^^^^^^^^^^^^^^^^^^

.. toctree::

   Adding CSS Sensors <Sensors/scenarioCSS>
   Estimating Sun Heading with CSS <Sensors/scenarioCSSFilters>


Three-Axis Magnetometers
^^^^^^^^^^^^^^^^^^^^^^^^

.. toctree::

   Adding a Three-Axis Magnetometer (TAM)  <Sensors/scenarioTAM>



Spacecraft Sub-Systems
----------------------

Power Sub-System
^^^^^^^^^^^^^^^^

.. toctree::

   Basic Power Usage and Tracking  <SubSystems/scenarioPowerDemo>


Monte Carlo Simulations
-----------------------

.. toctree::

   MC run with RW control  <MonteCarlo/scenarioMonteCarloAttRW>
   MC run using Python Spice setup  <MonteCarlo/scenarioMonteCarloSpice>


bskSim()-Based Simulation
-------------------------

.. toctree::

   Basic Orbital Simulation  <BskSim/scenarios/scenario_BasicOrbit>
   Attitude Detumble Control  <BskSim/scenarios/scenario_FeedbackRW>
   Hill Pointing Attitude Control  <BskSim/scenarios/scenario_AttGuidance>
   Velocity Frame Pointing Control  <BskSim/scenarios/scenario_AttGuidHyperbolic>
   MRP Steering Attitude Control  <BskSim/scenarios/scenario_AttSteering>
   Sun Pointing Mode Include Eclipse Evaluation  <BskSim/scenarios/scenario_AttitudeEclipse>



Spacecraft Formation Flying
---------------------------

Formation Flying Dynamics
^^^^^^^^^^^^^^^^^^^^^^^^^

.. toctree::

   Two-Spacecraft Formation  <BskSim/scenarios/scenario_BasicOrbitFormation>


Formation Flying Control
^^^^^^^^^^^^^^^^^^^^^^^^

.. toctree::

   Relative Pointing Control  <BskSim/scenarios/scenario_RelativePointingFormation>



Advanced Simulation Options
---------------------------

.. toctree::

   Setting the Integrator  <AdvancedOptions/scenarioIntegrators>



Advanced Visualizations
-----------------------

Live Plotting
^^^^^^^^^^^^^

.. toctree::

   Regular Basilisk simulation using Live Plotting  <AdvVisualization/scenarioBasicOrbitLivePlot>
   bskSim Basilisk simulation using Live Plotting <BskSim/scenarios/scenario_BasicOrbit_LivePlot>


Interfacing with Vizard
^^^^^^^^^^^^^^^^^^^^^^^

.. toctree::

   Live Streaming to Vizard  <AdvVisualization/scenarioBasicOrbitStream>
   Pointing a Vizard Camera  <AdvVisualization/scenarioVizPoint>




