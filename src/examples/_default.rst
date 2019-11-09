.. _examples:


Example Scripts
===============

To learn how to use Basilisk it is often convenient to study sample scripts and see how they were implemented.  Below are a series of example Basilisk scripts grouped by topics and complexity.  The first sections illustrate how to setup simple spacecraft simulations and then the following sections begin to add more complexity.



The example script source code can be viewed directly within the browser by clicking on the Source link next to the python method name as shown in the image below:

.. image:: /_images/static/scenarioSourceLink.png
   :width: 1316px
   :height: 106px
   :scale: 50%
   :alt: scenario python source link
   :align: center

The python code contains additional comments to explain what is being setup and tested here.  The example instructions are all stored as extensive comments within the python script.  Read the script source to study how this example functions.


Orbital Simulations
-------------------
This section contains scripts that illustrate how to setup a simple spacecraft orbital simulations.

.. toctree::
   :maxdepth: 1

   Basic Orbit Simulations <scenarioBasicOrbit>
   Delta_v Orbit Maneuvers <scenarioOrbitManeuver>
   Multiple Gravitational Bodies <scenarioOrbitMultiBody>
   Defining Motion Relative to Planet <scenarioCentralBody>
   Simulating Trajectory about Multiple Celestial Bodies <scenarioPatchedConics>

.. toctree::
   :maxdepth: 1

    OrbitalSimulations/


Attitude Simulations
--------------------

Attitude Regulation Control
^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. toctree::
   :maxdepth: 1

   Inertial Attitude Pointing <scenarioAttitudeFeedback>
   Using Separate TAsk Group for Control <scenarioAttitudeFeedback2T>
   Inertial Pointing with Python Module <scenarioAttitudePythonPD>
   Basic Attitude Pointing in Deep Space <scenarioAttitudePointing>
   Complex Attitude Pointing in Deep Space <scenarioAttitudeFeedbackNoEarth>


Attitude Guidance
^^^^^^^^^^^^^^^^^

.. toctree::
   :maxdepth: 1

   Hill Frame Pointing <scenarioAttitudeGuidance>
   Velocity Frame Pointing <scenarioAttGuideHyperbolic>


Attitude Control with Actuators
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. toctree::
   :maxdepth: 1

   Pointing with Reaction Wheels <scenarioAttitudeFeedbackRW>
   Pointing with Attitude Thrusters <scenarioAttitudeFeedback2T_TH>


Attitude Steering Control
^^^^^^^^^^^^^^^^^^^^^^^^^

.. toctree::
   :maxdepth: 1

   MRP Steering Law <scenarioAttitudeSteering>

Complex Spacecraft Dynamics Simulations
---------------------------------------

.. toctree::
   :maxdepth: 1

   Fuel Slosh <scenarioFuelSlosh>
   Flexible (Hinged) Panels <scenarioHingedRigidBody>


Planetary Environments
----------------------

Magnetic Field Models
^^^^^^^^^^^^^^^^^^^^^

.. toctree::
   :maxdepth: 1

   Centered Dipole Model <scenarioMagneticFieldCenteredDipole>
   World Magnetic Model WMM <scenarioMagneticFieldWMM>



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



Spacecraft Sub-Systems
----------------------

Power Sub-System
^^^^^^^^^^^^^^^^

.. toctree::
   :maxdepth: 1

   Basic Power Usage and Tracking  <scenarioPowerDemo>


Monte Carlo Simulations
-----------------------

.. toctree::
   :maxdepth: 1

   MC run with RW control  <scenarioMonteCarloAttRW>
   MC run using Python Spice setup  <scenarioMonteCarloSpice>


bskSim()-Based Simulation
-------------------------

.. toctree::
   :maxdepth: 1

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
   :maxdepth: 1

   Two-Spacecraft Formation  <BskSim/scenarios/scenario_BasicOrbitFormation>


Formation Flying Control
^^^^^^^^^^^^^^^^^^^^^^^^

.. toctree::
   :maxdepth: 1

   Relative Pointing Control  <BskSim/scenarios/scenario_RelativePointingFormation>



Advanced Simulation Options
---------------------------

.. toctree::
   :maxdepth: 1

   Setting the Integrator  <scenarioIntegrators>



Advanced Visualizations
-----------------------

Live Plotting
^^^^^^^^^^^^^

.. toctree::
   :maxdepth: 1

   Regular Basilisk simulation using Live Plotting  <scenarioBasicOrbitLivePlot>
   bskSim Basilisk simulation using Live Plotting <BskSim/scenarios/scenario_BasicOrbit_LivePlot>


Interfacing with Vizard
^^^^^^^^^^^^^^^^^^^^^^^

.. toctree::
   :maxdepth: 1

   Live Streaming to Vizard  <scenarioBasicOrbitStream>
   Pointing a Vizard Camera  <scenarioVizPoint>




