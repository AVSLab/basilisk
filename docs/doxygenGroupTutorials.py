
## \defgroup Tutorials Basilisk Tutorials
# @brief Basilisk Tutorials and Demonstration Scripts.
# The collection provide a self-study guide
# to learn how to setup Basilisk astrodynamics simulations.  The chapters provide groupings of
# common sub-topics.
## @{
#   \defgroup Tutorial_1 Chapter 1: Orbital Simulations
#   @brief This chapter covers spacecraft orbital simulations where only translation is considered.
#   @{
#       \defgroup scenarioBasicOrbitGroup                   1.1 Basic Orbits
#       \defgroup scenarioIntegratorsGroup                  1.2 Using Integrators
#       \defgroup scenarioOrbitManeuverGroup                1.3 Doing Impulsive Orbit Maneuvers
#       \defgroup scenarioOrbitMultiBodyGroup               1.4 Using Multiple Gravitational Bodies
#       \defgroup scenarioCentralBodyGroup                  1.5 Setting States Relative to Planets
#   @}
#
#   \defgroup Tutorial_2 Chapter 2: Attitude Simulations
#   @brief This chapter covers simulations that include attitude motion a spacecraft.
#   Modular flight software modules are discussed to create full attitude tracking
#   solution.
#   @{
#       \defgroup Tutorials_21 2.1  Attitude Regulation Control
#       @brief This section covers spacecraft attitude control feedback examples using a range of
#       simulation options.  In each case regulation attitude control solution is applied to
#       stabilize the spacecraft orientation.
#       @{
#           \defgroup scenarioAttitudeFeedbackGroup         2.1.1 Using 1 Task Group
#           \defgroup scenarioAttitudeFeedback2TGroup       2.1.2 Using 2 Task Groups (SIM and FSW)
#           \defgroup scenarioAttitudePythonPDGroup         2.1.3 Using Python FSW Module
#           \defgroup scenarioAttitudePointingGroup         2.1.4 Basic Attitude Control in Deep Space
#           \defgroup scenarioAttitudeFeedbackNoEarthGroup  2.1.5 Complex Control in Deep Space
#       @}
#       \defgroup Tutorials_22 2.2  Attitude Guidance
#       @brief This section covers spacecraft attitude guidance examples.  The attitude
#       guidance modules are designed to be connected in a modular manner using baseline
#       and dynamics behaviors.
#       @{
#           \defgroup scenarioAttitudeGuidanceGroup         2.2.1 Hill Frame Guidance
#           \defgroup scenarioAttGuideHyperbolicGroup       2.2.2 Velocity Frame Guidance
#       @}
#       \defgroup scenarioAttitudeFeedbackRWGroup           2.3 Reaction Wheel Control
#       \defgroup scenarioAttitudeSteeringGroup             2.4 MRP Steering Attitude Control
#   @}
#
#   \defgroup Tutorial_3 Chapter 3: Complex Spacecraft Dynamics Simulations
#   @brief This chapter discusses spacecraft dynamics components that reach beyond the
#   classical translational and rotational degrees of freedom.
#   @{
#       \defgroup scenarioFuelSloshGroup                    3.1 Fuel Slosh
#       \defgroup scenarioHingedRigidBodyGroup              3.2 Flexible (Hinged) Panels
#   @}
#
#   \defgroup Tutorial_4 Chapter 4: Planetary Environments
#   @brief This chapter discusses setting up planetary environments such a a neutral density atmosphere
#   or a magnetic field.
#   @{
#       \defgroup Tutorials_41 4.1  Magnetic Field Models
#       @brief This section covers planetary magnetic field models.
#       @{
#           \defgroup scenarioMagneticFieldCenteredDipoleGroup   4.1.1 Centered Dipole Model
#       @}
#   @}
#
#   \defgroup Tutorial_5 Chapter 5: Spacecraft Sensors
#   @brief This chapter discusses setting up and using spacecraft sensors within Basilisk.
#   @{
#       \defgroup Tutorials_51 5.1  Coarse Sun Sensors
#       @brief This section covers coarse sun sensor or CSS devices.
#       @{
#           \defgroup scenarioCSSGroup                      5.1.1 Adding CSS to simulation
#           \defgroup scenarioCSSFiltersGroup               5.1.2 Estimating Sun Heading with CSS
#       @}
#   @}
#
#   \defgroup Tutorial_6 Chapter 6: Monte Carlo Simulations
#   @brief This chapter discusses how to setup and run Basilisk simulations in a
#   Monte Carlo (MC) configuration.  Here the initial spacecraft states and parameters
#   can be statistically varied and the resulting performance observed over a large
#   number of simulation runs.
#   @{
#       \defgroup scenarioMonteCarloAttRWGroup              6.1 MC run with RW control
#   @}
#
#   \defgroup Tutorial_7 Chapter 7: bskSim()-Based Simulation
#   @brief This chapter discusses how to use the `bskSim()` simulation class within Basilisk.
#   Here the spacecraft dynamics and flight algorithms have been packaged into a class
#   allowing for compact Python simulation code to be written.  Further, the flight software
#   algorithm can be setup in a range of flight mode, including the ability to switch between
#   flight modes.
#   @{
#       \defgroup scenario_BasicOrbitGroup                  7.1 Basic Orbital Simulation
#       \defgroup scenario_FeedbackRWGroup                  7.2 Attitude Detumble Control
#       \defgroup scenario_AttGuidanceGroup                 7.3 Hill Pointing Attitude Control
#       \defgroup scenario_AttGuidHyperbolicGroup           7.4 Velocity Frame Pointing Control
#       \defgroup scenario_AttSteeringGroup                 7.5 MRP Steering Attitude Control
#       \defgroup scenario_AttEclipseGroup                  7.6 Sun Pointing Mode Include Eclipse Evaluation
#   @}
#
#   \defgroup Tutorial_8 Chapter 8: Spacecraft Formation Flying
#   @brief This chapter discusses how setup simulations that involve more than one satellite.
#   @{
#       \defgroup Tutorials_81 8.1  Formation Flying Dynamics
#       @brief This section covers simulations involving more than one spacecraft.
#       @{
#           \defgroup scenario_BasicOrbitFormationGroup     8.1.1 Two-Spacecraft Formation
#       @}
#       \defgroup Tutorials_82 8.2  Formation Flying Control
#       @brief This section covers FSW control simulations that depend on the relative
#       states of two or more spacecraft.
#       @{
#           \defgroup scenario_RelativePointingFormationGroup 8.2.1 Relative Pointing Control
#       @}
#   @}
#
#   \defgroup Tutorial_9 Chapter 9: Simulations Interfacing with Vizard
#   @brief This chapter illustrates scenarios where the BSK simulation interfaces with teh
#   Vizard Unit-Based Visualization tool.
#   @{
#       \defgroup scenarioVizPointGroup                     9.1 Pointing a Vizard Camera.
#   @}
#
## @}

