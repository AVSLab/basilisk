
## \defgroup Tutorials Basilisk Tutorials
# @brief Basilisk Tutorials and Demonstration Scripts.
# The collection provide a self-study guide
# to learn how to setup Basilisk astrodynamics simulations.  The chapters provide groupings of
# common sub-topics.
## @{
#   \defgroup Tutorial_1 Chapter 1: Orbital Simulations
#   @brief This chapter covers spacecraft orbital simulations where only translation is considered.
#   @{
#       \defgroup Tutorials_1_0 1.1 Basic Orbits
#       \defgroup Tutorials_1_1 1.2 Using Integrators
#       \defgroup Tutorials_1_2 1.3 Doing Impulsive Orbit Maneuvers
#       \defgroup Tutorials_1_3 1.4 Using Multiple Gravitational Bodies
#       \defgroup Tutorials_1_4 1.5 Setting States Relative to Planets
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
#           \defgroup Tutorials_2_0     2.1.1 Using 1 Task Group
#           \defgroup Tutorials_2_0_2   2.1.2 Using 2 Task Groups (SIM and FSW)
#           \defgroup Tutorials_2_0_3   2.1.3 Using Python FSW Module
#           \defgroup Tutorials_2_0_1   2.1.4 Basic Attitude Control in Deep Space
#           \defgroup Tutorials_2_0_4   2.1.5 Complex Control in Deep Space
#       @}
#       \defgroup Tutorials_22 2.2  Attitude Guidance
#       @brief This section covers spacecraft attitude guidance examples.  The attitude
#       guidance modules are designed to be connected in a modular manner using baseline
#       and dynamics behaviors.
#       @{
#           \defgroup Tutorials_2_1     2.2.1 Hill Frame Guidance
#           \defgroup Tutorials_2_1_1   2.2.2 Velocity Frame Guidance
#       @}
#       \defgroup Tutorials_2_2         2.3 Reaction Wheel Control
#       \defgroup Tutorials_2_3         2.4 MRP Steering Attitude Control
#   @}
#
#   \defgroup Tutorial_3 Chapter 3: Complex Spacecraft Dynamics Simulations
#   @brief This chapter discusses spacecraft dynamics components that reach beyond the
#   classical translational and rotational degrees of freedom.
#   @{
#       \defgroup Tutorials_3_0         3.1 Fuel Slosh
#       \defgroup Tutorials_3_1         3.2 Flexible (Hinged) Panels
#   @}
#
#   \defgroup Tutorial_4 Chapter 4: Spacecraft Sensors
#   @brief This chapter discusses setting up and using spacecraft sensors within Basilisk.
#   @{
#       \defgroup Tutorials_41 4.1  Coarse Sun Sensors
#       @brief This section covers coarse sun sensor or CSS devices.
#       @{
#           \defgroup Tutorials_4_0     4.1.1 Adding CSS to simulation
#           \defgroup Tutorials_4_0_1   4.1.2 Estimating Sun Heading with CSS
#       @}
#   @}
#
#   \defgroup Tutorial_5 Chapter 5: Monte Carlo Simulations
#   @brief This chapter discusses how to setup and run Basilisk simulations in a
#   Monte Carlo (MC) configuration.  Here the initial spacecraft states and parameters
#   can be statistically varied and the resulting performance observed over a large
#   number of simulation runs.
#   @{
#       \defgroup Tutorials_5_0         5.1 MC run with RW control
#   @}
#
#   \defgroup Tutorial_6 Chapter 6: bskSim()-Based Simulation
#   @brief This chapter discusses how to use the `bskSim()` simulation class within Basilisk.
#   Here the spacecraft dynamics and flight alorithms have been packaged into a class
#   allowing for compact Python simulation code to be written.  Further, the flight software
#   algorithm can be setup in a range of flight mode, including the ability to switch between
#   flight modes.
#   @{
#       \defgroup Tutorials_6_0         6.1 Basic Orbital Simulation
#       \defgroup Tutorials_6_1         6.2 Attitude Detumble Control
#       \defgroup Tutorials_6_2         6.3 Hill Pointing Attitude Control
#       \defgroup Tutorials_6_3         6.4 Velocity Frame Pointing Control
#       \defgroup Tutorials_6_4         6.5 MRP Steering Attitude Control
#       \defgroup Tutorials_6_5         6.6 Sun Pointing Mode Include Eclipse Evaluation
#   @}
#
#   \defgroup Tutorial_7 Chapter 7: Spacecraft Formation Flying
#   @brief This chapter discusses how setup simulations that involve more than one satellite.
#   @{
#       \defgroup Tutorials_71 7.1  Formation Flying Dynamics
#       @brief This section covers simulations involving more than one spacecraft.
#       @{
#           \defgroup Tutorials_7_0     7.1.1 Two-Spacecraft Formation
#       @}
#       \defgroup Tutorials_72 7.2  Formation Flying Control
#       @brief This section covers FSW control simulations that depend on the relative
#       states of two or more spacecraft.
#       @{
#           \defgroup Tutorials_7_1     7.2.1 Relative Pointing Control
#       @}
#   @}
#
## @}

