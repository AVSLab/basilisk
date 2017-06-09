# Basilisk Release Notes {#bskReleaseNotes}

**Note:**
This software is currently in a limited alpha public-release.  The Basilisk development is still in progress, and some behaviors and setups API's are bound to change.  That said, we are very excited by the great capabilities that this software already has, and plan to be updating this software regularly.  There is some documentation in terms of Doxygen generated HTML documentation, but also documentation within the code, and several Basilisk modules that are math heavy have LaTeX'd documentation folders as well.  Best place to start is to run the integrated tutorial scripts inside the SimScenarios folder.  More documentation and tutorial scripts are in development.

## In Progress Features
* new Python packaging of the Basilisk modules
* ability to integrate custom Basilisk modules that are kept outside of the core Basilisk folder
* more integrated BSK scenarios and tutorials
* GPU based methods to evaluate solar radiation pressure forces and torques
* atmospheric drag evaluation using multi-faceted spacecraft model

## Version 0.1.3
<ul>
	   <li>There is a new capability to now write BSK modules in Python, and integrated them directly with the C and C++ BSK modules.  Documentation is still in progress, but a sample is found in <code>SimScenarios/test_scenarioAttitudePythonPD.py</code>.</li>
	   <li>A new Variable Speed Control Moment Gyroscope (VSCMG) state effector module has been created.  This module provides a torque-level VSCMG simulation which also includes the gyro frame or wheel being imbalanced.  If the latter modes are engaged, the simulation does slow down noticeably, but you get the full physics.</li>
	   <li>In the simulation the initial spacecraft position and velocity states are now specified now using the spacecraft center of mass location C, not the body fixed point B.  This greatly simplifies the simulation setup.  Upon initialization, the sim determines what the true center of mass of the spacecraft is using all time varying mass components, and sets the proper B point position and velocity vectors.</li> 
	   <li>Specifying the initial spacecraft position and velocity states can now be done anywhere before the BSK initialization.  The user sets init versions of the position and velocity vectors.  The setState() method on the state engine thus doesn't have to be used. </li>
	   <li>There is a new <code>initializeSimulationAndDiscover</code> method to init the BSK simulation that automatically checks if messages are shared across multiple simulation threads.  See the modified <code> SimScenarios/test_scenarioAttitudeFeedback2T.py</code> file for how this simplifies the dual-threaded setup.</li>
	   <li>The <code>MRP_Steering</code> and <code>PRV_Steering</code> FSW modules have been broken up into a separate kinematic steering command (commanded desired angular velcocity vector) and an associated angular velocity servo module name <code>rateServoFullNonlinear</code>.  This will break any existing code that used either of these two attitude steering modules.  The Python simulation code must be updated to to account for these new modules as done in the MRP_Steering integrated test <code>test_MRP_steeringInt.py</code>.</li>    

</ul>

## Version 0.1.2
<ul>
	   <li>All unit and integrated tests now pass on Linux.  The root issue was a varaible length string variable in an output message.  These strings have now been removed as they are no longer needed.</li>
	   <li>The position and velocity of the center of mass of the spacecraft was added to the messaging system, so now the spacecraft’s translational states can be logged by the center of mass of the spacecraft (r_CN_N and v_CN_N) or the origin of the body frame which is fixed to the hub (r_BN_N and v_BN_N). Additionally, the mass properties of the spacecraft was organized into an updateSCMassProps method that incapsulates the calculations of mass property calculations.</li>
	   <li>Updated UKF FSW module to be able to run on gryo only information when the star tracker is not available.</li> 
</ul>

## Version 0.1.1
<ul>
	   <li>On Linux, simplified the processing running BSK modules that require boost.  This makes the Viz related communicaiton modules working again.</li>
	   <ul>
	       <li>Added boost libs built on Ubunutu against gcc 5.4.0 20160609.</li>
	       <li>Added RPATH settings to allow for build directory to be placed outside source directory</li>
	   </ul>
	   <li>Major addition with new depleatable mass dynamic modeling, including some fuel tank dynamic models.</li>
	   <li>minor fix for Monte Carlo dispersions</li> 
</ul>


## Version 0.1.0
### Simulation modules include:
<ul>
<li>Flexible integration structure with fixed time step RK1, RK2 and RK4 included</li>
	   <li>Rigid spacecraft simulated through <code>spacecratPlus()</code> module.  The spacecraft object makes it simple to add external disturbances through <code>dynEffectors</code> and state depended actuation through <code>stateEffectors</code>.
	   <ul>
    	   <li>Dynamics Effectors (acuation methods which do not have their own states to integrate)</li>
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
</ul>

### The AVS Lab Flight Algorithm folder contains:
<ul>
<li>FSW template module</li>
	   <li>CSS based sun heading estimation module</li>
	   <li>UKF filter to determine inertial attitude</li>
	   <li>UKF fitler to determine CSS based body-relative sun heading</li>
	   <li>Attitude Guidance modules:</li>
	       <ul>
	           <li>Pointing towards two celestial objects</li>
	           <li>Inertial Pointing</li>
	           <li>Hill Frame Pointing</li>
	           <li>Euler rotation sequence to add dynamics capabiliteis to the attitude reference generation</li>
	           <li>Spinning about an inertially fixed axis</li>
	           <li>A raster manager module that can change the guidance module states</li>
	           <li>Velocity frame pointing</li>
	           <li>attitude tracking error evaluation module</li>
	           <li>Deadband module for attitude trackign error</li>
	       </ul>
	   <li>DV guidance module</li>
	   <li>Effector Interfaces</li>
	       <ul>
	           <li>mapping of control torque onto RW motor torques</li>
	           <li>Converting RW motor torques to voltages</li>
	           <li>RW null motion module to equalize the wheel speeds continuously</li>
	           <li>Thruster (THR) firing logic using a Schmitt trigger</li>
	           <li>THR firign logic using a remainder calculation</li>
	           <li>mappign of a command torque onto a set of THR devices</li>
	           <li>module to evalute the net momentum to dump with thrusters</li>
	       </ul>
</ul>