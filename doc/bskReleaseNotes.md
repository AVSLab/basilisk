# Basilisk Release Notes {#bskReleaseNotes}

**Note:**
This software is currently in a limited alpha public-release.  The Basilisk development is still in progress, and some behaviors and setups API's are bound to change.  That said, we are very excited by the great capabilities that this software already has, and plan to be updating this software regularly.  There is some documentation in terms of Doxygen generated HTML documentation, but also documentation within the code, and several Basilisk modules that are math heavy have LaTeX'd documentation folders as well.  Best place to start is to run the integrated tutorial scripts inside the SimScenarios folder.  More documentation and tutorial scripts are in development.

## In Progress Features
* new Python packaging of the Basilisk modules
* ability to integrate custom Basilisk modules that are kept outside of the core Basilisk folder
* more integrated BSK scenarios and tutorials
* Variable Speed CMG module that include imbalanced dynamics
* GPU based methods to evaluate solar radiation pressure forces and torques
* atmospheric drag evaluation using multi-faceted spacecraft model
* depletable mass dynamics model


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