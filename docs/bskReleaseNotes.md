# Basilisk Release Notes {#bskReleaseNotes}

**Note:**
This software is currently in a limited alpha public-release.  The Basilisk development is still in progress, and some behaviors and setups API's are bound to change.  That said, we are very excited by the great capabilities that this software already has, and plan to be updating this software regularly.  There is some documentation in terms of Doxygen generated HTML documentation, but also documentation within the code, and several Basilisk modules that are math heavy have LaTeX'd documentation folders as well.  Best place to start is to run the integrated tutorial scripts inside the `src/tests/scenarios` folder.  More documentation and tutorial scripts are in development.

## In Progress Features
* ability to integrate custom Basilisk modules that are kept outside of the core Basilisk folder
* more integrated BSK scenarios and tutorials
* GPU based methods to evaluate solar radiation pressure forces and torques
* atmospheric drag evaluation using multi-faceted spacecraft model

## Version 0.X.X (latest develop)
<ul>
    <li>Improved how the `fuelSloshSpringMassDamper` effector class works.  It is now renamed to `LinearSpringMassDamper`.  It can be used to simulate both fuel sloshing, but also structural modes.  If the `LinearSpringMassDamper` is connected to a fuel tank, then it's mass depends on the amount of fuel left. The associated unit test illustrated how to setup this last capability.  The module also contains documentation on the associated math.
    </li>
    <li>A new `SphericalPendulum` effector class has been added.  For rotations a spherical pendulum is a better approximation rotational fuel slosh.  This effector can model rotational fuel slosh if connected to a tank (see unit test again), or it can model a torsional structural mode if not connected to a tank. Associated math documentation is included with the class.
    </li>
    <li>The booleans useTranslation and useRotation have been removed from the `HubEffector()` class. The defaults in hubEffector for mass properties: `mHub = 1`, `IHubPntBc_B = diag`(1), and `r_BcB_B = zeros(3)`, enable us to evaluate the same code no matter if the desire is only to have translational states, only rotational states, or both. This allows for less logic in hubEffector and removes possibility of fringe cases that result in unexpected results from a developer standpoint. The fix for if your python script is not working related to this change:
    <ul>
    <li>FIX: Remove any instances of useTranslation or useRotation defined in the hubEffector class.</li>
    </ul>
    <li>Changed name of the method `computeBodyForceTorque` to `computeForceTorque` in the `dynamicEffector` abstract class and any inheriting classes. This avoids the confusion of thinking that only body frame relative forces can be defined, but in reality this class gives the ability to define both external forces defined in the body frame and the inertial frame.
    </li>
    <li>
    Fixed an issue in `RadiationPressure` where the cannonball model was not computed in the proper frame.  An integrated test has been added, and the unit test is updated.  Note that the `RadiationPressure` model specification has changes slightly.  The default model is still the cannonball model.  To specify another model, the python methods `setUseCannonballModel()` or `setUseFacetedCPUModel()` are used.  Note that these take no argument anymore.
    <ul>
        <li>FIX: remove the argument from `setUseCannonballModel(true)` and use the methods `setUseCannonballModel()` or `setUseFacetedCPUModel()` without any arguments instead.</li>
    </ul>
    </li>
</ul>

## Version 0.2.2
<ul>
    <li>Fixed a build issues on the Windows platform is Visual Studio 2017 or later is used.</li>
    <li>Unified the Coarse Sun Sensor (CSS) sun heading filtering modules to use the same I/O messages.  All used messages are now in the fswMessage folder.</li>
    <li>Made the CSS sun heading filter messages consistently use the CBias value.  This allows particular sensors to have an indiviual (known) scaling coorection factor.  For example, if the return of one sensor is 10% stronger then that of the other sensors, then CBias is set to 1.10.  Default value is 1.0 assuming all CSS units have the same gain.</li>
    <li>The `src\tests\bskSimScenarios` folder now functions properly with the `bskSim` spacecraft class.</li>
    <li>The tutorial scripts in `src\tests\scenarios` are now simplified to pull out the unit testing functionality.  The unit testing is now down with the `test_XXX.py` scripts inside the `src\tests\testScripts` folder.</li>
    <li>The `bskSim` tutorial files are now tested through pytest as well.  The file `testScripts\bskTestScript.py` calls all the `bskSim` tutorial fails and ensures they run without error.</li>
</ul>

## Version 0.2.1
<ul>
    <li>Added messages for current fuel tank mass, fuel tank mDot, and thruster force and torque</li>
    <li>Changed the linearAlgebra.c/h support library to avoid using any dynamic memory allocation.</li>
    <li>Added some new function to linearAlgebra.c/h while making the library use the new BSK_PRINT() function.</li>
    <li>Added ability to simulate noise to the RW devices.</li>
    <li>Created a more complete spacecraft python simulation class called BSKsim, and recreated some BSK tutorial scripts to use BSKsim instead of the more manual spacecraft setup in the earlier scripts.</li>
    <li>Developed general functions to add saturation, discretization and Gauss Markov processes to signals.</li>
    <li>Created a new BSK_PRINT() function.  Here the coder can tag a message as an ERROR, WARNING, DEBUG or INFORMATION status.  The printout can be set to selectively show these print statements. </li>
</ul>

## Version 0.2.0 (First public beta)
<ul>
    <li>First open beta release of Basilisk</li>
    <li>Moved to a new file architecture.  This means older BSK python scripts need to be updated as the method to import BSK has changed.</li>
    <li>The source an now be forked from Bitbucket</li>
    <li>Percompiled binaries are provided through a python pip install wheel file.</li>
    <li>The Doxygen documentation now pulls in the BSK module description PDF file and makes it available via the class defintion html page.</li>
    <li>The tutorial python scripts are now moved to `src/test/scenarios`</li>
    <li>The `pytest` comman should now be run within the `src` sub-directory</li>
    <li>Updated fuel slosh model documentation</li>
    <li>Updated fuel tank documentation</li>
    <li>Adding noise and corruptions using a new utility to the BSK modules (in progress)</li>
    <li>New N-panel hinged rigid body module</li>
    <li>New 2-panel hinged rigid body module</li>
    <li>Added CSS sun-heading estimation tutorial script</li>
    <li>Added O'Keefe CSS sun-heading estimation module</li>
    </ul>
    
    
    
## Version 0.1.7
<ul>
    <li>New Monte-Carlo capability that uses multiple cores and hyperthreading to accelerate the MC evaluations.  Data is retained and stored for each MC run for robustness.  See `test_scenarioMonteCarloAttRW.py` for an example.</li>
    <li>Coarse Sun Sensor (CSS) modules can now scale the sensor output with the distance from the sun.</li>
    <li>CSS now have updated documentation that includes validation results.</li>
    <li>CSS, IMU have updated means to apply sensor corruptions.</li>
    <li>IMU, simple_nav and star tracker modules have been updated to use now internally Eigen vectors rather than C-Arrays.  <b>NOTE: if you have simulation scripts that use these modules you may have to update the script to set sensor states as Eigen vectors from python.</b></li>
    <li>All the dynamics, thruster and sensor simulation modules have expanded documentation and valdiation unit and integrated tests.  The validation results are automatically included in the module TeX documentation.</li>
</ul>

## Version 0.1.6
<ul>
    <li>new unit tests to validate the multi-body gravity simulation code in `SimCode/dynamics/gravityEffector/_UnitTest/test_gavityDynEffector.py` </li>
    <li>new hinged rigid body tutorial script in `SimScenarios/test_scenarioAttGuideHyperbolic.py`</li>
    <li>new tutorial to do velicity frame pointing on a hyperbolic orbit in `SimScenarios/test_scenarioHingedRigidBody.py` </li>
    <li>fixed various unit test issues that came up on the non-macOS builds</li>
    <li>added reaction wheel effector documentation</li>
    <li>added `orb_elem_convert` documentation</li>
    <li>added `boreAngCalc` documentation</li>
    
</ul>

## Version 0.1.5
<ul>
    <li>
        Lots of new module documentation which includes a discussion of what is being modeled, the validation tests, as well as a user guide to the module.  The new documentation includes:
        <ul>
            <li>Thruster `DynEffector` module</li>
            <li>ephemeris conversion module</li>
            <li>Coarse Sun Sensor module</li>
            <li>Updated BSK module template documentation</li>            
            <li>Updated documentation for IMU Sensor module</li>            
            <li>Gravity Effector module</li>            
            <li>SimpleNav Sensor module</li>            
            <li>Hinged Panel `StateEffector` module</li>            
        </ul>
    </li>
    <li>New tutorial scripts on
        <ul>
            <li>using CSS modules</li>
            <li>using fuel tank module and the fuel slosh particle `StateEffector`</li>
            <li>How to use `MRP_Steering()` along with the rate tracking sub-servo module</li>
        </ul>
    </li>
    <li>The CSS modules now use the planetary shadow message information to simulated being in a planet's shadow</li>
    <li>SRP DynEffector modules now simulates the impact of being in a planets shadow</li>
    <li>Included a method to validate all the AVS C-Function libraries like `rigidBodyKinematics`, `linearAlgebra` and `orbitalMotion` when the Basilisk `pytest` command is called.  There is also some documentation on using these libraries in `/SimCode/utilitiesSelfCheck/_Documentation` </li>
    <li>Updated the RW and gravitational body (i.e. adding Earth, sun, etc. to the simulation) to use new factory classes.  If you did use the older `simIncludeRW.py` or `simIncludeGravity.py` libraries, you'll need to update your python code to work with the new factory classes.</li>
</ul>


## Version 0.1.4
<ul>
    <li>A planetary eclipse model has been added.  This allows for the shadow of one or multiple planets to be taken into account, including the penumbra region.  This module writes an output message indicating if the spacecraft is in full sun light, partial shadow, or full shadow of the sun.</li>
	   <li>The body-fixed spacecraft structure frame has now been removed from the simulation and flight algorithm codes.  All spacecraft vectors and tensors are now set directly in the body frame B.  If the spacecraft parameters are given in terms of an alternate structure frame, these vectors and tensor must be transformed into the body frame first before being set in BSK.</li>
	   <li>The integrated tutorial test for using a Python based BSK module now has some documentation.</li>
	   <li>Created a method to compute the orbital potential and angular momentum energy.  This allows for the kinetic energy and angular momentum checks to flat-line even if the satellite is in orbit.  The spherical harmonics of the planet are taken into account as well.</li>
	   <li>Included a new Extended Kalman Filter module that determines the body-relative sun heading using the CSS signals.</li>
</ul>

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