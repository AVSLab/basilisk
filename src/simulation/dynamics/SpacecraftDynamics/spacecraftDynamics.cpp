/*
 ISC License

 Copyright (c) 2016-2018, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

 Permission to use, copy, modify, and/or distribute this software for any
 purpose with or without fee is hereby granted, provided that the above
 copyright notice and this permission notice appear in all copies.

 THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

 */

#include "spacecraftDynamics.h"
#include "simFswInterfaceMessages/macroDefinitions.h"
#include "../_GeneralModuleFiles/svIntegratorRK4.h"
#include "utilities/avsEigenSupport.h"
#include "utilities/avsEigenMRP.h"
#include <iostream>

/*! This is the constructor, setting variables to default values */
SpacecraftDynamics::SpacecraftDynamics()
{
    // - Set default names
    this->sysTimePropertyName = "systemTime";
    this->scStateOutMsgName = "inertial_state_output";
    this->scMassStateOutMsgName = "mass_state_output";

    // - Set values to either zero or default values
    this->currTimeStep = 0.0;
    this->timePrevious = 0.0;
    this->simTimePrevious = 0;

    // - Set integrator as RK4 by default
    this->integrator = new svIntegratorRK4(this);

    return;
}

/*! This is the destructor, nothing to report here */
SpacecraftDynamics::~SpacecraftDynamics()
{
    return;
}

/*! This method creates the messages for s/c output data and initializes the gravity field*/
void SpacecraftDynamics::SelfInit()
{
    // - Call the gravity fields selfInit method
    this->gravField.SelfInit();

    return;
}

/*! This method is used to cross link the messages and to initialize the dynamics */
void SpacecraftDynamics::CrossInit()
{
    // - Call gravity field cross initialization
    this->gravField.CrossInit();
    // - Call method for initializing the dynamics of spacecraftDynamics
    this->initializeDynamics();

    return;
}

/*! This is the method where the messages of the state of vehicle are written */
void SpacecraftDynamics::writeOutputMessages(uint64_t clockTime)
{
    return;
}

/*! This method is a part of sysModel and is used to integrate the state and update the state in the messaging system */
void SpacecraftDynamics::UpdateState(uint64_t CurrentSimNanos)
{
    // - Convert current time to seconds
    double newTime = CurrentSimNanos*NANO2SEC;

    // - Get access to the spice bodies
    this->gravField.UpdateState(CurrentSimNanos);

    // - Integrate the state forward in time
    this->integrateState(newTime);
    this->gravField.updateInertialPosAndVel();

    // - Write the state of the vehicle into messages
    this->writeOutputMessages(CurrentSimNanos);
    this->simTimePrevious = CurrentSimNanos;

    return;
}

/*! This method allows the spacecraftDynamics to have access to the current state of the hub for MRP switching, writing 
 messages, and calculating energy and momentum */
void SpacecraftDynamics::linkInStates(DynParamManager& statesIn)
{
    return;
}

/*! This method is used to initialize the simulation by registering all of the states, linking the dynamicEffectors,
 stateEffectors, and the hub, initialize gravity, and initialize the sim with the initial conditions specified in python
 for the simulation */
void SpacecraftDynamics::initializeDynamics()
{
    // - SpacecraftDynamics initiates all of the spaceCraft mass properties
    Eigen::MatrixXd systemTime(2,1);
    systemTime.setZero();

    // - Create the properties
    this->sysTime = this->dynManager.createProperty(this->sysTimePropertyName, systemTime);
    
    // - Register the gravity properties with the dynManager, 'erbody wants g_N!
    this->gravField.registerProperties(this->dynManager);
    
    // - Register the hub states
    
    // - Loop through stateEffectors to register their states
    
    // - Link in states for the spacecraftDynamis, gravity and the hub
    this->linkInStates(this->dynManager);
    this->gravField.linkInStates(this->dynManager);

    // - Update the mass properties of the spacecraft to retrieve c_B and cDot_B to update r_BN_N and v_BN_N
    this->updateSCMassProps(0.0);

    // - Edit r_BN_N and v_BN_N to take into account that point B and point C are not coincident
    // - Pulling the state from the hub at this time gives us r_CN_N

    // - Loop through the stateEffectros to link in the states needed
    
    // - Loop through the dynamicEffectors to link in the states needed

    // - Call equations of motion at time zero
    this->equationsOfMotion(0.0);
    
    return;
}

/*! This method is used to update the mass properties of the entire spacecraft using contributions from stateEffectors */
void SpacecraftDynamics::updateSCMassProps(double time)
{
    // - Zero the properties which will get populated in this method

    // Add in hubs mass props to the spacecraft mass props

    // - Loop through state effectors to get mass props

    // Divide c_B and cPrime_B by the total mass of the spaceCraft to finalize c_B and cPrime_B

    return;
}

/*! This method is solving Xdot = F(X,t) for the system. The hub needs to calculate its derivatives, along with all of 
 the stateEffectors. The hub also has gravity and dynamicEffectors acting on it and these relationships are controlled 
 in this method. At the end of this method all of the states will have their corresponding state derivatives set in the 
 dynParam Manager thus solving for Xdot*/
void SpacecraftDynamics::equationsOfMotion(double integTimeSeconds)
{
    // - Update time to the current time
    uint64_t integTimeNanos = this->simTimePrevious + (integTimeSeconds-this->timePrevious)/NANO2SEC;
    (*this->sysTime) << integTimeNanos, integTimeSeconds;

    // - Zero all Matrices and vectors for back-sub and the dynamics

    // - Update the mass properties of the spacecraft
    this->updateSCMassProps(integTimeSeconds);

    // - This is where gravity is computed (gravity needs to know c_B to calculated gravity about r_CN_N)
    this->gravField.computeGravityField();

    // - Loop through dynEffectors to compute force and torque on the s/c

    // - Loop through state effectors to get contributions for back-substitution

    // - Compute the derivatives of the hub states before looping through stateEffectors

    return;
}

/*! This method is used to integrate the state forward in time, switch MRPs, calculate energy and momentum, and 
 calculate the accumulated deltaV */
void SpacecraftDynamics::integrateState(double integrateToThisTime)
{
    // - Find the time step
	double localTimeStep = integrateToThisTime - timePrevious;

    // - Find v_CN_N before integration for accumulated DV

    // - Integrate the state from the last time (timeBefore) to the integrateToThisTime
    double timeBefore = integrateToThisTime - localTimeStep;
    this->integrator->integrate(timeBefore, localTimeStep);
    this->timePrevious = integrateToThisTime;     // - copy the current time into previous time for next integrate state call

    // - Call hubs modify states to allow for switching of MRPs

    // - Loop over stateEffectors to call modifyStates

    // - Call mass properties to get current info on the mass props of the spacecraft
    this->updateSCMassProps(integrateToThisTime);

    // - Find v_CN_N after the integration for accumulated DV

    // - Find change in velocity

    // - Find accumulated DV of the center of mass in the body frame
    
    // - Find the accumulated DV of the body frame in the body frame
    
    // - non-conservative acceleration of the body frame in the body frame
    
    // - angular acceleration in the body frame

    // - Compute Energy and Momentum
    this->computeEnergyMomentum(integrateToThisTime);

    return;
}

/*! This method is used to find the total energy and momentum of the spacecraft. It finds the total orbital energy,
 total orbital angular momentum, total rotational energy and total rotational angular momentum. These values are used 
 for validation purposes. */
void SpacecraftDynamics::computeEnergyMomentum(double time)
{
    // - Grab values from state Manager
    // - Find DCM's

    // - Convert from inertial frame to body frame

    // - zero necessarry variables

    // - Get the hubs contribution

    // - Loop over stateEffectors to get their contributions to energy and momentum

    // - Get cDot_B from manager

    // - Add in orbital kinetic energy into the total orbital energy calculations

    // - Call gravity effector and add in its potential contributions to the total orbital energy calculations

    // - Find total rotational energy

    // - Find orbital angular momentum for the spacecraft

    // - Find rotational angular momentum for the spacecraft
    
    return;
}
