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

Spacecraft::Spacecraft()
{
    // - Set default names
    this->spacecraftName = "spacecraft";
    this->scStateOutMsgName = "inertial_state_output";
    this->scMassStateOutMsgName = "mass_state_output";

    // - Set values to either zero or default values
    this->scStateOutMsgId = -1;
    this->dvAccum_B.setZero();
    this->dvAccum_BN_B.setZero();

    return;
}

Spacecraft::~Spacecraft()
{
    return;
}

void Spacecraft::addStateEffector(StateEffector *newStateEffector)
{
    this->states.push_back(newStateEffector);

    // Give the stateEffector the name of the spacecraft it is attached to
    newStateEffector->nameOfSpacecraftAttachedTo = this->spacecraftName;

    return;
}

void Spacecraft::addDynamicEffector(DynamicEffector *newDynamicEffector)
{
    this->dynEffectors.push_back(newDynamicEffector);

    return;
}

void Spacecraft::writeOutputMessages(uint64_t clockTime)
{
    return;
}
void Spacecraft::linkInStates(DynParamManager& statesIn)
{
    return;
}

/*! This is the constructor, setting variables to default values */
SpacecraftDynamics::SpacecraftDynamics()
{
    // - Set default names
    this->sysTimePropertyName = "systemTime";

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
    // - Call this for the primary spacecraft
    // - Create the message for the spacecraft state
    this->primaryCentralSpacecraft.scStateOutMsgName = this->primaryCentralSpacecraft.spacecraftName + this->primaryCentralSpacecraft.scStateOutMsgName;
    this->primaryCentralSpacecraft.scMassStateOutMsgName = this->primaryCentralSpacecraft.spacecraftName + this->primaryCentralSpacecraft.scMassStateOutMsgName;
    this->primaryCentralSpacecraft.scStateOutMsgId = SystemMessaging::GetInstance()->CreateNewMessage(this->primaryCentralSpacecraft.scStateOutMsgName,
                                                                             sizeof(SCStatesSimMsg),
                                                                             this->numOutMsgBuffers,
                                                                             "SCPlusStatesSimMsg", this->moduleID);
    // - Create the message for the spacecraft mass state
    this->primaryCentralSpacecraft.scMassStateOutMsgId = SystemMessaging::GetInstance()->CreateNewMessage(this->primaryCentralSpacecraft.scMassStateOutMsgName,
                                                                                 sizeof(SCMassPropsSimMsg),
                                                                                 this->numOutMsgBuffers,
                                                                                 "SCPlusMassPropsSimMsg", this->moduleID);
    // - Call the gravity fields selfInit method
    this->primaryCentralSpacecraft.gravField.SelfInit();

    return;
}

/*! This method is used to cross link the messages and to initialize the dynamics */
void SpacecraftDynamics::CrossInit()
{
    // - Call gravity field cross initialization for all spacecraft
    this->primaryCentralSpacecraft.gravField.CrossInit();
    
    // - Call method for initializing the dynamics of spacecraftPlus
    this->initializeDynamics();

    return;
}

/*! This method attaches a stateEffector to the dynamicObject */
void SpacecraftDynamics::addSpacecraftUndocked(Spacecraft *newSpacecraft)
{
    this->unDockedSpacecraft.push_back(newSpacecraft);

    return;
}

/*! This method attaches a stateEffector to the dynamicObject */
void SpacecraftDynamics::attachSpacecraftToPrimary(Spacecraft *newSpacecraft)
{
    this->spacecraftDockedToPrimary.push_back(newSpacecraft);

    return;
}

/*! This method attaches a stateEffector to the dynamicObject */
void SpacecraftDynamics::attachSpacecraftToSecondary(Spacecraft *newSpacecraft)
{
    this->spacecraftDockedToSecondary.push_back(newSpacecraft);

    return;
}

/*! This is the method where the messages of the state of vehicle are written */
void SpacecraftDynamics::writeOutputMessages(uint64_t clockTime)
{
    // - Write output messages for each spacecraft
    // - Populate state output message
    SCStatesSimMsg stateOut;
    eigenMatrixXd2CArray(*this->primaryCentralSpacecraft.inertialPositionProperty, stateOut.r_BN_N);
    eigenMatrixXd2CArray(*this->primaryCentralSpacecraft.inertialVelocityProperty, stateOut.v_BN_N);
    Eigen::MRPd sigmaLocal_BN;
    sigmaLocal_BN = (Eigen::Vector3d) this->primaryCentralSpacecraft.hubSigma->getState();
    Eigen::Matrix3d dcm_NB = sigmaLocal_BN.toRotationMatrix();
    Eigen::Vector3d rLocal_CN_N = (*this->primaryCentralSpacecraft.inertialPositionProperty) + dcm_NB*(*this->primaryCentralSpacecraft.c_B);
    Eigen::Vector3d vLocal_CN_N = (*this->primaryCentralSpacecraft.inertialVelocityProperty) + dcm_NB*(*this->primaryCentralSpacecraft.cDot_B);
    eigenVector3d2CArray(rLocal_CN_N, stateOut.r_CN_N);
    eigenVector3d2CArray(vLocal_CN_N, stateOut.v_CN_N);
    eigenMatrixXd2CArray(this->primaryCentralSpacecraft.hubSigma->getState(), stateOut.sigma_BN);
    eigenMatrixXd2CArray(this->primaryCentralSpacecraft.hubOmega_BN_B->getState(), stateOut.omega_BN_B);
    eigenMatrixXd2CArray(this->primaryCentralSpacecraft.dvAccum_B, stateOut.TotalAccumDVBdy);
    stateOut.MRPSwitchCount = this->primaryCentralSpacecraft.hub.MRPSwitchCount;
    eigenMatrixXd2CArray(this->primaryCentralSpacecraft.dvAccum_BN_B, stateOut.TotalAccumDV_BN_B);
    eigenVector3d2CArray(this->primaryCentralSpacecraft.nonConservativeAccelpntB_B, stateOut.nonConservativeAccelpntB_B);
    eigenVector3d2CArray(this->primaryCentralSpacecraft.omegaDot_BN_B, stateOut.omegaDot_BN_B);
    SystemMessaging::GetInstance()->WriteMessage(this->primaryCentralSpacecraft.scStateOutMsgId, clockTime, sizeof(SCStatesSimMsg),
                                                 reinterpret_cast<uint8_t*> (&stateOut), this->moduleID);

    // - Populate mass state output message
    SCMassPropsSimMsg massStateOut;
    massStateOut.massSC = (*this->primaryCentralSpacecraft.m_SC)(0,0);
    eigenMatrixXd2CArray(*this->primaryCentralSpacecraft.c_B, massStateOut.c_B);
    eigenMatrixXd2CArray(*this->primaryCentralSpacecraft.ISCPntB_B, (double *)massStateOut.ISC_PntB_B);
    SystemMessaging::GetInstance()->WriteMessage(this->primaryCentralSpacecraft.scMassStateOutMsgId, clockTime, sizeof(SCMassPropsSimMsg),
                                                 reinterpret_cast<uint8_t*> (&massStateOut), this->moduleID);

    return;
}

/*! This method is a part of sysModel and is used to integrate the state and update the state in the messaging system */
void SpacecraftDynamics::UpdateState(uint64_t CurrentSimNanos)
{
    // - Convert current time to seconds
    double newTime = CurrentSimNanos*NANO2SEC;

    // - Get access to the spice bodies
    this->primaryCentralSpacecraft.gravField.UpdateState(CurrentSimNanos);

    // - Integrate the state forward in time
    this->integrateState(newTime);
    // - Update the inertial position of each spacecraft
    Eigen::Vector3d rLocal_BN_N = this->primaryCentralSpacecraft.hubR_N->getState();
    Eigen::Vector3d vLocal_BN_N = this->primaryCentralSpacecraft.hubV_N->getState();
    this->primaryCentralSpacecraft.gravField.updateInertialPosAndVel(rLocal_BN_N, vLocal_BN_N);

    // - Write the state of the vehicle into messages
    this->writeOutputMessages(CurrentSimNanos);
    this->simTimePrevious = CurrentSimNanos;

    return;
}

/*! This method allows the spacecraftDynamics to have access to the current state of the hub for MRP switching, writing 
 messages, and calculating energy and momentum */
void SpacecraftDynamics::linkInStates(DynParamManager& statesIn)
{
    // - Get access to all spacecraft hub states
    this->primaryCentralSpacecraft.hubR_N = statesIn.getStateObject(this->primaryCentralSpacecraft.hub.nameOfHubPosition);
    this->primaryCentralSpacecraft.hubV_N = statesIn.getStateObject(this->primaryCentralSpacecraft.hub.nameOfHubVelocity);
    this->primaryCentralSpacecraft.hubSigma = statesIn.getStateObject(this->primaryCentralSpacecraft.hub.nameOfHubSigma);   /* Need sigmaBN for MRP switching */
    this->primaryCentralSpacecraft.hubOmega_BN_B = statesIn.getStateObject(this->primaryCentralSpacecraft.hub.nameOfHubOmega);

    // - Get access to the hubs position and velocity in the property manager
    this->primaryCentralSpacecraft.inertialPositionProperty = statesIn.getPropertyReference(this->primaryCentralSpacecraft.gravField.inertialPositionPropName);
    this->primaryCentralSpacecraft.inertialVelocityProperty = statesIn.getPropertyReference(this->primaryCentralSpacecraft.gravField.inertialVelocityPropName);
    this->primaryCentralSpacecraft.g_N = statesIn.getPropertyReference(this->primaryCentralSpacecraft.gravField.vehicleGravityPropName);

    return;
}

/*! This method is used to initialize the simulation by registering all of the states, linking the dynamicEffectors,
 stateEffectors, and the hub, initialize gravity, and initialize the sim with the initial conditions specified in python
 for the simulation */
void SpacecraftDynamics::initializeDynamics()
{
    // - SpacecraftPlus initiates all of the spaceCraft mass properties
    Eigen::MatrixXd initM_SC(1,1);
    Eigen::MatrixXd initMDot_SC(1,1);
    Eigen::MatrixXd initC_B(3,1);
    Eigen::MatrixXd initISCPntB_B(3,3);
    Eigen::MatrixXd initCPrime_B(3,1);
    Eigen::MatrixXd initCDot_B(3,1);
    Eigen::MatrixXd initISCPntBPrime_B(3,3);
    Eigen::MatrixXd systemTime(2,1);
    systemTime.setZero();
    // - Create the properties
    std::string tmpName;
    tmpName = this->primaryCentralSpacecraft.spacecraftName + "_" + "m_SC";
    this->primaryCentralSpacecraft.m_SC = this->dynManager.createProperty(tmpName, initM_SC);
    tmpName = this->primaryCentralSpacecraft.spacecraftName + "_" + "mDot_SC";
    this->primaryCentralSpacecraft.mDot_SC = this->dynManager.createProperty(tmpName, initMDot_SC);
    tmpName = this->primaryCentralSpacecraft.spacecraftName + "_" + "centerOfMassSC";
    this->primaryCentralSpacecraft.c_B = this->dynManager.createProperty(tmpName, initC_B);
    tmpName = this->primaryCentralSpacecraft.spacecraftName + "_" + "inertiaSC";
    this->primaryCentralSpacecraft.ISCPntB_B = this->dynManager.createProperty(tmpName, initISCPntB_B);
    tmpName = this->primaryCentralSpacecraft.spacecraftName + "_" + "inertiaPrimeSC";
    this->primaryCentralSpacecraft.ISCPntBPrime_B = this->dynManager.createProperty(tmpName, initISCPntBPrime_B);
    tmpName = this->primaryCentralSpacecraft.spacecraftName + "_" + "centerOfMassPrimeSC";
    this->primaryCentralSpacecraft.cPrime_B = this->dynManager.createProperty(tmpName, initCPrime_B);
    tmpName = this->primaryCentralSpacecraft.spacecraftName + "_" + "centerOfMassDotSC";
    this->primaryCentralSpacecraft.cDot_B = this->dynManager.createProperty(tmpName, initCDot_B);
    this->sysTime = this->dynManager.createProperty(this->sysTimePropertyName, systemTime);

    // - Give name of all spacecraft to attached hubEffector
    this->primaryCentralSpacecraft.hub.nameOfSpacecraftAttachedTo = this->primaryCentralSpacecraft.spacecraftName;
    
    // - Before er'body registers their properties, we need to prepend their state names with the spacecraft
    this->primaryCentralSpacecraft.hub.prependSpacecraftNameToStates();
    this->primaryCentralSpacecraft.gravField.prependSpacecraftNameToStates();
    std::vector<StateEffector*>::iterator stateIt;
    for(stateIt = this->primaryCentralSpacecraft.states.begin(); stateIt != this->primaryCentralSpacecraft.states.end(); stateIt++)
    {
        (*stateIt)->prependSpacecraftNameToStates();
    }
    

    // - Register the gravity properties with the dynManager, 'erbody wants g_N!
    this->primaryCentralSpacecraft.gravField.registerProperties(this->dynManager);

    // - Register the hub states
    this->primaryCentralSpacecraft.hub.registerStates(this->dynManager);

    // - Loop through stateEffectors to register their states
    for(stateIt = this->primaryCentralSpacecraft.states.begin(); stateIt != this->primaryCentralSpacecraft.states.end(); stateIt++)
    {
        (*stateIt)->registerStates(this->dynManager);
    }
    
    // - Link in states for the spaceCraftPlus, gravity and the hub
    this->linkInStates(this->dynManager);
    this->primaryCentralSpacecraft.gravField.linkInStates(this->dynManager);
    this->primaryCentralSpacecraft.hub.linkInStates(this->dynManager);

    // - Update the mass properties of the spacecraft to retrieve c_B and cDot_B to update r_BN_N and v_BN_N
    this->updateSystemMassProps(0.0);

    // - Edit r_BN_N and v_BN_N to take into account that point B and point C are not coincident
    // - Pulling the state from the hub at this time gives us r_CN_N
    Eigen::Vector3d rInit_BN_N = this->primaryCentralSpacecraft.hubR_N->getState();
    Eigen::MRPd sigma_BN;
    sigma_BN = (Eigen::Vector3d) this->primaryCentralSpacecraft.hubSigma->getState();
    Eigen::Matrix3d dcm_NB = sigma_BN.toRotationMatrix();
    // - Substract off the center mass to leave r_BN_N
    rInit_BN_N -= dcm_NB*(*this->primaryCentralSpacecraft.c_B);
    // - Subtract off cDot_B to get v_BN_N
    Eigen::Vector3d vInit_BN_N = this->primaryCentralSpacecraft.hubV_N->getState();
    vInit_BN_N -= dcm_NB*(*this->primaryCentralSpacecraft.cDot_B);
    // - Finally set the translational states r_BN_N and v_BN_N with the corrections
    this->primaryCentralSpacecraft.hubR_N->setState(rInit_BN_N);
    this->primaryCentralSpacecraft.hubV_N->setState(vInit_BN_N);

    // - Loop through the stateEffectros to link in the states needed
    for(stateIt = this->primaryCentralSpacecraft.states.begin(); stateIt != this->primaryCentralSpacecraft.states.end(); stateIt++)
    {
        (*stateIt)->linkInStates(this->dynManager);
    }

    // - Loop through the dynamicEffectors to link in the states needed
    std::vector<DynamicEffector*>::iterator dynIt;
    for(dynIt = this->primaryCentralSpacecraft.dynEffectors.begin(); dynIt != this->primaryCentralSpacecraft.dynEffectors.end(); dynIt++)
    {
        (*dynIt)->linkInStates(this->dynManager);
    }

    // - Call equations of motion at time zero
    this->equationsOfMotion(0.0);

    return;
}

/*! This method is used to update the mass properties of the entire spacecraft using contributions from stateEffectors */
void SpacecraftDynamics::updateSystemMassProps(double time)
{
    // - Zero the properties which will get populated in this method
    (*this->primaryCentralSpacecraft.m_SC).setZero();
    (*this->primaryCentralSpacecraft.mDot_SC).setZero();
    (*this->primaryCentralSpacecraft.c_B).setZero();
    (*this->primaryCentralSpacecraft.ISCPntB_B).setZero();
    (*this->primaryCentralSpacecraft.cPrime_B).setZero();
    (*this->primaryCentralSpacecraft.cDot_B).setZero();
    (*this->primaryCentralSpacecraft.ISCPntBPrime_B).setZero();

    // Add in hubs mass props to the spacecraft mass props
    this->primaryCentralSpacecraft.hub.updateEffectorMassProps(time);
    (*this->primaryCentralSpacecraft.m_SC)(0,0) += this->primaryCentralSpacecraft.hub.effProps.mEff;
    (*this->primaryCentralSpacecraft.ISCPntB_B) += this->primaryCentralSpacecraft.hub.effProps.IEffPntB_B;
    (*this->primaryCentralSpacecraft.c_B) += this->primaryCentralSpacecraft.hub.effProps.mEff*this->primaryCentralSpacecraft.hub.effProps.rEff_CB_B;

    // - Loop through state effectors to get mass props
    std::vector<StateEffector*>::iterator it;
    for(it = this->primaryCentralSpacecraft.states.begin(); it != this->primaryCentralSpacecraft.states.end(); it++)
    {
        (*it)->updateEffectorMassProps(time);
        // - Add in effectors mass props into mass props of spacecraft
        (*this->primaryCentralSpacecraft.m_SC)(0,0) += (*it)->effProps.mEff;
        (*this->primaryCentralSpacecraft.mDot_SC)(0,0) += (*it)->effProps.mEffDot;
        (*this->primaryCentralSpacecraft.ISCPntB_B) += (*it)->effProps.IEffPntB_B;
        (*this->primaryCentralSpacecraft.c_B) += (*it)->effProps.mEff*(*it)->effProps.rEff_CB_B;
        (*this->primaryCentralSpacecraft.ISCPntBPrime_B) += (*it)->effProps.IEffPrimePntB_B;
        (*this->primaryCentralSpacecraft.cPrime_B) += (*it)->effProps.mEff*(*it)->effProps.rEffPrime_CB_B;
        // For high fidelity mass depletion, this is left out: += (*it)->effProps.mEffDot*(*it)->effProps.rEff_CB_B
    }

    // Divide c_B and cPrime_B by the total mass of the spaceCraft to finalize c_B and cPrime_B
    (*this->primaryCentralSpacecraft.c_B) = (*this->primaryCentralSpacecraft.c_B)/(*this->primaryCentralSpacecraft.m_SC)(0,0);
    (*this->primaryCentralSpacecraft.cPrime_B) = (*this->primaryCentralSpacecraft.cPrime_B)/(*this->primaryCentralSpacecraft.m_SC)(0,0)
    - (*this->primaryCentralSpacecraft.mDot_SC)(0,0)*(*this->primaryCentralSpacecraft.c_B)/(*this->primaryCentralSpacecraft.m_SC)(0,0)/(*this->primaryCentralSpacecraft.m_SC)(0,0);
    Eigen::Vector3d omegaLocal_BN_B = this->primaryCentralSpacecraft.hubOmega_BN_B->getState();
    Eigen::Vector3d cLocal_B = (*this->primaryCentralSpacecraft.c_B);
    (*this->primaryCentralSpacecraft.cDot_B) = (*this->primaryCentralSpacecraft.cPrime_B) + omegaLocal_BN_B.cross(cLocal_B);

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
    this->updateSystemMassProps(integTimeSeconds);

    // - This is where gravity is computed (gravity needs to know c_B to calculated gravity about r_CN_N)
//    this->gravField.computeGravityField();

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
    this->updateSystemMassProps(integrateToThisTime);

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
