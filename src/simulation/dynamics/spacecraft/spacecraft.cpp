/*
 ISC License

 Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#include "spacecraft.h"
#include "architecture/utilities/macroDefinitions.h"
#include "../_GeneralModuleFiles/svIntegratorRK4.h"
#include "architecture/utilities/avsEigenSupport.h"
#include "architecture/utilities/avsEigenMRP.h"
#include <iostream>


/*! This is the constructor, setting variables to default values */
Spacecraft::Spacecraft()
{
    // - Set default propery names
    this->sysTimePropertyName = "systemTime";
    this->propName_m_SC = "m_SC";
    this->propName_mDot_SC = "mDot_SC";
    this->propName_centerOfMassSC = "centerOfMassSC";
    this->propName_inertiaSC = "inertiaSC";
    this->propName_inertiaPrimeSC = "inertiaPrimeSC";
    this->propName_centerOfMassPrimeSC = "centerOfMassPrimeSC";
    this->propName_centerOfMassDotSC = "centerOfMassDotSC";

    // - Set values to either zero or default values
    this->dvAccum_CN_B.setZero();
    this->dvAccum_BN_B.setZero();
    this->dvAccum_CN_N.setZero();

    // - Set integrator as RK4 by default
    this->integrator = new svIntegratorRK4(this);
}

/*! This is the destructor, nothing to report here */
Spacecraft::~Spacecraft()
{
}


/*! This method is used to reset the module.

 */
void Spacecraft::Reset(uint64_t CurrentSimNanos)
{
    this->gravField.Reset(CurrentSimNanos);
    // - Call method for initializing the dynamics of spacecraft
    this->initializeDynamics();

    // compute initial spacecraft states relative to inertial frame, taking into account initial sc states might be defined relative to a planet
    this->gravField.updateInertialPosAndVel(this->hubR_N->getState(), this->hubV_N->getState());
    this->writeOutputStateMessages(CurrentSimNanos);
    // - Loop over stateEffectors to call writeOutputStateMessages and write initial state output messages
    std::vector<StateEffector*>::iterator it;
    for(it = this->states.begin(); it != this->states.end(); it++)
    {
        // - Call writeOutputStateMessages for stateEffectors
        (*it)->writeOutputStateMessages(CurrentSimNanos);
    }

    this->timeBefore = CurrentSimNanos * NANO2SEC;
    this->timeBeforeNanos = CurrentSimNanos;
}


/*! This method attaches a stateEffector to the dynamicObject */
void Spacecraft::addStateEffector(StateEffector *newStateEffector)
{
    this->assignStateParamNames<StateEffector *>(newStateEffector);

    this->states.push_back(newStateEffector);
}

/*! This method attaches a dynamicEffector to the dynamicObject */
void Spacecraft::addDynamicEffector(DynamicEffector *newDynamicEffector)
{
    this->assignStateParamNames<DynamicEffector *>(newDynamicEffector);

    this->dynEffectors.push_back(newDynamicEffector);
}

/*! This is the method where the messages of the state of vehicle are written */
void Spacecraft::writeOutputStateMessages(uint64_t clockTime)
{
    // - Populate state output message
    SCStatesMsgPayload stateOut;
    stateOut = this->scStateOutMsg.zeroMsgPayload;
    eigenMatrixXd2CArray(*this->inertialPositionProperty, stateOut.r_BN_N);
    eigenMatrixXd2CArray(*this->inertialVelocityProperty, stateOut.v_BN_N);
    Eigen::MRPd sigmaLocal_BN;
    sigmaLocal_BN = (Eigen::Vector3d) this->hubSigma->getState();
    Eigen::Matrix3d dcm_NB = sigmaLocal_BN.toRotationMatrix();
    Eigen::Vector3d rLocal_CN_N = (*this->inertialPositionProperty) + dcm_NB*(*this->c_B);
    Eigen::Vector3d vLocal_CN_N = (*this->inertialVelocityProperty) + dcm_NB*(*this->cDot_B);
    eigenVector3d2CArray(rLocal_CN_N, stateOut.r_CN_N);
    eigenVector3d2CArray(vLocal_CN_N, stateOut.v_CN_N);
    eigenMatrixXd2CArray(this->hubSigma->getState(), stateOut.sigma_BN);
    eigenMatrixXd2CArray(this->hubOmega_BN_B->getState(), stateOut.omega_BN_B);
    eigenMatrixXd2CArray(this->dvAccum_CN_B, stateOut.TotalAccumDVBdy);
    stateOut.MRPSwitchCount = this->hub.MRPSwitchCount;
    eigenMatrixXd2CArray(this->dvAccum_BN_B, stateOut.TotalAccumDV_BN_B);
    eigenMatrixXd2CArray(this->dvAccum_CN_N, stateOut.TotalAccumDV_CN_N);
    eigenVector3d2CArray(this->nonConservativeAccelpntB_B, stateOut.nonConservativeAccelpntB_B);
    eigenVector3d2CArray(this->omegaDot_BN_B, stateOut.omegaDot_BN_B);
    this->scStateOutMsg.write(&stateOut, this->moduleID, clockTime);

    // - Populate mass state output message
    SCMassPropsMsgPayload massStateOut;
    massStateOut = this->scMassOutMsg.zeroMsgPayload;
    massStateOut.massSC = (*this->m_SC)(0,0);
    eigenMatrixXd2CArray(*this->c_B, massStateOut.c_B);
    eigenMatrixXd2CArray(*this->ISCPntB_B, (double *)massStateOut.ISC_PntB_B);
    this->scMassOutMsg.write(&massStateOut, this->moduleID, clockTime);
}

/*! If the optional attitude reference input message is set, then read in the reference attitude and set it for the hub*/
void Spacecraft::readOptionalRefMsg()
{
    if (this->attRefInMsg.isLinked()) {
        Eigen::Vector3d omega_BN_B;
        AttRefMsgPayload attRefMsgBuffer;
        attRefMsgBuffer = this->attRefInMsg();
        Eigen::MRPd sigma_BN = cArray2EigenMRPd(attRefMsgBuffer.sigma_RN);
        Eigen::Vector3d omega_BN_N = cArray2EigenVector3d(attRefMsgBuffer.omega_RN_N);
        Eigen::Matrix3d dcm_BN = sigma_BN.toRotationMatrix().transpose();
        omega_BN_B = dcm_BN * omega_BN_N;

        this->hubSigma->setState(eigenMRPd2Vector3d(sigma_BN));
        this->hubOmega_BN_B->setState(omega_BN_B);
    }

    if (this->transRefInMsg.isLinked()) {
        Eigen::Vector3d r_RN_N;
        Eigen::Vector3d v_RN_N;
        TransRefMsgPayload transRefMsgBuffer;
        transRefMsgBuffer = this->transRefInMsg();

        r_RN_N = cArray2EigenVector3d(transRefMsgBuffer.r_RN_N);
        v_RN_N = cArray2EigenVector3d(transRefMsgBuffer.v_RN_N);

        this->hubR_N->setState(r_RN_N);
        this->hubV_N->setState(v_RN_N);
    }
}

/*! This method is a part of sysModel and is used to integrate the state and update the state in the messaging system */
void Spacecraft::UpdateState(uint64_t CurrentSimNanos)
{
    // - Get access to the spice bodies
    this->gravField.UpdateState(CurrentSimNanos);

    // - Integrate the state forward in time
    this->integrateState(CurrentSimNanos);

    // If set, read in and prescribe attitude reference motion
    readOptionalRefMsg();

    Eigen::Vector3d rLocal_BN_N = this->hubR_N->getState();
    Eigen::Vector3d vLocal_BN_N = this->hubV_N->getState();
    this->gravField.updateInertialPosAndVel(rLocal_BN_N, vLocal_BN_N);

    // - Write the state of the vehicle into messages
    this->writeOutputStateMessages(CurrentSimNanos);
    // - Loop over stateEffectors to call writeOutputStateMessages
    std::vector<StateEffector*>::iterator it;
    for(it = this->states.begin(); it != this->states.end(); it++)
    {
        // - Call writeOutputStateMessages for stateEffectors
        (*it)->writeOutputStateMessages(CurrentSimNanos);
    }
}

/*! This method allows the spacecraft to have access to the current state of the hub for MRP switching, writing
 messages, and calculating energy and momentum */
void Spacecraft::linkInStates(DynParamManager& statesIn)
{
    // - Get access to all hub states
    this->hubR_N = statesIn.getStateObject(this->hub.nameOfHubPosition);
    this->hubV_N = statesIn.getStateObject(this->hub.nameOfHubVelocity);
    this->hubSigma = statesIn.getStateObject(this->hub.nameOfHubSigma);   /* Need sigmaBN for MRP switching */
    this->hubOmega_BN_B = statesIn.getStateObject(this->hub.nameOfHubOmega);
    this->hubGravVelocity = statesIn.getStateObject(this->hub.nameOfHubGravVelocity);
    this->BcGravVelocity = statesIn.getStateObject(this->hub.nameOfBcGravVelocity);

    // - Get access to the hubs position and velocity in the property manager
    this->inertialPositionProperty = statesIn.getPropertyReference(this->gravField.inertialPositionPropName);
    this->inertialVelocityProperty = statesIn.getPropertyReference(this->gravField.inertialVelocityPropName);
    this->g_N = statesIn.getPropertyReference(this->gravField.vehicleGravityPropName);
}

/*! This method is used to initialize the simulation by registering all of the states, linking the dynamicEffectors,
 stateEffectors, and the hub, initialize gravity, and initialize the sim with the initial conditions specified in python
 for the simulation */
void Spacecraft::initializeDynamics()
{
    // - Spacecraft initiates all of the spaceCraft mass properties
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
    this->m_SC = this->dynManager.createProperty(this->propName_m_SC, initM_SC);
    this->mDot_SC = this->dynManager.createProperty(this->propName_mDot_SC, initMDot_SC);
    this->c_B = this->dynManager.createProperty(this->propName_centerOfMassSC, initC_B);
    this->ISCPntB_B = this->dynManager.createProperty(this->propName_inertiaSC, initISCPntB_B);
    this->ISCPntBPrime_B = this->dynManager.createProperty(this->propName_inertiaPrimeSC, initISCPntBPrime_B);
    this->cPrime_B = this->dynManager.createProperty(this->propName_centerOfMassPrimeSC, initCPrime_B);
    this->cDot_B = this->dynManager.createProperty(this->propName_centerOfMassDotSC, initCDot_B);
    this->sysTime = this->dynManager.createProperty(this->sysTimePropertyName, systemTime);

    // - Register the gravity properties with the dynManager, 'erbody wants g_N!
    this->gravField.registerProperties(this->dynManager);

    // - Register the hub states
    this->hub.registerStates(this->dynManager);

    // - Loop through stateEffectors to register their states
    std::vector<StateEffector*>::iterator stateIt;
    for(stateIt = this->states.begin(); stateIt != this->states.end(); stateIt++)
    {
        (*stateIt)->registerStates(this->dynManager);
    }

    // - Link in states for the Spacecraft, gravity and the hub
    this->linkInStates(this->dynManager);
    this->gravField.linkInStates(this->dynManager);
    this->hub.linkInStates(this->dynManager);

    // - Update the mass properties of the spacecraft to retrieve c_B and cDot_B to update r_BN_N and v_BN_N
    this->updateSCMassProps(0.0);

    // - Edit r_BN_N and v_BN_N to take into account that point B and point C are not coincident
    // - Pulling the state from the hub at this time gives us r_CN_N
    Eigen::Vector3d rInit_BN_N = this->hubR_N->getState();
    Eigen::MRPd sigma_BN;
    sigma_BN = (Eigen::Vector3d) this->hubSigma->getState();
    Eigen::Matrix3d dcm_NB = sigma_BN.toRotationMatrix();
    // - Substract off the center mass to leave r_BN_N
    rInit_BN_N -= dcm_NB*(*this->c_B);
    // - Subtract off cDot_B to get v_BN_N
    Eigen::Vector3d vInit_BN_N = this->hubV_N->getState();
    vInit_BN_N -= dcm_NB*(*this->cDot_B);
    // - Finally set the translational states r_BN_N and v_BN_N with the corrections
    this->hubR_N->setState(rInit_BN_N);
    this->hubV_N->setState(vInit_BN_N);

    // - Loop through the stateEffectros to link in the states needed
    for(stateIt = this->states.begin(); stateIt != this->states.end(); stateIt++)
    {
        (*stateIt)->linkInStates(this->dynManager);
    }

    // - Loop through the dynamicEffectors to link in the states needed
    std::vector<DynamicEffector*>::iterator dynIt;
    for(dynIt = this->dynEffectors.begin(); dynIt != this->dynEffectors.end(); dynIt++)
    {
        (*dynIt)->linkInStates(this->dynManager);
    }

    // If set, read in and prescribe attitude reference motion as initial states
    readOptionalRefMsg();

    // - Call equations of motion at time zero
    this->equationsOfMotion(0.0, 1.0);
}

/*! This method is used to update the mass properties of the entire spacecraft using contributions from stateEffectors */
void Spacecraft::updateSCMassProps(double time)
{
    // - Zero the properties which will get populated in this method
    (*this->m_SC).setZero();
    (*this->mDot_SC).setZero();
    (*this->c_B).setZero();
    (*this->ISCPntB_B).setZero();
    (*this->cPrime_B).setZero();
    (*this->cDot_B).setZero();
    (*this->ISCPntBPrime_B).setZero();

    // Add in hubs mass props to the spacecraft mass props
    this->hub.updateEffectorMassProps(time);
    (*this->m_SC)(0,0) += this->hub.effProps.mEff;
    (*this->ISCPntB_B) += this->hub.effProps.IEffPntB_B;
    (*this->c_B) += this->hub.effProps.mEff*this->hub.effProps.rEff_CB_B;

    // - Loop through state effectors to get mass props
    std::vector<StateEffector*>::iterator it;
    for(it = this->states.begin(); it != this->states.end(); it++)
    {
        (*it)->updateEffectorMassProps(time);
        // - Add in effectors mass props into mass props of spacecraft
        (*this->m_SC)(0,0) += (*it)->effProps.mEff;
        (*this->mDot_SC)(0,0) += (*it)->effProps.mEffDot;
        (*this->ISCPntB_B) += (*it)->effProps.IEffPntB_B;
        (*this->c_B) += (*it)->effProps.mEff*(*it)->effProps.rEff_CB_B;
        (*this->ISCPntBPrime_B) += (*it)->effProps.IEffPrimePntB_B;
        (*this->cPrime_B) += (*it)->effProps.mEff*(*it)->effProps.rEffPrime_CB_B;
        // For high fidelity mass depletion, this is left out: += (*it)->effProps.mEffDot*(*it)->effProps.rEff_CB_B
    }

    // Divide c_B and cPrime_B by the total mass of the spaceCraft to finalize c_B and cPrime_B
    (*this->c_B) = (*this->c_B)/(*this->m_SC)(0,0);
    (*this->cPrime_B) = (*this->cPrime_B)/(*this->m_SC)(0,0)
                                             - (*this->mDot_SC)(0,0)*(*this->c_B)/(*this->m_SC)(0,0)/(*this->m_SC)(0,0);
    Eigen::Vector3d omegaLocal_BN_B = hubOmega_BN_B->getState();
    Eigen::Vector3d cLocal_B = (*this->c_B);
    (*this->cDot_B) = (*this->cPrime_B) + omegaLocal_BN_B.cross(cLocal_B);
}

/*! This method is solving Xdot = F(X,t) for the system. The hub needs to calculate its derivatives, along with all of
 the stateEffectors. The hub also has gravity and dynamicEffectors acting on it and these relationships are controlled
 in this method. At the end of this method all of the states will have their corresponding state derivatives set in the
 dynParam Manager thus solving for Xdot*/
void Spacecraft::equationsOfMotion(double integTimeSeconds, double timeStep)
{
    // - Update time to the current time
    uint64_t integTimeNanos = secToNano(integTimeSeconds);

    (*this->sysTime) << (double) integTimeNanos, integTimeSeconds;

    // - Zero all Matrices and vectors for back-sub and the dynamics
    this->hub.hubBackSubMatrices.matrixA.setZero();
    this->hub.hubBackSubMatrices.matrixB.setZero();
    this->hub.hubBackSubMatrices.matrixC.setZero();
    this->hub.hubBackSubMatrices.matrixD.setZero();
    this->hub.hubBackSubMatrices.vecTrans.setZero();
    this->hub.hubBackSubMatrices.vecRot.setZero();
    this->sumForceExternal_B.setZero();
    this->sumForceExternal_N.setZero();
    this->sumTorquePntB_B.setZero();

    // - Update the mass properties of the spacecraft
    this->updateSCMassProps(integTimeSeconds);

    // - This is where gravity is computed (gravity needs to know c_B to calculated gravity about r_CN_N)
    Eigen::MRPd sigmaBNLoc;
    Eigen::Matrix3d dcm_NB;
    Eigen::Vector3d cLocal_N;

    sigmaBNLoc = (Eigen::Vector3d) this->hubSigma->getState();
    dcm_NB = sigmaBNLoc.toRotationMatrix();
    cLocal_N = dcm_NB*(*this->c_B);
    Eigen::Vector3d rLocal_CN_N = this->hubR_N->getState() + dcm_NB*(*this->c_B);
    Eigen::Vector3d vLocal_CN_N = this->hubV_N->getState() + dcm_NB*(*this->cDot_B);

    this->gravField.computeGravityField(rLocal_CN_N, vLocal_CN_N);

    // - Loop through dynEffectors to compute force and torque on the s/c
    std::vector<DynamicEffector*>::iterator dynIt;
    for(dynIt = this->dynEffectors.begin(); dynIt != this->dynEffectors.end(); dynIt++)
    {
        // - Compute the force and torque contributions from the dynamicEffectors
        (*dynIt)->computeForceTorque(integTimeSeconds, timeStep);
        this->sumForceExternal_N += (*dynIt)->forceExternal_N;
        this->sumForceExternal_B += (*dynIt)->forceExternal_B;
        this->sumTorquePntB_B += (*dynIt)->torqueExternalPntB_B;
    }

    // - Loop through state effectors to get contributions for back-substitution
    std::vector<StateEffector*>::iterator it;
    for(it = this->states.begin(); it != this->states.end(); it++)
    {
        /* - Set the contribution matrices to zero (just in case a stateEffector += on the matrix or the stateEffector
         doesn't have a contribution for a matrix and doesn't set the matrix to zero */
        this->backSubContributions.matrixA.setZero();
        this->backSubContributions.matrixB.setZero();
        this->backSubContributions.matrixC.setZero();
        this->backSubContributions.matrixD.setZero();
        this->backSubContributions.vecTrans.setZero();
        this->backSubContributions.vecRot.setZero();

        // - Call the update contributions method for the stateEffectors and add in contributions to the hub matrices
        (*it)->updateContributions(integTimeSeconds, this->backSubContributions, this->hubSigma->getState(), this->hubOmega_BN_B->getState(), *this->g_N);
        this->hub.hubBackSubMatrices.matrixA += this->backSubContributions.matrixA;
        this->hub.hubBackSubMatrices.matrixB += this->backSubContributions.matrixB;
        this->hub.hubBackSubMatrices.matrixC += this->backSubContributions.matrixC;
        this->hub.hubBackSubMatrices.matrixD += this->backSubContributions.matrixD;
        this->hub.hubBackSubMatrices.vecTrans += this->backSubContributions.vecTrans;
        this->hub.hubBackSubMatrices.vecRot += this->backSubContributions.vecRot;
    }

    // - Finish the math that is needed
    Eigen::Vector3d cLocal_B;
    Eigen::Vector3d cPrimeLocal_B;
    cLocal_B = *this->c_B;
    cPrimeLocal_B = *cPrime_B;

    Eigen::Matrix3d intermediateMatrix;
    Eigen::Vector3d intermediateVector;
    Eigen::Vector3d omegaLocalBN_B = this->hubOmega_BN_B->getState();
    this->hub.hubBackSubMatrices.matrixA += (*this->m_SC)(0,0)*intermediateMatrix.Identity();
    intermediateMatrix = eigenTilde((*this->c_B));  // make c_B skew symmetric matrix
    this->hub.hubBackSubMatrices.matrixB += -(*this->m_SC)(0,0)*intermediateMatrix;
    this->hub.hubBackSubMatrices.matrixC += (*this->m_SC)(0,0)*intermediateMatrix;
    this->hub.hubBackSubMatrices.matrixD += *ISCPntB_B;
    this->hub.hubBackSubMatrices.vecTrans += -2.0*(*this->m_SC)(0, 0)*omegaLocalBN_B.cross(cPrimeLocal_B)
    - (*this->m_SC)(0, 0)*omegaLocalBN_B.cross(omegaLocalBN_B.cross(cLocal_B))
    - 2.0*(*mDot_SC)(0,0)*(cPrimeLocal_B+omegaLocalBN_B.cross(cLocal_B));
    intermediateVector = *ISCPntB_B*omegaLocalBN_B;
    this->hub.hubBackSubMatrices.vecRot += -omegaLocalBN_B.cross(intermediateVector) - *ISCPntBPrime_B*omegaLocalBN_B;

    // - Map external force_N to the body frame
    Eigen::Vector3d sumForceExternalMappedToB;
    sumForceExternalMappedToB = dcm_NB.transpose()*this->sumForceExternal_N;

    // - Edit both v_trans and v_rot with gravity and external force and torque
    Eigen::Vector3d gLocal_N = *this->g_N;

    // -  Make additional contributions to the matrices from the hub

    // - Need to find force of gravity on the spacecraft
    Eigen::Vector3d gravityForce_N;
    gravityForce_N = (*this->m_SC)(0,0)*gLocal_N;

    Eigen::Vector3d gravityForce_B;
    gravityForce_B = dcm_NB.transpose()*gravityForce_N;
    this->hub.hubBackSubMatrices.vecTrans += gravityForce_B + sumForceExternalMappedToB + this->sumForceExternal_B;
    this->hub.hubBackSubMatrices.vecRot += cLocal_B.cross(gravityForce_B) + this->sumTorquePntB_B;

    // - Compute the derivatives of the hub states before looping through stateEffectors
    this->hub.computeDerivatives(integTimeSeconds, this->hubV_N->getStateDeriv(), this->hubOmega_BN_B->getStateDeriv(), this->hubSigma->getState());

    // - Loop through state effectors for compute derivatives
    for(it = states.begin(); it != states.end(); it++)
    {
        (*it)->computeDerivatives(integTimeSeconds, this->hubV_N->getStateDeriv(), this->hubOmega_BN_B->getStateDeriv(), this->hubSigma->getState());
    }
}

/*! Prepare for integration process
 @param integrateToThisTimeNanos Time to integrate to
 */
void Spacecraft::preIntegration(uint64_t integrateToThisTimeNanos) {
    this->timeStep = diffNanoToSec(integrateToThisTimeNanos, this->timeBeforeNanos); // - Find the time step in seconds

    // - Find v_CN_N before integration for accumulated DV
    Eigen::Vector3d oldV_BN_N = this->hubV_N->getState();  // - V_BN_N before integration
    Eigen::Vector3d oldV_CN_N;  // - V_CN_N before integration
    Eigen::Vector3d oldC_B;     // - Center of mass offset before integration
    Eigen::MRPd oldSigma_BN;    // - Sigma_BN before integration
    // - Get the angular rate, oldOmega_BN_B from the dyn manager
    this->oldOmega_BN_B = this->hubOmega_BN_B->getState();
    // - Get center of mass, v_BN_N and dcm_NB from the dyn manager
    oldSigma_BN = (Eigen::Vector3d) this->hubSigma->getState();
    // - Finally find v_CN_N
    Eigen::Matrix3d oldDcm_NB = oldSigma_BN.toRotationMatrix(); // - dcm_NB before integration
    oldV_CN_N = oldV_BN_N + oldDcm_NB*(*this->cDot_B);

    // - Integrate the state from the last time (timeBefore) to the integrateToThisTime
    this->hub.matchGravitytoVelocityState(oldV_CN_N); // Set gravity velocity to base velocity for DV estimation
}

/*! Perform post-integration steps
 @param integrateToThisTimeNanos Time to integrate to
 */
void Spacecraft::postIntegration(uint64_t integrateToThisTimeNanos) {
    this->timeBeforeNanos = integrateToThisTimeNanos;     // - copy the current time into previous time for next integrate state call
    this->timeBefore = integrateToThisTimeNanos*NANO2SEC;
    double integrateToThisTime = integrateToThisTimeNanos*NANO2SEC; // - convert to seconds

    // - Call mass properties to get current info on the mass props of the spacecraft
    this->updateSCMassProps(integrateToThisTime);

    // - Find v_CN_N after the integration for accumulated DV
    Eigen::Vector3d newV_BN_N = this->hubV_N->getState(); // - V_BN_N after integration
    Eigen::Vector3d newV_CN_N;  // - V_CN_N after integration
    Eigen::MRPd newSigma_BN;    // - Sigma_BN after integration
    // - Get center of mass, v_BN_N and dcm_NB
    Eigen::Vector3d sigmaBNLoc;
    sigmaBNLoc = (Eigen::Vector3d) this->hubSigma->getState();
    newSigma_BN = sigmaBNLoc;
    Eigen::Matrix3d newDcm_NB = newSigma_BN.toRotationMatrix();  // - dcm_NB after integration
    newV_CN_N = newV_BN_N + newDcm_NB*(*this->cDot_B);

    // - Find accumulated DV of the center of mass in the body frame
    this->dvAccum_CN_B += newDcm_NB.transpose()*(newV_CN_N -
                                              this->BcGravVelocity->getState());

    // - Find the accumulated DV of the body frame in the body frame
    this->dvAccum_BN_B += newDcm_NB.transpose()*(newV_BN_N -
                                                 this->hubGravVelocity->getState());

    // - Find the accumulated DV of the center of mass in the inertial frame
    this->dvAccum_CN_N += newV_CN_N - this->BcGravVelocity->getState();

    // - non-conservative acceleration of the body frame in the body frame
    this->nonConservativeAccelpntB_B = (newDcm_NB.transpose()*(newV_BN_N -
                                                               this->hubGravVelocity->getState()))/this->timeStep;

    // - angular acceleration in the body frame
    Eigen::Vector3d newOmega_BN_B;
    newOmega_BN_B = this->hubOmega_BN_B->getState();
    if (fabs(this->timeStep) > 1e-10) {
        this->omegaDot_BN_B = (newOmega_BN_B - this->oldOmega_BN_B)/this->timeStep; //angular acceleration of B wrt N in the Body frame
    } else {
        this->omegaDot_BN_B = {0., 0., .0};
    }

    // - Compute Energy and Momentum
    this->computeEnergyMomentum(integrateToThisTime);

    // - Call hubs modify states to allow for switching of MRPs
    this->hub.modifyStates(integrateToThisTime);

    // - Loop over stateEffectors to call modifyStates
    std::vector<StateEffector*>::iterator it;
    for(it = this->states.begin(); it != this->states.end(); it++)
    {
        // - Call energy and momentum calulations for stateEffectors
        (*it)->modifyStates(integrateToThisTime);
    }
    // - Compute force and torque on the body due to stateEffectors
    this->calcForceTorqueFromStateEffectors(integrateToThisTime, newOmega_BN_B);
}

/*! This method is used to find the total energy and momentum of the spacecraft. It finds the total orbital energy,
 total orbital angular momentum, total rotational energy and total rotational angular momentum. These values are used
 for validation purposes. */
void Spacecraft::computeEnergyMomentum(double time)
{
    // - Grab values from state Manager
    Eigen::Vector3d rLocal_BN_N = hubR_N->getState();
    Eigen::Vector3d rDotLocal_BN_N = hubV_N->getState();
    Eigen::MRPd sigmaLocal_BN;
    sigmaLocal_BN = (Eigen::Vector3d ) hubSigma->getState();

    // - Find DCM's
    Eigen::Matrix3d dcmLocal_NB = sigmaLocal_BN.toRotationMatrix();
    Eigen::Matrix3d dcmLocal_BN = dcmLocal_NB.transpose();

    // - Convert from inertial frame to body frame
    Eigen::Vector3d rBNLocal_B;
    Eigen::Vector3d rDotBNLocal_B;
    rBNLocal_B = dcmLocal_BN*rLocal_BN_N;
    rDotBNLocal_B = dcmLocal_BN*rDotLocal_BN_N;

    // - zero necessarry variables
    Eigen::Vector3d totOrbAngMomPntN_B;
    Eigen::Vector3d totRotAngMomPntC_B;
    totOrbAngMomPntN_B.setZero();
    totRotAngMomPntC_B.setZero();
    this->totOrbAngMomPntN_N.setZero();
    this->totRotAngMomPntC_N.setZero();
    this->rotAngMomPntCContr_B.setZero();
    this->totOrbEnergy = 0.0;
    this->totRotEnergy = 0.0;
    this->rotEnergyContr = 0.0;

    // - Get the hubs contribution
    this->hub.updateEnergyMomContributions(time, this->rotAngMomPntCContr_B, this->rotEnergyContr, this->hubOmega_BN_B->getState());
    totRotAngMomPntC_B += this->rotAngMomPntCContr_B;
    this->totRotEnergy += this->rotEnergyContr;

    // - Loop over stateEffectors to get their contributions to energy and momentum
    std::vector<StateEffector*>::iterator it;
    for(it = this->states.begin(); it != this->states.end(); it++)
    {
        // - Set the matrices to zero
        this->rotAngMomPntCContr_B.setZero();
        this->rotEnergyContr = 0.0;

        // - Call energy and momentum calulations for stateEffectors
        (*it)->updateEnergyMomContributions(time, this->rotAngMomPntCContr_B, this->rotEnergyContr, this->hubOmega_BN_B->getState());
        totRotAngMomPntC_B += this->rotAngMomPntCContr_B;
        this->totRotEnergy += this->rotEnergyContr;
    }

    // - Get cDot_B from manager
    Eigen::Vector3d cDotLocal_B = (*this->cDot_B);

    // - Add in orbital kinetic energy into the total orbital energy calculations
    this->totOrbEnergy += 1.0/2.0*(*this->m_SC)(0,0)*(rDotBNLocal_B.dot(rDotBNLocal_B) + 2.0*rDotBNLocal_B.dot(cDotLocal_B)
                                               + cDotLocal_B.dot(cDotLocal_B));

    // - Call gravity effector and add in its potential contributions to the total orbital energy calculations
    this->orbPotentialEnergyContr = 0.0;
    Eigen::Vector3d rLocal_CN_N = this->hubR_N->getState() + dcmLocal_NB*(*this->c_B);
    gravField.updateEnergyContributions(rLocal_CN_N, this->orbPotentialEnergyContr);
    this->totOrbEnergy += (*this->m_SC)(0,0)*this->orbPotentialEnergyContr;

    // - Find total rotational energy
    this->totRotEnergy += -1.0/2.0*(*this->m_SC)(0,0)*cDotLocal_B.dot(cDotLocal_B);

    // - Find orbital angular momentum for the spacecraft
    Eigen::Vector3d rCN_N;
    Eigen::Vector3d rDotCN_N;
    rCN_N = rLocal_BN_N + dcmLocal_NB*(*this->c_B);
    rDotCN_N = rDotLocal_BN_N + dcmLocal_NB*cDotLocal_B;
    this->totOrbAngMomPntN_N = (*this->m_SC)(0,0)*(rCN_N.cross(rDotCN_N));

    // - Find rotational angular momentum for the spacecraft
    totRotAngMomPntC_B += -(*this->m_SC)(0,0)*(Eigen::Vector3d (*this->c_B)).cross(cDotLocal_B);
    this->totRotAngMomPntC_N = dcmLocal_NB*totRotAngMomPntC_B;
}

/*! This method is used to find the force and torque that each stateEffector is applying to the spacecraft. These values
 are held in the stateEffector class. Additionally, the stateDerivative value is behind the state values because they
 are calculated in the intergrator calls */
void Spacecraft::calcForceTorqueFromStateEffectors(double time, Eigen::Vector3d omega_BN_B)
{
    // - Loop over stateEffectors to get their contributions to energy and momentum
    std::vector<StateEffector*>::iterator it;
    for(it = this->states.begin(); it != this->states.end(); it++)
    {
        (*it)->calcForceTorqueOnBody(time, omega_BN_B);
    }
}
