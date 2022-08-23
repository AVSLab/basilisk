/*
 ISC License

 Copyright (c) 2022, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#include "spinningBodyStateEffector.h"
#include "architecture/utilities/avsEigenSupport.h"
#include "architecture/utilities/rigidBodyKinematics.h"
#include "architecture/utilities/avsEigenSupport.h"
#include "architecture/utilities/macroDefinitions.h"
#include <iostream>
#include <string>

/*! This is the constructor, setting variables to default values */
SpinningBodyStateEffector::SpinningBodyStateEffector()
{
    // - zero the mass props and mass prop rates contributions
    this->effProps.mEff = 0.0;
    this->effProps.rEff_CB_B.fill(0.0);
    this->effProps.IEffPntB_B.fill(0.0);
    this->effProps.rEffPrime_CB_B.fill(0.0);
    this->effProps.IEffPrimePntB_B.fill(0.0);

    // - Initialize variables to working values
    this->mass = 0.0;
    this->u = 0.0;
    this->thetaInit = 0.00;
    this->OmegaInit = 0.0;
    this->IPntS_S.Identity();
    this->r_SB_B.setZero();
    this->dcm_SB.Identity();
    this->nameOfThetaState = "spinningBodyTheta" + std::to_string(this->effectorID);
    this->nameOfOmegaState = "spinningBodyOmega" + std::to_string(this->effectorID);
    this->effectorID++;

    return;
}

uint64_t SpinningBodyStateEffector::effectorID = 1;

/*! This is the destructor, nothing to report here */
SpinningBodyStateEffector::~SpinningBodyStateEffector()
{
    this->effectorID = 1;    /* reset the panel ID*/
    return;
}


/*! This method takes the computed theta states and outputs them to the m
 messaging system.
 @return void
 @param CurrentClock The current simulation time (used for time stamping)
 */
void SpinningBodyStateEffector::writeOutputStateMessages(uint64_t CurrentClock)
{

    if (this->spinningBodyOutMsg.isLinked()) {
        this->SBoutputStates = this->spinningBodyOutMsg.zeroMsgPayload;
        this->SBoutputStates.theta = this->theta;
        this->SBoutputStates.Omega = this->Omega;
        this->spinningBodyOutMsg.write(&this->SBoutputStates, this->moduleID, CurrentClock);
    }

    // write out the panel state config log message
    if (this->spinningBodyConfigLogOutMsg.isLinked()) {
        SCStatesMsgPayload configLogMsg;
        configLogMsg = this->spinningBodyConfigLogOutMsg.zeroMsgPayload;
        // Note, logging the hinge frame S is the body frame B of that object
        eigenVector3d2CArray(this->r_SN_N, configLogMsg.r_BN_N);
        eigenVector3d2CArray(this->v_SN_N, configLogMsg.v_BN_N);
        eigenVector3d2CArray(this->sigma_SN, configLogMsg.sigma_BN);
        eigenVector3d2CArray(this->omega_SN_S, configLogMsg.omega_BN_B);
        this->spinningBodyConfigLogOutMsg.write(&configLogMsg, this->moduleID, CurrentClock);
    }

}

void SpinningBodyStateEffector::prependSpacecraftNameToStates()
{
    this->nameOfThetaState = this->nameOfSpacecraftAttachedTo + this->nameOfThetaState;
    this->nameOfOmegaState = this->nameOfSpacecraftAttachedTo + this->nameOfOmegaState;

    return;
}

/*! This method allows the HRB state effector to have access to the hub states and gravity*/
void SpinningBodyStateEffector::linkInStates(DynParamManager& statesIn)
{
    // - Get access to the hubs sigma, omegaBN_B and velocity needed for dynamic coupling and gravity
    std::string tmpMsgName;
    tmpMsgName = this->nameOfSpacecraftAttachedTo + "centerOfMassSC";
    this->c_B = statesIn.getPropertyReference(tmpMsgName);
    tmpMsgName = this->nameOfSpacecraftAttachedTo + "centerOfMassPrimeSC";
    this->cPrime_B = statesIn.getPropertyReference(tmpMsgName);

    this->sigma_BN = statesIn.getStateObject(this->nameOfSpacecraftAttachedTo + "hubSigma");
    this->omega_BN_B = statesIn.getStateObject(this->nameOfSpacecraftAttachedTo + "hubOmega");
    this->r_BN_N = statesIn.getStateObject(this->nameOfSpacecraftAttachedTo + "hubPosition");
    this->v_BN_N = statesIn.getStateObject(this->nameOfSpacecraftAttachedTo + "hubVelocity");

    return;
}

/*! This method allows the HRB state effector to register its states: theta and Omega with the dyn param manager */
void SpinningBodyStateEffector::registerStates(DynParamManager& states)
{
    // - Register the states associated with hinged rigid bodies - theta and Omega
    this->thetaState = states.registerState(1, 1, this->nameOfThetaState);
    Eigen::MatrixXd thetaInitMatrix(1,1);
    thetaInitMatrix(0,0) = this->thetaInit;
    this->thetaState->setState(thetaInitMatrix);
    this->OmegaState = states.registerState(1, 1, this->nameOfOmegaState);
    Eigen::MatrixXd OmegaInitMatrix(1,1);
    OmegaInitMatrix(0,0) = this->OmegaInit;
    this->OmegaState->setState(OmegaInitMatrix);

    return;
}

/*! This method allows the HRB state effector to provide its contributions to the mass props and mass prop rates of the
 spacecraft */
void SpinningBodyStateEffector::updateEffectorMassProps(double integTime)
{
    

    return;
}

/*! This method allows the HRB state effector to give its contributions to the matrices needed for the back-sub 
 method */
void SpinningBodyStateEffector::updateContributions(double integTime, BackSubMatrices & backSubContr, Eigen::Vector3d sigma_BN, Eigen::Vector3d omega_BN_B, Eigen::Vector3d g_N)
{
    

    return;
}

/*! This method is used to find the derivatives for the HRB stateEffector: thetaDDot and the kinematic derivative */
void SpinningBodyStateEffector::computeDerivatives(double integTime, Eigen::Vector3d rDDot_BN_N, Eigen::Vector3d omegaDot_BN_B, Eigen::Vector3d sigma_BN)
{
    

    return;
}

/*! This method is for calculating the contributions of the HRB state effector to the energy and momentum of the s/c */
void SpinningBodyStateEffector::updateEnergyMomContributions(double integTime, Eigen::Vector3d & rotAngMomPntCContr_B,
                                                                double & rotEnergyContr, Eigen::Vector3d omega_BN_B)
{
    
    return;
}
/*! This method is used so that the simulation will ask HRB to update messages.
 @return void
 @param CurrentSimNanos The current simulation time in nanoseconds
 */
void SpinningBodyStateEffector::UpdateState(uint64_t CurrentSimNanos)
{

    this->writeOutputStateMessages(CurrentSimNanos);
    
    return;
}

void SpinningBodyStateEffector::calcForceTorqueOnBody(double integTime, Eigen::Vector3d omega_BN_B)
{

    
    return;
}


