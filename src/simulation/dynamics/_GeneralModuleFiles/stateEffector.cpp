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

#include "stateEffector.h"

/*! This is the constructor, just setting the variables to zero */
StateEffector::StateEffector()
{
    // - Set effector mass props to zero
    this->effProps.mEff = 0.0;
    this->effProps.mEffDot = 0.0;
    this->effProps.IEffPntB_B.fill(0.0);
    this->effProps.IEffPrimePntB_B.fill(0.0);
    this->effProps.rEff_CB_B.fill(0.0);
    this->effProps.rEffPrime_CB_B.fill(0.0);

    // - set force and torques equal to zero
    this->forceOnBody_B = this->torqueOnBodyPntB_B = this->torqueOnBodyPntC_B.setZero();

    this->nameOfSpacecraftAttachedTo = "";
    this->r_BP_P.setZero();
    this->dcm_BP.setIdentity();
    return;
}

/*! This is the destructor, nothing to report here */
StateEffector::~StateEffector()
{
    return;
}

/*! This method is for the state effector to provide its contributions of mass and mass rates to the dynamicObject. This
 allows for the dynamicObject to have access to the total mass, and inerita, mass and inertia rates*/
void StateEffector::updateEffectorMassProps(double integTime)
{
    return;
}

void StateEffector::receiveMotherSpacecraftData(Eigen::Vector3d rSC_BP_P, Eigen::Matrix3d dcmSC_BP)
{
    this->r_BP_P = rSC_BP_P;
    this->dcm_BP = dcmSC_BP;

    return;
}

/*! This method is strictly for the back-substituion method for computing the dynamics of the spacecraft. The back-sub
 method first computes rDDot_BN_N and omegaDot_BN_B for the spacecraft using these contributions from the state
 effectors. Then computeDerivatives is called to compute the stateEffectors derivatives using rDDot_BN_N omegaDot_BN_B*/
void StateEffector::updateContributions(double integTime, BackSubMatrices & backSubContr, Eigen::Vector3d sigma_BN, Eigen::Vector3d omega_BN_B, Eigen::Vector3d g_N)
{
    return;
}

/*! This method allows for an individual stateEffector to add its energy and momentum calculations to the dynamicObject.
 The analytical devlopement of these contributions can be seen in
 Basilisk/simulation/dynamics/_Documentation/Basilisk-EnergyAndMomentum-20161219.pdf*/
void StateEffector::updateEnergyMomContributions(double integTime, Eigen::Vector3d & rotAngMomPntCContr_B,
                                                 double & rotEnergyContr, Eigen::Vector3d omega_BN_B)
{
    return;
}

/*! This method allows for an individual stateEffector to modify their states after integration*/
void StateEffector::modifyStates(double integTime)
{
    return;
}

/*! This method allows for an individual stateEffector to find the force and torque that the stateEffector is placing on to the body */
void StateEffector::calcForceTorqueOnBody(double integTime, Eigen::Vector3d omega_BN_B)
{
    return;
}

/*! This method ensures that all dynamics states have their messages written after integation */
void StateEffector::writeOutputStateMessages(uint64_t integTimeNanos)
{
    return;
}

/*! This method ensures that stateEffectors can be implemented using the multi-spacecraft archticture */
void StateEffector::prependSpacecraftNameToStates()
{
    return;
}

void StateEffector::setStateNameOfPosition(std::string value)
{
    // check that value is acceptable
    if (!value.empty()) {
        this->stateNameOfPosition = value;
    } else {
        bskLogger.bskLog(BSK_ERROR, "StateEffector: stateNameOfPosition variable must be a non-empty string");
    }
}

void StateEffector::setStateNameOfVelocity(std::string value)
{
    // check that value is acceptable
    if (!value.empty()) {
        this->stateNameOfVelocity = value;
    } else {
        bskLogger.bskLog(BSK_ERROR, "StateEffector: stateNameOfVelocity variable must be a non-empty string");
    }
}

void StateEffector::setStateNameOfSigma(std::string value)
{
    // check that value is acceptable
    if (!value.empty()) {
        this->stateNameOfSigma = value;
    } else {
        bskLogger.bskLog(BSK_ERROR, "StateEffector: stateNameOfSigma variable must be a non-empty string");
    }
}

void StateEffector::setStateNameOfOmega(std::string value)
{
    // check that value is acceptable
    if (!value.empty()) {
        this->stateNameOfOmega = value;
    } else {
        bskLogger.bskLog(BSK_ERROR, "StateEffector: stateNameOfOmega variable must be a non-empty string");
    }
}

void StateEffector::setPropName_m_SC(std::string value)
{
    // check that value is acceptable
    if (!value.empty()) {
        this->propName_m_SC = value;
    } else {
        bskLogger.bskLog(BSK_ERROR, "StateEffector: propName_m_SC variable must be a non-empty string");
    }
}

void StateEffector::setPropName_mDot_SC(std::string value)
{
    // check that value is acceptable
    if (!value.empty()) {
        this->propName_mDot_SC = value;
    } else {
        bskLogger.bskLog(BSK_ERROR, "StateEffector: propName_mDot_SC variable must be a non-empty string");
    }
}

void StateEffector::setPropName_centerOfMassSC(std::string value)
{
    // check that value is acceptable
    if (!value.empty()) {
        this->propName_centerOfMassSC = value;
    } else {
        bskLogger.bskLog(BSK_ERROR, "StateEffector: propName_centerOfMassSC variable must be a non-empty string");
    }
}

void StateEffector::setPropName_inertiaSC(std::string value)
{
    // check that value is acceptable
    if (!value.empty()) {
        this->propName_inertiaSC = value;
    } else {
        bskLogger.bskLog(BSK_ERROR, "StateEffector: propName_inertiaSC variable must be a non-empty string");
    }
}

void StateEffector::setPropName_inertiaPrimeSC(std::string value)
{
    // check that value is acceptable
    if (!value.empty()) {
        this->propName_inertiaPrimeSC = value;
    } else {
        bskLogger.bskLog(BSK_ERROR, "StateEffector: propName_inertiaPrimeSC variable must be a non-empty string");
    }
}

void StateEffector::setPropName_centerOfMassPrimeSC(std::string value)
{
    // check that value is acceptable
    if (!value.empty()) {
        this->propName_centerOfMassPrimeSC = value;
    } else {
        bskLogger.bskLog(BSK_ERROR, "StateEffector: propName_centerOfMassPrimeSC variable must be a non-empty string");
    }
}

void StateEffector::setPropName_centerOfMassDotSC(std::string value)
{
    // check that value is acceptable
    if (!value.empty()) {
        this->propName_centerOfMassDotSC = value;
    } else {
        bskLogger.bskLog(BSK_ERROR, "StateEffector: propName_centerOfMassDotSC variable must be a non-empty string");
    }
}

void StateEffector::setPropName_inertialPosition(std::string value)
{
    // check that value is acceptable
    if (!value.empty()) {
        this->propName_inertialPosition = value;
    } else {
        bskLogger.bskLog(BSK_ERROR, "StateEffector: propName_inertialPosition variable must be a non-empty string");
    }
}

void StateEffector::setPropName_inertialVelocity(std::string value)
{
    // check that value is acceptable
    if (!value.empty()) {
        this->propName_inertialVelocity = value;
    } else {
        bskLogger.bskLog(BSK_ERROR, "StateEffector: propName_inertialVelocity variable must be a non-empty string");
    }
}

void StateEffector::setPropName_vehicleGravity(std::string value)
{
    // check that value is acceptable
    if (!value.empty()) {
        this->propName_vehicleGravity = value;
    } else {
        bskLogger.bskLog(BSK_ERROR, "StateEffector: propName_vehicleGravity variable must be a non-empty string");
    }
}
