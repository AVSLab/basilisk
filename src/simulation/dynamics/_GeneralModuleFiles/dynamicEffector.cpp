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

#include "dynamicEffector.h"

/*! This is the constructor, just setting the variables to zero */
DynamicEffector::DynamicEffector()
{
    // Set forces and torques to zero
    this->forceExternal_N.setZero();
    this->forceExternal_B.setZero();
    this->torqueExternalPntB_B.setZero();

    return;
}

/*! This is the destructor, nothing to report here */
DynamicEffector::~DynamicEffector()
{
    return;
}

/*! This method is an optional method by a dynamic effector and allows the dynamics effector to add direct contributions
    to a state effector derivative. Example - a thruster's mDot will impact a fuel tanks total mDot */
void DynamicEffector::computeStateContribution(double integTime)
{
    return;
}

void DynamicEffector::setStateNameOfPosition(std::string value)
{
    // check that value is acceptable
    if (!value.empty()) {
        this->stateNameOfPosition = value;
    } else {
        bskLogger.bskLog(BSK_ERROR, "DynamicEffector: stateNameOfPosition variable must be a non-empty string");
    }
}

void DynamicEffector::setStateNameOfVelocity(std::string value)
{
    // check that value is acceptable
    if (!value.empty()) {
        this->stateNameOfVelocity = value;
    } else {
        bskLogger.bskLog(BSK_ERROR, "DynamicEffector: stateNameOfVelocity variable must be a non-empty string");
    }
}

void DynamicEffector::setStateNameOfSigma(std::string value)
{
    // check that value is acceptable
    if (!value.empty()) {
        this->stateNameOfSigma = value;
    } else {
        bskLogger.bskLog(BSK_ERROR, "DynamicEffector: stateNameOfSigma variable must be a non-empty string");
    }
}

void DynamicEffector::setStateNameOfOmega(std::string value)
{
    // check that value is acceptable
    if (!value.empty()) {
        this->stateNameOfOmega = value;
    } else {
        bskLogger.bskLog(BSK_ERROR, "DynamicEffector: stateNameOfOmega variable must be a non-empty string");
    }
}

void DynamicEffector::setPropName_m_SC(std::string value)
{
    // check that value is acceptable
    if (!value.empty()) {
        this->propName_m_SC = value;
    } else {
        bskLogger.bskLog(BSK_ERROR, "DynamicEffector: propName_m_SC variable must be a non-empty string");
    }
}

void DynamicEffector::setPropName_mDot_SC(std::string value)
{
    // check that value is acceptable
    if (!value.empty()) {
        this->propName_mDot_SC = value;
    } else {
        bskLogger.bskLog(BSK_ERROR, "DynamicEffector: propName_mDot_SC variable must be a non-empty string");
    }
}

void DynamicEffector::setPropName_centerOfMassSC(std::string value)
{
    // check that value is acceptable
    if (!value.empty()) {
        this->propName_centerOfMassSC = value;
    } else {
        bskLogger.bskLog(BSK_ERROR, "DynamicEffector: propName_centerOfMassSC variable must be a non-empty string");
    }
}

void DynamicEffector::setPropName_inertiaSC(std::string value)
{
    // check that value is acceptable
    if (!value.empty()) {
        this->propName_inertiaSC = value;
    } else {
        bskLogger.bskLog(BSK_ERROR, "DynamicEffector: propName_inertiaSC variable must be a non-empty string");
    }
}

void DynamicEffector::setPropName_inertiaPrimeSC(std::string value)
{
    // check that value is acceptable
    if (!value.empty()) {
        this->propName_inertiaPrimeSC = value;
    } else {
        bskLogger.bskLog(BSK_ERROR, "DynamicEffector: propName_inertiaPrimeSC variable must be a non-empty string");
    }
}

void DynamicEffector::setPropName_centerOfMassPrimeSC(std::string value)
{
    // check that value is acceptable
    if (!value.empty()) {
        this->propName_centerOfMassPrimeSC = value;
    } else {
        bskLogger.bskLog(BSK_ERROR, "DynamicEffector: propName_centerOfMassPrimeSC variable must be a non-empty string");
    }
}

void DynamicEffector::setPropName_centerOfMassDotSC(std::string value)
{
    // check that value is acceptable
    if (!value.empty()) {
        this->propName_centerOfMassDotSC = value;
    } else {
        bskLogger.bskLog(BSK_ERROR, "DynamicEffector: propName_centerOfMassDotSC variable must be a non-empty string");
    }
}

void DynamicEffector::setPropName_inertialPosition(std::string value)
{
    // check that value is acceptable
    if (!value.empty()) {
        this->propName_inertialPosition = value;
    } else {
        bskLogger.bskLog(BSK_ERROR, "DynamicEffector: propName_inertialPosition variable must be a non-empty string");
    }
}

void DynamicEffector::setPropName_inertialVelocity(std::string value)
{
    // check that value is acceptable
    if (!value.empty()) {
        this->propName_inertialVelocity = value;
    } else {
        bskLogger.bskLog(BSK_ERROR, "DynamicEffector: propName_inertialVelocity variable must be a non-empty string");
    }
}

void DynamicEffector::setPropName_vehicleGravity(std::string value)
{
    // check that value is acceptable
    if (!value.empty()) {
        this->propName_vehicleGravity = value;
    } else {
        bskLogger.bskLog(BSK_ERROR, "DynamicEffector: propName_vehicleGravity variable must be a non-empty string");
    }
}
