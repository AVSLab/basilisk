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

#ifndef DYNAMIC_EFFECTOR_H
#define DYNAMIC_EFFECTOR_H

#include <Eigen/Dense>
#include "dynParamManager.h"
#include "architecture/utilities/bskLogging.h"

/*! @brief dynamic effector class */
class DynamicEffector {
public:
    DynamicEffector();                      //!< Constructor
    virtual ~DynamicEffector();             //!< Destructor
    virtual void computeStateContribution(double integTime);
    virtual void linkInStates(DynParamManager& states) = 0;  //!< Method to get access to other states/stateEffectors
    virtual void computeForceTorque(double integTime, double timeStep) = 0;  //!< -- Method to computeForce and torque on the body

public:
    Eigen::VectorXd stateDerivContribution; //!< DynamicEffectors contribution to a stateEffector
    Eigen::Vector3d forceExternal_N = Eigen::Vector3d::Zero();      //!< [N] External force applied by this effector in inertial components
    Eigen::Vector3d forceExternal_B = Eigen::Vector3d::Zero();      //!< [N] External force applied by this effector in body frame components
    Eigen::Vector3d torqueExternalPntB_B = Eigen::Vector3d::Zero(); //!< [Nm] External torque applied by this effector

    /** setter for `stateNameOfPosition` property */
    void setStateNameOfPosition(std::string value);
    /** getter for `stateNameOfPosition` property */
    const std::string getStateNameOfPosition() const {return this->stateNameOfPosition; }
    /** setter for `stateNameOfVelocity` property */
    void setStateNameOfVelocity(std::string value);
    /** getter for `stateNameOfVelocity` property */
    const std::string getStateNameOfVelocity() const { return this->stateNameOfVelocity; }
    /** setter for `stateNameOfSigma` property */
    void setStateNameOfSigma(std::string value);
    /** getter for `stateNameOfSigma` property */
    const std::string getStateNameOfSigma() const { return this->stateNameOfSigma; }
    /** setter for `stateNameOfOmega` property */
    void setStateNameOfOmega(std::string value);
    /** getter for `stateNameOfOmega` property */
    const std::string getStateNameOfOmega() const { return this->stateNameOfOmega; }
    /** setter for `propName_m_SC` property */
    void setPropName_m_SC(std::string value);
    /** getter for `propName_m_SC` property */
    const std::string getPropName_m_SC() const { return this->propName_m_SC; }
    /** setter for `propName_mDot_SC` property */
    void setPropName_mDot_SC(std::string value);
    /** getter for `propName_mDot_SC` property */
    const std::string getPropName_mDot_SC() const { return this->propName_mDot_SC; }
    /** setter for `propName_centerOfMassSC` property */
    void setPropName_centerOfMassSC(std::string value);
    /** getter for `propName_centerOfMassSC` property */
    const std::string getPropName_centerOfMassSC() const { return this->propName_centerOfMassSC; }
    /** setter for `propName_inertiaSC` property */
    void setPropName_inertiaSC(std::string value);
    /** getter for `propName_inertiaSC` property */
    const std::string getPropName_inertiaSC() const { return this->propName_inertiaSC; }
    /** setter for `propName_inertiaPrimeSC` property */
    void setPropName_inertiaPrimeSC(std::string value);
    /** getter for `propName_inertiaPrimeSC` property */
    const std::string getPropName_inertiaPrimeSC() const { return this->propName_inertiaPrimeSC; }
    /** setter for `propName_centerOfMassPrimeSC` property */
    void setPropName_centerOfMassPrimeSC(std::string value);
    /** getter for `propName_centerOfMassPrimeSC` property */
    const std::string getPropName_centerOfMassPrimeSC() const { return this->propName_centerOfMassPrimeSC; }
    /** setter for `propName_centerOfMassDotSC` property */
    void setPropName_centerOfMassDotSC(std::string value);
    /** getter for `propName_centerOfMassDotSC` property */
    const std::string getPropName_centerOfMassDotSC() const { return this->propName_centerOfMassDotSC; }
    /** setter for `propName_inertialPosition` property */
    void setPropName_inertialPosition(std::string value);
    /** getter for `propName_inertialPosition` property */
    const std::string getPropName_inertialPosition() const { return this->propName_inertialPosition; }
    /** setter for `propName_inertialVelocity` property */
    void setPropName_inertialVelocity(std::string value);
    /** getter for `propName_inertialVelocity` property */
    const std::string getPropName_inertialVelocity() const { return this->propName_inertialVelocity; }
    /** setter for `propName_vehicleGravity` property */
    void setPropName_vehicleGravity(std::string value);
    /** getter for `propName_vehicleGravity` property */
    const std::string getPropName_vehicleGravity() const { return this->propName_vehicleGravity; }

    BSKLogger bskLogger;                    //!< BSK Logging

protected:
    std::string stateNameOfPosition = "";                           //!< state engine name of the parent rigid body inertial position vector
    std::string stateNameOfVelocity = "";                           //!< state engine name of the parent rigid body inertial velocity vector
    std::string stateNameOfSigma = "";                              //!< state engine name of the parent rigid body inertial attitude
    std::string stateNameOfOmega = "";                              //!< state engine name of the parent rigid body inertial angular velocity vector

    std::string propName_m_SC = "";                                 //!< property name of m_SC
    std::string propName_mDot_SC = "";                              //!< property name of mDot_SC
    std::string propName_centerOfMassSC = "";                       //!< property name of centerOfMassSC
    std::string propName_inertiaSC = "";                            //!< property name of inertiaSC
    std::string propName_inertiaPrimeSC = "";                       //!< property name of inertiaPrimeSC
    std::string propName_centerOfMassPrimeSC = "";                  //!< property name of centerOfMassPrimeSC
    std::string propName_centerOfMassDotSC = "";                    //!< property name of centerOfMassDotSC
    std::string propName_inertialPosition = "";                     //!< property name of inertialPosition
    std::string propName_inertialVelocity = "";                     //!< property name of inertialVelocity
    std::string propName_vehicleGravity = "";                       //!< property name of vehicleGravity

};


#endif /* DYNAMIC_EFFECTOR_H */
