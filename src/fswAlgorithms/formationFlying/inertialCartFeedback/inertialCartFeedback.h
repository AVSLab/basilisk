/*
 ISC License

 Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado Boulder

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

#ifndef INERTIAL_CART_FEEDBACK_H
#define INERTIAL_CART_FEEDBACK_H

#include <cstdint>
#include <vector>

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/messaging/messaging.h"
#include "cMsgCInterface/CmdForceInertialMsg_C.h"
#include "architecture/msgPayloadDefC/NavTransMsgPayload.h"
#include "architecture/msgPayloadDefC/VehicleConfigMsgPayload.h"
#include "architecture/utilities/avsEigenSupport.h"
#include "architecture/utilities/bskLogging.h"

/*! @brief Inertial Cartesian translational feedback controller for deputy desired-state tracking. */
class InertialCartFeedback : public SysModel {
public:
    InertialCartFeedback() = default; //!< This is the constructor for the module class.
    ~InertialCartFeedback() = default; //!< This is the destructor for the module class.
    void SelfInit(); //!< Self initialization method for the c-wrapped message.
    /*!
    * This method is used to reset the module and checks that required input messages are connected.
    */
    void Reset(uint64_t CurrentSimNanos);
    /*!
    * This is the main method that gets called every time the module is updated.
    * It computes the inertial force for the relative motion control.
    * */
    void UpdateState(uint64_t CurrentSimNanos);

public:
    ReadFunctor<NavTransMsgPayload> deputyTransInMsg;                  //!< deputy translational navigation input msg
    ReadFunctor<NavTransMsgPayload> deputyTransDesiredInMsg;           //!< desired deputy translational navigation input msg
    ReadFunctor<VehicleConfigMsgPayload> deputyVehicleConfigInMsg;     //!< deputy spacecraft mass/config input msg
    ReadFunctor<CmdForceInertialMsgPayload> forceFeedforwardInMsg;     //!< (optional) inertial feed-forward thrust for non-natural deputy motion input msg

    Message<CmdForceInertialMsgPayload> forceOutMsg;                   //!< inertial force command output msg
    CmdForceInertialMsg_C forceOutMsgC = {};                           //!< C-wrapped inertial force output msg

    BSKLogger bskLogger;                                               //!< BSK logging interface

    /** setter for `mu` */
    void setMu(const double value);
    /** getter for `mu` */
    double getMu() const {return this->mu;}
    /** setter for `K` */
    void setK(const std::vector<double>& value);
    /** getter for `K` */
    Eigen::Matrix3d getK() const {return this->K;}
    /** setter for `P` */
    void setP(const std::vector<double>& value);
    /** getter for `P` */
    Eigen::Matrix3d getP() const {return this->P;}

private:
    double deputyMass;                                                 //!< [kg] deputy system mass
    double mu = -1;                                                    //!< [m^3/s^2] (optional) gravitational parameter
    bool setKFlag = false;                                             //!< flag to check if K has been set
    bool setPFlag = false;                                             //!< flag to check if P has been set
    Eigen::Matrix3d K;                                                 //!< [1/s^2] proportional acceleration gain (mass-scaled in force law)
    Eigen::Matrix3d P;                                                 //!< [1/s] derivative acceleration gain (mass-scaled in force law)
};

#endif
