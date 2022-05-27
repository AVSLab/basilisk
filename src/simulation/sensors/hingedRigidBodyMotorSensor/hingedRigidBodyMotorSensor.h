/*
 ISC License

 Copyright (c) 2022, Autonomous Vehicle Systems Lab, University of Colorado Boulder

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


#ifndef HINGEDRIGIDBODYMOTORSENSOR_H
#define HINGEDRIGIDBODYMOTORSENSOR_H

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/msgPayloadDefC/HingedRigidBodyMsgPayload.h"
#include "architecture/utilities/bskLogging.h"
#include "architecture/messaging/messaging.h"
#include "architecture/utilities/discretize.h"
#include <stdint.h>
#include <random>
/*! @brief Outputs measured angle and angle rate for a hinged rigid body, adding optional noise, bias, and discretization.
 */
class HingedRigidBodyMotorSensor: public SysModel {
public:
    HingedRigidBodyMotorSensor();
    ~HingedRigidBodyMotorSensor();

    void Reset(uint64_t CurrentSimNanos);
    void UpdateState(uint64_t CurrentSimNanos);
    
    void setRNGSeed(unsigned int newSeed); //!< for setting the seed

public:

    double thetaNoiseStd;       //!< [rad] standard deviation for Gaussian noise to theta
    double thetaDotNoiseStd;    //!< [rad/s] standard deviation for Gaussian noise to theta dot
    double thetaBias;           //!< [rad] bias added to true theta
    double thetaDotBias;        //!< [rad/s] bias added to true theta dot
    double thetaLSB;            //!< [rad] discretization for theta
    double thetaDotLSB;         //!< [rad/s] discretization for theta dot
    
    ReadFunctor<HingedRigidBodyMsgPayload> hingedRigidBodyMotorSensorInMsg;  //!< input message for true rigid body state (theta, theta dot)

    Message<HingedRigidBodyMsgPayload> hingedRigidBodyMotorSensorOutMsg;  //!< output message for sensed rigid body state

    BSKLogger bskLogger;              //!< -- BSK Logging

private:
    std::minstd_rand rGen; //!< -- Random number generator for model
    std::normal_distribution<double> rNum;  //!< -- Random number distribution for model
};


#endif
