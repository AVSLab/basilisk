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

#ifndef RW_VOLTAGE_INTERFACE_H
#define RW_VOLTAGE_INTERFACE_H

#include <vector>
#include "_GeneralModuleFiles/sys_model.h"
#include "messaging2/messaging2.h"

#include "msgPayloadDefC/RWArrayVoltageMsgPayload.h"
#include "msgPayloadDefC/ArrayMotorTorqueMsgPayload.h"

#include "../../utilities/macroDefinitions.h"
#include "utilities/bskLogging.h"
#include <Eigen/Dense>

/*! @brief RW voltage interface class */
class RWVoltageInterface: public SysModel {
public:
    RWVoltageInterface();
    ~RWVoltageInterface();
   
    void computeRWMotorTorque();
    void UpdateState(uint64_t CurrentSimNanos);
    void readInputMessages();
    void writeOutputMessages(uint64_t Clock);
    void setGains(Eigen::VectorXd gains); //!< --     Takes in an array of gains to set for rws and sets them, leaving blanks up to MAX_EFF_COUNT
    void setScaleFactors(Eigen::VectorXd scaleFactors); //!< --     Takes in an array of scale factors to set for rws and sets them, leaving blanks up to MAX_EFF_COUNT
    void setBiases(Eigen::VectorXd biases); //!< --     Takes in an array of biases to set for rws and sets them, leaving blanks up to MAX_EFF_COUNT
    
public:
    ReadFunctor<RWArrayVoltageMsgPayload> rwVoltageInMsg;     //!< --     Message that contains RW voltage input states
    Message<ArrayMotorTorqueMsgPayload> rwMotorTorqueOutMsg;//!< --     Output Message for RW motor torques
    Eigen::VectorXd voltage2TorqueGain;          //!< Nm/V   gain to convert voltage to motor torque
    Eigen::VectorXd scaleFactor;                 //!<        scale the output - like a constant gain error
    Eigen::VectorXd bias;                        //!< Nm     A bias to add to the torque output
    BSKLogger bskLogger;                      //!< -- BSK Logging

private:
    ArrayMotorTorqueMsgPayload outputRWTorqueBuffer;//!< [Nm] copy of module output buffer
    uint64_t prevTime;                  //!< -- Previous simulation time observed
    RWArrayVoltageMsgPayload inputVoltageBuffer;//!< [V] One-time allocation for time savings
};


#endif
