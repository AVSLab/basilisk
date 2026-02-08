/*
 ISC License

 Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

 Permission to use, copy, modify, and/or distribute this software for any
 purpose with or without fee is hereby granted, provided that the above
 copyright notice and this permission notice appear in all copies.

 THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR DISCLAIMS ALL WARRANTIES
 WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

 */

#ifndef ARRAY_MOTOR_TORQUE_TO_SINGLE_ACTUATORS_H
#define ARRAY_MOTOR_TORQUE_TO_SINGLE_ACTUATORS_H

#include <cstdint>
#include <vector>

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/messaging/messaging.h"
#include "architecture/utilities/bskLogging.h"

#include "architecture/msgPayloadDefC/ArrayMotorTorqueMsgPayload.h"
#include "architecture/msgPayloadDefC/SingleActuatorMsgPayload.h"

/*! @brief Adapter module that converts an ArrayMotorTorqueMsg into one
    SingleActuatorMsg per actuator using direct index mapping.
*/
class ArrayMotorTorqueToSingleActuators : public SysModel
{
public:
    /*! @brief Default constructor. */
    ArrayMotorTorqueToSingleActuators();

    /*! @brief Constructor specifying number of actuators.
        @param numActuators Number of output actuators.
    */
    explicit ArrayMotorTorqueToSingleActuators(uint32_t numActuators);

    /*! @brief Destructor. */
    ~ArrayMotorTorqueToSingleActuators() override = default;

    /*! @brief Reset the module.
        @param CurrentSimNanos Current simulation time [ns].
    */
    void Reset(uint64_t CurrentSimNanos) override;

    /*! @brief Read array torque message and publish scalar actuator commands.
        @param CurrentSimNanos Current simulation time [ns].
    */
    void UpdateState(uint64_t CurrentSimNanos) override;

    /*! @brief Set the number of actuators.
        @param numActuators Number of actuators.
    */
    void setNumActuators(uint32_t numActuators);

public:
    ReadFunctor<ArrayMotorTorqueMsgPayload> torqueInMsg;            //!< Input motor torque array
    std::vector<Message<SingleActuatorMsgPayload>> actuatorOutMsgs; //!< Scalar actuator outputs
    BSKLogger bskLogger;                                            //!< BSK Logging

private:
    /*! @brief Ensure internal buffers are correctly sized. */
    void ensureSizes();

private:
    uint32_t numActuators;                               //!< Number of actuators
};

#endif
