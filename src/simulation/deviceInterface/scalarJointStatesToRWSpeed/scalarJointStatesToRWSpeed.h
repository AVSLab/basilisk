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

#ifndef SCALAR_JOINT_STATES_TO_RW_SPEED_H
#define SCALAR_JOINT_STATES_TO_RW_SPEED_H

#include <cstdint>
#include <vector>

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/messaging/messaging.h"
#include "architecture/utilities/bskLogging.h"

#include "architecture/msgPayloadDefC/ScalarJointStateMsgPayload.h"
#include "architecture/msgPayloadDefC/RWSpeedMsgPayload.h"

/*! @brief Adapter module that maps multiple ScalarJointStateMsg inputs
    directly into a single RWSpeedMsg output.
*/
class ScalarJointStatesToRWSpeed : public SysModel
{
public:
    /*! @brief Default constructor. */
    ScalarJointStatesToRWSpeed();

    /*! @brief Constructor specifying number of joints.
        @param numJoints Number of joint inputs.
    */
    explicit ScalarJointStatesToRWSpeed(uint32_t numJoints);

    /*! @brief Destructor. */
    ~ScalarJointStatesToRWSpeed() override;

    /*! @brief Reset the module.
        @param CurrentSimNanos Current simulation time [ns].
    */
    void Reset(uint64_t CurrentSimNanos) override;

    /*! @brief Read joint states and publish RW speed message.
        @param CurrentSimNanos Current simulation time [ns].
    */
    void UpdateState(uint64_t CurrentSimNanos) override;

    /*! @brief Set the number of joints.
        @param numJoints Number of joints.
    */
    void setNumJoints(uint32_t numJoints);

public:
    std::vector<ReadFunctor<ScalarJointStateMsgPayload>> jointStateInMsgs; //!< Joint state readers
    Message<RWSpeedMsgPayload> rwSpeedOutMsg;                               //!< RW speed output
    BSKLogger bskLogger;                                                    //!< BSK Logging

private:
    /*! @brief Ensure internal buffers are correctly sized. */
    void ensureSizes();

private:
    uint32_t numJoints;            //!< Number of joints
};

#endif
