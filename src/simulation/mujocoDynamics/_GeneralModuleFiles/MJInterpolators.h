/*
 ISC License

 Copyright (c) 2025, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#ifndef MJ_INTERPOLATORS_H
#define MJ_INTERPOLATORS_H

#include "simulation/dynamics/_GeneralModuleFiles/interpolator.h"

#include "architecture/msgPayloadDefC/ScalarJointStateMsgPayload.h"
#include "architecture/msgPayloadDefC/SingleActuatorMsgPayload.h"

/**
 * @brief Interpolates a series of user-given data points to publish
 *  a message of type `SingleActuatorMsg`
 */
class SingleActuatorInterpolator : public Interpolator<SingleActuatorMsgPayload, 1>
{
protected:
    /**
     * @brief Sets the payload values for the SingleActuatorMsgPayload.
     *
     * This method is called by the UpdateState method of the Interpolator class
     * to set the interpolated values into the payload.
     *
     * @param payload The payload object to set values in.
     * @param CurrentSimNanos The current simulation time in nanoseconds.
     * @param interp The interpolated values.
     */
    virtual void setPayloadValues(SingleActuatorMsgPayload& payload,
                                  uint64_t CurrentSimNanos,
                                  const Eigen::Array<double, 1, 1>& interp) override
    {
        payload.input = interp(0);
    }
};


/**
 * @brief Interpolates a series of user-given data points to publish
 *  a message of type `ScalarJointStateMsg`
 */
class ScalarJointStateInterpolator : public Interpolator<ScalarJointStateMsgPayload, 1>
{
protected:
    /**
     * @brief Sets the payload values for the ScalarJointStateMsgPayload.
     *
     * This method is called by the UpdateState method of the Interpolator class
     * to set the interpolated values into the payload.
     *
     * @param payload The payload object to set values in.
     * @param CurrentSimNanos The current simulation time in nanoseconds.
     * @param interp The interpolated values.
     */
    virtual void setPayloadValues(ScalarJointStateMsgPayload& payload,
                                  uint64_t CurrentSimNanos,
                                  const Eigen::Array<double, 1, 1>& interp) override
    {
        payload.state = interp(0);
    }
};

#endif
