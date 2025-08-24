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

#ifndef MUJOCO_PID_CONTROLLER
#define MUJOCO_PID_CONTROLLER

#include <Eigen/Dense>

#include "architecture/messaging/messaging.h"
#include "architecture/utilities/bskLogging.h"
#include "simulation/mujocoDynamics/_GeneralModuleFiles/StatefulSysModel.h"
#include "simulation/mujocoDynamics/_GeneralModuleFiles/MJUtils.h"

/**
 * @brief Generic PID controller.
 *
 * This templated class implements a proportional-integral-derivative (PID)
 * controller for use in MuJoCo-based simulations. It reads measured and desired
 * position/velocity messages, computes the control output, and writes it to an
 * output message. The controller supports custom payload types for measured and
 * desired states, as well as output.
 *
 * The controller requires the relevant input messages to be linked if the
 * corresponding gain (Kp or Kd) is non-zero. Throws a runtime error if a required
 * message is not linked.
 *
 * @tparam MeasuredPositionMsgPayload Type of measured position message payload.
 * @tparam MeasuredVelocityMsgPayload Type of measured velocity message payload.
 * @tparam OutputMsgPayload Type of output message payload.
 * @tparam DesiredPositionMsgPayload Type of desired position message payload (defaults to measured).
 * @tparam DesiredVelocityMsgPayload Type of desired velocity message payload (defaults to measured).
 */
template<typename MeasuredPositionMsgPayload, typename MeasuredVelocityMsgPayload, typename OutputMsgPayload, typename DesiredPositionMsgPayload = MeasuredPositionMsgPayload, typename DesiredVelocityMsgPayload = MeasuredVelocityMsgPayload>
class PIDController : public StatefulSysModel
{
public:
    /** @brief Virtual destructor for safe inheritance. */
    virtual ~PIDController() = default;

    /** @brief Get the proportional gain. */
    double getProportionalGain() const { return Kp; }

    /** @brief Set the proportional gain. */
    void setProportionalGain(double value) { Kp = value; }

    /** @brief Get the derivative gain. */
    double getDerivativeGain() const { return Kd; }

    /** @brief Set the derivative gain. */
    void setDerivativeGain(double value) { Kd = value; }

    /** @brief Get the integral gain. */
    double getIntegralGain() const { return Ki; }

    /** @brief Set the integral gain. */
    void setIntegralGain(double value) { Ki = value; }

    /**
     * @brief Get the current integral error state.
     * @return The integral error value.
     */
    double getIntegralError() {
        if (!this->integralErrorState)
        {
            MJBasilisk::detail::logAndThrow<std::runtime_error>(
                "Tried to get integral error before simulation has been initialized",
                &bskLogger
            );
        }
        return this->integralErrorState->getState()(0,0);
    }

    /**
     * @brief Set the integral error state.
     * @param val Value to set.
     */
    void setIntegralError(double val) {
        if (!this->integralErrorState)
        {
            MJBasilisk::detail::logAndThrow<std::runtime_error>(
                "Tried to set integral error before simulation has been initialized",
                &bskLogger
            );
        }
        this->integralErrorState->setState(Eigen::Matrix<double, 1, 1>::Constant(val));
    }

    /**
     * @brief Register the integral error state with the state engine.
     * @param registerer State registration helper.
     */
    void registerStates(DynParamRegisterer registerer) override
    {
        this->integralErrorState = registerer.registerState(1, 1, "integralError");
        this->setIntegralError(0); // initialize state to zero
    }

    /**
     * @brief Update the controller state and compute the control output.
     *
     * Reads the measured and desired position/velocity messages, computes the
     * control output using the PID formula, and writes the result to the output
     * message. Throws a runtime error if required messages are not linked when
     * Kp or Kd are non-zero.
     *
     * @param CurrentSimNanos Current simulation time in nanoseconds.
     */
    void UpdateState(uint64_t CurrentSimNanos) override
    {
        double positionError = 0;
        if (this->Kp != 0)
        {
            if (!desiredPosInMsg.isLinked())
            {
                MJBasilisk::detail::logAndThrow<std::runtime_error>(
                    "PIDController [" + ModelTag + "]: desiredPosInMsg must be linked when Kp is non-zero.",
                    &bskLogger
                );
            }
            if (!measuredPosInMsg.isLinked())
            {
                MJBasilisk::detail::logAndThrow<std::runtime_error>(
                    "PIDController [" + ModelTag + "]: measuredPosInMsg must be linked when Kp is non-zero.",
                    &bskLogger
                );
            }
            positionError = readDesiredPosition(desiredPosInMsg()) - readMeasuredPosition(measuredPosInMsg());
        }

        double velocityError = 0;
        if (this->Kd != 0)
        {
            if (!desiredVelInMsg.isLinked())
            {
                MJBasilisk::detail::logAndThrow<std::runtime_error>(
                    "PIDController [" + ModelTag + "]: desiredVelInMsg must be linked when Kd is non-zero.",
                    &bskLogger
                );
            }
            if (!measuredVelInMsg.isLinked())
            {
                MJBasilisk::detail::logAndThrow<std::runtime_error>(
                    "PIDController [" + ModelTag + "]: measuredVelInMsg must be linked when Kd is non-zero.",
                    &bskLogger
                );
            }
            velocityError = readDesiredVelocity(desiredVelInMsg()) - readMeasuredVelocity(measuredVelInMsg());
        }

        double integralError = this->integralErrorState->getState()(0,0);

        double outputVal = this->Kp * positionError + this->Kd * velocityError + this->Ki * integralError;

        auto& payload = outputOutMsg.zeroMsgPayload;
        writeOutput(payload, outputVal);
        outputOutMsg.write(&payload, CurrentSimNanos, this->moduleID);
    }

public:
    /** @brief Input message functor for measured position.*/
    ReadFunctor<MeasuredPositionMsgPayload> measuredPosInMsg;

    /** @brief Input message functor for desired position.*/
    ReadFunctor<DesiredPositionMsgPayload> desiredPosInMsg;

    /** @brief Input message functor for measured velocity.*/
    ReadFunctor<MeasuredVelocityMsgPayload> measuredVelInMsg;

    /** @brief Input message functor for desired velocity.*/
    ReadFunctor<DesiredVelocityMsgPayload> desiredVelInMsg;

    /** @brief Output message for control value.*/
    Message<OutputMsgPayload> outputOutMsg;

    BSKLogger bskLogger; ///< BSK Logging

protected:
    /**
     * @brief Read the measured position from the input payload.
     * @param i Measured position message payload.
     * @return The measured position value.
     */
    virtual double readMeasuredPosition(const MeasuredPositionMsgPayload& i) const = 0;

    /**
     * @brief Read the measured velocity from the input payload.
     * @param i Measured velocity message payload.
     * @return The measured velocity value.
     */
    virtual double readMeasuredVelocity(const MeasuredVelocityMsgPayload& i) const = 0;

    /**
     * @brief Read the desired position from the input payload.
     * Defaults to using the read method ``readMeasuredPosition``.
     * @param i Desired position message payload.
     * @return The desired position value.
     */
    virtual double readDesiredPosition(const DesiredPositionMsgPayload& i) const { return readMeasuredPosition(i); }

    /**
     * @brief Read the desired velocity from the input payload.
     * Defaults to using the read method ``readMeasuredVelocity``.
     * @param i Desired velocity message payload.
     * @return The desired velocity value.
     */
    virtual double readDesiredVelocity(const DesiredVelocityMsgPayload& i) const { return readMeasuredVelocity(i); }

    /**
     * @brief Write the computed output value to the output payload.
     * @param o Output message payload.
     * @param val Value to write.
     */
    virtual void writeOutput(OutputMsgPayload& o, double val) = 0;

protected:
    /** @brief State data for the integral error term.*/
    StateData* integralErrorState = nullptr;

    /** @brief Proportional gain.*/
    double Kp = 0;

    /** @brief Derivative gain.*/
    double Kd = 0;

    /** @brief Integral gain.*/
    double Ki = 0;
};

#endif
