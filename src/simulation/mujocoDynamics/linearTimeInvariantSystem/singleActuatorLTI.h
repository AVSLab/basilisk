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

#ifndef SINGLE_ACTUATOR_LTI_H
#define SINGLE_ACTUATOR_LTI_H

#include "linearTimeInvariantSystem.h"

#include "architecture/messaging/messaging.h"
#include "architecture/msgPayloadDefC/SingleActuatorMsgPayload.h"

/**
 * @brief Linear SISO actuator model built on top of LinearTimeInvariantSystem.
 *
 * This class implements a single-input, single-output LTI system where
 * both the input and output are carried through a SingleActuatorMsgPayload.
 *
 * The internal dynamics are defined by the A, B, C, D matrices stored in the
 * base class LinearTimeInvariantSystem.
 */
class SingleActuatorLTI : public LinearTimeInvariantSystem
{
public:
    /**
     * @brief Default constructor.
     *
     * The underlying LinearTimeInvariantSystem matrices must be configured
     * (for example, via configureSecondOrder or direct setA/setB/setC/setD
     * calls) before the simulation is run.
     */
    SingleActuatorLTI() = default;

    /**
     * @brief Get the dimension of the input vector.
     *
     * This model is strictly SISO, so the input dimension is always 1.
     *
     * @return Number of inputs (always 1).
     */
    size_t getInputSize() const override { return 1; }

    /**
     * @brief Get the dimension of the output vector.
     *
     * This model is strictly SISO, so the output dimension is always 1.
     *
     * @return Number of outputs (always 1).
     */
    size_t getOutputSize() const override { return 1; }

    /**
     * @brief Read the current input vector from the subscribed input message.
     *
     * This method constructs a 1x1 Eigen::VectorXd whose single element is
     * taken from inMsg().input.
     *
     * @param CurrentSimNanos Current simulation time in nanoseconds.
     * @return Input vector u of size 1.
     */
    Eigen::VectorXd readInput(uint64_t CurrentSimNanos) override;

    /**
     * @brief Write the current output vector to the output message.
     *
     * The first element of the output vector y is written to outMsg as the
     * 'input' field of SingleActuatorMsgPayload.
     *
     * @param CurrentSimNanos Current simulation time in nanoseconds.
     * @param y Output vector of size 1.
     */
    void writeOutput(uint64_t CurrentSimNanos,
                     const Eigen::VectorXd &y) override;

public:
    /**
     * @brief Output message carrying the actuator command.
     *
     * The 'input' field is populated from the first element of the output
     * vector y computed by LinearTimeInvariantSystem.
     */
    Message<SingleActuatorMsgPayload> outMsg;

    /**
     * @brief Input message read functor providing the actuator command.
     *
     * The 'input' field of the payload is mapped to the input vector u(0).
     */
    ReadFunctor<SingleActuatorMsgPayload> inMsg;
};

#endif /* SINGLE_ACTUATOR_LTI_H */
