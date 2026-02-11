/*
 ISC License

 Copyright (c) 2025, Autonomous Vehicle Systems Lab

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

#ifndef FORCE_AT_SITE_LTI_H
#define FORCE_AT_SITE_LTI_H

#include "linearTimeInvariantSystem.h"

#include "architecture/messaging/messaging.h"
#include "architecture/msgPayloadDefC/ForceAtSiteMsgPayload.h"

/**
 * @brief Linear 3D force model based on LinearTimeInvariantSystem.
 *
 * This class implements a three input, three output LTI system where the
 * input and output are carried through ForceAtSiteMsgPayload messages.
 *
 * Input:
 *   u = [Fx, Fy, Fz]^T in the site S frame, read from inMsg().force_S.
 *
 * Output:
 *   y = [Fx, Fy, Fz]^T in the site S frame, written to outMsg.force_S.
 *
 * The internal dynamics are defined by the A, B, C, D matrices stored in the
 * base class LinearTimeInvariantSystem.
 */
class ForceAtSiteLTI : public LinearTimeInvariantSystem
{
public:
    /**
     * @brief Default constructor.
     *
     * The system matrices must be configured by the user before running
     * the simulation, either by direct calls to setA, setB, setC, setD
     * or through convenience configuration helpers.
     */
    ForceAtSiteLTI() = default;

    /**
     * @brief Get the dimension of the input vector.
     *
     * This model is strictly 3D, so the input dimension is always 3.
     *
     * @return Number of inputs (always 3).
     */
    size_t getInputSize() const override { return 3; }

    /**
     * @brief Get the dimension of the output vector.
     *
     * This model is strictly 3D, so the output dimension is always 3.
     *
     * @return Number of outputs (always 3).
     */
    size_t getOutputSize() const override { return 3; }

    /**
     * @brief Read the current input vector from the subscribed input message.
     *
     * This method constructs a 3 by 1 Eigen::VectorXd whose elements are
     * taken from the force_S array of the ForceAtSiteMsgPayload read via
     * inMsg().
     *
     * @param CurrentSimNanos Current simulation time in nanoseconds.
     *                        (Unused in this implementation.)
     * @return Input vector u of size 3.
     */
    Eigen::VectorXd readInput(uint64_t CurrentSimNanos) override;

    /**
     * @brief Write the current output vector to the output message.
     *
     * The first three elements of the output vector y are written to the
     * force_S array of the ForceAtSiteMsgPayload and sent on outMsg.
     *
     * @param CurrentSimNanos Current simulation time in nanoseconds.
     * @param y Output vector of size at least 3.
     */
    void writeOutput(uint64_t CurrentSimNanos,
                     const Eigen::VectorXd &y) override;

public:
    /**
     * @brief Output message carrying the 3D force command at the site.
     *
     * The force_S array is populated from the first three elements of
     * the output vector y computed by LinearTimeInvariantSystem.
     */
    Message<ForceAtSiteMsgPayload> outMsg;

    /**
     * @brief Input message read functor providing the 3D force at the site.
     *
     * The force_S array of the payload is mapped to the input vector u.
     */
    ReadFunctor<ForceAtSiteMsgPayload> inMsg;
};

#endif /* FORCE_AT_SITE_LTI_H */
