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
#include "forceAtSiteLTI.h"

Eigen::VectorXd ForceAtSiteLTI::readInput(uint64_t /*CurrentSimNanos*/)
{
    // Read the current payload from the input message
    const ForceAtSiteMsgPayload payload = this->inMsg();

    // Construct a 3 by 1 input vector from the force components
    Eigen::VectorXd u(3);
    u(0) = payload.force_S[0];
    u(1) = payload.force_S[1];
    u(2) = payload.force_S[2];

    return u;
}

void ForceAtSiteLTI::writeOutput(uint64_t CurrentSimNanos,
                                 const Eigen::VectorXd &y)
{
    ForceAtSiteMsgPayload payload{};
    payload.force_S[0] = y(0);
    payload.force_S[1] = y(1);
    payload.force_S[2] = y(2);

    this->outMsg.write(&payload, this->moduleID, CurrentSimNanos);
}
