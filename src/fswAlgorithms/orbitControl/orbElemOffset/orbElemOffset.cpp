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

#include "orbElemOffset.h"

#include "architecture/utilities/orbitalMotion.h"

void OrbElemOffset::Reset(uint64_t /*CurrentSimNanos*/)
{
    if (!this->mainElementsInMsg.isLinked()) {
        this->bskLogger.bskLog(
            BSK_ERROR,
            "OrbElemOffset::Reset: mainElementsInMsg is not linked."
        );
    }

    if (!this->offsetElementsInMsg.isLinked()) {
        this->bskLogger.bskLog(
            BSK_ERROR,
            "OrbElemOffset::Reset: offsetElementsInMsg is not linked."
        );
    }
}

void OrbElemOffset::UpdateState(uint64_t CurrentSimNanos)
{
    ClassicElementsMsgPayload outPayload{};
    const ClassicElementsMsgPayload main   = this->mainElementsInMsg();
    const ClassicElementsMsgPayload offset = this->offsetElementsInMsg();

    // Element wise addition for all fields except f which is handled separately
    outPayload.a       = main.a       + offset.a;
    outPayload.e       = main.e       + offset.e;
    outPayload.i       = main.i       + offset.i;
    outPayload.Omega   = main.Omega   + offset.Omega;
    outPayload.omega   = main.omega   + offset.omega;

    if (useMeanAnomalyOffset) {
        // Interpret offset.f as a mean anomaly increment DeltaM
        // Step 1: mean anomaly of the main orbit
        const double e_main = main.e;
        const double e_out  = outPayload.e;

        double E_main   = f2E(main.f, e_main);
        double M_main   = E2M(E_main, e_main);
        double M_offset = offset.f;

        // Step 2: final mean anomaly and conversion back to true anomaly
        double M_out = M_main + M_offset;
        double E_out = M2E(M_out, e_out);
        outPayload.f = E2f(E_out, e_out);
    } else {
        // Plain true anomaly increment
        outPayload.f = main.f + offset.f;
    }

    this->elementsOutMsg.write(&outPayload, this->moduleID, CurrentSimNanos);
}
