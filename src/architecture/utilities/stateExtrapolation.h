/*
 ISC License

 Copyright (c) 2026, PIC4SeR & AVS Lab, Politecnico di Torino & Argotec S.R.L., University of Colorado Boulder

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

#ifndef STATE_EXTRAPOLATION_H
#define STATE_EXTRAPOLATION_H

#include <cstdint>

#include "architecture/msgPayloadDefC/SCStatesMsgPayload.h"
#include "architecture/utilities/macroDefinitions.h"

/*! @brief Extrapolate a spacecraft state message to the middle of the interval the next spacecraft update integrates.
 *
 * Environment modules (atmosphere, wind, magnetic field, eclipse, solar flux, albedo) run before the
 * spacecraft within a task, so the state they read was written at the end of the previous step. The
 * spacecraft update at the current time integrates the interval between that message time and the
 * current time, so the best estimate of where the spacecraft is while the environment output acts is the state
 * advanced by half of the message age. The position is advanced with the message velocity.
 *
 * @note The extrapolation assumes the environment module and the spacecraft run at the same task rate.
 * The state is returned unchanged if the message was written at the current time, or if it was written before
 * the previous update of the environment module. The latter means the message is stale (for example a state
 * written once at the start of the simulation) and not the output of the previous spacecraft update.
 *
 * @param scState spacecraft state message payload as read from the message
 * @param currentSimNanos [ns] current simulation time
 * @param timeWrittenNanos [ns] time the spacecraft state message was written
 * @param previousUpdateNanos [ns] time of the previous update of the calling environment module
 * @return copy of the payload with `r_BN_N` and `r_CN_N` advanced by `v * 0.5 * (currentSimNanos - timeWrittenNanos)`
 */
static inline SCStatesMsgPayload
extrapolateScStateToStepMidpoint(const SCStatesMsgPayload& scState,
                                 uint64_t currentSimNanos,
                                 uint64_t timeWrittenNanos,
                                 uint64_t previousUpdateNanos)
{
    SCStatesMsgPayload extrapolated = scState;
    if (currentSimNanos <= timeWrittenNanos || timeWrittenNanos < previousUpdateNanos) {
        return extrapolated;
    }
    const double halfAge = 0.5 * diffNanoToSec(currentSimNanos, timeWrittenNanos); // [s]
    for (int i = 0; i < 3; i++) {
        extrapolated.r_BN_N[i] += scState.v_BN_N[i] * halfAge;
        extrapolated.r_CN_N[i] += scState.v_CN_N[i] * halfAge;
    }
    return extrapolated;
}

#endif
