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

#include <cstddef>
#include <cstdint>
#include <vector>

#include "architecture/msgPayloadDefC/SCStatesMsgPayload.h"
#include "architecture/utilities/bskLogging.h"
#include "architecture/utilities/macroDefinitions.h"

/*! @brief Extrapolate a spacecraft state message to the middle of the interval the next spacecraft update integrates.
 *
 * Environment modules (atmosphere, wind, magnetic field, eclipse, solar flux, albedo) run before the
 * spacecraft within a task, so the state they read was written at the end of the previous step. The
 * spacecraft update at the current time integrates the interval between that message time and the
 * current time, so the best estimate of where the spacecraft is while the environment output acts is the state
 * advanced by half of the message age. The position is advanced with the message velocity.
 *
 * @note The extrapolation assumes the environment module and the spacecraft run at the same task rate and that the
 * spacecraft uses a constant step. With different task rates, or a variable spacecraft step, the offset applied
 * is not the half step the spacecraft integrates and it can alternate from one update to the next.
 * The state is returned unchanged if the message was written at the current time, or if it was written before
 * the previous update of the environment module. The latter means the message is stale (for example a state
 * written once at the start of the simulation) and not the output of the previous spacecraft update.
 * A message written exactly at the previous update, which is what the spacecraft does in every step, or at the start of
 * the simulation, is extrapolated.
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

/*! @brief Opt-in switch and rate-mismatch monitor for the spacecraft state extrapolation of an environment module.
 *
 * The extrapolation is disabled by default, in which case the state message is used as written. When enabled,
 * the state is extrapolated with extrapolateScStateToStepMidpoint() and a warning is logged once if a message that
 * is rewritten by the spacecraft is found older than the previous module update, which means the module and the
 * spacecraft do not run at the same task rate.
 */
class ScStateExtrapolation
{
  public:
    /*! @brief Enable or disable the extrapolation.
     * @param enable true to extrapolate the spacecraft state to the step midpoint
     */
    void setEnabled(bool enable) { this->enabled = enable; }

    /*! @brief Returns whether the extrapolation is enabled.
     * @return true if the spacecraft state is extrapolated
     */
    bool isEnabled() const { return this->enabled; }

    /*! @brief Clear the observed message history and re-arm the warning. */
    void reset()
    {
        this->lastWriteNanos.clear();
        this->rewritten.clear();
        this->warned = false;
    }

    /*! @brief Return the spacecraft state, extrapolated if the extrapolation is enabled.
     * @param index index of the spacecraft state input message
     * @param scState spacecraft state message payload as read from the message
     * @param currentSimNanos [ns] current simulation time
     * @param timeWrittenNanos [ns] time the spacecraft state message was written
     * @param previousUpdateNanos [ns] time of the previous update of the calling environment module
     * @param logger logger of the calling module
     * @return the payload, with the position advanced to the step midpoint if the extrapolation is enabled
     */
    SCStatesMsgPayload apply(std::size_t index,
                             const SCStatesMsgPayload& scState,
                             uint64_t currentSimNanos,
                             uint64_t timeWrittenNanos,
                             uint64_t previousUpdateNanos,
                             BSKLogger& logger)
    {
        if (!this->enabled) {
            return scState;
        }
        if (this->lastWriteNanos.size() <= index) {
            this->lastWriteNanos.resize(index + 1, timeWrittenNanos);
            this->rewritten.resize(index + 1, false);
        }
        if (timeWrittenNanos != this->lastWriteNanos[index]) {
            this->rewritten[index] = true;
            this->lastWriteNanos[index] = timeWrittenNanos;
        }
        if (this->rewritten[index] && timeWrittenNanos < previousUpdateNanos && !this->warned) {
            this->warned = true;
            logger.bskLog(BSK_WARNING,
                          "The spacecraft state extrapolation is enabled, but the spacecraft state message is "
                          "older than the previous module update. The module and the spacecraft do not run at the "
                          "same task rate, or the spacecraft step is variable: the extrapolation is inconsistent.");
        }
        return extrapolateScStateToStepMidpoint(scState, currentSimNanos, timeWrittenNanos, previousUpdateNanos);
    }

  private:
    bool enabled = false;                    //!< true if the spacecraft state is extrapolated
    bool warned = false;                     //!< true once the rate mismatch warning was logged
    std::vector<uint64_t> lastWriteNanos{};  //!< [ns] last observed write time of each state message
    std::vector<bool> rewritten{};           //!< true if the state message was seen with more than one write time
};

#endif
