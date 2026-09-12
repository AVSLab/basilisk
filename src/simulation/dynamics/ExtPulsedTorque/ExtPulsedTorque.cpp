/*
 ISC License

 Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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
#include "simulation/dynamics/ExtPulsedTorque/ExtPulsedTorque.h"
#include <algorithm>
#include <cmath>
#include <limits>

/*! This is the constructor.  It sets some default initializers that can be
 overridden by the user.*/
ExtPulsedTorque::ExtPulsedTorque()
{
    CallCounts = 0;
    return;
}

/*! The destructor.  Nothing of note is performed here*/
ExtPulsedTorque::~ExtPulsedTorque()
{
    return;
}


/*! link the states
 *
 * @param[in] statesIn Dynamic parameter manager containing the required states.
 */
void ExtPulsedTorque::linkInStates(DynParamManager& statesIn [[maybe_unused]])
{
    this->validateConfiguration();
}


/*! @brief Validate pulse configuration without changing its phase relative to simulation time.
 * @param CurrentSimNanos [ns] Scheduler reset time; it does not shift the pulse sequence.
 */
void ExtPulsedTorque::Reset(uint64_t CurrentSimNanos [[maybe_unused]])
{
    this->validateConfiguration();
}

/*! @brief Validate pulse parameters before attachment, reset, or torque evaluation. */
void ExtPulsedTorque::validateConfiguration()
{
    if (!this->pulsedTorqueExternalPntB_B.allFinite()) {
        this->bskLogger.bskError("ExtPulsedTorque: pulsedTorqueExternalPntB_B must contain only finite values.");
    }
    if (this->countOnPulse < 0 || this->countOff < 0) {
        this->bskLogger.bskError("ExtPulsedTorque: countOnPulse and countOff must be non-negative.");
    }
    if (!std::isfinite(this->pulseInterval) || this->pulseInterval <= 0.0) {
        this->bskLogger.bskError("ExtPulsedTorque: pulseInterval must be finite and greater than zero.");
    }
    const double countPeriod = 2.0 * static_cast<double>(this->countOnPulse) + this->countOff;
    if (!std::isfinite(countPeriod * this->pulseInterval)) {
        this->bskLogger.bskError("ExtPulsedTorque: pulse counts and pulseInterval must produce a finite cycle duration.");
    }
}


/*! This module does not write any output messages.
 @param currentClock The current time used for time-stamping the message

 */
void ExtPulsedTorque::writeOutputMessages(uint64_t currentClock [[maybe_unused]])
{
    return;
}

/*! This method is used to read the incoming message and set the
 associated buffer structure.

 */
void ExtPulsedTorque::readInputMessages()
{
    return;
}

/*! This method is used to compute the RHS forces and torques.
    Note:   the module can set any of these three vectors, or a subset.  Regarding the external force, the
            matrix representations in the body (B) and inertial (N) frame components are treated as 2
            separate vectors.  Only set both if you mean to, as both vectors will be included.
 *
 * @param[in] integTime [s] Current integration time.
 * @param[in] timeStep [s] Integration time step; unused because pulse timing uses pulseInterval.
 */
void ExtPulsedTorque::computeForceTorque(double integTime, double timeStep [[maybe_unused]])
{
    this->validateConfiguration();
    if (!std::isfinite(integTime) || integTime < 0.0) {
        this->bskLogger.bskError("ExtPulsedTorque: integTime must be finite and non-negative, in seconds.");
    }

    this->forceExternal_N.setZero();
    this->forceExternal_B.setZero();
    this->torqueExternalPntB_B.setZero();
    if (this->countOnPulse == 0) {
        return;
    }

    const double onDuration = static_cast<double>(this->countOnPulse) * this->pulseInterval; // [s]
    const double countPeriod = 2.0 * static_cast<double>(this->countOnPulse) + this->countOff;
    const double cycleDuration = countPeriod * this->pulseInterval; // [s]
    double phase = std::fmod(integTime, cycleDuration); // [s]
    // Roundoff in decimal intervals can place a transition just inside the preceding segment.
    // Scale each tolerance to its own transition and the evaluation time, so a long future off
    // period cannot advance the initial pulse edges. Bound it to keep adjacent windows separate.
    const auto isNearTransition = [this, integTime, phase](double transition) {
        const double tolerance = std::min(0.25 * this->pulseInterval,
            4.0 * std::numeric_limits<double>::epsilon() * std::max(integTime, transition)); // [s]
        return std::abs(phase - transition) <= tolerance;
    };
    if (isNearTransition(cycleDuration)) {
        phase = 0.0; // [s]
    } else if (isNearTransition(onDuration)) {
        phase = onDuration;
    } else if (isNearTransition(2.0 * onDuration)) {
        phase = 2.0 * onDuration; // [s]
    }
    if (phase < onDuration) {
        this->torqueExternalPntB_B = this->pulsedTorqueExternalPntB_B;
    } else if (phase < 2.0 * onDuration) {
        this->torqueExternalPntB_B = -this->pulsedTorqueExternalPntB_B;
    }
}

/*! Module update method
 *
 * @param[in] CurrentSimNanos [ns] Current simulation time.
 */
void ExtPulsedTorque::UpdateState(uint64_t CurrentSimNanos [[maybe_unused]])
{
    return;
}
