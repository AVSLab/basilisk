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

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "architecture/msgPayloadDefC/SCStatesMsgPayload.h"
#include "architecture/msgPayloadDefC/SpicePlanetStateMsgPayload.h"
#include "architecture/utilities/bskLogging.h"
#include "architecture/utilities/macroDefinitions.h"

/*! @brief Advance a planet-fixed orientation matrix by a time offset as a rotation about the planet angular velocity.
 *
 * A first-order update of the matrix elements (`dcm_NPfix + dcm_NPfix_dot * dt`) is not orthonormal. This function
 * extracts the angular velocity from `dcm_NPfix_dot * dcm_NPfix^T` and applies the rotation `AngleAxis(|omega| dt)`,
 * so the result stays orthonormal.
 *
 * @param dcm_NPfix [-] [NP] orientation of the inertial frame relative to the planet-fixed frame, which maps
 * planet-fixed components to inertial components
 * @param dcm_NPfix_dot [1/s] time derivative of dcm_NPfix
 * @param dt [s] signed time offset to advance the orientation by
 * @return the advanced orientation matrix, or dcm_NPfix if the derivative or the offset is zero
 * @note The callers pass the transpose of `J20002Pfix` (which is [PN]). GravBodyData::computeGravityInertial() holds
 * the same matrix in a variable named `dcm_PfixN`.
 */
static inline Eigen::Matrix3d
extrapolateDcm(const Eigen::Matrix3d& dcm_NPfix, const Eigen::Matrix3d& dcm_NPfix_dot, double dt)
{
    if (dcm_NPfix_dot.isZero() || dt == 0.0) {
        return dcm_NPfix;
    }
    // dcm_NPfix_dot = [omega_N x] dcm_NPfix, hence [omega_N x] = dcm_NPfix_dot * dcm_NPfix^T
    const Eigen::Matrix3d omegaTilde_N = dcm_NPfix_dot * dcm_NPfix.transpose();
    const Eigen::Vector3d omega_N(0.5 * (omegaTilde_N(2, 1) - omegaTilde_N(1, 2)),
                                  0.5 * (omegaTilde_N(0, 2) - omegaTilde_N(2, 0)),
                                  0.5 * (omegaTilde_N(1, 0) - omegaTilde_N(0, 1))); // [rad/s]
    const double rotationAngle = omega_N.norm() * dt;                               // [rad]
    if (rotationAngle == 0.0) {
        return dcm_NPfix;
    }
    return Eigen::AngleAxisd(rotationAngle, omega_N.normalized()).toRotationMatrix() * dcm_NPfix;
}

/*! @brief Returns true if a spacecraft state message is the output of the previous update of the environment module.
 *
 * The message-age based extrapolation is only valid if the spacecraft message was written at the previous update of
 * the environment module. Then the age of the message is exactly the update interval of the module, and half of the
 * age is the middle of that interval. This holds if the module and the spacecraft are updated with the same task
 * period. A message written at any other earlier time is either stale (for example a state written once at the start
 * of the simulation), or the spacecraft is updated at a different task period than the environment module.
 *
 * @param currentSimNanos [ns] current simulation time
 * @param timeWrittenNanos [ns] time the spacecraft state message was written
 * @param previousUpdateNanos [ns] time of the previous update of the calling environment module
 * @return true if the message is older than the current time and was written at the previous module update
 */
static inline bool
isStepLagMessage(uint64_t currentSimNanos, uint64_t timeWrittenNanos, uint64_t previousUpdateNanos)
{
    return timeWrittenNanos < currentSimNanos && timeWrittenNanos == previousUpdateNanos;
}

/*! @brief Extrapolate a spacecraft state message to the middle of the interval the next spacecraft update integrates.
 *
 * Environment modules (atmosphere, wind, magnetic field, eclipse, solar flux) run before the
 * spacecraft within a task, so the state they read was written at the end of the previous step. The
 * spacecraft update at the current time integrates the interval between that message time and the
 * current time, so the best estimate of where the spacecraft is while the environment output acts is the state
 * advanced by half of the message age. The position is advanced with the message velocity.
 *
 * @note The requirement concerns the task period, not the integrator. The environment module must be updated with
 * the same task period as the spacecraft, so that the message written by the spacecraft at the previous module
 * update is exactly one update interval old. The internal substeps of a variable-step integrator do not matter:
 * they still cover the update interval requested of the spacecraft. The state is returned unchanged if the message
 * was written at the current time, or if it was not written at the previous update of the environment module.
 * The latter is either a stale message (written once at the start of the simulation), or a spacecraft that runs
 * faster or slower than the environment module, for which half of the message age is not the middle of the module
 * interval. ScStateExtrapolation additionally waits until the spacecraft period has been observed.
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
    if (!isStepLagMessage(currentSimNanos, timeWrittenNanos, previousUpdateNanos)) {
        return extrapolated;
    }
    const double halfAge = 0.5 * diffNanoToSec(currentSimNanos, timeWrittenNanos); // [s]
    for (int i = 0; i < 3; i++) {
        extrapolated.r_BN_N[i] += scState.v_BN_N[i] * halfAge;
        extrapolated.r_CN_N[i] += scState.v_CN_N[i] * halfAge;
    }
    return extrapolated;
}

/*! @brief Move a planet state message to the epoch the spacecraft state was extrapolated to.
 *
 * The relative position of the spacecraft with respect to a planet is only meaningful if both are evaluated at the
 * same epoch. The position is advanced with the planet velocity, and the planet-fixed orientation is advanced with
 * extrapolateDcm(). The offset is signed: it is negative if the planet message is newer than the target epoch.
 *
 * @param planetState planet state message payload as read from the message
 * @param targetNanos [ns] epoch to evaluate the planet at
 * @param timeWrittenNanos [ns] time the planet state message was written
 * @return copy of the payload with the position and the orientation advanced to the target epoch
 * @note As in GravBodyData::computeGravityInertial(), the planet is advanced by the time between the message write and
 * the target epoch, with no check on the age of the message. A message written once with a non-zero velocity is
 * projected forward over the whole simulation.
 */
static inline SpicePlanetStateMsgPayload
extrapolatePlanetStateToEpoch(const SpicePlanetStateMsgPayload& planetState,
                              uint64_t targetNanos,
                              uint64_t timeWrittenNanos)
{
    SpicePlanetStateMsgPayload extrapolated = planetState;
    const double dt = diffNanoToSec(targetNanos, timeWrittenNanos); // [s]
    if (dt == 0.0) {
        return extrapolated;
    }
    for (int i = 0; i < 3; i++) {
        extrapolated.PositionVector[i] += planetState.VelocityVector[i] * dt;
    }
    using RowMajorMatrix3d = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>;
    // J20002Pfix is [PN], it maps inertial to planet-fixed components. Its transpose [NP] is advanced.
    const Eigen::Matrix3d dcm_NPfix = RowMajorMatrix3d(Eigen::Map<const RowMajorMatrix3d>(&planetState.J20002Pfix[0][0])).transpose();
    const Eigen::Matrix3d dcm_NPfix_dot = RowMajorMatrix3d(Eigen::Map<const RowMajorMatrix3d>(&planetState.J20002Pfix_dot[0][0])).transpose();
    const Eigen::Matrix3d dcmAdvanced_NPfix = extrapolateDcm(dcm_NPfix, dcm_NPfix_dot, dt);
    Eigen::Map<RowMajorMatrix3d>(&extrapolated.J20002Pfix[0][0]) = dcmAdvanced_NPfix.transpose();
    // keep the derivative consistent with the advanced matrix: dcm_NPfix_dot = [omega_N x] dcm_NPfix, with the
    // inertial angular velocity unchanged, so dcm_NPfix_dot advances as [omega_N x] dcmAdvanced_NPfix
    const Eigen::Matrix3d omegaTilde_N = dcm_NPfix_dot * dcm_NPfix.transpose();
    const Eigen::Matrix3d dcmAdvanced_NPfix_dot = omegaTilde_N * dcmAdvanced_NPfix;
    Eigen::Map<RowMajorMatrix3d>(&extrapolated.J20002Pfix_dot[0][0]) = dcmAdvanced_NPfix_dot.transpose();
    return extrapolated;
}

/*! @brief Opt-in switch and rate-mismatch monitor for the spacecraft state extrapolation of an environment module.
 *
 * The extrapolation is disabled by default, in which case the state message is used as written. When enabled, the
 * module calls prepare() once per update with the write times of all its spacecraft state messages. The spacecraft
 * states are extrapolated with extrapolateScStateToStepMidpoint() and the planets with
 * extrapolatePlanetStateToEpoch() only if all spacecraft messages were written at the previous module update and two
 * successive write times were observed with an interval equal to the module update interval. The extrapolation is
 * therefore deferred until the spacecraft period is known (the first updates are never extrapolated). If
 * one of them was not nothing is extrapolated, so the spacecraft and the shared planets stay at one epoch, and a warning is
 * logged once.
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

    /*! @brief Returns whether a task period mismatch between the module and the spacecraft was detected.
     * @return true if the warning was logged since the last reset
     */
    bool mismatchDetected() const { return this->warned; }

    /*! @brief Clear the observed message history and re-arm the warning. */
    void reset()
    {
        this->lastWriteNanos.clear();
        this->writeIntervalNanos.clear();
        this->rewritten.clear();
        this->seen.clear();
        this->warned = false;
        this->lagConsistent = false;
    }

    /*! @brief Decide once per module update whether the extrapolation applies to all spacecraft and the planets.
     *
     * The planet state messages are shared by all spacecraft of a module, so the decision cannot be made per
     * spacecraft: a planet moved to the step midpoint is inconsistent with a spacecraft left at the message epoch, and
     * the other way around. The extrapolation is therefore applied to every spacecraft and to the planets only if all
     * spacecraft state messages were written at the previous module update (see isStepLagMessage()). If one of them
     * was not, nothing is extrapolated and a warning is logged once.
     *
     * Must be called once per update, before apply() and applyPlanet().
     * @param currentSimNanos [ns] current simulation time
     * @param previousUpdateNanos [ns] time of the previous update of the calling environment module
     * @param timesWrittenNanos [ns] write time of each spacecraft state message, in the order of the apply() indexes
     * @param logger logger of the calling module
     */
    void prepare(uint64_t currentSimNanos,
                 uint64_t previousUpdateNanos,
                 const std::vector<uint64_t>& timesWrittenNanos,
                 BSKLogger& logger)
    {
        this->lagConsistent = false;
        if (!this->enabled) {
            return;
        }
        if (this->lastWriteNanos.size() < timesWrittenNanos.size()) {
            this->lastWriteNanos.resize(timesWrittenNanos.size(), 0);
            this->writeIntervalNanos.resize(timesWrittenNanos.size(), 0);
            this->rewritten.resize(timesWrittenNanos.size(), false);
            this->seen.resize(timesWrittenNanos.size(), false);
        }
        const uint64_t moduleIntervalNanos = currentSimNanos - previousUpdateNanos; // [ns]
        bool allLagged = true;
        bool mismatch = false;
        for (std::size_t index = 0; index < timesWrittenNanos.size(); index++) {
            const uint64_t timeWrittenNanos = timesWrittenNanos[index];
            if (!this->seen[index]) {
                this->seen[index] = true;
                this->lastWriteNanos[index] = timeWrittenNanos;
            } else if (timeWrittenNanos != this->lastWriteNanos[index]) {
                this->rewritten[index] = true;
                this->writeIntervalNanos[index] = timeWrittenNanos - this->lastWriteNanos[index];
                this->lastWriteNanos[index] = timeWrittenNanos;
            }
            // extrapolate only once two successive write times show that the spacecraft period equals the module
            // update interval
            const bool periodKnown = this->writeIntervalNanos[index] != 0;
            const bool periodDiffers = periodKnown && this->writeIntervalNanos[index] != moduleIntervalNanos;
            const bool lagged = periodKnown && !periodDiffers &&
                                isStepLagMessage(currentSimNanos, timeWrittenNanos, previousUpdateNanos);
            allLagged = allLagged && lagged;
            mismatch = mismatch ||
                       (this->rewritten[index] && timeWrittenNanos < currentSimNanos && (periodDiffers || !lagged));
        }
        this->lagConsistent = allLagged;
        if (mismatch && !this->warned) {
            this->warned = true;
            logger.bskLog(BSK_WARNING,
                          "The spacecraft state extrapolation is enabled, but a spacecraft state message was not "
                          "written at the previous module update. The module and the spacecraft are not updated with "
                          "the same task period (spacecraft slower or faster than the module): no spacecraft state "
                          "and no planet state is extrapolated.");
        }
    }

    /*! @brief Return the spacecraft state, extrapolated if prepare() found all spacecraft consistent.
     * @param scState spacecraft state message payload as read from the message
     * @param currentSimNanos [ns] current simulation time
     * @param timeWrittenNanos [ns] time the spacecraft state message was written
     * @param previousUpdateNanos [ns] time of the previous update of the calling environment module
     * @return the payload, with the position advanced to the step midpoint if the extrapolation applies
     */
    SCStatesMsgPayload apply(const SCStatesMsgPayload& scState,
                             uint64_t currentSimNanos,
                             uint64_t timeWrittenNanos,
                             uint64_t previousUpdateNanos) const
    {
        if (!this->lagConsistent) {
            return scState;
        }
        return extrapolateScStateToStepMidpoint(scState, currentSimNanos, timeWrittenNanos, previousUpdateNanos);
    }

    /*! @brief Return the planet state at the epoch the spacecraft states were extrapolated to.
     *
     * The planet is moved to the middle of the module update interval only if prepare() found the extrapolation
     * applicable to all spacecraft, so that the relative geometry of every spacecraft is evaluated at one epoch.
     * Otherwise the planet is returned as written.
     * @param planetState planet state message payload as read from the message
     * @param currentSimNanos [ns] current simulation time
     * @param timeWrittenNanos [ns] time the planet state message was written
     * @param previousUpdateNanos [ns] time of the previous update of the calling environment module
     * @return the planet payload with the position and the orientation advanced to the step midpoint
     * @note prepare() must be called first in every update.
     */
    SpicePlanetStateMsgPayload applyPlanet(const SpicePlanetStateMsgPayload& planetState,
                                           uint64_t currentSimNanos,
                                           uint64_t timeWrittenNanos,
                                           uint64_t previousUpdateNanos) const
    {
        if (!this->lagConsistent) {
            return planetState;
        }
        const uint64_t midpointNanos = previousUpdateNanos + (currentSimNanos - previousUpdateNanos) / 2; // [ns]
        return extrapolatePlanetStateToEpoch(planetState, midpointNanos, timeWrittenNanos);
    }

  private:
    bool enabled = false;                    //!< true if the spacecraft state is extrapolated
    bool warned = false;                     //!< true once the rate mismatch warning was logged
    bool lagConsistent = false;              //!< true if all spacecraft messages were written at the previous update
    std::vector<uint64_t> lastWriteNanos{};  //!< [ns] last observed write time of each state message
    std::vector<uint64_t> writeIntervalNanos{}; //!< [ns] last observed interval between two write times, 0 if unknown
    std::vector<bool> rewritten{};           //!< true if the state message was seen with more than one write time
    std::vector<bool> seen{};                //!< true if the state message was observed in a previous update
};

#endif
