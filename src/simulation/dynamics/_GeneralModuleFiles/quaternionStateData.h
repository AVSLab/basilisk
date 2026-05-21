/*
 ISC License

 Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#ifndef QUATERNION_STATE_DATA_H
#define QUATERNION_STATE_DATA_H

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "stateData.h"

/**
 * @brief A `StateData` whose value is a unit quaternion advanced from a body
 * angular velocity.
 *
 * `state` is a 4x1 unit quaternion in `(w, x, y, z)` order.
 * `stateDeriv` is a 3x1 body angular velocity in rad/s.
 * Each `stateDiffusion` entry, when present, must be a 3x1 rotational
 * increment vector that is composed into the quaternion with the matching
 * pseudo-step.
 *
 * Sizes intentionally differ — the standard Euler step `state += deriv * dt`
 * is not valid on SO(3).  `propagateState` instead applies the exact
 * exponential-map integrator and renormalizes so `|q| = 1` after each step.
 */
class QuaternionStateData : public StateData
{
  public:
    /** Constructs a 4x1 quaternion state with a 3x1 derivative. */
    QuaternionStateData(std::string inName, const Eigen::MatrixXd& newState);

    /** Returns a deep copy of this state. */
    std::unique_ptr<StateData> clone() const override;

    /**
     * @brief Integrates `state` over `dt` using the body angular velocity in
     * `stateDeriv`.  Composes the current quaternion with
     * `exp(0.5 * dt * omega)`, applies any stochastic rotational diffusion,
     * and renormalizes so `|q| = 1` after each step.
     */
    void propagateState(double dt, std::vector<double> pseudoStep = {}) override;
};

#endif
