/*
 ISC License

 Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

 Permission to use, copy, modify, and/or distribute this software for any
 purpose with or without fee is hereby granted, provided that the above
 copyright notice and this permission notice appear in all copies.

 THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES WHATSOEVER
 RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF
 CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN
 CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#ifndef MJQUATERNION_STATE_POLICY_H
#define MJQUATERNION_STATE_POLICY_H

#include "simulation/dynamics/_GeneralModuleFiles/stateData.h"

/** @brief MuJoCo exponential-map update for a scalar-first quaternion.
 * State shape is 4x1; drift and diffusion tangents are 3x1 body angular increments.
 * Drift construction starts from the supplied base and calls MuJoCo's quaternion
 * integration rule. Ordered noise increments use the same rule in place.
 * The registry owns the stateless policy and compares its concrete type on Reset.
 */
class MJNativeQuaternionStatePolicy final : public StateUpdatePolicy
{
  public:
    /** @brief Accept another native quaternion policy as the same immutable topology. */
    bool topologyEquals(const StateUpdatePolicy& other) const override;
    /** @brief Require 4x1 state, 3x1 drift/tangent, and special-policy dispatch. */
    void validate(const StateSpec& spec) const override;
    /** @brief Exponentiate the weighted body-rate drift from the base quaternion. */
    void buildDriftCandidate(ConstMatrixView base,
                             ConstMatrixView combinedDrift,
                             double timeStep,
                             MutableMatrixView output) const override;
    /** @brief Apply one angular diffusion increment through the exponential map. */
    void applyNoiseIncrement(MutableMatrixView state,
                             ConstMatrixView diffusionTangent,
                             double pseudoStep) const override;
};

/** @brief RK update using a four-component quaternion derivative, followed by normalization.
 * State and drift shapes are 4x1. Diffusion remains a 3x1 angular tangent and uses
 * MuJoCo's exponential-map update.
 */
class MJHighOrderQuaternionStatePolicy final : public StateUpdatePolicy
{
  public:
    /** @brief Accept another high-order quaternion policy as the same immutable topology. */
    bool topologyEquals(const StateUpdatePolicy& other) const override;
    /** @brief Require 4x1 state/drift, 3x1 tangent, and special-policy dispatch. */
    void validate(const StateSpec& spec) const override;
    /** @brief Add the weighted quaternion derivative and normalize the result. */
    void buildDriftCandidate(ConstMatrixView base,
                             ConstMatrixView combinedDrift,
                             double timeStep,
                             MutableMatrixView output) const override;
    /** @brief Apply one angular diffusion increment through the exponential map. */
    void applyNoiseIncrement(MutableMatrixView state,
                             ConstMatrixView diffusionTangent,
                             double pseudoStep) const override;
};

#endif
