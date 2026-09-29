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

#include "MJQuaternionStatePolicy.h"

#include <mujoco/mujoco.h>

#include <cmath>
#include <stdexcept>

namespace {
void
requireQuaternionSpec(const StateSpec& spec, uint32_t derivativeRows)
{
    const MatrixShape quaternion{ 4, 1 };
    const MatrixShape tangent{ 3, 1 };
    if (spec.state != quaternion || spec.derivative != MatrixShape{ derivativeRows, 1 } ||
        spec.diffusionTangent != tangent) {
        throw std::invalid_argument("A MuJoCo quaternion state requires shapes 4x1, " + std::to_string(derivativeRows) +
                                    "x1, and 3x1.");
    }
    if (spec.updateKind != StateUpdateKind::Special) {
        throw std::invalid_argument("A MuJoCo quaternion requires a special state update policy.");
    }
}

void
integrateQuaternion(MutableMatrixView quaternion, ConstMatrixView angularVelocity, double timeStep)
{
    mju_quatIntegrate(quaternion.data(), angularVelocity.data(), timeStep);
}

void
normalizeQuaternion(MutableMatrixView quaternion)
{
    const double norm = std::sqrt(quaternion.squaredNorm());
    if (norm > 0.0) {
        quaternion /= norm;
    }
}
}

bool
MJNativeQuaternionStatePolicy::topologyEquals(const StateUpdatePolicy& other) const
{
    return dynamic_cast<const MJNativeQuaternionStatePolicy*>(&other) != nullptr;
}

void
MJNativeQuaternionStatePolicy::validate(const StateSpec& spec) const
{
    requireQuaternionSpec(spec, 3);
}

void
MJNativeQuaternionStatePolicy::buildDriftCandidate(ConstMatrixView base,
                                                   ConstMatrixView combinedDrift,
                                                   double timeStep,
                                                   MutableMatrixView output) const
{
    output = base;
    integrateQuaternion(output, combinedDrift, timeStep);
}

void
MJNativeQuaternionStatePolicy::applyNoiseIncrement(MutableMatrixView state,
                                                   ConstMatrixView diffusionTangent,
                                                   double pseudoStep) const
{
    integrateQuaternion(state, diffusionTangent, pseudoStep);
}

bool
MJHighOrderQuaternionStatePolicy::topologyEquals(const StateUpdatePolicy& other) const
{
    return dynamic_cast<const MJHighOrderQuaternionStatePolicy*>(&other) != nullptr;
}

void
MJHighOrderQuaternionStatePolicy::validate(const StateSpec& spec) const
{
    requireQuaternionSpec(spec, 4);
}

void
MJHighOrderQuaternionStatePolicy::buildDriftCandidate(ConstMatrixView base,
                                                      ConstMatrixView combinedDrift,
                                                      double timeStep,
                                                      MutableMatrixView output) const
{
    output = base;
    output += combinedDrift * timeStep;
    normalizeQuaternion(output);
}

void
MJHighOrderQuaternionStatePolicy::applyNoiseIncrement(MutableMatrixView state,
                                                      ConstMatrixView diffusionTangent,
                                                      double pseudoStep) const
{
    integrateQuaternion(state, diffusionTangent, pseudoStep);
}
