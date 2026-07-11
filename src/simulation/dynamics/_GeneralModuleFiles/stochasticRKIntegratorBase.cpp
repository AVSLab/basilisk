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
#include "stochasticRKIntegratorBase.h"

const std::vector<StateIdToIndexMap>& StochasticRKIntegratorBase::noiseIndexMaps()
{
    if (!this->noiseIndexMapsCached) {
        this->cachedNoiseIndexMaps = this->getStateIdToNoiseIndexMaps();
        this->noiseIndexMapsCached = true;
    }
    return this->cachedNoiseIndexMaps;
}

ExtendedStateVector StochasticRKIntegratorBase::computeDerivatives(double time, double timeStep)
{
    for (auto dynPtr : this->dynPtrs) {
        dynPtr->equationsOfMotion(time, timeStep);
    }
    return ExtendedStateVector::fromStateDerivs(this->dynPtrs);
}

ExtendedStateVector StochasticRKIntegratorBase::computeDiffusion(
    double time, double timeStep, const StateIdToIndexMap& stateIdToNoiseIndexMap)
{
    for (auto dynPtr : this->dynPtrs) {
        dynPtr->equationsOfMotionDiffusion(time, timeStep);
    }
    return ExtendedStateVector::fromStateDiffusions(this->dynPtrs, stateIdToNoiseIndexMap);
}

std::vector<ExtendedStateVector> StochasticRKIntegratorBase::computeDiffusions(
    double time, double timeStep, const std::vector<StateIdToIndexMap>& stateIdToNoiseIndexMaps)
{
    for (auto dynPtr : this->dynPtrs) {
        dynPtr->equationsOfMotionDiffusion(time, timeStep);
    }
    return ExtendedStateVector::fromStateDiffusions(this->dynPtrs, stateIdToNoiseIndexMaps);
}
