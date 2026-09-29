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
#include "svStochasticIntegratorMayurama.h"

void
svStochasticIntegratorMayurama::integrateImpl(double currentTime, double timeStep)
{
    // Initialization binds topology and scratch but consumes no random sample.
    if (timeStep == 0.0) {
        return;
    }

    this->gatherStochasticStates();
    this->generateWienerNoise(timeStep);

    try {
        this->evaluateDerivatives(currentTime, timeStep);
        this->gatherStochasticDerivatives();

        this->evaluateDiffusions(currentTime, timeStep);
        if (this->stochasticUpdatesAreAllEuclidean()) {
            this->advanceEuclideanEulerMaruyama(timeStep, this->flatDW());
        } else {
            this->gatherStochasticDiffusions();
            this->buildEulerMaruyamaCandidate(timeStep, this->flatDW());
        }
    } catch (...) {
        this->restoreStochasticStates();
        throw;
    }
}
