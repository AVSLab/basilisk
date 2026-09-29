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

#include "stateVecIntegrator.h"
#include "dynamicObject.h"

StateVecIntegrator::StateVecIntegrator(DynamicObject* dynIn) : dynPtrs{dynIn}
{
}

void StateVecIntegrator::integrate(double currentTime, double timeStep)
{
    this->ensureIntegrationBinding();
    this->integrateImpl(currentTime, timeStep);
}

void StateVecIntegrator::ensureIntegrationBinding()
{
    if (!this->bindingPrepared) {
        this->prepareIntegrationBinding();
        this->bindingPrepared = true;
    } else {
        this->validateIntegrationBinding();
    }
}

void StateVecIntegrator::evaluateDerivatives(double time, double timeStep)
{
    for (DynamicObject* object : this->dynPtrs) {
        object->equationsOfMotion(time, timeStep);
    }
}

void StateVecIntegrator::evaluateDiffusions(double time, double timeStep)
{
    for (DynamicObject* object : this->dynPtrs) {
        object->equationsOfMotionDiffusion(time, timeStep);
    }
}
