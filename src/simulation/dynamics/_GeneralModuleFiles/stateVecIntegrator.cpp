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
#include <algorithm>
#include <exception>

StateVecIntegrator::StateVecIntegrator(DynamicObject* dynIn)
    : dynPtrs{ dynIn }
{
}

void StateVecIntegrator::integrate(double currentTime, double timeStep, std::optional<uint64_t> endTick)
{
    this->scheduledEndTick = endTick;
    this->ensureIntegrationBinding();
    // Stochastic methods skip their numerical stages and rollback buffers on zero-duration steps.
    if (timeStep == 0.0) {
        this->validateFinalStates();
    }
    this->integrateImpl(currentTime, timeStep);
}

void StateVecIntegrator::validateFinalStates() const
{
    for (DynamicObject* object : this->dynPtrs) {
        object->validateIntegratedState();
    }
}

void StateVecIntegrator::restoreOutputsAfterRollback() const
{
    for (DynamicObject* object : this->dynPtrs) {
        try {
            object->restoreAfterFailedStep();
        } catch (const std::exception& error) {
            object->bskLogger.bskLog(BSK_WARNING, "Failed to restore dynamics output: %s", error.what());
        }
    }
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

void StateVecIntegrator::prepareConstraintEndpoints(double time)
{
    for (DynamicObject* object : this->dynPtrs) {
        object->prepareConstraintEndpoints(time);
    }
}

void StateVecIntegrator::evaluateDerivatives(double time, double timeStep)
{
    this->prepareConstraintEndpoints(time);
    for (DynamicObject* object : this->dynPtrs) {
        object->equationsOfMotion(time, timeStep);
    }
}

void StateVecIntegrator::evaluateFixedStepDerivatives(double startTime, double timeStep, double fraction)
{
    if (this->scheduledEndTick) {
        // After years of simulation, distinct RK stage ticks can round to the same double in seconds.
        const auto start = this->dynPtrs.front()->timeBeforeNanos;
        const auto end = *this->scheduledEndTick;
        if (fraction == 0.0) {
            this->fixedStageTick = start;
        } else if (fraction == 0.5) {
            const auto first = std::min(start, end);
            this->fixedStageTick = first + (std::max(start, end) - first) / 2;
        } else if (fraction == 1.0) {
            this->fixedStageTick = end;
        }
    }
    struct ClearStageTick
    {
        std::optional<uint64_t>& tick;
        ~ClearStageTick() { tick.reset(); }
    } clear{ this->fixedStageTick };
    this->evaluateDerivatives(startTime + fraction * timeStep, timeStep);
}

void StateVecIntegrator::evaluateDiffusions(double time, double timeStep)
{
    this->prepareConstraintEndpoints(time);
    for (DynamicObject* object : this->dynPtrs) {
        object->equationsOfMotionDiffusion(time, timeStep);
    }
}
