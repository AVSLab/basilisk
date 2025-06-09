/*
 ISC License

 Copyright (c) 2023, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#include "dynamicObject.h"
#include "architecture/utilities/macroDefinitions.h"

void DynamicObject::setIntegrator(StateVecIntegrator* newIntegrator)
{
    if (this->isDynamicsSynced) {
        bskLogger.bskLog(
            BSK_WARNING,
            "You cannot set the integrator of a DynamicObject with synced integration. "
            "If you want to change the integrator, change the integrator of the primary "
            "DynamicObject.");
        return;
    }

    if (!newIntegrator) {
        bskLogger.bskLog(BSK_ERROR, "New integrator cannot be a null pointer");
        return;
    }

    if (newIntegrator->dynPtrs.at(0) != this) {
        bskLogger.bskLog(BSK_ERROR,
                         "New integrator must have been created using this DynamicObject");
        return;
    }

    // If there was already an integrator set, then whatever dynPtrs that the
    // original integrator had take priority over the dynPtrs of newIntegrator
    if (this->integrator) {
        newIntegrator->dynPtrs = std::move(this->integrator->dynPtrs);
    }

    delete this->integrator;

    this->integrator = newIntegrator;
}

void DynamicObject::syncDynamicsIntegration(DynamicObject* dynPtr)
{
    this->integrator->dynPtrs.push_back(dynPtr);
    dynPtr->isDynamicsSynced = true;
}

void DynamicObject::integrateState(uint64_t integrateToThisTimeNanos)
{
    if (this->isDynamicsSynced) return;

    for (const auto& dynPtr : this->integrator->dynPtrs) {
        dynPtr->preIntegration(integrateToThisTimeNanos);
    }

    this->integrator->integrate(this->timeBefore, this->timeStep);

    for (const auto& dynPtr : this->integrator->dynPtrs) {
        dynPtr->postIntegration(integrateToThisTimeNanos);
    }
}
