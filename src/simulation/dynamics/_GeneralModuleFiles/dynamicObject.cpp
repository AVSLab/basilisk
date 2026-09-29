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
#include <algorithm>
#include <stdexcept>
#include <string>
#include <utility>

DynamicObject::~DynamicObject()
{
    if (this->integrationOwner && this->integrationOwner->integrator) {
        auto& primaryIntegrator = *this->integrationOwner->integrator;
        auto& dynamics = primaryIntegrator.dynPtrs;
        dynamics.erase(std::remove(dynamics.begin(), dynamics.end(), this), dynamics.end());
        // Cached state pointers must be rebuilt before the surviving group advances.
        primaryIntegrator.bindingPrepared = false;
    }
    if (this->integrator) {
        for (auto* object : this->integrator->dynPtrs) {
            if (object != this) {
                object->integrationOwner = nullptr;
            }
        }
    }
    // Custom integrator destructors may still use this object's states and logger.
    this->integrator.reset();
}

void DynamicObject::setIntegrator(StateVecIntegrator* newIntegrator)
{
    if (newIntegrator && newIntegrator == this->integrator.get()) {
        return;
    }

    std::unique_ptr<StateVecIntegrator> ownedIntegrator(newIntegrator);
    if (this->integrationOwner != nullptr) {
        bskLogger.bskLog(BSK_WARNING,
                         "You cannot set the integrator of a DynamicObject with synced integration. "
                         "Change the integrator of the primary DynamicObject.");
        return;
    }
    if (!ownedIntegrator) {
        bskLogger.bskError("New integrator cannot be a null pointer");
    }
    if (ownedIntegrator->dynPtrs.empty() || ownedIntegrator->dynPtrs.front() != this) {
        bskLogger.bskError("New integrator must have been created using this DynamicObject");
    }
    if (this->integrator) {
        ownedIntegrator->dynPtrs = std::move(this->integrator->dynPtrs);
    }
    this->integrator = std::move(ownedIntegrator);
}

void DynamicObject::syncDynamicsIntegration(DynamicObject* dynPtr)
{
    if (dynPtr == nullptr || dynPtr == this || this->integrationOwner != nullptr || !this->integrator) {
        bskLogger.bskError("Synchronization requires two independent DynamicObjects with integrators");
    }
    if (dynPtr->integrationOwner == this) {
        return;
    }
    if (dynPtr->integrationOwner != nullptr || !dynPtr->integrator || dynPtr->integrator->dynPtrs.size() != 1) {
        bskLogger.bskError("Synchronization requires two independent DynamicObjects with integrators");
    }
    if (this->integrator->bindingPrepared || dynPtr->integrator->bindingPrepared) {
        bskLogger.bskError("Configure synchronized dynamics before their first integration step");
    }
    this->integrator->dynPtrs.push_back(dynPtr);
    dynPtr->integrationOwner = this;
}

void DynamicObject::integrateState(uint64_t integrateToThisTimeNanos)
{
    if (this->integrationOwner != nullptr) {
        return;
    }
    for (DynamicObject* object : this->integrator->dynPtrs) {
        object->preIntegration(integrateToThisTimeNanos);
    }
    this->integrator->integrate(this->timeBefore, this->timeStep);
    for (DynamicObject* object : this->integrator->dynPtrs) {
        object->postIntegration(integrateToThisTimeNanos);
    }
}

DynamicObject* DynamicObject::getIntegrationOwner() const noexcept
{
    return this->integrationOwner;
}

StateVecIntegrator* DynamicObject::getIntegrator() const noexcept
{
    return this->integrator.get();
}

void DynamicObject::requireMutableTopology(const char* operation) const
{
    if (this->dynManager.statesAreFinalized()) {
        throw std::logic_error(std::string(operation) + " cannot change finalized state topology");
    }
}
