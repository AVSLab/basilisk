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

/**
 * @file dynParamManager.cpp
 * @brief DynParamManager facade and named property operations.
 * State lifecycle operations delegate to the owned registry.
 */

#include "dynParamManager.h"
#include "stateRegistry.h"

#include <stdexcept>
#include <utility>

DynParamManager::DynParamManager()
  : registry(new StateRegistry())
{
}

DynParamManager::~DynParamManager() = default;

void
DynParamManager::finalizeStates()
{
    this->registry->finalizeStates();
}

StateData*
DynParamManager::registerState(std::string stateName, const StateSpec& spec)
{
    return this->registry->registerState(std::move(stateName), spec);
}

StateData*
DynParamManager::registerState(std::string stateName, const StateSpec& spec, std::unique_ptr<StateUpdatePolicy> policy)
{
    return this->registry->registerState(std::move(stateName), spec, std::move(policy));
}

StateData*
DynParamManager::registerState(uint32_t nRow, uint32_t nCol, std::string stateName)
{
    return this->registry->registerState(nRow, nCol, std::move(stateName));
}

StateData*
DynParamManager::getStateObject(std::string stateName)
{
    StateData* state = this->registry->getStateObject(stateName);
    if (state == nullptr) {
        this->bskLogger.bskLog(BSK_WARNING, "You requested this non-existent state name: %s.", stateName.c_str());
    }
    return state;
}

void
DynParamManager::registerSharedNoiseSource(std::vector<std::pair<const StateData&, size_t>> sharedNoises)
{
    this->registry->registerSharedNoiseSource(std::move(sharedNoises));
}

bool
DynParamManager::statesAreFinalized() const noexcept
{
    return this->registry->statesAreFinalized();
}

Eigen::MatrixXd*
DynParamManager::createProperty(std::string propName, const Eigen::MatrixXd& propValue)
{
    auto [property, inserted] = this->dynProperties.emplace(std::move(propName), propValue);
    if (!inserted) {
        property->second = propValue;
    }
    return &property->second;
}

Eigen::MatrixXd*
DynParamManager::getPropertyReference(std::string propName)
{
    auto property = this->dynProperties.find(propName);
    if (property == this->dynProperties.end()) {
        this->bskLogger.bskError("You requested the property: %s which doesn't exist.", propName.c_str());
    }
    return &property->second;
}

void
DynParamManager::setPropertyValue(const std::string propName, const Eigen::MatrixXd& propValue)
{
    auto property = this->dynProperties.find(propName);
    if (property == this->dynProperties.end()) {
        this->bskLogger.bskError("You tried to set property: %s before creating it.", propName.c_str());
    }
    if (property->second.rows() != propValue.rows() || property->second.cols() != propValue.cols()) {
        throw std::invalid_argument("Property '" + propName + "' shape mismatch.");
    }
    property->second = propValue;
}
