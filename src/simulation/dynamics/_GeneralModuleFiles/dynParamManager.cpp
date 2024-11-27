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

#include "dynParamManager.h"
#include <algorithm>
#include <iostream>

StateVector::StateVector(const StateVector& other)
{
    *this = other;
}

StateVector&
StateVector::operator=(const StateVector& other)
{
    stateMap.clear();
    for (const auto& [key, value] : other.stateMap) {
        stateMap.emplace(key, value->clone());
    }
    return *this;
}

void StateVector::setStates(const StateVector& operand)
{
    for (auto [it1, it2] = std::tuple{std::begin(this->stateMap), std::begin(operand.stateMap)};
         it1 != std::end(this->stateMap) && it2 != std::end(operand.stateMap);
         ++it1, ++it2) {
        it1->second->setState(it2->second->getState());
    }
}

void StateVector::addStates(const StateVector& operand)
{
    for (auto [it1, it2] = std::tuple{std::begin(this->stateMap), std::begin(operand.stateMap)};
         it1 != std::end(this->stateMap) && it2 != std::end(operand.stateMap);
         ++it1, ++it2) {
        it1->second->addState(*it2->second);
    }
}

void StateVector::scaleStates(double scaleFactor)
{
    for (const auto& [key, value] : stateMap) {
        value->scaleState(scaleFactor);
    }
}

void StateVector::propagateStates(double dt)
{
    for (const auto& [key, value] : stateMap) {
        value->propagateState(dt);
    }
}

StateData* DynParamManager::getStateObject(std::string stateName)
{
    if (stateContainer.stateMap.count(stateName) > 0) {
        return stateContainer.stateMap.at(stateName).get();
    }

    /*  The requested state could not be found.
        Either the state name was miss-spelled, or the state simply
        doesn't exit in the current simulaiton setup (i.e. asking for the
        hub attitude in a translation only simulation setup */
    bskLogger.bskLog(
        BSK_WARNING,
        "You requested this non-existent state name: %s You either miss-typed the stateName, or "
        "you asked for a state that doesn't exist in your simulation setup.",
        stateName.c_str());

    return nullptr;
}

void DynParamManager::updateStateVector(const StateVector& newState)
{
    this->stateContainer.setStates(newState);
}

void DynParamManager::propagateStateVector(double dt) { this->stateContainer.propagateStates(dt); }

Eigen::MatrixXd* DynParamManager::createProperty(std::string propName,
                                                 const Eigen::MatrixXd& propValue)
{
    std::map<std::string, Eigen::MatrixXd>::iterator it;
    it = dynProperties.find(propName);
    if (it == dynProperties.end()) {
        dynProperties.insert(std::pair<std::string, Eigen::MatrixXd>(propName, propValue));
    }
    else {
        bskLogger.bskLog(
            BSK_WARNING,
            "You created the dynamic property: %s more than once.  You shouldn't be doing that.",
            propName.c_str());
        it->second = propValue;
    }
    return (&(dynProperties.find(propName)->second));
}

Eigen::MatrixXd* DynParamManager::getPropertyReference(std::string propName)
{
    std::map<std::string, Eigen::MatrixXd>::iterator it;
    it = dynProperties.find(propName);
    if (it == dynProperties.end()) {
        bskLogger.bskLog(BSK_ERROR,
                         "You requested the property: %s which doesn't exist.  Null returned.",
                         propName.c_str());
        return nullptr;
    }
    else {
        return (&(it->second));
    }
}

void DynParamManager::setPropertyValue(const std::string propName, const Eigen::MatrixXd& propValue)
{
    std::map<std::string, Eigen::MatrixXd>::iterator it;
    it = dynProperties.find(propName);
    if (it == dynProperties.end()) {
        bskLogger.bskLog(BSK_ERROR,
                         "You tried to set the property value for: %s which has not been created "
                         "yet. I can't do that.",
                         propName.c_str());
    }
    else {
        it->second = propValue;
    }
}
