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
#include <iostream>

DynParamManager::DynParamManager()
{
    return;
}

DynParamManager::~DynParamManager()
{
    return;
}

StateData* DynParamManager::registerState(uint32_t nRow, uint32_t nCol,
    std::string stateName)
{
    if (stateName == "") {
        bskLogger.bskLog(BSK_ERROR, "Your state name can't be an empty string.  Come on.  You get null.");
        return nullptr;
    }

    std::map<std::string, StateData>::iterator it;
    it = stateContainer.stateMap.find(stateName);
    if(it != stateContainer.stateMap.end())
    {
        bskLogger.bskLog(BSK_WARNING, "You created a state with the name: %s more than once.  Go ahead and don't do this.", stateName.c_str());
        if(it->second.getRowSize() != nRow || it->second.getColumnSize() != nCol)
        {
            bskLogger.bskLog(BSK_ERROR, "In addition to that, you tried to change the size of the state in question.  Come on.  You get null.");
            return nullptr;
        }
    }
    else
    {
        Eigen::MatrixXd stateMatrix;
        stateMatrix.resize(nRow, nCol);
        StateData newState(stateName, stateMatrix);
        stateContainer.stateMap.insert(std::pair<std::string, StateData>
                              (stateName, newState));
        it = stateContainer.stateMap.find(stateName);
    }
    return (&(it->second));
}

StateData* DynParamManager::getStateObject(std::string stateName)
{
    StateData *statePtr;
    std::map<std::string, StateData>::iterator it;
    
    statePtr = nullptr;
    it = stateContainer.stateMap.find(stateName);
    if(it != stateContainer.stateMap.end())
    {
        statePtr = &(it->second);
    }

    if (statePtr == nullptr)
    {
        /*  The requested state could not be found.
            Either the state name was miss-spelled, or the state simply
            doesn't exit in the current simulaiton setup (i.e. asking for the
            hub attitude in a translation only simulation setup */
        bskLogger.bskLog(BSK_WARNING, "You requested this non-existent state name: %s You either miss-typed the stateName, or you asked for a state that doesn't exist in your simulation setup. stateName = ", stateName.c_str());
    }
    
    return(statePtr);
}

StateVector DynParamManager::getStateVector()
{
    return(stateContainer);
}

void DynParamManager::updateStateVector(const StateVector & newState)
{

    std::map<std::string, StateData>::iterator it;
    std::map<std::string, StateData>::const_iterator inIt;
    for (it = stateContainer.stateMap.begin(), inIt = newState.stateMap.begin();
         it != stateContainer.stateMap.end() && inIt != newState.stateMap.end(); it++, inIt++)
    {
        it->second.setState(inIt->second.getState());

    }
}

void DynParamManager::propagateStateVector(double dt)
{
    std::map<std::string, StateData>::iterator it;
    for (it = stateContainer.stateMap.begin();
         it != stateContainer.stateMap.end(); it++)
    {
        it->second.propagateState(dt);

    }
}

StateVector StateVector::operator+(const StateVector& operand)
{
    std::map<std::string, StateData>::iterator it;
    std::map<std::string, StateData>::const_iterator opIt;
    StateVector outVector;
    for (it = stateMap.begin(), opIt = operand.stateMap.begin();
         it != stateMap.end() && opIt != operand.stateMap.end(); it++, opIt++)
    {
        StateData newState = it->second + opIt->second;
        outVector.stateMap.insert(std::pair<std::string, StateData>
                                  (it->first, newState));

    }
    return outVector;
}

StateVector StateVector::operator*(double scaleFactor)
{
    StateVector outVector;
    std::map<std::string, StateData>::iterator it;
    for (it = stateMap.begin(); it != stateMap.end(); it++)
    {
        outVector.stateMap.insert(std::pair<std::string, StateData>
                                  (it->first, it->second*scaleFactor));

    }

    return outVector;
}

Eigen::MatrixXd* DynParamManager::createProperty(std::string propName,
    const Eigen::MatrixXd & propValue)
{
    std::map<std::string, Eigen::MatrixXd>::iterator it;
    it = dynProperties.find(propName);
    if(it == dynProperties.end())
    {
        dynProperties.insert(std::pair<std::string, Eigen::MatrixXd>
                             (propName, propValue));
    }
    else{
        bskLogger.bskLog(BSK_WARNING, "You created the dynamic property: %s more than once.  You shouldn't be doing that.", propName.c_str());
        it->second = propValue;
    }
    return(&(dynProperties.find(propName)->second));
}

Eigen::MatrixXd* DynParamManager::getPropertyReference(std::string propName)
{
    std::map<std::string, Eigen::MatrixXd>::iterator it;
    it = dynProperties.find(propName);
    if(it == dynProperties.end())
    {
        bskLogger.bskLog(BSK_ERROR, "You requested the property: %s which doesn't exist.  Null returned.", propName.c_str());
        return nullptr;
    }
    else
    {
        return(&(it->second));
    }
}

void DynParamManager::setPropertyValue(const std::string propName,
                      const Eigen::MatrixXd & propValue)
{
    std::map<std::string, Eigen::MatrixXd>::iterator it;
    it = dynProperties.find(propName);
    if(it == dynProperties.end())
    {
        bskLogger.bskLog(BSK_ERROR, "You tried to set the property value for: %s which has not been created yet. I can't do that.", propName.c_str());
    }
    else
    {
        it->second = propValue;
    }
}
