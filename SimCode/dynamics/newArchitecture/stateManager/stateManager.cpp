/*
 Copyright (c) 2016, Autonomous Vehicle Systems Lab, Univeristy of Colorado at Boulder
 
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


#include "stateManager.h"
#include <iostream>

StateManager::StateManager()
{
    return;
}

StateManager::~StateManager()
{
    return;
}

StateData* StateManager::registerState(uint32_t nRow, uint32_t nCol,
    std::string stateName)
{
    std::map<std::string, StateData>::iterator it;
    it = stateContainer.stateMap.find(stateName);
    if(it != stateContainer.stateMap.end())
    {
        std::cout << "WARNING: You created a state with the name: " <<stateName;
        std::cout << " more than once.  Go ahead and don't do this.";
        std::cout << std::endl;
        if(it->second.getRowSize() != nRow || it->second.getColumnSize() != nCol)
        {
            std::cout << "ERROR: In addition to that, you tried to change the ";
            std::cout << "size of the state in question.  Come on.  You get null";
            std::cout << std::endl;
            return nullptr;
        }
    }
    else
    {
        Eigen::MatrixXd stateMatrix;
        stateMatrix.resize(nRow, nCol);
        StateData newState(stateName, stateMatrix);
        newState.zeroDerivative();
        stateContainer.stateMap.insert(std::pair<std::string, StateData>
                              (stateName, newState));
        it = stateContainer.stateMap.find(stateName);
    }
    return (&(it->second));
}

StateData* StateManager::getStateObject(std::string stateName)
{
    StateData *statePtr;
    std::map<std::string, StateData>::iterator it;
    
    statePtr = nullptr;
    it = stateContainer.stateMap.find(stateName);
    if(it != stateContainer.stateMap.end())
    {
        statePtr = &(it->second);
    }
    
    return(statePtr);
}

StateVector StateManager::getStateVector()
{
    return(stateContainer);
}

void StateManager::setStateVector(const StateVector & newState)
{
    stateContainer = newState;
}

void StateManager::zeroContributions()
{
    std::map<std::string, StateData>::iterator it;
    for(it = stateContainer.stateMap.begin();
        it != stateContainer.stateMap.end(); it++)
    {
        it->second.zeroDerivative();
    }
}