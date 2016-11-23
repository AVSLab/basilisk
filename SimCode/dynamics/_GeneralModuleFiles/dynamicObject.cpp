/*
 ISC License

 Copyright (c) 2016-2017, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

DynamicObject::DynamicObject()
{
    return;
}


DynamicObject::~DynamicObject()
{
    return;
}

void DynamicObject::computeEnergyMomentum()
{
    return;
}

void DynamicObject::initializeDynamics()
{
    std::vector<StateEffector*>::iterator it;
    std::vector<DynamicEffector*>::iterator dynIt;
    
    for(it = states.begin(); it != states.end(); it++)
    {
        (*it)->registerStates(dynManager);
    }
    
    for(it = states.begin(); it != states.end(); it++)
    {
        (*it)->linkInStates(dynManager);
    }

    for(dynIt = dynEffectors.begin(); dynIt != dynEffectors.end(); dynIt++)
    {
        (*dynIt)->linkInStates(dynManager);
    }

    return;
}

void DynamicObject::addStateEffector(StateEffector *newStateEffector)
{
	this->states.push_back(newStateEffector);

    return;
}

void DynamicObject::addDynamicEffector(DynamicEffector *newDynamicEffector)
{
    this->dynEffectors.push_back(newDynamicEffector);

    return;
}
