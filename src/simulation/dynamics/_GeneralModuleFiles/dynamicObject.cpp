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

#include "dynamicObject.h"

/*! This is the constructor, just setting the variables to zero */
DynamicObject::DynamicObject()
{
    return;
}

/*! This is the destructor, nothing to report here */
DynamicObject::~DynamicObject()
{
    return;
}

/*! This method initializes the stateEffectors and dynamicEffectors and links the necessarry components together */
void DynamicObject::initializeDynamics()
{
    return;
}

/*! This method allows a dynamicObject to compute energy and momentum. Great for sim validation purposes */
void DynamicObject::computeEnergyMomentum(double t)
{
    return;
}

/*! This method changes the integrator in use (Default integrator: RK4) */
void DynamicObject::setIntegrator(StateVecIntegrator *newIntegrator)
{
    if (newIntegrator != nullptr) {
        delete integrator;
        integrator = newIntegrator;
    }

    return;
}


/*! This method is used to connect the integration of another DynamicObject
    to the integration of this spacecraft module */
void DynamicObject::syncDynamicsIntegration(DynamicObject *dynPtr)
{
    this->integrator->dynPtrs.push_back(dynPtr);
    dynPtr->isDynamicsSynced = true;
}
