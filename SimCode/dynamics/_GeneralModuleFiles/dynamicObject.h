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

#ifndef DYNAMICOBJECT_H
#define DYNAMICOBJECT_H

#include "dynParamManager.h"
#include "stateEffector.h"
#include "dynamicEffector.h"
#include "stateVecIntegrator.h"
#include "_GeneralModuleFiles/sys_model.h"
#include <vector>
#include <stdint.h>

/*! @brief Object that is to be used by an integrator. This holds the equations of motion, integrate state, energy and
    momentum calculations. dynamicObject is what puts all of the pieces together for your system */
class DynamicObject : public SysModel {
public:
    DynParamManager dynManager;                       //!< -- Dynamics parameter manager for all effectors
	StateVecIntegrator *integrator;                   //!< -- Integrator used to propagate state forward
    std::vector<StateEffector*> states;               //!< -- Vector of state effectors attached to dynObject
    std::vector<DynamicEffector*> dynEffectors;       //!< -- Vector of dynamic effectors attached to dynObject
    
public:
    DynamicObject();
    virtual ~DynamicObject();
    virtual void initializeDynamics();
    virtual void computeEnergyMomentum(double t);
    virtual void addStateEffector(StateEffector *newSateEffector) = 0;
    virtual void addDynamicEffector(DynamicEffector *newDynamicEffector) = 0;
    virtual void setIntegrator(StateVecIntegrator *newIntegrator) = 0;
    virtual void UpdateState(uint64_t callTime) = 0;  //!< -- This hooks the dyn-object into Basilisk architecture
    virtual void equationsOfMotion(double t) = 0;     //!< -- This is computing F = Xdot(X,t)
    virtual void integrateState(double t) = 0;        /*!< -- This is where the integration call happens, steps the
                                                       state forward in time */
};

#endif /* DYNAMICOBJECT_H */
