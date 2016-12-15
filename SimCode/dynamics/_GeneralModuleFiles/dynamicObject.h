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

/*! @brief Object that is to be used by an integrator. It's basically an interface with only one method: the F function describing a dynamic model X_dot = F(X,t)
 */
class DynamicObject : public SysModel {
public:
    DynParamManager dynManager;                       //! [-] Dynamics parameter manager for all effectors
    std::vector<StateEffector*> states;               //! [-] Vector of states for dyn-object to handle
    std::vector<DynamicEffector*> dynEffectors;       //! [-] Vector of dynamic effectors attached to dyn
	StateVecIntegrator *integrator;                   //! [-] Integrator used to propagate state forward
    
public:
    DynamicObject();
    virtual ~DynamicObject();
    virtual void UpdateState(uint64_t callTime) = 0;  //! [-] This hooks the dyn-object into Basilisk arch
    virtual void initializeDynamics();                //! [-] Method to cross-link all states
    virtual void equationsOfMotion(double t) = 0;     //! [-] Everyone will need to provide this EOM
    virtual void integrateState(double t) = 0;        //! [-] Everyone will need to integrate the state
    virtual void computeEnergyMomentum();             //! [-] User can implement NRG/moment check 
	virtual void setIntegator(StateVecIntegrator *newInt) { integrator = newInt; } //!< [-] Setter for integrator
	void addStateEffector(StateEffector *newSateEffector);   //! [-] Method to add a hinged rigid body to the stateEffector list
	void addDynamicEffector(DynamicEffector *newDynamicEffector);   //! [-] Method to add a hinged rigid body to the stateEffector list
    void setIntegrator(StateVecIntegrator *newIntegrator);

};

#endif /* DYNAMICOBJECT_H */
