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

#ifndef DYNAMICOBJECT_H
#define DYNAMICOBJECT_H

#include <vector>
#include <stdint.h>
#include "dynParamManager.h"
#include "stateEffector.h"
#include "dynamicEffector.h"
#include "stateVecIntegrator.h"
#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/utilities/bskLogging.h"

/*! @brief dynamic object class */
class DynamicObject : public SysModel {
public:
    DynParamManager dynManager;                       //!< -- Dynamics parameter manager for all effectors
    StateVecIntegrator *integrator;                   //!< -- Integrator used to propagate state forward
    BSKLogger bskLogger;                      //!< -- BSK Logging

public:
    virtual ~DynamicObject() = default;               //!< -- Destructor
    virtual void initializeDynamics();                //!< -- Initializes the dynamics and variables
    virtual void computeEnergyMomentum(double t);     //!< -- Method to compute energy and momentum of the system
    virtual void UpdateState(uint64_t callTime) = 0;  //!< -- This hooks the dyn-object into Basilisk architecture
    virtual void equationsOfMotion(double t, double timeStep) = 0;     //!< -- This is computing F = Xdot(X,t)
    void integrateState(double t);                    //!< -- This method steps the state forward in time
    void setIntegrator(StateVecIntegrator *newIntegrator);  //!< -- Sets a new integrator
    virtual void preIntegration(double callTime) = 0;       //!< -- method to perform pre-integration steps
    virtual void postIntegration(double callTime) = 0;      //!< -- method to perform post-integration steps
    void syncDynamicsIntegration(DynamicObject *dynPtr);    //!< add another DynamicObject to be intregated simultaneously
    bool isDynamicsSynced = false;                    //!< flag indicating that another spacecraft object is controlling the integration
    double timeStep;                                  //!< [s] integration time step
    double timeBefore;                                //!< [s] prior time value

};


#endif /* DYNAMICOBJECT_H */
