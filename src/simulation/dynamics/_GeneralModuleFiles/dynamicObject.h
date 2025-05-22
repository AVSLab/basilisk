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

#ifndef DYNAMICOBJECT_H
#define DYNAMICOBJECT_H

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/utilities/bskLogging.h"
#include "dynamicEffector.h"
#include "dynParamManager.h"
#include "stateEffector.h"
#include "stateVecIntegrator.h"
#include <stdint.h>
#include <vector>

/** A DynamicObject is a Basilisk model with states that must be integrated */
class DynamicObject : public SysModel {
  public:
    DynParamManager dynManager;     /**< Dynamics parameter manager for all effectors */
    StateVecIntegrator* integrator; /**< Integrator used to propagate state forward */
    BSKLogger bskLogger;            /**< BSK Logging */

  public:
    DynamicObject() = default;
    DynamicObject(const DynamicObject&) = delete;
    DynamicObject& operator=(const DynamicObject&) = delete;
    DynamicObject(DynamicObject&&) = delete;
    DynamicObject& operator=(DynamicObject&&) = delete;
    virtual ~DynamicObject() = default;

    /** Hooks the dyn-object into Basilisk architecture */
    virtual void UpdateState(uint64_t callTime) = 0;

    /** Computes F = Xdot(X,t) */
    virtual void equationsOfMotion(double t, double timeStep) = 0;

    /** Performs pre-integration steps */
    virtual void preIntegration(uint64_t callTimeNanos) = 0;

    /** Performs post-integration steps */
    virtual void postIntegration(uint64_t callTimeNanos) = 0;

    /** Initializes the dynamics and variables */
    virtual void initializeDynamics(){};

    /** Computes energy and momentum of the system */
    virtual void computeEnergyMomentum(double t){};

    /** Prepares the dynamic object to be integrated, integrates the states
     * forward in time, and finally performs the post-integration steps.
     *
     * This is only done if the DynamicObject integration is not sync'd to another DynamicObject
     */
    void integrateState(uint64_t t);

    /** Sets a new integrator in use */
    void setIntegrator(StateVecIntegrator* newIntegrator);

    /** Connects the integration of a DynamicObject to the integration of this DynamicObject. */
    void syncDynamicsIntegration(DynamicObject* dynPtr);

  public:
    /** flag indicating that another spacecraft object is controlling the integration */
    bool isDynamicsSynced = false;
    double timeStep = 0.0;   /**< [s] integration time step */
    double timeBefore = 0.0; /**< [s] prior time value */
    uint64_t timeBeforeNanos = 0; /**< [ns] prior time value */
};

#endif /* DYNAMICOBJECT_H */
