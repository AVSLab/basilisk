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

/** @file dynamicObject.h
 * @brief Model lifecycle, synchronized integration, and protected dynamics extension hooks.
 */

#ifndef DYNAMICOBJECT_H
#define DYNAMICOBJECT_H

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/utilities/bskLogging.h"
#include "dynParamManager.h"
#include "dynamicEffector.h"
#include "stateEffector.h"
#include "stateVecIntegrator.h"
#include <memory>
#include <stdint.h>
#include <vector>

/**
 * @brief A model that owns continuous states and coordinates their integration.
 *
 * Reset registers and initializes states, then calls DynParamManager::finalizeStates().
 * Dynamics callbacks write drift and diffusion; the integrator advances the values
 * in fixed contiguous storage. One primary object can integrate several synchronized
 * objects. Configure this group before its first step. Python retains synchronized
 * secondaries; native connections are detached when either object is destroyed.
 * Reset and integration exceptions propagate to callers.
 */
class DynamicObject : public SysModel {
    friend class StateVecIntegrator;

  public:
    DynParamManager dynManager;     /**< Dynamics parameter manager for all effectors */
    BSKLogger bskLogger;            /**< BSK Logging */

  public:
    DynamicObject() = default;
    DynamicObject(const DynamicObject&) = delete;
    DynamicObject& operator=(const DynamicObject&) = delete;
    DynamicObject(DynamicObject&&) = delete;
    DynamicObject& operator=(DynamicObject&&) = delete;
    virtual ~DynamicObject();

    /** Hooks the dyn-object into Basilisk architecture */
    virtual void UpdateState(uint64_t callTime) = 0;

    /** Computes the time derivative of the states:
     *
     * \f[
     *     dx = f(t,x)\,dt
     * \f]
     *
     * ``equationsOfMotion`` computes \f$f(t,x)\f$ in the equation above.
     */
    virtual void equationsOfMotion(double t, double timeStep) = 0;

    /** Computes the diffusion of the states:
     *
     * \f[
     *     dx = f(t,x)\,dt + g_0(t,x)\,dW_0 + g_1(t,x)\,dW_1 + \cdots + g_{n-1}(t,x)\,dW_{n-1}
     * \f]
     *
     * ``equationsOfMotionDiffusion`` is equivalent to evaluating
     * \f$g_0(t,x), g_1(t,x), \ldots, g_{n-1}(t,x)\f$ in the equation above.
     *
     * Note that not all ``DynamicObjects`` may support this functionality.
     */
    virtual void equationsOfMotionDiffusion(double t [[maybe_unused]], double timeStep [[maybe_unused]]) {
    };

    /** Performs pre-integration steps */
    virtual void preIntegration(uint64_t callTimeNanos) = 0;

    /** Performs post-integration steps */
    virtual void postIntegration(uint64_t callTimeNanos) = 0;

    /** Computes energy and momentum of the system */
    virtual void computeEnergyMomentum(double t [[maybe_unused]]){
    };

    /** Prepares the dynamic object to be integrated, integrates the states
     * forward in time, and finally performs the post-integration steps.
     *
     * This is only done if the DynamicObject integration is not sync'd to another DynamicObject
     */
    void integrateState(uint64_t t);

    /** @brief Take ownership of a replacement integrator.
     * @param newIntegrator Integrator constructed for this object, with ownership
     * available for transfer, or the currently active integrator.
     * @note A newly supplied integrator is destroyed if rejected. Passing the active
     * integrator again is a no-op. Replacement invalidates borrowed integrator pointers.
     * C++ callers must not pass an integrator owned by another object.
     */
    void setIntegrator(StateVecIntegrator* newIntegrator);

    /** @brief Borrow the active integrator until its replacement or this object's destruction.
     * @return Active integrator, or nullptr if no integrator has been installed.
     */
    StateVecIntegrator* getIntegrator() const noexcept;

    /** @brief Integrate another dynamics object with this object's numerical method.
     * @param dynPtr Secondary object; must be independent, non-null, and different from this object.
     * @note Repeating an existing connection is a no-op. Configure new connections
     * before the first integration step and outside dynamics callbacks.
     * Python retains the secondary; native connections are removed during destruction.
     */
    void syncDynamicsIntegration(DynamicObject* dynPtr);

    /** @brief Borrow the primary object, or return nullptr for independent integration.
     * @note The returned pointer does not retain the primary.
     */
    DynamicObject* getIntegrationOwner() const noexcept;

  public:
    double timeStep = 0.0;   /**< [s] integration time step */
    double timeBefore = 0.0; /**< [s] prior time value */
    uint64_t timeBeforeNanos = 0; /**< [ns] prior time value */

  private:
    std::unique_ptr<StateVecIntegrator> integrator; ///< Owned numerical method and workspaces.
    DynamicObject* integrationOwner = nullptr; ///< Borrowed primary object, or null for independent integration.

  protected:
    /** @brief Reject changes that would invalidate the fixed state layout. */
    void requireMutableTopology(const char* operation) const;
};

#endif /* DYNAMICOBJECT_H */
