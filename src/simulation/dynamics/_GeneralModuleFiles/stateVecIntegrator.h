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

/** @file stateVecIntegrator.h
 * @brief Shared dynamics callbacks and integrator workspace preparation.
 */
#ifndef stateVecIntegrator_h
#define stateVecIntegrator_h

#include <cstddef>
#include <vector>

class DynamicObject;
namespace integrator_test {
class StateVecIntegratorTestAccess;
}

/**
 * @brief Base for numerical methods that advance a fixed group of dynamics objects.
 *
 * DynamicObject owns its integrator. The first step prepares state descriptors and
 * numerical buffers; subsequent steps reuse them. If a synchronized secondary is
 * destroyed outside a callback, the next step prepares storage for the surviving
 * group. Borrowed state storage must remain alive while a step uses it.
 */
class StateVecIntegrator
{
  public:
    /** @brief Construct an integrator for its future owning dynamics object. */
    explicit StateVecIntegrator(DynamicObject* dynIn);
    /** @brief Release numerical storage without taking ownership of dynamics objects. */
    virtual ~StateVecIntegrator() = default;

    /** @brief Return the number of objects advanced together. */
    std::size_t getDynamicsCount() const noexcept { return this->dynPtrs.size(); }
    /** @brief Borrow the dynamics list in integration order, with the primary first. */
    const std::vector<DynamicObject*>& getDynamics() const noexcept { return this->dynPtrs; }

  protected:
    /** @brief Resolve the current group's state layouts and allocate numerical workspace. */
    virtual void prepareIntegrationBinding() = 0;
    /** @brief Check that cached descriptors still refer to finalized dynamics. */
    virtual void validateIntegrationBinding() const = 0;
    /** @brief Advance from currentTime by timeStep, both in seconds. */
    virtual void integrateImpl(double currentTime, double timeStep) = 0;
    /** @brief Prepare storage on first use and validate it on later calls. */
    void ensureIntegrationBinding();
    /** @brief Evaluate each object's drift at the supplied time and step in seconds. */
    void evaluateDerivatives(double time, double timeStep);
    /** @brief Evaluate each object's diffusion at the supplied time and step in seconds. */
    void evaluateDiffusions(double time, double timeStep);
    /** @brief Borrow dynamics objects in callback order. */
    const std::vector<DynamicObject*>& dynamics() const noexcept { return this->dynPtrs; }

  private:
    friend class DynamicObject;
    friend class integrator_test::StateVecIntegratorTestAccess;
    /** @brief Prepare workspace and execute the numerical method. */
    void integrate(double currentTime, double timeStep);

    std::vector<DynamicObject*> dynPtrs; ///< Borrowed callback targets, with the primary first.
    bool bindingPrepared = false;        ///< True when workspace preparation has succeeded for the current group.
};

#endif /* stateVecIntegrator_h */
