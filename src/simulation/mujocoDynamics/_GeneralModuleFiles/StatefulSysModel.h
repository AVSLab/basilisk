/*
 ISC License

 Copyright (c) 2025, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#ifndef STATEFUL_SYS_MODEL_H
#define STATEFUL_SYS_MODEL_H

#include "simulation/dynamics/_GeneralModuleFiles/dynParamManager.h"
#include "architecture/_GeneralModuleFiles/sys_model.h"
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

/** @brief Helper passed to ``StatefulSysModel`` instances while they register
 * their states.
 *
 * The scene supplies a unique model prefix during state registration.
 * This helper prepends it to state names, forwards specifications and owned update
 * policies, and supports shared-noise declarations. It does not own the manager or
 * start/finalize registration. The scene must outlive the helper and its returned
 * state handles.
 *
 * Models should exchange ordinary information through messages. Keep returned state
 * handles for equations of motion, and reacquire views after lifecycle transitions.
 */
class DynParamRegisterer
{
public:
    /** @brief Construct a state-registering helper.
     *
     * @param manager Underlying dynamics-parameter manager.
     * @param stateNamePrefix Prefix appended to every registered state name.
     */
    DynParamRegisterer(DynParamManager& manager, std::string stateNamePrefix)
        : manager(manager)
        , stateNamePrefix(stateNamePrefix)
        {}

    /** @brief Create and return a new state managed by the underlying
     * ``DynParamManager``.
     *
     * The state name should be unique: registering two states with the
     * same name on this class will cause an error. Different
     * ``StatefulSysModel`` instances are allowed to use the same state name,
     * however.
     *
     * @param nRow Number of rows in the state storage.
     * @param nCol Number of columns in the state storage.
     * @param stateName State name local to the registering model.
     * @return Pointer to the newly registered state object.
     */
        inline StateData* registerState(uint32_t nRow, uint32_t nCol, std::string stateName)
    {
            return this->manager.registerState(nRow, nCol, this->stateNamePrefix + stateName);
    }

        /** @brief Register a state with complete immutable topology metadata.
         * @param stateName Name local to this model, before prefixing.
         * @param spec State, drift, tangent, noise, and error-control declarations.
         * @return Borrowed handle owned by the underlying manager.
         */
        inline StateData* registerState(std::string stateName, const StateSpec& spec)
        {
            return this->manager.registerState(this->stateNamePrefix + stateName, spec);
        }

        /** @brief Register a state with complete topology and an owned update policy.
         * @param stateName Name local to this model, before prefixing.
         * @param spec Immutable state topology.
         * @param policy Special update rule whose ownership transfers to the registry.
         * @return Borrowed handle owned by the underlying manager.
         */
        inline StateData* registerState(std::string stateName,
                                        const StateSpec& spec,
                                        std::unique_ptr<StateUpdatePolicy> policy)
        {
            return this->manager.registerState(this->stateNamePrefix + stateName, spec, std::move(policy));
        }

    /** @brief Register a shared stochastic noise source across multiple states.
     *
     * Used when more than one state has dynamics perturbed
     * by the same noise process.
     *
     * For example, consider the following SDE:
     *
     * \f[
     *   dx_0 = f_0(t,x)\,dt + g_{00}(t,x)\,dW_0 + g_{01}(t,x)\,dW_1
     * \f]
     * \f[
     *   dx_1 = f_1(t,x)\,dt + g_{11}(t,x)\,dW_1
     * \f]
     *
     * In this case, state 'x_0' is affected by 2 sources of noise
     * and 'x_1' by 1 source of noise. However, the source 'W_1' is
     * shared between 'x_0' and 'x_1'.
     *
     * This function is called like:
     *
     * \code
     *     dynParamManager.registerSharedNoiseSource({
     *         {myStateX0, 1},
     *         {myStateX1, 0}
     *     });
     * \endcode
     *
     * which means that the 2nd noise source of the ``StateData`` 'myStateX0'
     * and the first noise source of the ``StateData`` 'myStateX1' actually
     * correspond to the same noise process.
     *
     * @param in List of state/noise-source-index pairs that share one process.
     *
     * @note Endpoints must belong to this manager and satisfy its registration
     * contract. The numerical method's noise assumptions still apply.
     */
    inline void registerSharedNoiseSource(std::vector<std::pair<const StateData&, size_t>> in)
    {
        manager.registerSharedNoiseSource(std::move(in));
    }

protected:
    DynParamManager& manager;      //!< Wrapped manager that owns the registered states.
    std::string stateNamePrefix;   //!< Prefix added to all registered state names.
};

/** @brief ``SysModel`` base class for modules with continuous-time states.
 *
 * ``StatefulSysModel`` instances are added to the dynamics task of an
 * ``MJScene``. The scene calls registerStates() during Reset,
 * then resets each distinct task model after finalization. Repeated registration must
 * reproduce the same topology. Models may also participate in the diffusion task.
 *
 * On ``UpdateState()``, a drift model should call each
 * state's ``setDerivative`` method. That derivative is then used by the
 * integrator to update the state for the next integration step.
 *
 * The sample code below shows how to get the current value of the state
 * and how to set its derivative. In this case, ``x`` would follow an
 * exponential trajectory:
 * \code{.cpp}
 * void UpdateState(uint64_t CurrentSimNanos) override {
 *     const double growthRate = 1.0; // [1/s]
 *     const auto x = this->xState->stateView();
 *     this->xState->derivativeView() = growthRate * x;
 * }
 * \endcode
 * @note The borrowed view aliases the active state buffer. Use getState() for an
 * owning snapshot, and reacquire views after registry lifecycle transitions.
 */
class StatefulSysModel : virtual public SysModel
{
public:
    /** @brief Construct a stateful system model. */
    StatefulSysModel() = default;

    /** @brief Register this model's continuous states.
     *
     * The main purpose of this method is to call ``registerState`` on the
     * supplied registerer. State names must not be repeated within the same
     * ``StatefulSysModel`` instance.
     *
     * \code{.cpp}
     * void registerStates(DynParamRegisterer registerer) override {
     *     this->posState = registerer.registerState(3, 1, "pos");
     *     this->massState = registerer.registerState(1, 1, "mass");
     *     // etc.
     * }
     * \endcode
     *
     * @param registerer Helper used to create namespaced state registrations.
     */
    virtual void registerStates(DynParamRegisterer registerer) = 0;
};

#endif
