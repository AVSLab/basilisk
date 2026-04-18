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

/** @brief Helper passed to ``StatefulSysModel`` instances while they register
 * their states.
 *
 * This class serves two purposes. First, it adds a prefix to every state
 * name before registering it on the actual DynParamManager. This prevents
 * state-name collisions between ``StatefulSysModel`` instances as long as the
 * prefixes are unique. Second, it exposes only the ``registerState`` method
 * from the ``DynParamManager``. This prevents ``StatefulSysModel`` instances
 * from registering
 * properties or accessing the states of other models, which would allow for
 * information to flow between models without going through the message system.
 * If a model needs to access information from another model, it should do so
 * through a message, not by sharing a state or property.
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
     * This method may optionally be templated to create StateData of
     * subclasses of ``StateData``.
     *
     * @tparam StateDataType Concrete state-data type to instantiate.
     * @param nRow Number of rows in the state storage.
     * @param nCol Number of columns in the state storage.
     * @param stateName State name local to the registering model.
     * @return Pointer to the newly registered state object.
     */
    template <typename StateDataType = StateData,
              std::enable_if_t<std::is_base_of_v<StateData, StateDataType>, bool> = true>
    inline StateDataType* registerState(uint32_t nRow, uint32_t nCol, std::string stateName)
    {
        return this->manager.registerState<StateDataType>(
            nRow, nCol, this->stateNamePrefix + stateName
        );
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
     * @note Some stochastic integrators do not support shared noise sources.
     * In this case, this method should raise ``std::logic_error``.
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
 * ``MJScene``. On ``UpdateState()``, a ``StatefulSysModel`` should call each
 * state's ``setDerivative`` method. That derivative is then used by the
 * integrator to update the state for the next integration step.
 *
 * The sample code below shows how to get the current value of the state
 * and how to set its derivative. In this case, ``x`` would follow an
 * exponential trajectory:
 * \code{.cpp}
 * void UpdateState(uint64_t CurrentSimNanos) override {
 *     auto x = this->xState->getState();
 *     this->xState->setDerivative( x );
 * }
 * \endcode
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
