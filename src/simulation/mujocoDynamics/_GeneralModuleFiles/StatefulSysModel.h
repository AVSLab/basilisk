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

/**A short-lived class passed to StatefulSysModel for them to register
 * their states.
 *
 * This class serves two purposes. First, it adds a prefix to every state
 * name before registering it on the actual DynParamManager. This prevents
 * state name collisions between StatefulSysModel as long as the prefix
 * are unique. Second, it exponses only the registerState method from
 * the DynParamManager. This prevents StatefulSysModel from registering
 * properties or accessing the states of other models, which would allow for
 * information to flow between models without going through the message system.
 * If a model needs to access information from another model, it should do so
 * thorugh a message, not by sharing a state or property.
 */
class DynParamRegisterer
{
public:
    /** Constructor */
    DynParamRegisterer(DynParamManager& manager, std::string stateNamePrefix)
        : manager(manager)
        , stateNamePrefix(stateNamePrefix)
        {}

    /** Creates and returns a new state, which will be managed by the
     * underlying ``DynParamManager``.
     *
     * The state name should be unique: registering two states with the
     * same name on this class will cause an error. Different StatefulSysModel
     * are allowed to use the same state name, however.
     *
     * This method may optionally be templated to create StateData of
     * subclasses of StateData.
     */
    template <typename StateDataType = StateData,
              std::enable_if_t<std::is_base_of_v<StateData, StateDataType>, bool> = true>
    inline StateDataType* registerState(uint32_t nRow, uint32_t nCol, std::string stateName)
    {
        return this->manager.registerState<StateDataType>(
            nRow, nCol, this->stateNamePrefix + stateName
        );
    }

protected:
    DynParamManager& manager; ///< wrapped manager
    std::string stateNamePrefix; ///< prefix added to all registered state names
};

/** A SysModel that has continuous-time states.
 *
 * StatefulSysModel are added on the dynamics task of an MJScene.
 * On its UpdateState method, a StatefulSysModel should call each state's
 * setDerivative method. This value will be used by the integrator to
 * update the state for the next integrator step.
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
class StatefulSysModel : public SysModel
{
public:
    /** Default constructor */
    StatefulSysModel() = default;

    /**Used to register states on the given DynParamRegisterer.
     *
     * The main purpose of this method is for this class to call
     * ``registerState`` on the registerer. Note that state names
     * should not be repeated within the same StatefulSysModel.
     *
     * \code{.cpp}
     * void registerStates(DynParamRegisterer& registerer) override {
     *     this->posState = registerer.register(3, 1, "pos");
     *     this->massState = registerer.register(1, 1, "mass");
     *     // etc.
     * }
     * \endcode
     *
     */
    virtual void registerStates(DynParamRegisterer registerer) = 0;
};

#endif
