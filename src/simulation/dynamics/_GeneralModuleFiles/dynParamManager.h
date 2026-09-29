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

/**
 * @file dynParamManager.h
 * @brief Module-facing state declarations, lookup, and shared properties.
 * Registration and contiguous storage are implemented by StateRegistry.
 */

#ifndef STATE_MANAGER_H
#define STATE_MANAGER_H

#include "architecture/utilities/bskLogging.h"
#include "stateData.h"
#include <Eigen/Core>
#include <cstddef>
#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

class StateRegistry;

/**
 * @brief Named integration states and shared property matrices for dynamics modules.
 *
 * Register states in your module's registration callback and retain the returned
 * StateData handles. Use getStateObject() to link another module's state and
 * getPropertyReference() to link a shared property. Properties are not integrated.
 *
 * The owning DynamicObject calls finalizeStates() after initial registration to
 * allocate contiguous storage. Later resets reuse existing states by name and
 * update their values in place. Shapes, update policies, and noise connections
 * cannot change after finalization. Repeated finalization has no effect.
 *
 * This manager owns all handles and properties and must outlive borrowed pointers.
 * Reacquire Eigen views after the first finalization; subsequent resets preserve
 * storage addresses. Setup errors propagate to the caller without value rollback.
 *
 * getStateRegistry() provides native access to layouts and contiguous segments.
 */
class DynParamManager
{
  public:
    /** @brief Construct an empty manager with no states or properties. */
    DynParamManager();
    /** @brief Release all states, properties, and registry storage. */
    ~DynParamManager();

    // Borrowed handles and property pointers require a stable owner identity.
    DynParamManager(const DynParamManager&) = delete;
    DynParamManager& operator=(const DynParamManager&) = delete;
    DynParamManager(DynParamManager&&) = delete;
    DynParamManager& operator=(DynParamManager&&) = delete;

    /**
     * @brief Allocate contiguous state storage and fix the layout after registration.
     * @note Call before integration. Repeated calls have no effect.
     * @throws std::exception If dimensions or shared-noise declarations are invalid.
     */
    void finalizeStates();

    /**
     * @brief Declare a state with explicit dimensions, noise count, and error scaling.
     * @param stateName Nonempty name, unique within this manager.
     * @param spec Fixed state topology; Euclidean states require equal state,
     * derivative, and diffusion-tangent shapes.
     * @return Manager-owned handle, reused when matching a repeated declaration.
     * @throws std::exception If the declaration is invalid or would change finalized topology.
     */
    StateData* registerState(std::string stateName, const StateSpec& spec);

    /**
     * @brief Declare a state whose update requires a non-Euclidean policy.
     * @param stateName Nonempty state name, reused by name on reset.
     * @param spec State topology with StateUpdateKind::Special.
     * @param policy Immutable update policy; ownership transfers to this call.
     * Repeated declarations must provide an equivalent policy.
     * @return Manager-owned handle for the declaration.
     * @throws std::exception If the declaration or policy is invalid.
     */
    StateData* registerState(std::string stateName, const StateSpec& spec, std::unique_ptr<StateUpdatePolicy> policy);

    /**
     * @brief Compatibility overload for an equal-shape Euclidean matrix state.
     * @param nRow Nonzero number of rows.
     * @param nCol Nonzero number of columns.
     * @param stateName Nonempty state name.
     * @return Manager-owned state handle.
     * @note A matching repeated declaration retains its established noise count
     * and error-control settings. Use StateSpec for explicit new declarations.
     */
    StateData* registerState(uint32_t nRow, uint32_t nCol, std::string stateName);

    /**
     * @brief Find a state declared in this manager.
     * @param stateName Name supplied to registerState().
     * @return Borrowed handle, or nullptr with a warning if the name is absent.
     */
    StateData* getStateObject(std::string stateName);

    /**
     * @brief Declare that several states use the same stochastic process.
     * @param sharedNoises Pairs of state handles and zero-based local noise indices.
     * All states must belong to this manager. A local source may appear in only
     * one group, and a group may contain at most one source from each state.
     * @throws std::exception If the group is invalid or would change finalized noise connections.
     * @note Group ordering is immaterial; repeat the same connections on reset.
     */
    void registerSharedNoiseSource(std::vector<std::pair<const StateData&, size_t>> sharedNoises);

    /** @brief Report whether fixed state storage has been allocated. */
    bool statesAreFinalized() const noexcept;

    /**
     * @brief Create or replace a named property without integrating it.
     * @param propName Property name used by modules to link this matrix.
     * @param propValue Initial matrix; replacing a property may change its shape.
     * @return Borrowed pointer to the matrix object, stable across later updates.
     */
    Eigen::MatrixXd* createProperty(std::string propName, const Eigen::MatrixXd& propValue);
    /**
     * @brief Look up a shared property matrix.
     * @param propName Previously created property name.
     * @return Borrowed pointer, valid until the manager is destroyed.
     * @throws BasiliskError If the property does not exist.
     */
    Eigen::MatrixXd* getPropertyReference(std::string propName);
    /**
     * @brief Update an existing property without changing its shape.
     * @param propName Previously created property name.
     * @param propValue Replacement values with the existing dimensions.
     * @throws BasiliskError If the property does not exist.
     * @throws std::invalid_argument If its dimensions differ.
     */
    void setPropertyValue(std::string propName, const Eigen::MatrixXd& propValue);

    /**
     * @brief Access state layouts, contiguous buffer segments, and shared-noise topology.
     * @return Borrowed registry owned by this manager. Include stateRegistry.h to
     * inspect layouts, resolve buffer segments, or bind shared-noise topology.
     * @note This interface is not exposed to Python.
     */
    StateRegistry& getStateRegistry() noexcept { return *this->registry; }
    /** @brief Inspect the native registry through a constant manager. @see getStateRegistry() */
    const StateRegistry& getStateRegistry() const noexcept { return *this->registry; }

    BSKLogger bskLogger; //!< Reports missing state names and invalid property access.

  private:
    std::map<std::string, Eigen::MatrixXd>
      dynProperties;                         //!< Named, non-integrated matrices; map nodes keep pointers stable.
    std::unique_ptr<StateRegistry> registry; //!< Owns registration and buffer storage at a stable address.
};

#endif /* STATE_MANAGER_H */
