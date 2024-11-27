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

#ifndef STATE_MANAGER_H
#define STATE_MANAGER_H

#include "architecture/utilities/bskLogging.h"
#include "stateData.h"
#include <Eigen/Dense>
#include <map>
#include <stdint.h>
#include <type_traits>
#include <vector>

/** StateVector represents an ordered collection of StateData,
 * with each state having a unique name.
 */
class StateVector {
  public:
    /** All states managed by the StateVector, cached
     * by name.
     *
     * StateData classes have virtual methods, and this class
     * should be capable of supporting subclasses of StateData.
     * To do so, we need to store StateData objects as pointers.
     * To facilitate memory management, we use unique_ptr.
     */
    std::map<std::string, std::unique_ptr<StateData>> stateMap;

  public:
    /** Default constructor */
    StateVector() = default;

    /** Copy constructor */
    StateVector(const StateVector& other);

    /** Assignment operator */
    StateVector& operator= (const StateVector&);

    /** Sets the values of the states of this StateVector to a copy the
     * values of the states of the other StateVector.
     *
     * Note that we assume that both StateVectors have the same states
     * and in the same order !!!*/
    void setStates(const StateVector& operand);

    /** Adds the states of the given StateVector to the states of
     * this StateVector.
     *
     * Note that we assume that both StateVectors have the same states
     * and in the same order !!!*/
    void addStates(const StateVector& operand);

    /** Scales the states of this StateVector by the given factor */
    void scaleStates(double scaleFactor);

    /** Scales the states of this StateVector by the given delta time */
    void propagateStates(double dt);
};

/** A class that manages a set of states and properties. */
class DynParamManager {
  public:
    /** A map of properties managed by this class.
     *
     * Properties are matrices of doubles that are defined and updated
     * by modules. They are not integrated at any point, so it is the
     * responsability of the user to update them so that they remain
     * current.
     */
    std::map<std::string, Eigen::MatrixXd> dynProperties;

    /** A collection of states managed by this class */
    StateVector stateContainer;

    /** Logger used by this class */
    BSKLogger bskLogger;

  public:
    /** Creates and stores a new state to be managed by this class.
     *
     * The state name should be unique: registering two states with the
     * same name will cause either an error or a warning.
     *
     * This method may optionally be templated to create StateData of
     * subclasses of StateData.
     */
    template <typename StateDataType = StateData,
              std::enable_if_t<std::is_base_of_v<StateData, StateDataType>, bool> = true>
    StateDataType* registerState(uint32_t nRow, uint32_t nCol, std::string stateName);

    /** Retrieves the handler to a previously registered StateData.
     *
     * Calling this method for a state name not previously registered will raise a warning
     * and return a nullptr.
     */
    StateData* getStateObject(std::string stateName);

    /** Creates and stores a new property to be managed by this class.
     *
     * The property name should be unique: registering two properties with the
     * same name will cause a warning or an error.
     */
    Eigen::MatrixXd* createProperty(std::string propName, const Eigen::MatrixXd& propValue);

    /** Retrieves the handler to a previously registered property.
     *
     * Calling this method for a property name not previously registered
     * will raise an error.
     */
    Eigen::MatrixXd* getPropertyReference(std::string propName);

    /** Sets the value for a property.
     *
     * An error will be raised if no property exists with name or the size of the
     * given matrix is different from the size of the existing property.
     */
    void setPropertyValue(const std::string propName, const Eigen::MatrixXd& propValue);

    /** Sets the values of the states managed by this class to a copy the
     * values of the states of the given StateVector.
     *
     * Note that we assume that given StateVector have the same states
     * and in the same order as this object !!!*/
    void updateStateVector(const StateVector& newState);

    /** Propagates the states managed by this class a given delta time.  */
    void propagateStateVector(double dt);
};

template <typename StateDataType,
          std::enable_if_t<std::is_base_of_v<StateData, StateDataType>, bool>>
StateDataType* DynParamManager::registerState(uint32_t nRow, uint32_t nCol, std::string stateName)
{
    if (stateName == "") {
        bskLogger.bskLog(BSK_ERROR,
                         "Your state name can't be an empty string.  Come on.  You get null.");
        return nullptr;
    }

    if (stateContainer.stateMap.count(stateName) > 0) {
        bskLogger.bskLog(
            BSK_WARNING,
            "You created a state with the name: %s more than once.  Go ahead and don't do this.",
            stateName.c_str());

        auto& stateData = stateContainer.stateMap.at(stateName);

        if (stateData->getRowSize() != nRow || stateData->getColumnSize() != nCol) {
            bskLogger.bskLog(BSK_ERROR,
                             "In addition to that, you tried to change the size of the state in "
                             "question.  Come on.  You get null.");
            return nullptr;
        }

        auto casted = dynamic_cast<StateDataType*>(stateData.get());
        if (!casted) {
            bskLogger.bskLog(BSK_ERROR,
                             "In addition to that, you tried to change the StateData type.  Come "
                             "on.  You get null.");
            return nullptr;
        }

        return casted;
    }

    Eigen::MatrixXd stateMatrix;
    stateMatrix.resize(nRow, nCol);

    // Emplacing this stateData in the map will wrap the raw
    // pointer in a unique_ptr, so no worries about leaks.
    // I didn't emplace `std::make_unique<StateDataType>(...)
    // because I needed the raw pointer anyway to return it.
    StateDataType* stateData = new StateDataType(stateName, stateMatrix);
    stateContainer.stateMap.emplace(stateName, stateData);
    return stateData;
}

#endif /* STATE_MANAGER_H */
