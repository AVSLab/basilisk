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

#ifndef STATE_DATA_H
#define STATE_DATA_H
#include <Eigen/Dense>
#include <memory>
#include <stdint.h>
#include "architecture/utilities/bskLogging.h"

/** @brief Represents a physical state, which has a name, a value, and a derivative. */
class StateData
{
public:
    Eigen::MatrixXd state;       //!< [-] State value storage
    Eigen::MatrixXd stateDeriv;  //!< [-] State derivative value storage
    const std::string stateName; //!< [-] Name of the state
    BSKLogger bskLogger;         //!< -- BSK Logging

public:
    /** Creates a new state with the given name and set's the initial state.
     *
     * The state derivative will be resized to the same size as the state and zero'd.
     */
    StateData(std::string inName, const Eigen::MatrixXd& newState);

    /** Clone constructor for polymorphic class */
    virtual std::unique_ptr<StateData> clone() const;

    /** Destructor */
    virtual ~StateData() = default;

    /** Updates the value of the state */
    void setState(const Eigen::MatrixXd& newState);

    /** Updates the derivative of the value of the state */
    void setDerivative(const Eigen::MatrixXd& newDeriv);

    /** Retrieves a copy of the current state */
    Eigen::MatrixXd getState() const { return state; }

    /** Retrieves a copy of the current state derivative */
    Eigen::MatrixXd getStateDeriv() const { return stateDeriv; }

    /** Returns the name of the state */
    std::string getName() const { return stateName; }

    /** Returns the row-size of the state */
    uint32_t getRowSize() const { return ((uint32_t)state.innerSize()); }

    /** Returns the column-size of the state */
    uint32_t getColumnSize() const { return ((uint32_t)state.outerSize()); }

    /** Returns the row-size of the derivative of the state */
    uint32_t getDerivativeRowSize() const { return ((uint32_t)stateDeriv.innerSize()); }

    /** Returns the column-size of the derivative of the state */
    uint32_t getDerivativeColumnSize() const { return ((uint32_t)stateDeriv.outerSize()); }

    /** Multiples the state by a scalar */
    void scaleState(double scaleFactor);

    /** Adds the values of the other state to this state */
    void addState(const StateData& other);

    /** Propagates the state in time with the stored derivative */
    virtual void propagateState(double dt);
};

#endif /* STATE_DATA_H */
