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
    std::vector<Eigen::MatrixXd> stateDiffusion; //!< [-] State diffusion value storage
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

    /** Sets the number of noise sources for this state.
     *
     * This is used for stochastic dynamics, where the
     * evolutions of the state are driven by a set of independent
     * noise sources:
     *
     *  dx = f(t,x)dt + g_0(t,x)dW_0 + g_1(t,x)dW_1 + ... + g_{n-1}(t,x)dW_{n-1}
     *
     * where dW_i are independent Wiener processes. The number of
     * noise sources is equal to the number of diffusion matrices
     * that are used to drive the stochastic dynamics (n above).
     *
     * @param numSources The number of noise sources
     */
    void setNumNoiseSources(size_t numSources);

    /** Get how many independent sources of noise drive the dynamics
     * of this state.
     *
     * Any number greater than zero indicates that this state
     * is driven by a stochastic differential equation.
     */
    size_t getNumNoiseSources() const;

    /** Updates the value of the state */
    void setState(const Eigen::MatrixXd& newState);

    /** Updates the derivative of the value of the state */
    void setDerivative(const Eigen::MatrixXd& newDeriv);

    /** Updates the diffusion of the value of the state.
     *
     * This is used for stochastic dynamics, where the
     * evolutions of the state are driven by a set of independent
     * noise sources:
     *
     *  dx = f(t,x)dt + g_0(t,x)dW_0 + g_1(t,x)dW_1 + ... + g_{n-1}(t,x)dW_{n-1}
     *
     * where dW_i are independent Wiener processes. The diffusion
     * matrices are used to drive the stochastic dynamics (g_i above).
     *
     * @param newDiffusion The new diffusion matrix
     * @param index The index of the diffusion matrix to update.
     * This must be less than the number of noise sources.
    */
    void setDiffusion(const Eigen::MatrixXd& newDiffusion, size_t index);

    /** Retrieves a copy of the current state */
    Eigen::MatrixXd getState() const { return state; }

    /** Retrieves a copy of the current state derivative */
    Eigen::MatrixXd getStateDeriv() const { return stateDeriv; }

    /** Retrieves a copy of the current state diffusion
     *
     * @param index The index of the diffusion matrix to retrieve.
     * This must be less than the number of noise sources.
    */
    Eigen::MatrixXd getStateDiffusion(size_t index) const { return stateDiffusion.at(index); }

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

    /**
     * @brief Propagates the state over a time step.
     *
     * This method integrates the position state using the state derivative
     * over the given time step::
     *
     *   x += f(t,x)*h + g_0(t,x)*pseudoStep[0] + g_1(t,x)*pseudoStep[1] ...
     *
     * @param h The time step for propagation.
     * @param pseudoStep For states driven by stochastic dynamics, this
     * represents the random pseudotimestep. The length of this input must
     * match the number of noise sources of this state (``getNumNoiseSources()``)
     */
    virtual void propagateState(double h, std::vector<double> pseudoStep = {});
};

#endif /* STATE_DATA_H */
