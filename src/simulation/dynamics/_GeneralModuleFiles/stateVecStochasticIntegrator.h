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


#ifndef stateVecStochasticIntegrator_h
#define stateVecStochasticIntegrator_h

#include <cstddef>
#include <unordered_map>
#include <string>

#include "../_GeneralModuleFiles/stateVecIntegrator.h"
#include "../_GeneralModuleFiles/extendedStateVector.h"

class StateData;

/*! @brief state vector integrator class for stochastic dynamics */
class StateVecStochasticIntegrator : public StateVecIntegrator
{

public:
    using StateVecIntegrator::StateVecIntegrator;

    /** Returns a vector with length equal to the total number of noise
     * sources in the system. Each index contains a map that indicates
     * how that noise source maps to individual noise indices for each
     * relevant ``StateData``.
     *
     * For example, consider the following SDE:
     *
     *   dx_0 = f_0(t,x)dt + g_00(t,x)dW_0 + g_01(t,x)dW_1
     *   dx_1 = f_1(t,x)dt + g_11(t,x)dW_1
     *
     * In this case, state 'x_0' is affected by 2 sources of noise
     * and 'x_1' by 1 source of noise. The source 'W_1' is
     * shared between 'x_0' and 'x_1'.
     *
     * This function would return (pseudo-code):
     *
     *   [
     *       {(0, "x_0") -> 0},
     *       {(0, "x_0") -> 1, (0, "x_1") -> 0},
     *   ]
     *
     * because there are 2 unique sources of noise, the first of
     * which affects the first noise index of 'x_0' and the second
     * of which affects the second noise index of 'x_0' and the
     * first noise index of "x_1".
     */
    std::vector<StateIdToIndexMap> getStateIdToNoiseIndexMaps();

    /** This method calls ``propagateStateVector`` on every state of
     * the dynamic objects handled by this integrator.
     *
     * The length of ``pseudoTimeSteps`` must match the size of
     * ``stateIdToNoiseIndexMaps``. The ``stateIdToNoiseIndexMaps``
     * are used to map each ``pseudoTimeStep`` to a noise index
     * for each state.
     *
     * Consider the following SDE:
     *
     *   dx_0 = f_0(t,x)dt + g_00(t,x)dW_0 + g_01(t,x)dW_1
     *   dx_1 = f_1(t,x)dt + g_11(t,x)dW_1
     *
     * Assume that the values of the drift and diffusion have
     * been computed and are stored in the state objects. If this
     * method is called with:
     *
     *  - ``timeStep``: h
     *  - ``pseudoTimeSteps``: [W_0, W_1]
     *  - ``stateIdToNoiseIndexMaps``:
     *   [
     *       {(0, "x_0") -> 0},
     *       {(0, "x_0") -> 1, (0, "x_1") -> 0},
     *   ]
     *
     * Then the states will be changed by:
     *
     *   x_0 += f_0(t,x)*h + g_00(t,x)*W_0 + g_01(t,x)*W_1
     *   x_1 += f_1(t,x)*h + g_11(t,x)*W_1
     *
     */
    void propagateState(
        double timeStep,
        const Eigen::VectorXd& pseudoTimeSteps,
        const std::vector<StateIdToIndexMap>& stateIdToNoiseIndexMaps
    );
};


#endif /* StateVecIntegrator_h */
