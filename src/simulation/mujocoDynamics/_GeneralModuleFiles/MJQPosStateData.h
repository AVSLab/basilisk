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

#ifndef MJSTATE_DATA_H
#define MJSTATE_DATA_H

#include <array>
#include <utility>

#include <mujoco/mujoco.h>
#include "simulation/dynamics/_GeneralModuleFiles/stateData.h"

/**
 * @brief Class representing the entire joint state data in a MuJoCo simulation.
 *
 * This class manages the position states (`qpos`) and their derivatives in a
 * MuJoCo simulation. It extends `StateData` to include specific
 * configurations and propagation logic for position states.
 */
class MJQPosStateData : public StateData
{
public:
    /**
     * @brief Constructs an MJQPosStateData object.
     *
     * @param inName The name of the state.
     * @param newState The initial state matrix.
     */
    MJQPosStateData(std::string inName, const Eigen::MatrixXd& newState)
        : StateData(std::move(inName), newState){};

    /**
     * @brief Creates a clone of the current state data object.
     *
     * @return A unique pointer to the cloned `StateData` object.
     */
    virtual std::unique_ptr<StateData> clone() const override;

    /**
     * @brief Configures the state data with the given MuJoCo model.
     *
     * This method initializes the state and derivative sizes
     * based on the given MuJoCo model.
     *
     * @param mujocoModel Pointer to the MuJoCo model.
     */
    void configure(mjModel* mujocoModel);

    /**
     * @brief Propagates the state over a time step.
     *
     * This method integrates the position state using the state derivative
     * over the given time step.
     *
     * @param dt The time step for propagation.
     */
    void propagateState(double dt) override;

protected:
    mjModel* mujocoModel; ///< Pointer to the MuJoCo model associated with the state.
};

#endif
