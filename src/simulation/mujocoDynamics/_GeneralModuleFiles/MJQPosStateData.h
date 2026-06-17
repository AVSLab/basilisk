/*
 ISC License

 Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#include <utility>
#include <vector>

#include <mujoco/mujoco.h>
#include "simulation/dynamics/_GeneralModuleFiles/stateData.h"

/**
 * @brief Holds and integrates the entire MuJoCo position vector (`qpos`).
 *
 * A MuJoCo scene integrates this single bulk position state (plus one bulk
 * velocity, one mass, and, when present, one actuator state) regardless of how
 * many bodies or joints it contains.  Keeping the state count constant keeps the
 * Runge-Kutta integrator's per-stage bookkeeping (`ExtendedStateVector`) constant
 * too, independent of model size.
 *
 * Because `qpos` mixes Euclidean coordinates (slide joints, translation) with
 * unit quaternions (free/ball joint orientation), this class integrates it with
 * MuJoCo's own `mj_integratePos`, which advances quaternion blocks on the SO(3)
 * manifold and Euclidean blocks linearly.
 *
 * Optionally (``highOrderIntegration``) the quaternion blocks are instead
 * advanced as a four-component rate ``qdot = 0.5 * q (x) (0, omega)`` so the
 * attitude inherits the full order of the Runge-Kutta method (see
 * ``MJScene::highOrderAttitudeIntegration``).  In that mode the state derivative
 * for a quaternion block is stored as the 4-vector ``qdot`` (filled by
 * ``MJScene``); every other block still stores the velocity it integrates.
 */
class MJQPosStateData : public StateData
{
public:
    /**
     * @brief Describes one orientation quaternion embedded in `qpos`.
     *
     * `qposAdr` is the index of the quaternion's `w` component in `qpos`
     * (4 contiguous entries, MuJoCo `(w,x,y,z)` order).  `qvelAdr` is the index
     * of the matching body angular velocity in `qvel` (3 contiguous entries).
     */
    struct QuaternionBlock {
        int qposAdr; ///< start index of the (w,x,y,z) quaternion in `qpos`
        int qvelAdr; ///< start index of the body angular velocity in `qvel`
    };

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
     * Sizes the state (`nq`) and derivative (`nv`), and records the location of
     * every orientation quaternion so the high-order integration mode can find
     * them.
     *
     * @param mujocoModel Pointer to the MuJoCo model.
     */
    void configure(mjModel* mujocoModel);

    /**
     * @brief Sets the derivative of the position state.
     *
     * In the default mode the derivative is simply `qvel` (length `nv`), which
     * `mj_integratePos` consumes.
     *
     * In the high-order mode the incoming `qvel` is expanded into a `qpos`-space
     * rate (length `nq`): Euclidean degrees of freedom keep their velocity while
     * each orientation quaternion is replaced by its four-component rate
     * ``qdot = 0.5 * q (x) (0, omega)``.  When the Runge-Kutta driver writes back
     * an already stage-combined `qpos`-space rate (length `nq`), it is stored
     * verbatim.
     */
    void setDerivative(const Eigen::MatrixXd& newDeriv) override;

    /**
     * @brief Propagates the position over a time step.
     *
     * In the default mode this defers to `mj_integratePos`.  In the high-order
     * mode, every entry is advanced linearly and each quaternion block is
     * renormalized so the attitude inherits the integrator's full order.
     *
     * @param h The time step for propagation.
     * @param pseudoStep For states driven by stochastic dynamics, the random
     * pseudotimestep (one per noise source).
     */
    void propagateState(double h, std::vector<double> pseudoStep = {}) override;

    /** @brief Whether to integrate quaternion blocks at the full RK order.
     *
     * See ``MJScene::highOrderAttitudeIntegration``. */
    bool highOrderIntegration = false;

    /** @brief Returns the quaternion blocks discovered at `configure`. */
    const std::vector<QuaternionBlock>& getQuaternionBlocks() const { return this->quaternionBlocks; }

protected:
    mjModel* mujocoModel = nullptr; ///< Pointer to the MuJoCo model associated with the state.
    std::vector<QuaternionBlock> quaternionBlocks; ///< Orientation quaternions embedded in `qpos`.

    // Index correspondence for the Euclidean (non-quaternion) degrees of freedom,
    // used only by the high-order mode to expand the `nv`-length `qvel` into the
    // `nq`-length `qpos`-space rate.  ``euclideanQpos[i]`` in `qpos` is driven by
    // ``euclideanQvel[i]`` in `qvel`.
    std::vector<int> euclideanQpos; ///< qpos indices of Euclidean DOFs
    std::vector<int> euclideanQvel; ///< matching qvel indices of Euclidean DOFs
};

#endif
