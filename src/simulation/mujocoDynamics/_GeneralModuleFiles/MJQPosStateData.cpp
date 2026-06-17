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

#include "MJQPosStateData.h"

#include <algorithm>

namespace
{
// Quaternion rate qdot = 0.5 * q (x) (0, omega), in MuJoCo (w,x,y,z) order.
// This is the same right-multiply body-frame convention as mj_integratePos's
// exponential map, so the high-order and default integrators are consistent in
// the small-step limit.
Eigen::Matrix<double, 4, 1> quaternionRate(const double* q, const double* omega)
{
    double w = q[0], x = q[1], y = q[2], z = q[3];
    double ox = omega[0], oy = omega[1], oz = omega[2];
    Eigen::Matrix<double, 4, 1> qd;
    qd(0) = 0.5 * (-x * ox - y * oy - z * oz);
    qd(1) = 0.5 * ( w * ox + y * oz - z * oy);
    qd(2) = 0.5 * ( w * oy - x * oz + z * ox);
    qd(3) = 0.5 * ( w * oz + x * oy - y * ox);
    return qd;
}
} // namespace

std::unique_ptr<StateData> MJQPosStateData::clone() const
{
    auto result = std::make_unique<MJQPosStateData>(this->stateName, this->state);
    result->state = this->state;
    result->stateDeriv = this->stateDeriv;
    result->perComponentErrorControl = this->perComponentErrorControl;
    result->mujocoModel = this->mujocoModel;
    result->quaternionBlocks = this->quaternionBlocks;
    result->euclideanQpos = this->euclideanQpos;
    result->euclideanQvel = this->euclideanQvel;
    result->highOrderIntegration = this->highOrderIntegration;
    result->stateDiffusion.resize(this->stateDiffusion.size());
    for (size_t i = 0; i < this->stateDiffusion.size(); i++) {
        result->stateDiffusion[i] = this->stateDiffusion[i];
    }
    return result;
}

void MJQPosStateData::configure(mjModel* mujocoModel)
{
    this->mujocoModel = mujocoModel;

    this->state.resize(mujocoModel->nq, 1);
    this->stateDeriv.resize(mujocoModel->nv, 1);

    // Record every orientation quaternion embedded in qpos so the high-order
    // mode can advance them as four-component rates.  Free joints carry the
    // quaternion three entries after their translation; ball joints carry it at
    // the joint address itself.  Also record the index correspondence of every
    // Euclidean DOF so the high-order mode can expand qvel into a qpos-space rate.
    this->quaternionBlocks.clear();
    this->euclideanQpos.clear();
    this->euclideanQvel.clear();
    for (int j = 0; j < mujocoModel->njnt; ++j) {
        int qp = mujocoModel->jnt_qposadr[j];
        int qv = mujocoModel->jnt_dofadr[j];
        switch (mujocoModel->jnt_type[j]) {
        case mjJNT_FREE:
            // 3 translation DOFs (Euclidean) then a 4-entry quaternion.
            for (int k = 0; k < 3; ++k) { this->euclideanQpos.push_back(qp + k); this->euclideanQvel.push_back(qv + k); }
            this->quaternionBlocks.push_back({qp + 3, qv + 3});
            break;
        case mjJNT_BALL:
            this->quaternionBlocks.push_back({qp, qv});
            break;
        case mjJNT_SLIDE:
        case mjJNT_HINGE:
            this->euclideanQpos.push_back(qp);
            this->euclideanQvel.push_back(qv);
            break;
        }
    }
}

void MJQPosStateData::setDerivative(const Eigen::MatrixXd& newDeriv)
{
    if (this->highOrderIntegration && newDeriv.rows() == this->mujocoModel->nv) {
        // Expand the body-velocity vector qvel (length nv) into a qpos-space rate
        // (length nq): Euclidean DOFs keep their velocity, each quaternion block
        // becomes the four-component rate qdot = 0.5 * q (x) (0, omega).  This is
        // the per-stage rate the RK driver linearly combines.
        Eigen::MatrixXd qposRate = Eigen::MatrixXd::Zero(this->mujocoModel->nq, 1);
        for (size_t i = 0; i < this->euclideanQpos.size(); ++i) {
            qposRate(this->euclideanQpos[i]) = newDeriv(this->euclideanQvel[i]);
        }
        for (const auto& block : this->quaternionBlocks) {
            Eigen::Matrix<double, 4, 1> qd =
                quaternionRate(this->state.data() + block.qposAdr, newDeriv.data() + block.qvelAdr);
            for (int k = 0; k < 4; ++k) qposRate(block.qposAdr + k) = qd(k);
        }
        this->stateDeriv = qposRate;
    } else {
        // Default mode: store qvel verbatim (consumed by mj_integratePos), or the
        // RK driver writing back an already qpos-space-combined rate (length nq).
        this->stateDeriv = newDeriv;
    }
}

void MJQPosStateData::propagateState(double dt, std::vector<double> pseudoStep)
{
    if (getNumNoiseSources() > 0 && pseudoStep.size() == 0)
    {
        auto errorMsg = "State " + this->getName() + " has stochastic dynamics, but "
            + "the integrator tried to propagate it without pseudoSteps. Are you sure "
            + "you are using a stochastic integrator?";
        bskLogger.bskError("%s", errorMsg.c_str());
        throw std::invalid_argument(errorMsg);
    }

    if (this->highOrderIntegration && this->stateDeriv.rows() == this->mujocoModel->nq) {
        // High-order mode: stateDeriv is the (possibly RK-stage-combined) rate in
        // qpos-space.  Every entry advances linearly; each quaternion block is
        // renormalized so the attitude inherits the integrator's full order.
        for (int i = 0; i < this->mujocoModel->nq; ++i) {
            this->state(i) += this->stateDeriv(i) * dt;
        }
        for (const auto& block : this->quaternionBlocks) {
            double* q = this->state.data() + block.qposAdr;
            double norm = std::sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
            if (norm > 0) {
                for (int k = 0; k < 4; ++k) q[k] /= norm;
            }
        }
    } else {
        // Default mode: mj_integratePos advances quaternion blocks on the SO(3)
        // manifold (a single exponential map of the stage-averaged body rate,
        // matching MuJoCo's native RK4 stepping) and Euclidean blocks linearly.
        mj_integratePos(this->mujocoModel, this->state.data(), this->stateDeriv.data(), dt);
    }

    // Stochastic increments are always applied on the manifold via
    // mj_integratePos, regardless of the deterministic integration order.
    for (size_t i = 0; i < getNumNoiseSources(); i++)
    {
        mj_integratePos(this->mujocoModel, this->state.data(),
                        this->stateDiffusion.at(i).data(), pseudoStep.at(i));
    }
}
