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

#include "quaternionStateData.h"

#include <stdexcept>

QuaternionStateData::QuaternionStateData(std::string inName, const Eigen::MatrixXd& newState)
  : StateData(std::move(inName), newState)
{
    // state holds (w,x,y,z). Derivative is the body angular velocity (3x1).
    this->state.resize(4, 1);
    this->state.setZero();
    this->state(0) = 1.0; // identity quaternion
    this->stateDeriv.resize(3, 1);
    this->stateDeriv.setZero();
}

std::unique_ptr<StateData>
QuaternionStateData::clone() const
{
    auto result = std::make_unique<QuaternionStateData>(this->stateName, this->state);
    result->state = this->state;
    result->stateDeriv = this->stateDeriv;
    result->stateDiffusion.resize(this->stateDiffusion.size());
    for (size_t i = 0; i < this->stateDiffusion.size(); i++) {
        result->stateDiffusion[i] = this->stateDiffusion[i];
    }
    return result;
}

void
QuaternionStateData::propagateState(double dt, std::vector<double> pseudoStep)
{
    Eigen::Quaterniond q(this->state(0), this->state(1), this->state(2), this->state(3));
    Eigen::Vector3d rotVec = Eigen::Vector3d(this->stateDeriv(0), this->stateDeriv(1), this->stateDeriv(2)) * dt;
    double angle = rotVec.norm();

    // AngleAxisd needs a unit axis. In the small-angle limit pick any axis
    // since the resulting rotation is the identity.  Use if/else rather than
    // a ternary so each branch yields a concrete Vector3d — the ternary's
    // two arms produce different Eigen expression templates and won't unify.
    Eigen::Vector3d axis;
    if (angle < 1e-12) {
        axis = Eigen::Vector3d::UnitX();
    } else {
        axis = rotVec / angle;
    }
    Eigen::Quaterniond dq(Eigen::AngleAxisd(angle, axis));
    Eigen::Quaterniond qNew = (q * dq).normalized();

    this->state(0) = qNew.w();
    this->state(1) = qNew.x();
    this->state(2) = qNew.y();
    this->state(3) = qNew.z();

    if (getNumNoiseSources() > 0 && pseudoStep.size() == 0) {
        auto errorMsg = "State " + this->getName() + " has stochastic dynamics, but " +
                        "the integrator tried to propagate it without pseudoSteps. Are you sure " +
                        "you are using a stochastic integrator?";
        bskLogger.bskLog(BSK_ERROR, "%s", errorMsg.c_str());
        throw std::invalid_argument(errorMsg);
    }
}
