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
#include "meanRevertingNoiseStateEffector.h"

#include <cmath>

uint64_t MeanRevertingNoiseStateEffector::effectorID = 1;

MeanRevertingNoiseStateEffector::MeanRevertingNoiseStateEffector()
{
    this->effProps.mEff = 0.0;
    this->effProps.rEff_CB_B.setZero();
    this->effProps.IEffPntB_B.setZero();
    this->effProps.rEffPrime_CB_B.setZero();
    this->effProps.IEffPrimePntB_B.setZero();

    this->nameOfState = "meanRevertingNoiseState"
                        + std::to_string(MeanRevertingNoiseStateEffector::effectorID);
    MeanRevertingNoiseStateEffector::effectorID++;
}

void MeanRevertingNoiseStateEffector::setStationaryStd(double sigmaStationary)
{
    if (sigmaStationary < 0.0) {
        this->bskLogger.bskLog(BSK_ERROR,
            "MeanRevertingNoiseStateEffector::setStationaryStd requires sigmaStationary >= 0.");
        return;
    }
    this->sigmaStationary = sigmaStationary;
}

void MeanRevertingNoiseStateEffector::setTimeConstant(double timeConstant)
{
    if (timeConstant <= 0.0) {
        this->bskLogger.bskLog(BSK_ERROR,
            "MeanRevertingNoiseStateEffector::setTimeConstant requires timeConstant > 0.");
        return;
    }
    this->timeConstant = timeConstant;
}

double MeanRevertingNoiseStateEffector::getStateValue() const
{
    if (this->state == nullptr) {
        return this->stateInit;
    }
    return this->state->getState()(0, 0);
}

void MeanRevertingNoiseStateEffector::setStateValue(double val)
{
    this->stateInit = val;
    if (this->state != nullptr) {
        Eigen::MatrixXd state(1, 1);
        state(0, 0) = val;
        this->state->setState(state);
    }
}

void MeanRevertingNoiseStateEffector::registerStates(DynParamManager& states)
{
    this->state = states.registerState(1, 1, this->nameOfState);
    this->state->setNumNoiseSources(1);

    Eigen::MatrixXd state(1, 1);
    state(0, 0) = this->stateInit;
    this->state->setState(state);
}

void MeanRevertingNoiseStateEffector::linkInStates(DynParamManager& /** states */)
{
}

void MeanRevertingNoiseStateEffector::computeDerivatives(double /** integTime */,
                                                         Eigen::Vector3d /** rDDot_BN_N */,
                                                         Eigen::Vector3d /** omegaDot_BN_B */,
                                                         Eigen::Vector3d /** sigma_BN */)
{
    if (this->state == nullptr) {
        this->bskLogger.bskLog(BSK_ERROR,
            "MeanRevertingNoiseStateEffector::computeDerivatives called before registerStates.");
        return;
    }

    const double x = this->state->getState()(0, 0);

    Eigen::MatrixXd derivative(1, 1);
    derivative(0, 0) = -(1.0 / this->timeConstant) * x;
    this->state->setDerivative(derivative);

    Eigen::MatrixXd diffusion(1, 1);
    diffusion(0, 0) = this->sigmaStationary * std::sqrt(2.0 / this->timeConstant);
    this->state->setDiffusion(diffusion, 0);
}
