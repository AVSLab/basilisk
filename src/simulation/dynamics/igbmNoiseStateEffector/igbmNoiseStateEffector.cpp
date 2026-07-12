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
#include "igbmNoiseStateEffector.h"

#include <cmath>

uint64_t IgbmNoiseStateEffector::effectorID = 1;

IgbmNoiseStateEffector::IgbmNoiseStateEffector()
{
    this->effProps.mEff = 0.0;
    this->effProps.rEff_CB_B.setZero();
    this->effProps.IEffPntB_B.setZero();
    this->effProps.rEffPrime_CB_B.setZero();
    this->effProps.IEffPrimePntB_B.setZero();

    this->nameOfState = "igbmNoiseState"
                        + std::to_string(IgbmNoiseStateEffector::effectorID);
    IgbmNoiseStateEffector::effectorID++;
}

void IgbmNoiseStateEffector::setMean(double mean)
{
    if (mean <= 0.0) {
        this->bskLogger.bskError("IgbmNoiseStateEffector::setMean requires mean > 0.");
    }
    this->mean = mean;
}

void IgbmNoiseStateEffector::setStationaryStd(double sigmaStationary)
{
    if (sigmaStationary < 0.0) {
        this->bskLogger.bskError("IgbmNoiseStateEffector::setStationaryStd requires sigmaStationary >= 0.");
    }
    this->sigmaStationary = sigmaStationary;
}

void IgbmNoiseStateEffector::setTimeConstant(double timeConstant)
{
    if (timeConstant <= 0.0) {
        this->bskLogger.bskError("IgbmNoiseStateEffector::setTimeConstant requires timeConstant > 0.");
    }
    this->timeConstant = timeConstant;
}

double IgbmNoiseStateEffector::getStateValue() const
{
    if (this->state == nullptr) {
        return this->stateInitSet ? this->stateInit : this->mean - 1.0;
    }
    return this->state->getState()(0, 0);
}

void IgbmNoiseStateEffector::setStateValue(double val)
{
    // The state is the correction delta; the process is only well-posed from a positive
    // initial factor 1 + delta, so reject an initial condition that is zero or below.
    // (This guards the initial condition; the pure-math process is integrated as written,
    // so a consumer needing positivity clamps the derived quantity downstream.)
    if (val <= -1.0) {
        this->bskLogger.bskError("IgbmNoiseStateEffector::setStateValue requires the correction > -1 "
                                 "(the initial factor 1 + delta must be positive).");
    }
    this->stateInit = val;
    this->stateInitSet = true;
    if (this->state != nullptr) {
        Eigen::MatrixXd state(1, 1);
        state(0, 0) = val;
        this->state->setState(state);
    }
}

void IgbmNoiseStateEffector::registerStates(DynParamManager& states)
{
    this->state = states.registerState(1, 1, this->nameOfState);
    this->state->setNumNoiseSources(1);

    // Default the correction to mu - 1 (the factor 1 + delta at its mean level): the
    // factor's zero is degenerate for the multiplicative diffusion, so a
    // stationary-consistent start is used unless the user set an explicit value.
    Eigen::MatrixXd state(1, 1);
    state(0, 0) = this->stateInitSet ? this->stateInit : this->mean - 1.0;
    this->state->setState(state);
}

void IgbmNoiseStateEffector::linkInStates(DynParamManager& /** states */)
{
}

void IgbmNoiseStateEffector::computeDerivatives(double /** integTime */,
                                                Eigen::Vector3d /** rDDot_BN_N */,
                                                Eigen::Vector3d /** omegaDot_BN_B */,
                                                Eigen::Vector3d /** sigma_BN */)
{
    if (this->state == nullptr) {
        this->bskLogger.bskError("IgbmNoiseStateEffector::computeDerivatives called before registerStates.");
    }

    // The factor X = 1 + delta follows dX = (mu - X)/tau dt + sigma X dW, so the
    // correction delta inherits d(delta) = (mu - 1 - delta)/tau dt + sigma (1+delta) dW.
    const double factor = 1.0 + this->state->getState()(0, 0);

    Eigen::MatrixXd derivative(1, 1);
    derivative(0, 0) = (this->mean - factor) / this->timeConstant;
    this->state->setDerivative(derivative);

    // SDE volatility derived from the stationary-form parameters:
    // sigma^2 = (2/tau) sigma_st^2 / (mu^2 + sigma_st^2)
    const double sigma = std::sqrt(
        (2.0 / this->timeConstant) * this->sigmaStationary * this->sigmaStationary /
        (this->mean * this->mean + this->sigmaStationary * this->sigmaStationary));

    Eigen::MatrixXd diffusion(1, 1);
    diffusion(0, 0) = sigma * factor;
    this->state->setDiffusion(diffusion, 0);
}
