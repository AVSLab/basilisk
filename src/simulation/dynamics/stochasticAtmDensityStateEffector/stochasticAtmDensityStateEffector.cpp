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
#include "stochasticAtmDensityStateEffector.h"

#include "architecture/utilities/macroDefinitions.h"

#include <cmath>

uint64_t StochasticAtmDensityStateEffector::effectorID = 1;

StochasticAtmDensityStateEffector::StochasticAtmDensityStateEffector()
{
    this->effProps.mEff = 0.0;
    this->effProps.rEff_CB_B.setZero();
    this->effProps.IEffPntB_B.setZero();
    this->effProps.rEffPrime_CB_B.setZero();
    this->effProps.IEffPrimePntB_B.setZero();

    this->nameOfCorrectionState = "stochasticAtmDensityCorrectionState"
                                  + std::to_string(StochasticAtmDensityStateEffector::effectorID);
    StochasticAtmDensityStateEffector::effectorID++;
}

void StochasticAtmDensityStateEffector::setStationaryStd(double sigmaStationary)
{
    if (sigmaStationary < 0.0) {
        this->bskLogger.bskLog(BSK_ERROR,
            "StochasticAtmDensityStateEffector::setStationaryStd requires sigmaStationary >= 0.");
        return;
    }
    this->sigmaStationary = sigmaStationary;
}

void StochasticAtmDensityStateEffector::setTimeConstant(double timeConstant)
{
    if (timeConstant <= 0.0) {
        this->bskLogger.bskLog(BSK_ERROR,
            "StochasticAtmDensityStateEffector::setTimeConstant requires timeConstant > 0.");
        return;
    }
    this->timeConstant = timeConstant;
}

double StochasticAtmDensityStateEffector::getStateValue() const
{
    if (this->correctionState == nullptr) {
        return this->correctionInit;
    }
    return this->correctionState->getState()(0, 0);
}

void StochasticAtmDensityStateEffector::setStateValue(double val)
{
    this->correctionInit = val;
    if (this->correctionState != nullptr) {
        Eigen::MatrixXd state(1, 1);
        state(0, 0) = val;
        this->correctionState->setState(state);
    }
}

void StochasticAtmDensityStateEffector::registerStates(DynParamManager& states)
{
    this->correctionState = states.registerState(1, 1, this->nameOfCorrectionState);
    this->correctionState->setNumNoiseSources(1);

    Eigen::MatrixXd state(1, 1);
    state(0, 0) = this->correctionInit;
    this->correctionState->setState(state);
}

void StochasticAtmDensityStateEffector::linkInStates(DynParamManager& /** states */)
{
}

void StochasticAtmDensityStateEffector::computeDerivatives(double integTime,
                                                           Eigen::Vector3d /** rDDot_BN_N */,
                                                           Eigen::Vector3d /** omegaDot_BN_B */,
                                                           Eigen::Vector3d /** sigma_BN */)
{
    if (this->correctionState == nullptr) {
        this->bskLogger.bskLog(BSK_ERROR,
            "StochasticAtmDensityStateEffector::computeDerivatives called before registerStates.");
        return;
    }

    const double x = this->correctionState->getState()(0, 0);

    Eigen::MatrixXd derivative(1, 1);
    derivative(0, 0) = -(1.0 / this->timeConstant) * x;
    this->correctionState->setDerivative(derivative);

    Eigen::MatrixXd diffusion(1, 1);
    diffusion(0, 0) = this->sigmaStationary * std::sqrt(2.0 / this->timeConstant);
    this->correctionState->setDiffusion(diffusion, 0);

    // unusual, but we need to write the output message at the integration sub-step
    // time so the DragDynamicEffector can see the corrected density also at the integrator
    // substep
    this->writeOutputStateMessages( secToNano(integTime) );
}

void StochasticAtmDensityStateEffector::writeOutputStateMessages(uint64_t integTimeNanos)
{
    if (!this->atmoDensInMsg.isLinked()) {
        if (!this->hasWarnedNoInputMsg) {
            this->bskLogger.bskLog(BSK_ERROR,
                "StochasticAtmDensityStateEffector.atmoDensInMsg was not linked.");
        }
        return;
    }

    AtmoPropsMsgPayload out = this->atmoDensInMsg();
    out.neutralDensity *= (1.0 + this->getStateValue());
    this->atmoDensOutMsg.write(&out, this->moduleID, integTimeNanos);
}
