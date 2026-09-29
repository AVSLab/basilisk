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
#include "svStochasticIntegratorSIESME.h"

#include <array>
#include <cmath>

// Coefficients for the SIEA/SMEA/SIEB/SMEB tableaux (Tocino & Vigo-Aguiar).

svStochasticIntegratorSIESME::svStochasticIntegratorSIESME(DynamicObject* dyn, const SIESMECoefficients& coefficients)
  : svIntegratorWeakSIESME(dyn, coefficients)
{
}

void
svStochasticIntegratorSIESME::integrateImpl(double currentTime, double timeStep)
{
    if (timeStep == 0.0) {
        return;
    }

    auto& workspace = this->flatWorkspace;
    this->gatherStochasticStates();
    this->generateWienerNoise(timeStep);
    const Eigen::VectorXd& dW = this->flatDW();

    const SIESMECoefficients& c = this->coefficients;
    const size_t m = this->globalNoiseCount();
    const double h = timeStep;
    const double sqh = std::sqrt(h);
    Eigen::VectorXd& W2 = workspace.vector(0);
    Eigen::VectorXd& W3 = workspace.vector(1);
    Eigen::VectorXd& pseudoStep = workspace.vector(2);

    for (size_t k = 0; k < m; ++k) {
        const Eigen::Index index = static_cast<Eigen::Index>(k);
        const double increment = dW(index);
        W2(index) = increment * increment / sqh;
        W3(index) = c.nu2 * increment * increment * increment / h;
    }

    auto evaluateDrift = [&](double time, size_t stage) {
        this->evaluateDerivatives(time, timeStep);
        workspace.captureDrift(stage);
    };
    auto evaluateDiffusions = [&](double time, size_t stage) {
        this->evaluateDiffusions(time, timeStep);
        workspace.captureAllDiffusions(stage);
    };
    auto buildCandidate = [&](double driftStep, const Eigen::VectorXd& diffusionSteps) {
        this->buildStochasticCandidate(
          this->stochasticAcceptedState(), workspace.drift(), driftStep, workspace.diffusions(), diffusionSteps);
    };

    try {
        evaluateDrift(currentTime, 0);
        evaluateDiffusions(currentTime, 0);

        workspace.writeDrift(std::array<double, 2>{ c.lambda0, 0.0 }, 1);
        for (size_t k = 0; k < m; ++k) {
            workspace.writeDiffusionStage(k, 0);
            const Eigen::Index index = static_cast<Eigen::Index>(k);
            pseudoStep(index) = c.nu1 * dW(index) + W3(index);
        }
        buildCandidate(timeStep, pseudoStep);
        evaluateDrift(currentTime + c.mu0 * timeStep, 1);

        workspace.writeDrift(std::array<double, 2>{ c.lambdabar0, 0.0 }, 1);
        for (size_t k = 0; k < m; ++k) {
            workspace.writeDiffusionStage(k, 0);
            const Eigen::Index index = static_cast<Eigen::Index>(k);
            pseudoStep(index) = c.beta2 * sqh + c.beta3 * W2(index);
        }
        buildCandidate(timeStep, pseudoStep);
        evaluateDiffusions(currentTime + c.mubar0 * timeStep, 1);

        workspace.writeDrift(std::array<double, 2>{ c.lambdabar0, 0.0 }, 1);
        for (size_t k = 0; k < m; ++k) {
            workspace.writeDiffusionStage(k, 0);
            const Eigen::Index index = static_cast<Eigen::Index>(k);
            pseudoStep(index) = c.delta2 * sqh + c.delta3 * W2(index);
        }
        buildCandidate(timeStep, pseudoStep);
        evaluateDiffusions(currentTime + c.mubar0 * timeStep, 2);

        workspace.writeDrift(std::array<double, 2>{ c.alpha1, c.alpha2 }, 2);
        for (size_t k = 0; k < m; ++k) {
            workspace.writeDiffusionStage(k, 0);
            const Eigen::Index index = static_cast<Eigen::Index>(k);
            pseudoStep(index) = c.gamma1 * dW(index);
        }
        const bool coalesceFinalUpdate = this->stochasticUpdatesAreAllEuclidean();
        if (coalesceFinalUpdate) {
            this->beginAllEuclideanFinalCandidate(
              this->stochasticAcceptedState(), workspace.drift(), timeStep, workspace.diffusions(), pseudoStep);
        } else {
            buildCandidate(timeStep, pseudoStep);
        }

        if (!coalesceFinalUpdate) {
            this->acceptStochasticCandidate();
        }
        for (size_t k = 0; k < m; ++k) {
            workspace.writeDiffusionStage(k, 1);
            const Eigen::Index index = static_cast<Eigen::Index>(k);
            pseudoStep(index) = c.lambda1 * dW(index) + c.lambda2 * sqh + c.lambda3 * W2(index);
        }
        if (coalesceFinalUpdate) {
            this->appendAllEuclideanFinalCandidate(workspace.drift(), 0.0, workspace.diffusions(), pseudoStep);
        } else {
            buildCandidate(0.0, pseudoStep);
        }

        if (!coalesceFinalUpdate) {
            this->acceptStochasticCandidate();
        }
        for (size_t k = 0; k < m; ++k) {
            workspace.writeDiffusionStage(k, 2);
            const Eigen::Index index = static_cast<Eigen::Index>(k);
            pseudoStep(index) = c.mu1 * dW(index) + c.mu2 * sqh + c.mu3 * W2(index);
        }
        if (coalesceFinalUpdate) {
            this->appendAllEuclideanFinalCandidate(workspace.drift(), 0.0, workspace.diffusions(), pseudoStep);
            this->commitAllEuclideanFinalCandidate();
        } else {
            buildCandidate(0.0, pseudoStep);
        }
    } catch (...) {
        this->restoreStochasticStates();
        throw;
    }
}

svStochasticIntegratorSIEA::svStochasticIntegratorSIEA(DynamicObject* dyn)
  : svStochasticIntegratorSIESME(dyn, svStochasticIntegratorSIEA::getCoefficients())
{}

SIESMECoefficients svStochasticIntegratorSIEA::getCoefficients()
{
    SIESMECoefficients c;
    c.alpha1 = 0.5;  c.alpha2 = 0.5;
    c.gamma1 = 0.5;
    c.lambda1 = 0.25; c.lambda2 = -0.25; c.lambda3 = 0.25;
    c.mu1 = 0.25; c.mu2 = 0.25; c.mu3 = -0.25;
    c.mu0 = 1.0; c.mubar0 = 1.0;
    c.lambda0 = 1.0; c.lambdabar0 = 1.0;
    c.nu1 = 1.0; c.nu2 = 0.0;
    c.beta2 = 1.0; c.beta3 = 0.0;
    c.delta2 = -1.0; c.delta3 = 0.0;
    return c;
}

svStochasticIntegratorSMEA::svStochasticIntegratorSMEA(DynamicObject* dyn)
  : svStochasticIntegratorSIESME(dyn, svStochasticIntegratorSMEA::getCoefficients())
{}

SIESMECoefficients svStochasticIntegratorSMEA::getCoefficients()
{
    SIESMECoefficients c;
    c.alpha1 = 0.0;  c.alpha2 = 1.0;
    c.gamma1 = 0.5;
    c.lambda1 = 0.25; c.lambda2 = -0.25; c.lambda3 = 0.25;
    c.mu1 = 0.25; c.mu2 = 0.25; c.mu3 = -0.25;
    c.mu0 = 0.5; c.mubar0 = 1.0;
    c.lambda0 = 0.5; c.lambdabar0 = 1.0;
    c.nu1 = (2.0 - std::sqrt(6.0)) / 4.0; c.nu2 = std::sqrt(6.0) / 12.0;
    c.beta2 = 1.0; c.beta3 = 0.0;
    c.delta2 = -1.0; c.delta3 = 0.0;
    return c;
}

svStochasticIntegratorSIEB::svStochasticIntegratorSIEB(DynamicObject* dyn)
  : svStochasticIntegratorSIESME(dyn, svStochasticIntegratorSIEB::getCoefficients())
{}

SIESMECoefficients svStochasticIntegratorSIEB::getCoefficients()
{
    SIESMECoefficients c;
    c.alpha1 = 0.5;  c.alpha2 = 0.5;
    c.gamma1 = -0.2;
    c.lambda1 = 0.6; c.lambda2 = 1.5; c.lambda3 = -0.5;
    c.mu1 = 0.6; c.mu2 = -1.5; c.mu3 = 0.5;
    c.mu0 = 1.0; c.mubar0 = 5.0 / 12.0;
    c.lambda0 = 1.0; c.lambdabar0 = 5.0 / 12.0;
    c.nu1 = 1.0; c.nu2 = 0.0;
    c.beta2 = 0.0; c.beta3 = -1.0 / 6.0;
    c.delta2 = 0.0; c.delta3 = 1.0 / 6.0;
    return c;
}

svStochasticIntegratorSMEB::svStochasticIntegratorSMEB(DynamicObject* dyn)
  : svStochasticIntegratorSIESME(dyn, svStochasticIntegratorSMEB::getCoefficients())
{}

SIESMECoefficients svStochasticIntegratorSMEB::getCoefficients()
{
    SIESMECoefficients c;
    c.alpha1 = 0.0;  c.alpha2 = 1.0;
    c.gamma1 = -0.2;
    c.lambda1 = 0.6; c.lambda2 = 1.5; c.lambda3 = -0.5;
    c.mu1 = 0.6; c.mu2 = -1.5; c.mu3 = 0.5;
    c.mu0 = 0.5; c.mubar0 = 5.0 / 12.0;
    c.lambda0 = 0.5; c.lambdabar0 = 5.0 / 12.0;
    c.nu1 = (2.0 - std::sqrt(6.0)) / 4.0; c.nu2 = std::sqrt(6.0) / 12.0;
    c.beta2 = 0.0; c.beta3 = -1.0 / 6.0;
    c.delta2 = 0.0; c.delta3 = 1.0 / 6.0;
    return c;
}
