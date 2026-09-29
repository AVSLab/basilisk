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
#include "svStochasticIntegratorRS.h"
#include "../_GeneralModuleFiles/stochasticWeakRandomVariables.h"

#include <cmath>

svStochasticIntegratorRS::svStochasticIntegratorRS(DynamicObject* dynIn,
                                                   const RSCoefficients& coefficients)
    : StochasticRKIntegratorBase(dynIn), coefficients(coefficients)
{
}

// ---- RS1 / RS2 coefficients (RS1 / RS2 tableaux) ----

svStochasticIntegratorRS1::svStochasticIntegratorRS1(DynamicObject* dyn)
    : svStochasticIntegratorRS(dyn, svStochasticIntegratorRS1::getCoefficients())
{}

RSCoefficients svStochasticIntegratorRS1::getCoefficients()
{
    RSCoefficients c;
    c.a021 = 0.0; c.a031 = 1.0; c.a032 = 0.0;
    c.a131 = 1.0; c.a141 = 1.0;
    c.b031 = 1.0 / 4.0; c.b032 = 3.0 / 4.0;
    c.b121 = 2.0 / 3.0;
    c.b131 = 1.0 / 12.0; c.b132 = 1.0 / 4.0;
    c.b141 = -5.0 / 4.0; c.b142 = 1.0 / 4.0; c.b143 = 2.0;
    c.b221 = 1.0; c.b231 = -1.0;
    c.b331 = 1.0 / 4.0; c.b332 = 3.0 / 4.0;
    c.b341 = 1.0 / 4.0; c.b342 = 3.0 / 4.0;
    c.alpha1 = 0.0; c.alpha2 = 0.0; c.alpha3 = 1.0 / 2.0; c.alpha4 = 1.0 / 2.0;
    c.c02 = 0.0; c.c03 = 1.0; c.c13 = 1.0; c.c14 = 1.0;
    c.beta11 = 1.0 / 8.0; c.beta12 = 3.0 / 8.0; c.beta13 = 3.0 / 8.0; c.beta14 = 1.0 / 8.0;
    c.beta22 = -1.0 / 4.0; c.beta23 = 1.0 / 4.0;
    return c;
}

svStochasticIntegratorRS2::svStochasticIntegratorRS2(DynamicObject* dyn)
    : svStochasticIntegratorRS(dyn, svStochasticIntegratorRS2::getCoefficients())
{}

RSCoefficients svStochasticIntegratorRS2::getCoefficients()
{
    RSCoefficients c;
    c.a021 = 2.0 / 3.0; c.a031 = 1.0 / 6.0; c.a032 = 1.0 / 2.0;
    c.a131 = 1.0; c.a141 = 1.0;
    c.b031 = 1.0 / 4.0; c.b032 = 3.0 / 4.0;
    c.b121 = 2.0 / 3.0;
    c.b131 = 1.0 / 12.0; c.b132 = 1.0 / 4.0;
    c.b141 = -5.0 / 4.0; c.b142 = 1.0 / 4.0; c.b143 = 2.0;
    c.b221 = 1.0; c.b231 = -1.0;
    c.b331 = 1.0 / 4.0; c.b332 = 3.0 / 4.0;
    c.b341 = 1.0 / 4.0; c.b342 = 3.0 / 4.0;
    c.alpha1 = 1.0 / 4.0; c.alpha2 = 1.0 / 4.0; c.alpha3 = 1.0 / 2.0; c.alpha4 = 0.0;
    c.c02 = 2.0 / 3.0; c.c03 = 2.0 / 3.0; c.c13 = 1.0; c.c14 = 1.0;
    c.beta11 = 1.0 / 8.0; c.beta12 = 3.0 / 8.0; c.beta13 = 3.0 / 8.0; c.beta14 = 1.0 / 8.0;
    c.beta22 = -1.0 / 4.0; c.beta23 = 1.0 / 4.0;
    return c;
}

void
svStochasticIntegratorRS::integrateImpl(double currentTime, double timeStep)
{
    if (timeStep == 0.0) {
        return;
    }

    this->gatherStochasticStates();
    const size_t m = this->globalNoiseCount();
    this->generateNoise(timeStep, m == 0 ? 0 : m - 1);

    auto& workspace = this->flatWorkspace;
    const RSCoefficients& c = this->coefficients;
    const double h = timeStep;
    const double sqh = std::sqrt(h);
    Eigen::VectorXd& Ihat = workspace.vector(0);
    Eigen::VectorXd& Itilde = workspace.vector(1);
    Eigen::VectorXd& pseudoStep = workspace.vector(2);
    Eigen::VectorXd& sqrtStep = workspace.vector(3);
    for (size_t k = 0; k < m; k++) {
        Ihat(static_cast<Eigen::Index>(k)) =
          stochasticWeakRV::threePoint(this->flatDW()(static_cast<Eigen::Index>(k)), h);
        Itilde(static_cast<Eigen::Index>(k)) = 0.0;
        sqrtStep(static_cast<Eigen::Index>(k)) = sqh;
    }
    for (size_t k = 0; k + 1 < m; k++) {
        Itilde(static_cast<Eigen::Index>(k)) =
          stochasticWeakRV::twoPoint(this->flatDZ()(static_cast<Eigen::Index>(k)), sqh);
    }
    auto Ihat2 = [&](size_t k, size_t l) -> double {
        if (l < k) {
            return Ihat(static_cast<Eigen::Index>(k)) * Itilde(static_cast<Eigen::Index>(l));
        }
        return -Ihat(static_cast<Eigen::Index>(l)) * Itilde(static_cast<Eigen::Index>(k));
    };

    auto evaluateDrift = [&](double time, size_t stage) {
        this->evaluateDerivatives(time, timeStep);
        workspace.captureDrift(stage);
    };
    auto evaluateAllDiffusions = [&](double time, size_t stage) {
        this->evaluateDiffusions(time, timeStep);
        workspace.captureAllDiffusions(stage);
    };
    auto evaluateDiffusion = [&](double time, size_t slot, size_t stage) {
        this->evaluateDiffusions(time, timeStep);
        workspace.captureDiffusion(slot, stage);
    };
    auto buildCandidate = [&](double driftStep, const Eigen::VectorXd& diffusionSteps) {
        this->buildStochasticCandidate(
          this->stochasticAcceptedState(), workspace.drift(), driftStep, workspace.diffusions(), diffusionSteps);
    };

    auto row = [](double x0, double x1, double x2, double x3) {
        return std::array<double, 4>{x0, x1, x2, x3};
    };
    const std::array<std::array<double, 4>, 4> A0 = {
        row(0, 0, 0, 0), row(c.a021, 0, 0, 0), row(c.a031, c.a032, 0, 0), row(0, 0, 0, 0)};
    const std::array<std::array<double, 4>, 4> A1 = {
        row(0, 0, 0, 0), row(0, 0, 0, 0), row(c.a131, 0, 0, 0), row(c.a141, 0, 0, 0)};
    const std::array<std::array<double, 4>, 4> B0 = {
        row(0, 0, 0, 0), row(0, 0, 0, 0), row(c.b031, c.b032, 0, 0), row(0, 0, 0, 0)};
    const std::array<std::array<double, 4>, 4> B1 = {
        row(0, 0, 0, 0), row(c.b121, 0, 0, 0), row(c.b131, c.b132, 0, 0),
        row(c.b141, c.b142, c.b143, 0)};
    const std::array<std::array<double, 4>, 4> B3 = {
        row(0, 0, 0, 0), row(0, 0, 0, 0), row(c.b331, c.b332, 0, 0), row(c.b341, c.b342, 0, 0)};
    const std::array<double, 4> c0nodes = {0.0, c.c02, c.c03, 0.0};
    const std::array<double, 4> c1nodes = {0.0, 0.0, c.c13, c.c14};

    try {
        evaluateDrift(currentTime, 0);
        evaluateAllDiffusions(currentTime, 0);

        for (size_t i = 1; i < 4; i++) {
            workspace.writeDrift(A0.at(i), i);
            for (size_t l = 0; l < m; l++) {
                workspace.writeDiffusion(l, B0.at(i), i);
            }
            buildCandidate(timeStep, Ihat);
            evaluateDrift(currentTime + c0nodes.at(i) * timeStep, i);

            workspace.writeDrift(A1.at(i), i);
            for (size_t l = 0; l < m; l++) {
                workspace.writeDiffusion(l, B3.at(i), i);
            }
            for (size_t k = 0; k < m; k++) {
                workspace.writeDiffusion(k, B1.at(i), i);
                buildCandidate(timeStep, Ihat);
                evaluateDiffusion(currentTime + c1nodes.at(i) * timeStep, k, i);
                workspace.writeDiffusion(k, B3.at(i), i);
            }
        }

        const std::array<std::array<double, 4>, 4> B2 = {
            row(0, 0, 0, 0), row(c.b221, 0, 0, 0), row(c.b231, 0, 0, 0), row(0, 0, 0, 0)
        };
        if (m > 1) {
            for (size_t k = 0; k < m; k++) {
                workspace.copyDiffusion(k, 4, 0);
                for (size_t i = 1; i < 4; i++) {
                    pseudoStep(static_cast<Eigen::Index>(k)) = 0.0;
                    for (size_t l = 0; l < m; l++) {
                        if (l == k) {
                            continue;
                        }
                        workspace.writeDiffusion(l, B2.at(i), i);
                        pseudoStep(static_cast<Eigen::Index>(l)) = Ihat2(k, l) / sqh;
                    }
                    buildCandidate(0.0, pseudoStep);
                    evaluateDiffusion(currentTime, k, 4 + i);
                }
            }
        }

        const std::array<double, 4> alpha = {c.alpha1, c.alpha2, c.alpha3, c.alpha4};
        workspace.writeDrift(alpha, 4);
        const std::array<double, 4> beta1 = {c.beta11, c.beta12, c.beta13, c.beta14};
        for (size_t k = 0; k < m; k++) {
            workspace.writeDiffusion(k, beta1, 4);
        }
        const bool coalesceFinalUpdate = this->stochasticUpdatesAreAllEuclidean();
        if (coalesceFinalUpdate) {
            this->beginAllEuclideanFinalCandidate(
              this->stochasticAcceptedState(), workspace.drift(), timeStep, workspace.diffusions(), Ihat);
        } else {
            buildCandidate(timeStep, Ihat);
        }

        if (m > 1) {
            if (!coalesceFinalUpdate) {
                this->acceptStochasticCandidate();
            }
            const std::array<double, 4> beta2 = { 0.0, c.beta22, c.beta23, 0.0 };
            for (size_t k = 0; k < m; k++) {
                workspace.writeDiffusion(k, beta2, 4, 4);
            }
            if (coalesceFinalUpdate) {
                this->appendAllEuclideanFinalCandidate(workspace.drift(), 0.0, workspace.diffusions(), sqrtStep);
            } else {
                buildCandidate(0.0, sqrtStep);
            }
        }
        if (coalesceFinalUpdate) {
            this->commitAllEuclideanFinalCandidate();
        }
    } catch (...) {
        this->restoreStochasticStates();
        throw;
    }
}
