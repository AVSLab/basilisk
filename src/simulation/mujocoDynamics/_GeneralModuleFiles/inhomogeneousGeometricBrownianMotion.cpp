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
#include "inhomogeneousGeometricBrownianMotion.h"

void InhomogeneousGeometricBrownianMotion::registerStates(DynParamRegisterer registerer) {
    xState = registerer.registerState(1, 1, "inhomogeneousGBMState");
    xState->setNumNoiseSources(1);
    // Default initial condition: start at the mean-reversion level mu. For this
    // multiplicative process x = 0 is degenerate (the diffusion sigma*x vanishes there),
    // so starting at mu keeps the process well-posed and stationary-consistent when the
    // caller does not set an explicit initial value.
    xState->setState(Eigen::Matrix<double, 1, 1>::Constant(mean));
}

void InhomogeneousGeometricBrownianMotion::UpdateState(uint64_t CurrentSimNanos) {
    // Safety
    if (!xState) {
        bskLogger.bskError("InhomogeneousGeometricBrownianMotion: state not registered");
    }

    // Current x
    const Eigen::MatrixXd &x = xState->getState();
    const double x0 = x(0, 0);

    // Drift: dx = (mu - x) / tau
    Eigen::Matrix<double, 1, 1> drift;
    drift(0, 0) = (mean - x0) / timeConstant;
    xState->setDerivative(drift);

    // Diffusion: sigma * x * dW on noise source 0 (multiplicative noise), with sigma
    // derived from the stationary-form parameters
    Eigen::Matrix<double, 1, 1> diff;
    diff(0, 0) = getSigma() * x0;
    xState->setDiffusion(diff, 0);

    // Delegate output
    writeOutput(CurrentSimNanos, x0);
}

double InhomogeneousGeometricBrownianMotion::getStateValue() const {
    if (!xState) {
        BSKLogger{}.bskError("InhomogeneousGeometricBrownianMotion: getStateValue before initialization");
    }
    return xState->getState()(0, 0);
}

void InhomogeneousGeometricBrownianMotion::setStateValue(double val) {
    if (!xState) {
        bskLogger.bskError("InhomogeneousGeometricBrownianMotion: setStateValue before initialization");
    }
    // The multiplicative process is only well-posed for a positive state (the drift
    // reverts toward mu > 0 and the diffusion sigma*x vanishes at x = 0), so a
    // non-positive initial value is rejected.
    if (val <= 0.0) {
        bskLogger.bskError("InhomogeneousGeometricBrownianMotion::setStateValue: state must be > 0");
    }
    xState->setState(Eigen::Matrix<double, 1, 1>::Constant(val));
}

void InhomogeneousGeometricBrownianMotion::setMean(double m) {
    if (m <= 0.0) {
        bskLogger.bskError("InhomogeneousGeometricBrownianMotion::setMean: mu must be > 0");
    }
    mean = m;
}

void InhomogeneousGeometricBrownianMotion::setTimeConstant(double t) {
    if (t <= 0.0) {
        bskLogger.bskError("InhomogeneousGeometricBrownianMotion::setTimeConstant: tau must be > 0");
    }
    timeConstant = t;
}

void InhomogeneousGeometricBrownianMotion::setStationaryStd(double s) {
    if (s < 0.0) {
        bskLogger.bskError("InhomogeneousGeometricBrownianMotion::setStationaryStd: sigma_st must be >= 0");
    }
    sigmaStationary = s;
}
