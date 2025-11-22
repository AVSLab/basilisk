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
#include "meanRevertingNoise.h"

void MeanRevertingNoise::registerStates(DynParamRegisterer registerer) {
    xState = registerer.registerState(1, 1, "meanRevertingState");
    xState->setNumNoiseSources(1);
    // Default initial condition
    xState->setState(Eigen::Matrix<double, 1, 1>::Constant(0.0));
}

void MeanRevertingNoise::UpdateState(uint64_t CurrentSimNanos) {
    // Safety
    if (!xState) {
        MJBasilisk::detail::logAndThrow<std::runtime_error>(
            "MeanRevertingNoise: state not registered",
            &bskLogger
        );
    }

    // Current x
    const Eigen::MatrixXd &x = xState->getState();
    const double x0 = x(0, 0);

    // Drift: dx = -theta * x
    Eigen::Matrix<double, 1, 1> drift;
    drift(0, 0) = -(1.0 / timeConstant) * x0;
    xState->setDerivative(drift);

    // Diffusion: sigma * dW on noise source 0
    Eigen::Matrix<double, 1, 1> diff;
    diff(0, 0) = sigmaStationary * std::sqrt(2.0 / timeConstant);
    xState->setDiffusion(diff, 0);

    // Delegate output
    writeOutput(CurrentSimNanos, x0);
}

double MeanRevertingNoise::getStateValue() const {
    if (!xState) {
        MJBasilisk::detail::logAndThrow<std::runtime_error>(
            "MeanRevertingNoise: getStateValue before initialization",
            &const_cast<BSKLogger&>(bskLogger)
        );
    }
    return xState->getState()(0, 0);
}

void MeanRevertingNoise::setStateValue(double val) {
    if (!xState) {
        MJBasilisk::detail::logAndThrow<std::runtime_error>(
            "MeanRevertingNoise: setStateValue before initialization",
            &bskLogger
        );
    }
    xState->setState(Eigen::Matrix<double, 1, 1>::Constant(val));
}

void MeanRevertingNoise::setTimeConstant(double t) {
    if (t <= 0.0) {
        MJBasilisk::detail::logAndThrow<std::invalid_argument>(
            "MeanRevertingNoise::setTimeConstant: tau must be > 0",
            &bskLogger
        );
    }
    timeConstant = t;
}

void MeanRevertingNoise::setStationaryStd(double s) {
    if (s < 0.0) {
        MJBasilisk::detail::logAndThrow<std::invalid_argument>(
            "MeanRevertingNoise::setStationaryStd: sigma_st must be >= 0",
            &bskLogger
        );
    }
    sigmaStationary = s;
}
