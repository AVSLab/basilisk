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
#ifndef MEAN_REVERTING_NOISE_H
#define MEAN_REVERTING_NOISE_H

#include <cstdint>
#include <cmath>
#include <stdexcept>
#include <Eigen/Dense>

#include "architecture/utilities/bskLogging.h"
#include "architecture/messaging/messaging.h"
#include "simulation/mujocoDynamics/_GeneralModuleFiles/StatefulSysModel.h"
#include "simulation/mujocoDynamics/_GeneralModuleFiles/MJUtils.h"

/**
 * @class MeanRevertingNoise
 * @brief Base class that provides a scalar Ornstein–Uhlenbeck state x and its
 *        stochastic evolution. Derived classes implement output handling.
 *
 * The state x follows
 * \f[
 *   dx = -\frac{1}{\tau}\,x\,dt + \sqrt{\frac{2}{\tau}}\,\sigma_{\text{st}}\,dW
 * \f]
 * The base class:
 *   1) registers a 1-by-1 state with one noise source
 *   2) computes drift and diffusion each step
 *   3) calls the user hook writeOutput(CurrentSimNanos, x)
 */
class MeanRevertingNoise : public StatefulSysModel {
public:
    /**
     * @brief Default constructor. Parameters remain at defaults.
     */
    MeanRevertingNoise() = default;

    /** @name Framework interface @{ */

    /**
     * @brief Register the scalar OU state with one noise source.
     * @param registerer Simulation state registerer.
     */
    void registerStates(DynParamRegisterer registerer) override;

    /**
     * @brief Update the OU state drift and diffusion, then call writeOutput.
     * @param CurrentSimNanos Current simulation time in nanoseconds.
     */
    void UpdateState(uint64_t CurrentSimNanos) override;

    /** @} */

    /** @name Parameter getters and setters in stationary form @{ */

    /**
     * @brief Get stationary standard deviation \f$\sigma_{\text{st}}\f$.
     */
    double getStationaryStd() const { return sigmaStationary; }
    /**
     * @brief Set stationary standard deviation \f$\sigma_{\text{st}}\f$.
     * @param s Nonnegative expected.
     */
    void setStationaryStd(double s);

    /**
     * @brief Get time constant \f$\tau\f$ [s].
     */
    double getTimeConstant() const { return timeConstant; }

    /**
     * @brief Set time constant \f$\tau\f$ [s].
     * @param t Positive expected.
     */
    void setTimeConstant(double t);

    /** @} */

    /** @name Canonical OU parameter accessors @{ */

    /**
     * @brief Get \f$\theta = 1/\tau\f$ [s^{-1}].
     */
    double getTheta() const { return 1.0 / timeConstant; }

    /**
     * @brief Get diffusion amplitude \f$\sigma = \sqrt{2/\tau}\,\sigma_{\text{st}}\f$.
     */
    double getSigma() const { return sigmaStationary * std::sqrt(2.0 / timeConstant); }

    /** @} */

    /** @name State access @{ */

    /**
     * @brief Get the current scalar state x.
     * @throws std::runtime_error if called before registerStates.
     */
    double getStateValue() const;

    /**
     * @brief Set the scalar state x.
     * @param val New value.
     * @throws std::runtime_error if called before registerStates.
     */
    void setStateValue(double val);

    /** @} */

    /// Logger
    BSKLogger bskLogger;

protected:
    /**
     * @brief Pure virtual output hook. Called every step after drift and diffusion
     *        are set. Use this to read inputs if needed and write outputs.
     * @param CurrentSimNanos Current simulation time in nanoseconds.
     * @param x Current scalar state value.
     */
    virtual void writeOutput(uint64_t CurrentSimNanos, double x) = 0;

private:
    // State storage
    StateData *xState = nullptr;

    // Parameters in stationary form
    double sigmaStationary = 0.0;  ///< Stationary std dev of x.
    double timeConstant    = 1.0;  ///< OU time constant in seconds.
};

#endif
