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
#ifndef INHOMOGENEOUS_GEOMETRIC_BROWNIAN_MOTION_H
#define INHOMOGENEOUS_GEOMETRIC_BROWNIAN_MOTION_H

#include <cstdint>
#include <cmath>
#include <stdexcept>
#include <Eigen/Dense>

#include "architecture/utilities/bskLogging.h"
#include "architecture/messaging/messaging.h"
#include "simulation/mujocoDynamics/_GeneralModuleFiles/StatefulSysModel.h"
#include "simulation/mujocoDynamics/_GeneralModuleFiles/MJUtils.h"

/**
 * @class InhomogeneousGeometricBrownianMotion
 * @brief Base class that provides a scalar inhomogeneous geometric Brownian
 *        motion (IGBM) state x and its stochastic evolution. Derived classes
 *        implement output handling.
 *
 * The state x follows the Itô SDE
 * \f[
 *   \text{d}X_t = \frac{1}{\tau}\,(\mu - X_t)\,\text{d}t + \sigma\,X_t\,\text{d}W_t
 * \f]
 * so the drift mean-reverts toward \f$\mu\f$ with time constant \f$\tau\f$ while the
 * diffusion \f$\sigma X_t\f$ is multiplicative (proportional to the state, as
 * in geometric Brownian motion). This process is also known as the GARCH
 * diffusion or the mean-reverting geometric Brownian motion.
 *
 * Like MeanRevertingNoise, the process is configured in stationary form: the user sets
 * the mean level \f$\mu\f$, the time constant \f$\tau\f$, and the stationary standard
 * deviation \f$\sigma_{\text{st}}\f$. The SDE volatility is derived as
 * \f[
 *   \sigma^2 = \frac{2}{\tau}\,\frac{\sigma_{\text{st}}^2}{\mu^2 + \sigma_{\text{st}}^2}
 * \f]
 * so that the stationary moments are exactly
 * \f[
 *   \mathbb{E}[X_\infty] = \mu, \qquad
 *   \operatorname{Var}[X_\infty] = \sigma_{\text{st}}^2.
 * \f]
 * Because \f$\sigma_{\text{st}}^2/(\mu^2+\sigma_{\text{st}}^2) < 1\f$, this
 * parameterization automatically satisfies the stationarity condition
 * \f$2/\tau > \sigma^2\f$, so a stationary distribution always exists.
 *
 * Assuming \f$\mu > 0\f$ and the initial value \f$X_0 > 0\f$, the *exact* continuous
 * process stays strictly positive for all time. Note, however, that this base class
 * integrates the SDE as written (pure math): an **explicit** stochastic integrator has no
 * such guarantee, and for a large time step or a large Wiener increment a
 * numerically-integrated \f$X\f$ can occasionally cross zero. Consumers that require a
 * non-negative quantity (e.g. a density) should clamp the value they derive from \f$X\f$
 * downstream rather than assume positivity here.
 *
 * The base class:
 *   1) registers a 1-by-1 state with one noise source
 *   2) computes drift and diffusion each step
 *   3) calls the user hook writeOutput(CurrentSimNanos, x)
 */
class InhomogeneousGeometricBrownianMotion : public StatefulSysModel {
public:
    /**
     * @brief Default constructor. Parameters remain at defaults.
     */
    InhomogeneousGeometricBrownianMotion() = default;

    /**
     * @brief Register the scalar IGBM state with one noise source.
     * @param registerer Simulation state registerer.
     */
    void registerStates(DynParamRegisterer registerer) override;

    /**
     * @brief Update the IGBM state drift and diffusion, then call writeOutput.
     * @param CurrentSimNanos Current simulation time in nanoseconds.
     */
    void UpdateState(uint64_t CurrentSimNanos) override;

    /**
     * @brief Get the mean-reversion level \f$\mu\f$.
     */
    double getMean() const { return mean; }

    /**
     * @brief Set the mean-reversion level \f$\mu\f$.
     * @param m Long-run mean the process reverts toward. Positive expected.
     */
    void setMean(double m);

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

    /**
     * @brief Get \f$\theta = 1/\tau\f$ [s^{-1}].
     */
    double getTheta() const { return 1.0 / timeConstant; }

    /**
     * @brief Get the derived SDE volatility \f$\sigma\f$ [s^{-1/2}] multiplying
     *        \f$X_t\,\text{d}W_t\f$: \f$\sigma = \sqrt{(2/\tau)\,
     *        \sigma_{\text{st}}^2/(\mu^2+\sigma_{\text{st}}^2)}\f$.
     */
    double getSigma() const
    {
        return std::sqrt((2.0 / timeConstant) * sigmaStationary * sigmaStationary /
                         (mean * mean + sigmaStationary * sigmaStationary));
    }

    /**
     * @brief Get the current scalar state x.
     * @throws std::runtime_error if called before registerStates.
     */
    double getStateValue() const;

    /**
     * @brief Set the scalar state x.
     * @param val New value. Must be strictly positive, since the multiplicative process
     *            is only well-posed for a positive initial state.
     * @throws std::runtime_error if called before registerStates.
     */
    void setStateValue(double val);

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

    // Parameters in stationary form (dX = (mu - X)/tau dt + sigma X dW with sigma derived)
    double mean            = 1.0;  ///< Mean-reversion level mu.
    double sigmaStationary = 0.0;  ///< Stationary std dev of x.
    double timeConstant    = 1.0;  ///< Mean-reversion time constant in seconds.
};

#endif
