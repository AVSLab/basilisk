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
#ifndef IGBM_NOISE_STATE_EFFECTOR_H
#define IGBM_NOISE_STATE_EFFECTOR_H

#include "simulation/dynamics/_GeneralModuleFiles/stateEffector.h"
#include "architecture/_GeneralModuleFiles/sys_model.h"

/*!
 * @brief State effector that propagates a scalar correction @f$\delta@f$ whose factor
 *        @f$(1+\delta)@f$ follows an inhomogeneous geometric Brownian motion (IGBM).
 *
 * The multiplicative factor @f$X = 1 + \delta@f$ follows the Ito SDE
 * @f[
 *     dX = \frac{1}{\tau}(\mu - X)\,dt + \sigma X\,dW,
 * @f]
 * so the registered correction state evolves as
 * @f[
 *     d\delta = \frac{\mu - 1 - \delta}{\tau}\,dt + \sigma (1+\delta)\,dW.
 * @f]
 * The correction form matches consumers that apply a @f$(1+\delta)@f$ correction (e.g.
 * ``DragDynamicEffector::densityCorrectionStateName``). The exact factor @f$X@f$ is
 * positive, but the module integrates the SDE as written (pure math), so an explicit
 * integrator can occasionally produce @f$X = 1+\delta \le 0@f$ for a large step or Wiener
 * increment. A consumer that needs a non-negative quantity should clamp it downstream:
 * ``DragDynamicEffector`` clamps the corrected density to be non-negative.
 *
 * Like the other IGBM/OU noise classes, the process is configured in stationary form:
 * the mean level @f$\mu@f$ of the factor, the time constant @f$\tau@f$, and the
 * stationary standard deviation @f$\sigma_{\mathrm{st}}@f$. The SDE volatility is derived
 * as @f$\sigma^2 = (2/\tau)\,\sigma_{\mathrm{st}}^2/(\mu^2 + \sigma_{\mathrm{st}}^2)@f$
 * so the factor's stationary mean and std are exactly @f$\mu@f$ and
 * @f$\sigma_{\mathrm{st}}@f$, and stationarity holds by construction.
 */
class IgbmNoiseStateEffector : public StateEffector, public SysModel {
public:
    /*! @brief Constructor. */
    IgbmNoiseStateEffector();
    /*! @brief Destructor. */
    ~IgbmNoiseStateEffector() override = default;

    /*!
     * @brief Register this effector's internal stochastic state with the dynamics manager.
     * @param states Dynamics parameter manager used to create the correction state.
     */
    void registerStates(DynParamManager& states) override;

    /*!
     * @brief Link any required external states and properties.
     * @param states Dynamics parameter manager.
     *
     * This effector has no required external state links.
     */
    void linkInStates(DynParamManager& states) override;

    /*!
     * @brief Compute deterministic and stochastic dynamics for the correction state.
     * @param integTime Integration time in seconds.
     * @param rDDot_BN_N Hub translational acceleration [m/s^2] (unused).
     * @param omegaDot_BN_B Hub angular acceleration [rad/s^2] (unused).
     * @param sigma_BN Hub attitude MRP (unused).
     */
    void computeDerivatives(double integTime,
                            Eigen::Vector3d rDDot_BN_N,
                            Eigen::Vector3d omegaDot_BN_B,
                            Eigen::Vector3d sigma_BN) override;

    /*!
     * @brief Set the mean level @f$\mu@f$ of the multiplicative factor @f$(1+\delta)@f$.
     * @param mean Mean level @f$\mu@f$ [-].
     *
     * Values less than or equal to zero are rejected.
     */
    void setMean(double mean);

    /*! @brief Get the mean level @f$\mu@f$ of the multiplicative factor [-]. */
    double getMean() const { return this->mean; }

    /*!
     * @brief Set the stationary standard deviation of the factor.
     * @param sigmaStationary Stationary standard deviation, @f$\sigma_{\mathrm{st}}@f$ [-].
     *
     * Values smaller than zero are rejected.
     */
    void setStationaryStd(double sigmaStationary);

    /*! @brief Get the stationary standard deviation @f$\sigma_{\mathrm{st}}@f$ [-]. */
    double getStationaryStd() const { return this->sigmaStationary; }

    /*!
     * @brief Set the IGBM time constant.
     * @param timeConstant Time constant @f$\tau@f$ [s].
     *
     * Values less than or equal to zero are rejected.
     */
    void setTimeConstant(double timeConstant);

    /*! @brief Get the IGBM time constant @f$\tau@f$ [s]. */
    double getTimeConstant() const { return this->timeConstant; }

    /*!
     * @brief Get the current state value.
     * @return Current scalar correction @f$\delta@f$ [-].
     *
     * If called before state registration, this returns the configured initial value
     * (which defaults to @f$\mu - 1@f$, i.e. the factor at its mean level).
     */
    double getStateValue() const;

    /*!
     * @brief Set the state value.
     * @param val New scalar correction @f$\delta@f$ [-]. Must be greater than @f$-1@f$ so
     *        the *initial* multiplicative factor @f$1 + \delta@f$ is positive (the
     *        process is only well-posed from a positive factor). This constrains the
     *        initial condition; it does not guarantee the numerically-integrated factor
     *        stays positive at every later step.
     *
     * This updates the configured initial value and also updates the registered
     * state immediately if registration has already occurred.
     */
    void setStateValue(double val);

    /*! @brief Get the registered state manager name. */
    std::string getStateName() const { return this->nameOfState; }

    /*! @brief Set the state manager name.
     * @param name State name.
     */
    void setStateName(std::string name) { this->nameOfState = name; }

private:
    static uint64_t effectorID;  //!< unique ID counter used to generate a distinct state name

    std::string nameOfState;      //!< state manager key for scalar correction @f$\delta@f$
    StateData* state = nullptr;   //!< pointer to scalar state container

    double mean = 1.0;             //!< mean level @f$\mu@f$ of the factor @f$(1+\delta)@f$ [-]
    double sigmaStationary = 0.0;  //!< stationary standard deviation @f$\sigma_{\mathrm{st}}@f$ [-]
    double timeConstant = 1.0;     //!< IGBM time constant @f$\tau@f$ [s]
    double stateInit = 0.0;        //!< initial value for @f$\delta@f$ used at state registration
    bool stateInitSet = false;     //!< whether the user set an explicit initial value
};

#endif
