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
#ifndef STOCHASTIC_ATM_DENSITY_STATE_EFFECTOR_H
#define STOCHASTIC_ATM_DENSITY_STATE_EFFECTOR_H

#include "simulation/dynamics/_GeneralModuleFiles/stateEffector.h"
#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/messaging/messaging.h"
#include "architecture/msgPayloadDefC/AtmoPropsMsgPayload.h"

/*!
 * @brief State effector that applies an Ornstein-Uhlenbeck correction to atmospheric density.
 *
 * This module propagates a scalar stochastic state @f$x@f$ using
 * @f[
 *     dx = -\frac{1}{\tau}x\,dt + \sqrt{\frac{2}{\tau}}\sigma_{\mathrm{st}}\,dW,
 * @f]
 * where @f$\tau@f$ is the time constant and @f$\sigma_{\mathrm{st}}@f$ is the stationary
 * standard deviation.
 *
 * At each output write, the incoming atmospheric density is corrected as
 * @f[
 *     \rho_{\mathrm{out}} = \rho_{\mathrm{in}}(1+x),
 * @f]
 * while all other atmospheric message fields pass through unchanged.
 */
class StochasticAtmDensityStateEffector : public StateEffector, public SysModel {
public:
    /*! @brief Constructor. */
    StochasticAtmDensityStateEffector();
    /*! @brief Destructor. */
    ~StochasticAtmDensityStateEffector() override = default;

    /*! @brief Input atmosphere properties message. */
    ReadFunctor<AtmoPropsMsgPayload> atmoDensInMsg;
    /*! @brief Output atmosphere properties message with stochastic density correction applied. */
    Message<AtmoPropsMsgPayload> atmoDensOutMsg;

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
     * @brief Write the corrected atmospheric properties output message.
     * @param integTimeNanos Current integration time in nanoseconds.
     */
    void writeOutputStateMessages(uint64_t integTimeNanos) override;

    /*!
     * @brief Set the stationary standard deviation of the OU state.
     * @param sigmaStationary Stationary standard deviation, @f$\sigma_{\mathrm{st}}@f$ [-].
     *
     * Values smaller than zero are rejected.
     */
    void setStationaryStd(double sigmaStationary);
    /*! @brief Get the stationary standard deviation @f$\sigma_{\mathrm{st}}@f$ [-]. */
    double getStationaryStd() const { return this->sigmaStationary; }

    /*!
     * @brief Set the OU time constant.
     * @param timeConstant Time constant @f$\tau@f$ [s].
     *
     * Values less than or equal to zero are rejected.
     */
    void setTimeConstant(double timeConstant);
    /*! @brief Get the OU time constant @f$\tau@f$ [s]. */
    double getTimeConstant() const { return this->timeConstant; }

    /*!
     * @brief Get the current correction state value.
     * @return Current scalar correction state @f$x@f$ [-].
     *
     * If called before state registration, this returns the configured initial value.
     */
    double getStateValue() const;
    /*!
     * @brief Set the correction state value.
     * @param val New scalar correction state @f$x@f$ [-].
     *
     * This updates the configured initial value and also updates the registered
     * state immediately if registration has already occurred.
     */
    void setStateValue(double val);

private:
    static uint64_t effectorID;  //!< unique ID counter used to generate a distinct state name

    std::string nameOfCorrectionState;      //!< state manager key for scalar correction state @f$x@f$
    StateData* correctionState = nullptr;   //!< pointer to scalar correction state container

    double sigmaStationary = 0.0;  //!< stationary standard deviation @f$\sigma_{\mathrm{st}}@f$ [-]
    double timeConstant = 1.0;     //!< OU time constant @f$\tau@f$ [s]
    double correctionInit = 0.0;   //!< initial value for @f$x@f$ used at state registration

    bool hasWarnedNoInputMsg = false;  //!< true after first missing-input warning is emitted
};

#endif
