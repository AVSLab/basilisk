/*
 ISC License

 Copyright (c) 2024, University of Colorado at Boulder

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
#ifndef KALMAN_FILTER_INTERFACE_HPP
#define KALMAN_FILTER_INTERFACE_HPP

#include <Eigen/Dense>
#include <functional>
#include <optional>
#include <array>
#include "architecture/utilities/avsEigenSupport.h"
#include "architecture/utilities/macroDefinitions.h"
#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/messaging/messaging.h"
#include "fswAlgorithms/_GeneralModuleFiles/filterInterfaceDefinitions.h"
#include "fswAlgorithms/_GeneralModuleFiles/measurementModels.h"
#include "fswAlgorithms/_GeneralModuleFiles/stateModels.h"
#include "fswAlgorithms/_GeneralModuleFiles/dynamicModels.h"

/*! @brief Square Root unscented Kalman Filter base class */
class KalmanFilter: public SysModel  {
public:
    KalmanFilter();
    ~KalmanFilter() override;
    void Reset(uint64_t currentSimNanos) override;
    void UpdateState(uint64_t currentSimNanos) final;

    void setInitialPosition(const Eigen::VectorXd &initialPositionInput);
    std::optional<Eigen::VectorXd> getInitialPosition() const;
    void setInitialVelocity(const Eigen::VectorXd &initialVelocityInput);
    std::optional<Eigen::VectorXd> getInitialVelocity() const;
    void setInitialAcceleration(const Eigen::VectorXd &initialAccelerationInput);
    std::optional<Eigen::VectorXd> getInitialAcceleration() const;
    void setInitialBias(const Eigen::VectorXd &initialBiasInput);
    std::optional<Eigen::VectorXd> getInitialBias() const;
    void setInitialConsiderParameters(const Eigen::VectorXd &initialConsiderInput);
    std::optional<Eigen::VectorXd> getInitialConsiderParameters() const;

    void setInitialCovariance(const Eigen::MatrixXd &initialCovariance);
    Eigen::MatrixXd getInitialCovariance() const;
    void setProcessNoise(const Eigen::MatrixXd &processNoise);
    Eigen::MatrixXd getProcessNoise() const;
    void setUnitConversionFromSItoState(double conversion);
    double getUnitConversionFromSItoState() const;
    void setMeasurementNoiseScale(double measurementNoiseScale);
    double getMeasurementNoiseScale() const;
    void setFilterDynamics(const std::function<const StateVector(double, const StateVector&)> &dynamicsPropagator);

protected:
    virtual void customReset(){/* virtual */};
    virtual void customInitializeUpdate(){/* virtual */};
    virtual void customFinalizeUpdate(){/* virtual */};
    /*! Read method neads to read incoming messages containing the measurements for the filter.
     * Their information must be added to the Measurement container class, and added to the measurements vector.
     * Each measurement must be paired with a measurement model provided in the measurementModels.h */
    virtual void readFilterMeasurements(){/* virtual */};
    virtual void writeOutputMessages(uint64_t CurrentSimNanos){/* virtual */};

    virtual void timeUpdate(double updateTime)=0;
    virtual void measurementUpdate(const MeasurementModel &measurement)=0;
    virtual Eigen::VectorXd computeResiduals(const MeasurementModel &measurement)=0;
    void orderMeasurementsChronologically();

    std::array<std::optional<MeasurementModel>, MAX_MEASUREMENT_NUMBER> measurements;  //!< [Measurements] All
    //!< measurement containers in chronological order
    DynamicsModel dynamics;
    double previousFilterTimeTag = 0; //!< [s]  Time tag for state covar/etc
    double unitConversion = 1; //!< [-] Scale that converts input units (SI) to a desired unit for the inner maths
    double measNoiseScaling = 1; //!< [-] Scale factor for the measurement noise

    StateVector state; //!< [-] State estimate for time TimeTag
    StateVector stateLogged; //!< [-] State variable for logging
    Eigen::MatrixXd covar; //!< [-] covariance
    StateVector xBar; //!< [-] Current mean state estimate
    Eigen::VectorXd stateError; //!< [-] Current mean state error
    Eigen::MatrixXd processNoise; //!< [-] process noise matrix
    StateVector stateInitial; //!< [-] State estimate for time TimeTag at previous time
    Eigen::MatrixXd covarInitial; //!< [-] covariance at previous time
};

#endif /* KALMAN_FILTER_INTERFACE_HPP */
