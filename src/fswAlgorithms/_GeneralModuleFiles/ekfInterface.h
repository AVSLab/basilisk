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
#ifndef EKF_INTERFACE_HPP
#define EKF_INTERFACE_HPP

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

/*! Enumerator class to set if the filter is meant to be used purely as a linear/classical KF,
 * no reference state updates will be performed in the classical filter, while the EKF updates the reference
 * with the computed state error at each measurement update */
enum class FilterType {Classical, Extended};

/*! @brief Extended or Classical/Linear Kalman Filter base class. */
class EkfInterface: public SysModel  {
public:
    EkfInterface(FilterType type);
    ~EkfInterface() override;
    void Reset(uint64_t CurrentSimNanos) override;
    void UpdateState(uint64_t CurrentSimNanos) override;

    void setMinimumCovarianceNormForEkf(double infiniteNorm);
    double getMinimumCovarianceNormForEkf() const;
    void setInitialState(const Eigen::VectorXd &initialState);
    Eigen::VectorXd getInitialState() const;
    void setInitialCovariance(const Eigen::MatrixXd &initialCovariance);
    Eigen::MatrixXd getInitialCovariance() const;
    void setProcessNoise(const Eigen::MatrixXd &processNoise);
    Eigen::MatrixXd getProcessNoise() const;
    void setUnitConversionFromSItoState(double conversion);
    double getUnitConversionFromSItoState() const ;
    void setConstantRateStates(const Eigen::VectorXd &rateStates);
    Eigen::VectorXd getConstantRateStates() const;

protected:
    virtual void customReset(){/* virtual */};
    virtual void customInitializeUpdate(){/* virtual */};
    virtual void customFinalizeUpdate(){/* virtual */};
    virtual Eigen::VectorXd propagate(std::array<double, 2> interval, const Eigen::VectorXd& X0, double dt)=0;
    virtual Eigen::MatrixXd computeDynamicsMatrix(const Eigen::VectorXd& state)=0;
    virtual Eigen::MatrixXd computeMeasurementMatrix(const Eigen::VectorXd& state)=0;
    /*! Read method neads to read incoming messages containing the measurements for the filter.
     * Their information must be added to the Measurement container class, and added to the measurements vector.
     * Each measurement must be paired with a measurement model provided in the measurementModels.h */
    virtual void readFilterMeasurements()=0;
    virtual void writeOutputMessages(uint64_t CurrentSimNanos)=0;

    Eigen::VectorXd rk4(const std::function<Eigen::VectorXd(double, Eigen::VectorXd)>& ODEfunction,
                const Eigen::VectorXd& X0,
                double t0,
                double dt) const;

    std::array<std::optional<Measurement>, MAX_MEASUREMENT_NUMBER> measurements;  //!< [Measurements] All measurement containers in chronological order
    double previousFilterTimeTag = 0; //!< [s]  Last filter time-tag
    double unitConversion = 1; //!< [-] Scale that converts input units (SI) to a desired unit for the inner maths
    bool updatedWithCkf = true; //!< [-] Flag to signal that filter was last updated with a Linear measurement update

    Eigen::VectorXd state; //!< [-] State estimate for time TimeTag
    Eigen::VectorXd stateError; //!< [-] State error for time TimeTag
    Eigen::VectorXd constantRateStates; //!< [-] Constant rate states if the filter doesn not estimate them
    Eigen::MatrixXd stateTransitionMatrix; //!< [-] State Transition Matrix
    Eigen::MatrixXd covar; //!< [-] covariance

private:
    void timeUpdate(const double updateTime);
    void measurementUpdate(const Measurement &measurement);
    Eigen::VectorXd computeResiduals(const Measurement &measurement);
    Eigen::MatrixXd computeKalmanGain(const Eigen::MatrixXd &covar,
                                    const Eigen::MatrixXd &measurementMatrix,
                                    const Eigen::MatrixXd &measurementNoise) const;
    void updateCovariance(const Eigen::MatrixXd &measMat, const Eigen::MatrixXd &noise, const Eigen::MatrixXd &kalmanGain);
    void ckfUpdate(const Eigen::MatrixXd &kalmanGain, const Eigen::VectorXd &yMeas, const Eigen::MatrixXd &measurementMatrix);
    void ekfUpdate(const Eigen::MatrixXd &kalmanGain, const Eigen::VectorXd &yMeas);

    void orderMeasurementsChronologically();

    Eigen::MatrixXd processNoise; //!< [-] process noise matrix
    Eigen::VectorXd stateInitial; //!< [-] State estimate for time TimeTag at previous time
    Eigen::MatrixXd covarInitial; //!< [-] covariance at previous time
    double minCovarNorm = 1E-5; /*!< [-] Infinite norm after which the filter will begin processing measurements as
    an extended kalman filter */
    FilterType ckfOnlyMode = FilterType::Extended; //!< [-] flag to know whether the filter is being run as a linear KF or extended KF
};

#endif /* EKF_INTERFACE_HPP */
