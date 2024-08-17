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
#include "fswAlgorithms/_GeneralModuleFiles/kalmanFilter.h"
#include "fswAlgorithms/_GeneralModuleFiles/stateModels.h"
#include "fswAlgorithms/_GeneralModuleFiles/dynamicModels.h"

/*! Enumerator class to set if the filter is meant to be used purely as a linear/classical KF,
 * no reference state updates will be performed in the classical filter, while the EKF updates the reference
 * with the computed state error at each measurement update */
enum class FilterType {Classical, Extended};

/*! @brief Extended or Classical/Linear Kalman Filter base class. */
class EkfInterface: public KalmanFilter {
public:
    EkfInterface(FilterType type);
    ~EkfInterface() override;
    void Reset(uint64_t CurrentSimNanos) override;

    void setFilterDynamicsMatrix(const std::function<const Eigen::MatrixXd(const double, const StateVector&)>&
        dynamicsMatrixCalculator);
    void setMinimumCovarianceNormForEkf(double infiniteNorm);
    double getMinimumCovarianceNormForEkf() const;


private:
    void timeUpdate(double updateTime) final;
    void measurementUpdate(const MeasurementModel &measurement) final;

    Eigen::VectorXd computeResiduals(const MeasurementModel &measurement) final;
    Eigen::MatrixXd computeKalmanGain(const Eigen::MatrixXd &covar,
                                    const Eigen::MatrixXd &measurementMatrix,
                                    const Eigen::MatrixXd &measurementNoise) const;
    void updateCovariance(const Eigen::MatrixXd &measMat, const Eigen::MatrixXd &noise, const Eigen::MatrixXd &kalmanGain);
    void ckfUpdate(const Eigen::MatrixXd &kalmanGain, const Eigen::VectorXd &yMeas, const Eigen::MatrixXd &measurementMatrix);
    void ekfUpdate(const Eigen::MatrixXd &kalmanGain, const Eigen::VectorXd &yMeas);

    Eigen::MatrixXd stateTransitionMatrix; //!< [-] State Transition Matrix
    double minCovarNorm = 1E-5; /*!< [-] Infinite norm after which the filter will begin processing measurements as
    an extended kalman filter */
    FilterType filterType = FilterType::Extended; //!< [-] flag to know whether the filter is being run as a linear KF or extended KF
};

#endif /* EKF_INTERFACE_HPP */
