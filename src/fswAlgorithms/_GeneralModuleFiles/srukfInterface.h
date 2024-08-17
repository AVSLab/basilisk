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
#ifndef SRUKF_INTERFACE_HPP
#define SRUKF_INTERFACE_HPP

#include <Eigen/Dense>
#include <functional>
#include <optional>
#include <array>
#include "architecture/utilities/avsEigenSupport.h"
#include "architecture/utilities/macroDefinitions.h"
#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/messaging/messaging.h"
#include "fswAlgorithms/_GeneralModuleFiles/filterInterfaceDefinitions.h"
#include "fswAlgorithms/_GeneralModuleFiles/kalmanFilter.h"
#include "fswAlgorithms/_GeneralModuleFiles/measurementModels.h"
#include "fswAlgorithms/_GeneralModuleFiles/stateModels.h"

/*! @brief Square Root unscented Kalman Filter base class */
class SRukfInterface: public KalmanFilter {
public:
    SRukfInterface();
    ~SRukfInterface() override;
    void Reset(uint64_t CurrentSimNanos) final;

    void setAlpha(double alpha);
    double getAlpha() const;
    void setBeta(double beta);
    double getBeta() const;

private:
    void timeUpdate(double updateTime) final;
    void measurementUpdate(const MeasurementModel &measurement) final;

    Eigen::VectorXd computeResiduals(const MeasurementModel &measurement) final;
    Eigen::MatrixXd qrDecompositionJustR(const Eigen::MatrixXd &input) const;
    Eigen::MatrixXd choleskyUpDownDate(const Eigen::MatrixXd &input,
                                       const Eigen::VectorXd &inputVector,
                                       double coefficient) const;
    Eigen::MatrixXd backSubstitution(const Eigen::MatrixXd &U, const Eigen::MatrixXd &b) const;
    Eigen::MatrixXd forwardSubstitution(const Eigen::MatrixXd &L, const Eigen::MatrixXd &b) const;
    Eigen::MatrixXd choleskyDecomposition(const Eigen::MatrixXd &input) const;

    Eigen::MatrixXd sBar; //!< [-] Time updated covariance
    std::array<StateVector, 2*MAX_STATES_VECTOR+1> sigmaPoints; //!< [-]    sigma point vector
    int numberSigmaPoints=0;//!< [-] number of sigma points
    Eigen::MatrixXd cholProcessNoise; //!< [-] cholesky of Qnoise
    Eigen::MatrixXd cholMeasNoise; //!< [-] cholesky of Measurement noise

    double beta=0;
    double alpha=0;
    double lambda=0;
    double eta=0;

    Eigen::VectorXd wM;
    Eigen::VectorXd wC;
    Eigen::MatrixXd sBarInitial; //!< [-] Time updated covariance at previous time
};

#endif /* SRUKF_INTERFACE_HPP */
