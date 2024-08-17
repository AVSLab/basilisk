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

#include <Eigen/Core>
#include "architecture/utilities/rigidBodyKinematics.hpp"
#include "fswAlgorithms/_GeneralModuleFiles/stateModels.h"

#ifndef FILTER_MEAS_MODELS_H
#define FILTER_MEAS_MODELS_H

/*! @brief Container class for measurement data and models */
class MeasurementModel{
public:
    MeasurementModel();
    ~MeasurementModel();

    static Eigen::VectorXd positionStates(const StateVector &state);
    static Eigen::VectorXd normalizedPositionStates(const StateVector &state);
    static Eigen::VectorXd mrpStates(const StateVector &state);
    static Eigen::VectorXd velocityStates(const StateVector &state);

    Eigen::MatrixXd model(const StateVector& state) const;
    void setMeasurementModel(const std::function<const Eigen::MatrixXd(const StateVector&)>& modelCalculator);
    Eigen::MatrixXd computeMeasurementMatrix(const StateVector& state) const;
    void setMeasurementMatrix(const std::function<const Eigen::MatrixXd(const StateVector&)>& hMatrixCalculator);

    size_t size() const;
    std::string getMeasurementName() const;
    void setMeasurementName(std::string_view measurementName);
    double getTimeTag() const;
    void setTimeTag(double measurementTimeTag);
    bool getValidity() const;
    void setValidity(bool measurementValidity);
    Eigen::VectorXd getObservation() const;
    void setObservation(const Eigen::VectorXd& measurementObserved);
    Eigen::MatrixXd getMeasurementNoise() const;
    void setMeasurementNoise(const Eigen::MatrixXd& measurementNoise);
    Eigen::VectorXd getPreFitResiduals() const;
    void setPreFitResiduals(const Eigen::VectorXd& measurementPreFit);
    Eigen::VectorXd getPostFitResiduals() const;
    void setPostFitResiduals(const Eigen::VectorXd& measurementPostFit);


private:
    std::string name{}; //!< [-] Name of measurement  type
    double timeTag{}; //!< [-] Observation time tag
    bool validity = false; //!< [-] Observation validity
    Eigen::VectorXd observation; //!< [-] Observation data vector
    Eigen::MatrixXd noise; //!< [-] Measurement Noise
    Eigen::MatrixXd choleskyNoise; //!< [-] Cholesky decomposition of measurement noise
    Eigen::VectorXd postFitResiduals; //!< [-] Observation post fit residuals
    Eigen::VectorXd preFitResiduals; //!< [-] Observation pre fit residuals

    std::function<const Eigen::MatrixXd(const StateVector&)> measurementModel; //!< [-] observation measurement model
    std::function<const Eigen::MatrixXd(const StateVector&)> measurementPartials; //!< [-] partial of measurement model wrt state

};

#endif
