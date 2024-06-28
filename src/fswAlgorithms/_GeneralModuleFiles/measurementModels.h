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

#ifndef FILTER_MEAS_MODELS_H
#define FILTER_MEAS_MODELS_H

/*! @brief Container class for measurement data and models */
class Measurement{
public:
    Measurement() = default;

    std::string name = ""; //!< [-] name of measurement  type
    size_t size = 0; //!< [-] size of observation vector
    double timeTag = 0; //!< [-] Observation time tag
    bool validity = false; //!< [-] Observation validity
    Eigen::VectorXd observation; //!< [-] Observation data vector
    Eigen::MatrixXd noise; //!< [-] Constant measurement Noise
    Eigen::MatrixXd choleskyNoise; //!< [-] cholesky of Qnoise
    Eigen::VectorXd postFitResiduals; //!< [-] Observation post fit residuals
    Eigen::VectorXd preFitResiduals; //!< [-] Observation pre fit residuals
     /*! Each measurement must be paired with a measurement model as a function which inputs the
     * sigma point matrix and outputs the modeled measurement for each sigma point */
    std::function<const Eigen::MatrixXd(const Eigen::MatrixXd)> model; //!< [-] observation measurement model
};



/*! @brief Measurement models used to map a state vector to a measurement */
Eigen::VectorXd normalizedFirstThreeStates(Eigen::VectorXd state);
Eigen::VectorXd firstThreeStates(Eigen::VectorXd state, size_t beginSlice, size_t endSlice);
Eigen::VectorXd lastThreeStates(Eigen::VectorXd state);
Eigen::VectorXd mrpFirstThreeStates(Eigen::VectorXd state);

#endif
