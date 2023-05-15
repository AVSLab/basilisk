/*
 ISC License

 Copyright (c) 2023, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

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

#ifndef _SICP_H_
#define _SICP_H_

#include <stdint.h>
#include <Eigen/Dense>
#include "sicpDefinitions.h"
#include "architecture/messaging/messaging.h"

#include "architecture/msgPayloadDefCpp/SICPMsgPayload.h"
#include "architecture/msgPayloadDefCpp/PointCloudMsgPayload.h"

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/utilities/avsEigenMRP.h"
#include "architecture/utilities/avsEigenSupport.h"
#include "architecture/utilities/bskLogging.h"

/*! @brief Scaling iterative Closest Point Algorithm */
class ScalingIterativeClosestPoint: public SysModel {
public:
    ScalingIterativeClosestPoint();
    ~ScalingIterativeClosestPoint();
    
    void UpdateState(uint64_t CurrentSimNanos);
    void Reset(uint64_t CurrentSimNanos);

    Message<PointCloudMsgPayload> outputPointCloud;  //!< The output fitted point cloud
    Message<SICPMsgPayload> outputSICPData;  //!< The output algorithm data
    ReadFunctor<PointCloudMsgPayload> measuredPointCloud;          //!< The input measured data
    ReadFunctor<PointCloudMsgPayload> referencePointCloud;          //!< The input reference data
    BSKLogger bskLogger;                //!< -- BSK Logging

    double scalingMax = 1.1; //!< Scaling maximums
    double scalingMin = 0.9; //!< Scaling minimums
    double errorTolerance = 1e-10; //!< Error tolerance for convergence
    int maxIterations = MAX_ITERATIONS; //!< Max iterations
    int numberScalePoints = 100; //!< Number of points in order to find the scale factor

    //!< Initial conditions that could be set by user to better start off the SICP apgorithm
    Eigen::MatrixXd R_init = Eigen::MatrixXd::Identity(POINT_DIM, POINT_DIM);
    Eigen::MatrixXd t_init = Eigen::VectorXd::Zero(POINT_DIM);
    double s_init = 1;

private:
    void computePointCorrespondance(const Eigen::MatrixXd& R_kmin1, const Eigen::MatrixXd& t_kmin1, const double s_kmin1,
                                    const Eigen::MatrixXd& measuredPoints, const Eigen::MatrixXd& referencePoints);
    void centerCloud(const Eigen::MatrixXd& measuredPoints);
    Eigen::MatrixXd computeRk(const double s_kmin1, const Eigen::MatrixXd& R_kmin1);
    double computeSk(const Eigen::MatrixXd& R_kmin1);
    Eigen::MatrixXd computeTk(const double s_k, const Eigen::MatrixXd& R_k, const Eigen::MatrixXd& measuredPoints);

    PointCloudMsgPayload outputCloudBuffer;
    PointCloudMsgPayload measuredCloudBuffer;
    PointCloudMsgPayload referenceCloudBuffer;
    SICPMsgPayload sicpBuffer;

    Eigen::MatrixXd correspondingPoints;
    Eigen::MatrixXd q;
    Eigen::MatrixXd n;

    int Np = 0; //!< Number of detected points
    int maxInteralIterations = 10; //!< Maximum iterations in the inner loop for scale factor and rotation

};


#endif
