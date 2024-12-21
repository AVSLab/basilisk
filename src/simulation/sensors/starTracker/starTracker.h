/*
 ISC License

 Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#ifndef STAR_TRACKER_H
#define STAR_TRACKER_H

#include <vector>
#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/utilities/gauss_markov.h"

#include "architecture/msgPayloadDefC/SCStatesMsgPayload.h"
#include "architecture/msgPayloadDefC/STSensorMsgPayload.h"
#include "architecture/messaging/messaging.h"

#include <Eigen/Dense>
#include "architecture/utilities/avsEigenMRP.h"
#include "architecture/utilities/bskLogging.h"


/*! @class StarTracker
 * @brief Star tracker sensor model that simulates quaternion measurements with configurable noise
 *
 * The star tracker supports noise configuration through:
 * - Walk bounds: Maximum allowed deviations from truth [rad]
 * - PMatrix: Noise covariance matrix (diagonal elements = noiseStd)
 * - AMatrix: Propagation matrix for error model (defaults to identity)
 *
 * Example Python usage:
 * @code
 *     starTracker = StarTracker()
 *
 *     # Configure noise (rad)
 *     noiseStd = 0.001  # Standard deviation
 *     PMatrix = [0.0] * 9  # 3x3 matrix
 *     PMatrix[0] = PMatrix[4] = PMatrix[8] = noiseStd
 *     starTracker.PMatrix = PMatrix
 *
 *     # Set maximum error bounds (rad)
 *     starTracker.setWalkBounds([0.01, 0.01, 0.01])
 * @endcode
 */
class StarTracker: public SysModel {
public:
    StarTracker();
    ~StarTracker();

    void UpdateState(uint64_t CurrentSimNanos);
    void Reset(uint64_t CurrentClock);          //!< Method for reseting the module
    void readInputMessages();
    void writeOutputMessages(uint64_t Clock);
    void computeSensorErrors();
    void applySensorErrors();
    void computeTrueOutput();
    void computeQuaternion(double *sigma, STSensorMsgPayload *sensorValue);

    void setAMatrix(const Eigen::MatrixXd& propMatrix);
    Eigen::MatrixXd getAMatrix() const;

    /*! Sets walk bounds [rad] */
    void setWalkBounds(const Eigen::Vector3d& bounds);

    /*! Gets current walk bounds [rad] */
    Eigen::Vector3d getWalkBounds() const;

public:

    uint64_t sensorTimeTag;            //!< [ns] Current time tag for sensor out
    ReadFunctor<SCStatesMsgPayload> scStateInMsg;    //!< [-] sc input state message
    Message<STSensorMsgPayload> sensorOutMsg;   //!< [-] sensor output state message

    Eigen::Matrix3d PMatrix;      //!< [-] Cholesky-decomposition or matrix square root of the covariance matrix to apply errors with
    Eigen::Vector3d walkBounds;   //!< [-] "3-sigma" errors to permit for states
    Eigen::Vector3d navErrors;    //!< [-] Current navigation errors applied to truth

    double dcm_CB[3][3];                 //!< [-] Transformation matrix from body to case
    STSensorMsgPayload trueValues;  //!< [-] total measurement without perturbations
    STSensorMsgPayload sensedValues;//!< [-] total measurement including perturbations
    double mrpErrors[3];              //!< [-] Errors to be applied to the input MRP set indicating whether
    SCStatesMsgPayload scState;      //!< [-] Module variable where the input State Data message is stored
    BSKLogger bskLogger;                      //!< -- BSK Logging




private:
    Eigen::Matrix3d AMatrix;      //!< [-] AMatrix that we use for error propagation
    GaussMarkov errorModel;           //!< [-] Gauss-markov error states
};


#endif
