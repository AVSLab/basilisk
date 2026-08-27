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

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/utilities/gauss_markov.h"
#include <cstdint>
#include <vector>

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
 * - PMatrix: Matrix square root of the process-noise covariance [rad]
 * - AMatrix: Propagation matrix for the error model (defaults to zero for white noise)
 * - Walk bounds: Optional hard bounds on the propagated error state [rad]
 *
 * Example Python usage:
 * @code
 *     tracker = starTracker.StarTracker()
 *
 *     # Configure noise (rad)
 *     tracker.PMatrix = [[0.001, 0.0, 0.0],
 *                        [0.0, 0.001, 0.0],
 *                        [0.0, 0.0, 0.001]]
 *
 *     # Optional: configure a bounded random walk explicitly
 *     tracker.setAMatrix([[1.0, 0.0, 0.0],
 *                         [0.0, 1.0, 0.0],
 *                         [0.0, 0.0, 1.0]])
 *     tracker.setWalkBounds([0.01, 0.01, 0.01])
 * @endcode
 */
class StarTracker: public SysModel {
public:
    StarTracker();
    ~StarTracker();

    void UpdateState(uint64_t CurrentSimNanos);
    void Reset(uint64_t CurrentClock); //!< Method for resetting the module
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

    Eigen::Matrix3d PMatrix;    //!< [rad] Matrix square root of the process-noise covariance
    Eigen::Vector3d walkBounds; //!< [rad] Hard bounds on error states; non-positive entries disable clipping
    Eigen::Vector3d navErrors;  //!< [rad] Current principal-rotation-vector errors applied to truth

    double dcm_CB[3][3];                 //!< [-] Transformation matrix from body to case
    STSensorMsgPayload trueValues;  //!< [-] total measurement without perturbations
    STSensorMsgPayload sensedValues;//!< [-] total measurement including perturbations
    double mrpErrors[3];              //!< [-] Errors to be applied to the input MRP set indicating whether
    SCStatesMsgPayload scState;      //!< [-] Module variable where the input State Data message is stored
    BSKLogger bskLogger;                      //!< -- BSK Logging




private:
  Eigen::Matrix3d AMatrix; //!< [-] Error propagation matrix; defaults to zero for white noise
  GaussMarkov errorModel;  //!< [-] Gauss-markov error states
};


#endif
