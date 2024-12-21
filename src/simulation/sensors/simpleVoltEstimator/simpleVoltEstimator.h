/*
 ISC License

 Copyright (c) 2022, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#ifndef SIMPLE_VOLT_ESTIMATOR_H
#define SIMPLE_VOLT_ESTIMATOR_H

#include <vector>
#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/utilities/gauss_markov.h"
#include "architecture/msgPayloadDefC/VoltMsgPayload.h"
#include "architecture/utilities/bskLogging.h"
#include <Eigen/Dense>
#include "architecture/messaging/messaging.h"

/*! @brief simple voltage estimation module class */
class SimpleVoltEstimator: public SysModel {
public:
    SimpleVoltEstimator();
    ~SimpleVoltEstimator();

    void Reset(uint64_t CurrentSimNanos);
    void UpdateState(uint64_t CurrentSimNanos);
    void computeErrors();
    void applyErrors();
    void readInputMessages();
    void writeOutputMessages(uint64_t Clock);

    void setAMatrix(const Eigen::MatrixXd& propMatrix);
    Eigen::MatrixXd getAMatrix() const;

public:
    Eigen::MatrixXd PMatrix;          //!< -- Cholesky-decomposition or matrix square root of the covariance matrix to apply errors with
    Eigen::VectorXd walkBounds;       //!< -- "3-sigma" errors to permit for states
    Eigen::VectorXd voltErrors;        //!< -- Current voltage errors applied to truth
    Message<VoltMsgPayload> voltOutMsg;    //!< voltage output msg
    VoltMsgPayload trueVoltState;    //!< -- voltage state without errors
    VoltMsgPayload estVoltState;     //!< -- voltage state including errors
    BSKLogger bskLogger;              //!< -- BSK Logging

    ReadFunctor<VoltMsgPayload> voltInMsg;          //!< voltage input msg

private:
    Eigen::MatrixXd AMatrix;           //!< -- The matrix used to propagate the state
    GaussMarkov errorModel;            //!< -- Gauss-markov error states
};


#endif
