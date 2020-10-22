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

#ifndef SIMPLE_NAV_H
#define SIMPLE_NAV_H

#include <vector>
#include "_GeneralModuleFiles/sys_model.h"
#include "utilities/gauss_markov.h"
#include "simMessages/scPlusStatesSimMsg.h"
#include "simMessages/spicePlanetStateSimMsg.h"
#include "simFswInterfaceMessages/navAttIntMsg.h"
#include "simFswInterfaceMessages/navTransIntMsg.h"
#include "utilities/bskLogging.h"
#include <Eigen/Dense>
#include "../../architecture/messaging2/message.h"

/*! @brief simple navigation module class */
class SimpleNav: public SysModel {
public:
    SimpleNav();
    ~SimpleNav();

    void SelfInit();
    void CrossInit();
    void UpdateState(uint64_t CurrentSimNanos);
    void computeTrueOutput(uint64_t Clock);
    void computeErrors(uint64_t CurrentSimNanos);
    void applyErrors();
    void readInputMessages();
    void writeOutputMessages(uint64_t Clock);

public:
    uint64_t outputBufferCount;       //!< -- Number of output state buffers in msg
    Eigen::MatrixXd PMatrix;          //!< -- Cholesky-decomposition or matrix square root of the covariance matrix to apply errors with
    Eigen::VectorXd walkBounds;       //!< -- "3-sigma" errors to permit for states
    Eigen::VectorXd navErrors;        //!< -- Current navigation errors applied to truth
    SimMessage<NavAttIntMsg> attOutMsg;
    SimMessage<NavTransIntMsg> transOutMsg;
    bool crossTrans;                  //!< -- Have position error depend on velocity
    bool crossAtt;                    //!< -- Have attitude depend on attitude rate
    NavAttIntMsg trueAttState;        //!< -- attitude nav state without errors
    NavAttIntMsg estAttState;         //!< -- attitude nav state including errors
    NavTransIntMsg trueTransState;    //!< -- translation nav state without errors
    NavTransIntMsg estTransState;     //!< -- translation nav state including errors
    SCPlusStatesSimMsg inertialState; //!< -- input inertial state from Star Tracker
    SpicePlanetStateSimMsg sunState;  //!< -- input Sun state
    BSKLogger bskLogger;              //!< -- BSK Logging

    ReadFunctor<SCPlusStatesSimMsg> scStateInMsg;
    ReadFunctor<SpicePlanetStateSimMsg> sunStateInMsg;

private:
    WriteFunctor<NavAttIntMsg>  writeAttOutMsg;
    WriteFunctor<NavTransIntMsg> writeTransOutMsg;

private:
    Eigen::MatrixXd AMatrix;           //!< -- The matrix used to propagate the state
    GaussMarkov errorModel;            //!< -- Gauss-markov error states
    uint64_t prevTime;                 //!< -- Previous simulation time observed
};


#endif
