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
#include <Eigen/Dense>

/*! \addtogroup SimModelGroup
 * @{
 */

//!@brief Simple navigation model used to provide error-ed truth (or truth)
/*! This class is used to perturb the truth state away using a gauss-markov 
 error model.  It is designed to look like a random walk process put on top of 
 the nominal position, velocity, attitude, and attitude rate.  This is meant to 
 be used in place of the nominal navigation system output.

 The module
 [PDF Description](Basilisk-SIMPLE_NAV20170712.pdf)
 contains further information on this module's function,
 how to run it, as well as testing.
*/
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
    uint64_t outputBufferCount;        //!< -- Number of output state buffers in msg
    Eigen::MatrixXd PMatrix;       //!< -- Covariance matrix used to perturb state
    Eigen::VectorXd walkBounds;    //!< -- "3-sigma" errors to permit for states
    Eigen::VectorXd navErrors;     //!< -- Current navigation errors applied to truth
    std::string inputStateName;        //!< -- Message that contains s/c state
    std::string outputAttName;         //!< -- Message that we output state to
    std::string outputTransName;       //!< -- Message that we output state to
    std::string inputSunName;          //!< -- Message name for the sun state
    bool crossTrans;                   //!< -- Have position error depend on velocity
    bool crossAtt;                     //!< -- Have attitude depend on attitude rate
    NavAttIntMsg trueAttState;        //!< -- attitude nav state without errors
    NavAttIntMsg estAttState;         //!< -- attitude nav state including errors
    NavTransIntMsg trueTransState;    //!< -- translation nav state without errors
    NavTransIntMsg estTransState;     //!< -- translation nav state including errors
    SCPlusStatesSimMsg inertialState; //!< -- input inertial state from Star Tracker
    SpicePlanetStateSimMsg sunState;  //!< -- input Sun state
private:
    int64_t inputStateID;              //!< -- Message ID associated with s/c state
    int64_t outputAttID;               //!< -- Message ID associated with att-nav state
    int64_t outputTransID;             //!< -- Message ID associated with trans-nav state
    int64_t inputSunID;                //!< -- Message ID associated with the sun position
    Eigen::MatrixXd AMatrix;       //!< -- The matrix used to propagate the state
    GaussMarkov errorModel;            //!< -- Gauss-markov error states
    uint64_t prevTime;                 //!< -- Previous simulation time observed
};

/*! @} */

#endif
