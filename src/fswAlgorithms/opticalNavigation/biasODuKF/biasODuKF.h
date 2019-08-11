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

#ifndef _BIASOD_UKF_H_
#define _BIASOD_UKF_H_

#include <stdint.h>

#include "messaging/static_messaging.h"
#include "simFswInterfaceMessages/navTransIntMsg.h"
#include "simFswInterfaceMessages/circlesOpNavMsg.h"
#include "simFswInterfaceMessages/cameraConfigMsg.h"
#include "simFswInterfaceMessages/navAttIntMsg.h"
#include "simFswInterfaceMessages/macroDefinitions.h"
#include "fswMessages/opNavFswMsg.h"
#include "fswMessages/biasOpNavMsg.h"
#include "fswMessages/biasOpNavFilterMsg.h"
#include "simulation/utilities/linearAlgebra.h"
#include "simulation/utilities/rigidBodyKinematics.h"
#include "simulation/utilities/bsk_Print.h"
#include "utilities/astroConstants.h"



/*! \defgroup biasODuKF
 @brief This module filters the biases present in the position measurements that have been processed from planet images. This allows to better estimate spacecraft relative position to an observed body in the inertial frame. The filter used is an unscented Kalman filter, and the images are first processed by houghCricles and pixelLineConverter.

 The module [PDF Description](Basilisk-biasOD-20190620.pdf) contains further information on this module's function,
 how to run it, as well as testing.
 
 * @{
 */

/*! @brief Top level structure for the bias estimator.
 Used to estimate the spacecraft's inertial position relative to a body.
 */
typedef struct {
    char biasStateOutMsgName[MAX_STAT_MSG_LENGTH]; //!< The name of the output message
    char biasFiltOutMsgName[MAX_STAT_MSG_LENGTH]; //!< The name of the output filter data message
    char opNavInMsgName[MAX_STAT_MSG_LENGTH];  //!< [-] The name of OpNav measurement
    char cameraConfigMsgName[MAX_STAT_MSG_LENGTH]; //!< The name of the camera config message
    char attInMsgName[MAX_STAT_MSG_LENGTH]; //!< The name of the attitude message
    char circlesInMsgName[MAX_STAT_MSG_LENGTH]; //!< The name of the circles message
    char navInMsgName[MAX_STAT_MSG_LENGTH];//!< The name of the filter nav message
    
    int imageObsNum;              //!< [-] Number of image parameters
    int numStates;                //!< [-] Number of states for this filter
    int countHalfSPs;             //!< [-] Number of sigma points over 2
    int numObs;                   //!< [-] Number of measurements including bias
    double beta;                  //!< [-] Beta parameter for filter
    double alpha;                 //!< [-] Alpha parameter for filter
    double kappa;                 //!< [-] Kappa parameter for filter
    double lambdaVal;             //!< [-] Lambda parameter for filter
    double gamma;                 //!< [-] Gamma parameter for filter
    double eta;                 //!< [-] Gamma parameter for filter
    double switchMag;             //!< [-] Threshold for where we switch MRP set
    
    double dt;                     //!< [s] seconds since last data epoch
    double timeTag;                //!< [s]  Time tag for statecovar/etc
    double gyrAggTimeTag;          //!< [s] Time-tag for aggregated gyro data
    double aggSigma_b2b1[3];       //!< [-] Aggregated attitude motion from gyros
    double dcm_BdyGyrpltf[3][3];   //!< [-] DCM for converting gyro data to body frame
    double wM[2 * ODUKF_N_STATES_B + 1]; //!< [-] Weighting vector for sigma points
    double wC[2 * ODUKF_N_STATES_B + 1]; //!< [-] Weighting vector for sigma points
    
    double stateInit[ODUKF_N_STATES_B];    //!< [-] State estimate to initialize filter to
    double state[ODUKF_N_STATES_B];        //!< [-] State estimate for time TimeTag
    double state_pxl[ODUKF_N_STATES_B];      //!< [-] Bias state estimate converted to pixels
    double statePrev[ODUKF_N_STATES_B];        //!< [-] State estimate for time TimeTag at previous time
    double sBar[ODUKF_N_STATES_B*ODUKF_N_STATES_B];         //!< [-] Time updated covariance
    double sBarPrev[ODUKF_N_STATES_B*ODUKF_N_STATES_B];     //!< [-] Time updated covariance at previous time
    double covar[ODUKF_N_STATES_B*ODUKF_N_STATES_B];        //!< [-] covariance
    double covarPrev[ODUKF_N_STATES_B*ODUKF_N_STATES_B];    //!< [-] covariance at previous time
    double covarInit[ODUKF_N_STATES_B*ODUKF_N_STATES_B];    //!< [-] Covariance to init filter with
    double xBar[ODUKF_N_STATES_B];            //!< [-] Current mean state estimate
    
    double obs[ODUKF_N_MEAS];                               //!< [-] Observation vector for frame
    double yMeas[ODUKF_N_STATES_B*(2*ODUKF_N_STATES_B+1)];        //!< [-] Measurement model data, can include bias making it 6x2n+1
    
    double SP[(2*ODUKF_N_STATES_B+1)*ODUKF_N_STATES_B];          //!< [-]    sigma point matrix
    
    double qNoise[ODUKF_N_STATES_B*ODUKF_N_STATES_B];       //!< [-] process noise matrix
    double sQnoise[ODUKF_N_STATES_B*ODUKF_N_STATES_B];      //!< [-] cholesky of Qnoise
    double measNoise[ODUKF_N_MEAS*ODUKF_N_MEAS];      //!< [-] Measurement Noise
    
    int planetIdInit;                    //!< [-] Planet being navigated inital value
    int planetId;                   //!< [-] Planet being navigated as per measurement
    uint32_t firstPassComplete;         //!< [-] Flag to know if first filter update
    double postFits[3];      //!< [-] PostFit residuals
    double timeTagOut;       //!< [s] Output time-tag information
    double maxTimeJump;      //!< [s] Maximum time jump to allow in propagation
    
    NavAttIntMsg attInfo; //!< [-] ST sensor data read in from message bus
    CameraConfigMsg cameraSpecs; //!< [-] Camera config data read in from message bus
    OpNavFswMsg pixelLineInMsg; //!< [-] Post processing measurement read in from fsw
    CirclesOpNavMsg circlesIn; //!< [-] Pre processing measurement read in from fsw
    NavTransIntMsg filterInMsg; //!< [-] Measurement from the filter
    int32_t biasStateOutMsgId;     //!< -- Id for the outgoing body estimate message
    int32_t biasFiltOutMsgId;     //!< [-] Id for the filter data output message
    int32_t opNavInMsgId;     //!< [-] Id for the incoming mass properties message
    int32_t attInMsgID;    //!< [-] The ID associated with the outgoing message
    int32_t circlesInMsgID;    //!< [-] The ID associated with the incoming circle message
    int32_t cameraConfigMsgID;  //!< [-] The ID associated with the incoming camera config message
    int32_t navInMsgId;     //!< [-] The ID associated of the OpNav filter nav message
}BiasODuKFConfig;


#ifdef __cplusplus
extern "C" {
#endif
    
    void SelfInit_biasODuKF(BiasODuKFConfig *configData, uint64_t moduleId);
    void CrossInit_biasODuKF(BiasODuKFConfig *configData, uint64_t moduleId);
    void Update_biasODuKF(BiasODuKFConfig *configData, uint64_t callTime,
                            uint64_t moduleId);
    void Reset_biasODuKF(BiasODuKFConfig *configData, uint64_t callTime,
                           uint64_t moduleId);
    int biasODuKFTimeUpdate(BiasODuKFConfig *configData, double updateTime);
    int biasODuKFMeasUpdate(BiasODuKFConfig *configData);
    void biasODuKFCleanUpdate(BiasODuKFConfig *configData);
    void biasODuKFMeasModel(BiasODuKFConfig *configData);
    
#ifdef __cplusplus
}
#endif

/*! @} */

#endif
