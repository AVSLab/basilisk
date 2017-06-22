/*
 ISC License

 Copyright (c) 2016-2017, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#ifndef _SUNLINE_EKF_H_
#define _SUNLINE_EKF_H_

#include "messaging/static_messaging.h"
#include <stdint.h>
#include "simFswInterfaceMessages/navAttIntMsg.h"
#include "simFswInterfaceMessages/cssArraySensorIntMsg.h"
#include "fswMessages/vehicleConfigFswMsg.h"
#include "fswMessages/cssConfigFswMsg.h"
#include "fswMessages/sunlineFilterFswMsg.h"


/*! \addtogroup ADCSAlgGroup
 * @{
 */


/*! @brief Top level structure for the CSS unscented kalman filter estimator.
 Used to estimate the sun state in the vehicle body frame*/
typedef struct {
    char navStateOutMsgName[MAX_STAT_MSG_LENGTH]; /*!< The name of the output message*/
    char filtDataOutMsgName[MAX_STAT_MSG_LENGTH]; /*!< The name of the output filter data message*/
    char cssDataInMsgName[MAX_STAT_MSG_LENGTH]; /*!< The name of the Input message*/
    char massPropsInMsgName[MAX_STAT_MSG_LENGTH]; /*!< [-] The name of the mass props message*/
    char cssConfInMsgName[MAX_STAT_MSG_LENGTH]; /*!< [-] The name of the CSS configuration message*/
    
    double qObsVal;               /*!< [-] CSS instrument noise parameter*/
    double qProcVal;               /*!< [-] Process noise parameter*/

	double dt;                     /*!< [s] seconds since last data epoch */
	double timeTag;                /*!< [s]  Time tag for statecovar/etc */

	double states[SKF_N_STATES];        /*!< [-] State estimate for time TimeTag*/
    double x[SKF_N_STATES];             /*! State errors */
    double xBar[SKF_N_STATES];            /*! [-] Current mean state estimate*/
	double covarBar[SKF_N_STATES*SKF_N_STATES];         /*!< [-] Time updated covariance */
	double covar[SKF_N_STATES*SKF_N_STATES];        /*!< [-] covariance */
    double stateTransition[SKF_N_STATES*SKF_N_STATES];        /*!< [-] covariance */
    double kalmanGain[SKF_N_STATES*MAX_N_CSS_MEAS];    /* Kalman Gain */

    double dynMat[SKF_N_STATES*SKF_N_STATES];        /*!< [-] Dynamics Matrix */
    double measMat[MAX_N_CSS_MEAS*SKF_N_STATES];        /*!< [-] Measurement Matrix H*/
    
	double obs[MAX_N_CSS_MEAS];          /*!< [-] Observation vector for frame*/
	double yMeas[MAX_N_CSS_MEAS];        /*!< [-] Measurement model data */

	double procNoise[SKF_N_STATES/2*SKF_N_STATES/2];       /*!< [-] process noise matrix */
	double measNoise[MAX_N_CSS_MEAS*MAX_N_CSS_MEAS];  /*!< [-] Maximally sized obs noise matrix*/
    
    double cssNHat_B[MAX_NUM_CSS_SENSORS*3];     /*!< [-] CSS normal vectors converted over to body*/

    uint32_t numStates;                /*!< [-] Number of states for this filter*/
    int numObs;                   /*!< [-] Number of measurements this cycle */
    uint32_t numActiveCss;   /*!< -- Number of currently active CSS sensors*/
    uint32_t numCSSTotal;    /*!< [-] Count on the number of CSS we have on the spacecraft*/
    double sensorUseThresh;  /*!< -- Threshold below which we discount sensors*/
    double eKFSwitch;       /*!< -- Max covariance element after which the filter switches to an EKF*/
	NavAttIntMsg outputSunline;   /*!< -- Output sunline estimate data */
    CSSArraySensorIntMsg cssSensorInBuffer; /*!< [-] CSS sensor data read in from message bus*/
    int32_t navStateOutMsgId;     /*!< -- ID for the outgoing body estimate message*/
    int32_t filtDataOutMsgId;   /*!< [-] ID for the filter data output message*/
    int32_t cssDataInMsgId;      /*!< -- ID for the incoming CSS sensor message*/
    int32_t massPropsInMsgId;    /*!< [-] ID for the incoming mass properties message*/
    int32_t cssConfInMsgId;   /*!< [-] ID associated with the CSS configuration data*/
}sunlineEKFConfig;

#ifdef __cplusplus
extern "C" {
#endif
    
    void SelfInit_sunlineEKF(sunlineEKFConfig *ConfigData, uint64_t moduleID);
    void CrossInit_sunlineEKF(sunlineEKFConfig *ConfigData, uint64_t moduleID);
	void Reset_sunlineEKF(sunlineEKFConfig *ConfigData, uint64_t callTime,
		uint64_t moduleID);
    void Update_sunlineEKF(sunlineEKFConfig *ConfigData, uint64_t callTime,
                           uint64_t moduleID);
	void sunlineTimeUpdate(sunlineEKFConfig *ConfigData, double updateTime);
    void sunlineMeasUpdate(sunlineEKFConfig *ConfigData, double updateTime);
	void sunlineStateSTMProp(double dynMat[SKF_N_STATES*SKF_N_STATES], double dt, double *stateInOut, double *stateTransition);
    
    void sunlineHMatrixYMeas(double states[SKF_N_STATES], int numCSS, double cssSensorCos[MAX_N_CSS_MEAS], double sensorUseThresh, double cssNHat_B[MAX_NUM_CSS_SENSORS*3], double *obs, double *yMeas, int *numObs, double *measMat);
    
    void sunlineKalmanGain(double covarBar[SKF_N_STATES*SKF_N_STATES], double hObs[MAX_N_CSS_MEAS*SKF_N_STATES], double qObsVal, int numObs, double *kalmanGain);
    
    void sunlineDynMatrix(double stateInOut[SKF_N_STATES], double *dynMat);
    
    void sunlineCKFUpdate(double xBar[SKF_N_STATES], double kalmanGain[SKF_N_STATES*MAX_N_CSS_MEAS], double covarBar[SKF_N_STATES*SKF_N_STATES], double qObsVal, int numObs, double yObs[MAX_N_CSS_MEAS], double hObs[MAX_N_CSS_MEAS*SKF_N_STATES], double *x, double *covar);
    
    void sunlineEKFUpdate(double kalmanGain[SKF_N_STATES*MAX_N_CSS_MEAS], double covarBar[SKF_N_STATES*SKF_N_STATES], double qObsVal, int numObs, double yObs[MAX_N_CSS_MEAS], double hObs[MAX_N_CSS_MEAS*SKF_N_STATES], double *states, double *x, double *covar);
    
#ifdef __cplusplus
}
#endif

/*! @} */

#endif
