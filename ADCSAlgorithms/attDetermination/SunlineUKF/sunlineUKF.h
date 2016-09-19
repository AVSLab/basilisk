/*
Copyright (c) 2016, Autonomous Vehicle Systems Lab, Univeristy of Colorado at Boulder

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

#ifndef _SUNLINE_UKF_H_
#define _SUNLINE_UKF_H_

#include "messaging/static_messaging.h"
#include "sensorInterfaces/CSSSensorData/cssComm.h"
#include <stdint.h>

#define SKF_N_STATES 2
#define MAX_N_CSS_MEAS 8

/*! \addtogroup ADCSAlgGroup
 * @{
 */

/*! @brief Top level structure for the CSS weighted least squares estimator.
 Used to estimate the sun state in the vehicle body frame*/
typedef struct {
//    SingleCSSConfig CSSData[MAX_NUM_CSS_SENSORS]; /*!< -- The config data for the estimator*/
    char outputNavStateName[MAX_STAT_MSG_LENGTH]; /*!< The name of the output message*/
    char inputCSSDataName[MAX_STAT_MSG_LENGTH]; /*!< The name of the Input message*/
    char inputPropsName[MAX_STAT_MSG_LENGTH]; /*!< [-] The name of the mass props message*/

	int numStates;                /*! [-] Number of states for this filter*/                
	int countHalfSPs;             /*! [-] Number of sigma points over 2 */
	int numObs;                   /*! [-] Number of measurements this cycle */
	double beta;                  /*! [-] Beta parameter for filter */
	double alpha;                 /*! [-] Alpha parameter for filter*/
	double kappa;                 /*! [-] Kappa parameter for filter*/
	double lambdaVal;             /*! [-] Lambda parameter for filter*/
	double gamma;                 /*! [-] Gamma parameter for filter*/

	double dt;                     /*! [s] seconds since last data epoch */
	double TimeTag;                /*! [s]  Time tag for statecovar/etc */

	double state[SKF_N_STATES];        // -- State estimate for time TimeTag
	double sBar[SKF_N_STATES*SKF_N_STATES];         // -- Time updated covariance
	double covar[SKF_N_STATES*SKF_N_STATES];        // -- covariance

	double obs[MAX_N_CSS_MEAS];          // -- pseudorange observations
	double yMeas[MAX_N_CSS_MEAS*SKF_N_STATES];        // -- Measurement model data

	double SP[(2*SKF_N_STATES+1)*SKF_N_STATES];          // --    sigma point matrix

	double Qnoise[SKF_N_STATES*SKF_N_STATES];       // -- process noise matrix
	double SQnoise[SKF_N_STATES*SKF_N_STATES];      // -- cholesky of Qnoise

	double Q_obs[MAX_N_CSS_MEAS];  // -- to put on diagonal of Q obs matrix

    uint32_t numActiveCss;   /*!< -- Number of currently active CSS sensors*/
    double sensorUseThresh;  /*!< -- Threshold below which we discount sensors*/
    //CSSWlsEstOut OutputData; /*!< -- Unit vector to the Sun in the spacecraft body frame*/
    int32_t outputStateID;     /*!< -- ID for the outgoing body estimate message*/
    int32_t inputCSSDataID;      /*!< -- ID for the incoming CSS sensor message*/
    int32_t inputPropsID;    /*!< [-] ID for the incoming mass properties message*/
}SunlineUKFConfig;

#ifdef __cplusplus
extern "C" {
#endif
    
    void SelfInit_sunlineUKF(SunlineUKFConfig *ConfigData, uint64_t moduleID);
    void CrossInit_sunlineUKF(SunlineUKFConfig *ConfigData, uint64_t moduleID);
    void Update_sunlineUKF(SunlineUKFConfig *ConfigData, uint64_t callTime,
        uint64_t moduleID);
    
#ifdef __cplusplus
}
#endif

/*! @} */

#endif
