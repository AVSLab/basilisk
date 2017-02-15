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

#ifndef _INERTIAL_UKF_H_
#define _INERTIAL_UKF_H_

#include "messaging/static_messaging.h"
#include "SimFswInterfaceMessages/rwSpeedIntMsg.h"
#include "SimFswInterfaceMessages/navAttIntMsg.h"
#include "fswMessages/inertialFilterMessage.h"
#include "fswMessages/stAttMessage.h"
#include "fswMessages/vehicleConfigMessage.h"
#include "fswMessages/rwArrayConfigFswMsg.h"
#include <stdint.h>


/*! \addtogroup ADCSAlgGroup
 * @{
 */



/*! @brief Top level structure for the CSS unscented kalman filter estimator.
 Used to estimate the sun state in the vehicle body frame*/
typedef struct {
    char navStateOutMsgName[MAX_STAT_MSG_LENGTH]; /*!< The name of the output message*/
    char filtDataOutMsgName[MAX_STAT_MSG_LENGTH]; /*!< The name of the output filter data message*/
    
    char rwParamsInMsgName[MAX_STAT_MSG_LENGTH];  /*!< The name of the RWConfigParams input message*/
    char rwSpeedsInMsgName[MAX_STAT_MSG_LENGTH]; /*!< [-] The name of the input RW speeds message*/
    char stDataInMsgName[MAX_STAT_MSG_LENGTH]; /*!< The name of the Input message*/
    char massPropsInMsgName[MAX_STAT_MSG_LENGTH]; /*!< [-] The name of the mass props message*/

	int numStates;                /*!< [-] Number of states for this filter*/
	int countHalfSPs;             /*!< [-] Number of sigma points over 2 */
	int numObs;                   /*!< [-] Number of measurements this cycle */
	double beta;                  /*!< [-] Beta parameter for filter */
	double alpha;                 /*!< [-] Alpha parameter for filter*/
	double kappa;                 /*!< [-] Kappa parameter for filter*/
	double lambdaVal;             /*!< [-] Lambda parameter for filter*/
	double gamma;                 /*!< [-] Gamma parameter for filter*/
    double switchMag;             /*!< [-] Threshold for where we switch MRP set*/

	double dt;                     /*!< [s] seconds since last data epoch */
	double timeTag;                /*!< [s]  Time tag for statecovar/etc */

	double wM[2 * AKF_N_STATES + 1]; /*!< [-] Weighting vector for sigma points*/
	double wC[2 * AKF_N_STATES + 1]; /*!< [-] Weighting vector for sigma points*/

	double state[AKF_N_STATES];        /*!< [-] State estimate for time TimeTag*/
	double sBar[AKF_N_STATES*AKF_N_STATES];         /*!< [-] Time updated covariance */
	double covar[AKF_N_STATES*AKF_N_STATES];        /*!< [-] covariance */
    double xBar[AKF_N_STATES];            /*! [-] Current mean state estimate*/

	double obs[3];          /*!< [-] Observation vector for frame*/
	double yMeas[3*(2*AKF_N_STATES+1)];        /*!< [-] Measurement model data */

	double SP[(2*AKF_N_STATES+1)*AKF_N_STATES];          /*!< [-]    sigma point matrix */

	double qNoise[AKF_N_STATES*AKF_N_STATES];       /*!< [-] process noise matrix */
	double sQnoise[AKF_N_STATES*AKF_N_STATES];      /*!< [-] cholesky of Qnoise */

	double qObs[3*3];  /*!< [-] Maximally sized obs noise matrix*/
    double IInv[3][3];

    uint32_t numActiveCss;   /*!< -- Number of currently active CSS sensors*/
    uint32_t numCSSTotal;    /*!< [-] Count on the number of CSS we have on the spacecraft*/
    double sensorUseThresh;  /*!< -- Threshold below which we discount sensors*/
    uint32_t firstPassComplete;
	NavAttIntMsg outputInertial;        /*!< -- Output inertial estimate data */
    STAttMessage stSensorIn;             /*!< [-] ST sensor data read in from message bus*/
    RWArrayConfigFswMsg rwConfigParams;       /*!< [-] struct to store message containing RW config parameters in body B frame */
    RWSpeedIntMsg rwSpeeds;             /*! [-] Local reaction wheel speeds */
    RWSpeedIntMsg rwSpeedPrev;          /*! [-] Local reaction wheel speeds */
    VehicleConfigMessage localConfigData;   /*! [-] Vehicle configuration data*/
    int32_t navStateOutMsgId;     /*!< -- ID for the outgoing body estimate message*/
    int32_t filtDataOutMsgId;   /*!< [-] ID for the filter data output message*/
    int32_t stDataInMsgId;      /*!< -- ID for the incoming CSS sensor message*/
    int32_t massPropsInMsgId;    /*!< [-] ID for the incoming mass properties message*/
    int32_t rwParamsInMsgID;     /*!< [-] ID for the RWArrayConfigFswMsg ingoing message */
    int32_t rwSpeedsInMsgID;      /*!< [-] ID for the incoming RW speeds*/
}InertialUKFConfig;

#ifdef __cplusplus
extern "C" {
#endif
    
    void SelfInit_inertialUKF(InertialUKFConfig *ConfigData, uint64_t moduleID);
    void CrossInit_inertialUKF(InertialUKFConfig *ConfigData, uint64_t moduleID);
    void Update_inertialUKF(InertialUKFConfig *ConfigData, uint64_t callTime,
        uint64_t moduleID);
	void Reset_inertialUKF(InertialUKFConfig *ConfigData, uint64_t callTime,
		uint64_t moduleID);
	void inertialUKFTimeUpdate(InertialUKFConfig *ConfigData, double updateTime);
    void inertialUKFMeasUpdate(InertialUKFConfig *ConfigData, double updateTime);
	void inertialStateProp(InertialUKFConfig *ConfigData, double *stateInOut, double dt);
    void inertialUKFMeasModel(InertialUKFConfig *ConfigData);
    
#ifdef __cplusplus
}
#endif

/*! @} */

#endif
