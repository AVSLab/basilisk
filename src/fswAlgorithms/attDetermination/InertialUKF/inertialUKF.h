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

#ifndef _INERTIAL_UKF_H_
#define _INERTIAL_UKF_H_

#include "messaging/static_messaging.h"
#include "simFswInterfaceMessages/rwSpeedIntMsg.h"
#include "simFswInterfaceMessages/navAttIntMsg.h"
#include "fswMessages/inertialFilterFswMsg.h"
#include "fswMessages/stAttFswMsg.h"
#include "fswMessages/vehicleConfigFswMsg.h"
#include "fswMessages/rwArrayConfigFswMsg.h"
#include "transDetermination/dvAccumulation/dvAccumulation.h"
#include "simulation/utilities/signalCondition.h"
#include <stdint.h>


/*! \addtogroup ADCSAlgGroup
 * @{
 */

typedef struct {
    char stInMsgName[MAX_STAT_MSG_LENGTH];    /*!< [-] Input message buffer from MIRU*/
    int32_t stInMsgID;                        /*!< [-] Input message ID from MIRU*/
    double noise[3*3];       /*!< [-] Per axis noise on the ST*/
}STMessage;

/*! Structure to gather the ST messages and content */
typedef struct {
    int numST;                                  /* Number of Star Trackers */
    STMessage STMessages[MAX_ST_VEH_COUNT];     /*!< [-] Decoded MIRU data for both camera heads*/
}STDataParsing;

/*! @brief Top level structure for the Inertial unscented kalman filter.
 Used to estimate the spacecraft's inertial attitude. Measurements are StarTracker data and gyro data. Please see the _Documentation folder for details on how this Kalman Filter Functions.*/
typedef struct {
    char navStateOutMsgName[MAX_STAT_MSG_LENGTH]; /*!< The name of the output message*/
    char filtDataOutMsgName[MAX_STAT_MSG_LENGTH]; /*!< The name of the output filter data message*/
    
	char massPropsInMsgName[MAX_STAT_MSG_LENGTH]; /*!< [-] The name of the mass props message*/
    char rwParamsInMsgName[MAX_STAT_MSG_LENGTH];  /*!< The name of the RWConfigParams input message*/
    char rwSpeedsInMsgName[MAX_STAT_MSG_LENGTH]; /*!< [-] The name of the input RW speeds message*/
    char gyrBuffInMsgName[MAX_STAT_MSG_LENGTH]; /*!< [-] Input message buffer from MIRU*/

	int numStates;                /*!< [-] Number of states for this filter*/
	int countHalfSPs;             /*!< [-] Number of sigma points over 2 */
	int numObs;                   /*!< [-] Number of measurements this cycle */
    int badUpdate;                /*!< [-] Flag to raise if there is a undefined term in the measurement update */
	double beta;                  /*!< [-] Beta parameter for filter */
	double alpha;                 /*!< [-] Alpha parameter for filter*/
	double kappa;                 /*!< [-] Kappa parameter for filter*/
	double lambdaVal;             /*!< [-] Lambda parameter for filter*/
	double gamma;                 /*!< [-] Gamma parameter for filter*/
    double switchMag;             /*!< [-] Threshold for where we switch MRP set*/

	double dt;                     /*!< [s] seconds since last data epoch */
	double timeTag;                /*!< [s]  Time tag for statecovar/etc */
    double gyrAggTimeTag;          /*!< [s] Time-tag for aggregated gyro data*/
    double aggSigma_b2b1[3];       /*!< [-] Aggregated attitude motion from gyros*/
    double dcm_BdyGyrpltf[3][3];   /*!< [-] DCM for converting gyro data to body frame*/
	double wM[2 * AKF_N_STATES + 1]; /*!< [-] Weighting vector for sigma points*/
	double wC[2 * AKF_N_STATES + 1]; /*!< [-] Weighting vector for sigma points*/

	double state[AKF_N_STATES];        /*!< [-] State estimate for time TimeTag*/
	double sBar[AKF_N_STATES*AKF_N_STATES];         /*!< [-] Time updated covariance */
	double covar[AKF_N_STATES*AKF_N_STATES];        /*!< [-] covariance */
    double xBar[AKF_N_STATES];            /*! [-] Current mean state estimate*/

	double obs[3];          /*!< [-] Observation vector for frame*/
	double yMeas[3*(2*AKF_N_STATES+1)];        /*!< [-] Measurement model data */

	double SP[(2*AKF_N_STATES+1)*AKF_N_STATES];          /*!< [-]    sigma point matrix */

	double qNoise[MAX_ST_VEH_COUNT*AKF_N_STATES*AKF_N_STATES];       /*!< [-] process noise matrix */
	double sQnoise[MAX_ST_VEH_COUNT*AKF_N_STATES*AKF_N_STATES];      /*!< [-] cholesky of Qnoise */

    double IInv[3][3];

    uint32_t numUsedGyros;   /*!< -- Number of currently active CSS sensors*/
    uint32_t firstPassComplete;
    double sigma_BNOut[3];   /*!< [-] Output MRP*/
    double omega_BN_BOut[3]; /*!< [r/s] Body rate output data*/
    double timeTagOut;       /*!< [s] Output time-tag information*/
	NavAttIntMsg outputInertial;        /*!< -- Output inertial estimate data */
    STAttFswMsg stSensorIn[MAX_ST_VEH_COUNT]; /*!< [-] ST sensor data read in from message bus*/
    int stSensorOrder[MAX_ST_VEH_COUNT]; /*!< [-] ST sensor data read in from message bus*/
    int ClockTimeST[MAX_ST_VEH_COUNT];  /*!< [-] All of the ClockTimes for the STs*/
    int ReadSizeST[MAX_ST_VEH_COUNT];  /*!< [-] All of the ReadSizes for the STs*/
    RWArrayConfigFswMsg rwConfigParams;       /*!< [-] struct to store message containing RW config parameters in body B frame */
    RWSpeedIntMsg rwSpeeds;             /*! [-] Local reaction wheel speeds */
    RWSpeedIntMsg rwSpeedPrev;          /*! [-] Local reaction wheel speeds */
    double speedDt;                     /*! [s] The time difference between speeds*/
    uint64_t timeWheelPrev;             /*! [ns] Previous wheel time-tag from msg*/
    VehicleConfigFswMsg localConfigData;   /*! [-] Vehicle configuration data*/
    LowPassFilterData gyroFilt[3];      /*! [-] Low-pass filters for input gyro data*/
    int32_t navStateOutMsgId;     /*!< -- ID for the outgoing body estimate message*/
    int32_t filtDataOutMsgId;   /*!< [-] ID for the filter data output message*/
    int32_t massPropsInMsgId;    /*!< [-] ID for the incoming mass properties message*/
    int32_t rwParamsInMsgID;     /*!< [-] ID for the RWArrayConfigFswMsg ingoing message */
    int32_t rwSpeedsInMsgID;      /*!< [-] ID for the incoming RW speeds*/
    int32_t gyrBuffInMsgID;         /*!< [-] ID of the input message buffer*/
    
    STDataParsing STDatasStruct;      /*!< [-] ID of the input message buffer*/
}InertialUKFConfig;


#ifdef __cplusplus
extern "C" {
#endif
    
    void SelfInit_inertialUKF(InertialUKFConfig *ConfigData, uint64_t moduleID);
    void CrossInit_inertialUKF(InertialUKFConfig *ConfigData, uint64_t moduleID);
    void Read_STMessages(InertialUKFConfig *ConfigData, uint64_t moduleID);
    void Update_inertialUKF(InertialUKFConfig *ConfigData, uint64_t callTime,
        uint64_t moduleID);
	void Reset_inertialUKF(InertialUKFConfig *ConfigData, uint64_t callTime,
		uint64_t moduleID);
    void inertialUKFAggGyrData(InertialUKFConfig *ConfigData, double prevTime,
                          double propTime, AccDataFswMsg *gyrData);
	void inertialUKFTimeUpdate(InertialUKFConfig *ConfigData, double updateTime);
    void inertialUKFMeasUpdate(InertialUKFConfig *ConfigData, double updateTime, int currentST);
	void inertialStateProp(InertialUKFConfig *ConfigData, double *stateInOut, double dt);
    void inertialUKFMeasModel(InertialUKFConfig *ConfigData, int currentST);
    
#ifdef __cplusplus
}
#endif

/*! @} */

#endif
