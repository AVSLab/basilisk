
#ifndef _CSS_WLS_EST_H_
#define _CSS_WLS_EST_H_

#include "messaging/static_messaging.h"
#include "sensorInterfaces/CSSSensorData/cssComm.h"
#include <stdint.h>

/*! \addtogroup ADCSAlgGroup
 * @{
 */

/*! @brief Structure used to define the output definition for the CSS estimator*/
typedef struct {
   double sHatBdy[3];      /*!< -- unit vector to the sun in ADCS body frame */
}CSSWlsEstOut;

/*! @brief Structure used to contain the configuration information for 
    each sun sensor*/
typedef struct {
   double nHatBdy[3];      /*!< -- Normal unit vector for sensor in body frame*/
   double CBias;           /*!< W  Calibration coefficient bias for CSS */
   double cssNoiseStd;     /*!< -- Measurement noise uncertainty*/
}SingleCSSConfig;

/*! @brief Top level structure for the CSS weighted least squares estimator.  
    Used to estimate the sun state in the vehicle body frame*/
typedef struct {
   SingleCSSConfig CSSData[MAX_NUM_CSS_SENSORS]; /*!< -- The config data for the estimator*/
   char OutputDataName[MAX_STAT_MSG_LENGTH]; /*!< The name of the output message*/
   char InputDataName[MAX_STAT_MSG_LENGTH]; /*!< The name of the Input message*/
   uint32_t numActiveCss;   /*!< -- Number of currently active CSS sensors*/
   uint32_t UseWeights;     /*!< -- Flag indicating whether or not to use weights for least squares*/
   double SensorUseThresh;  /*!< -- Threshold below which we discount sensors*/
   CSSWlsEstOut OutputData; /*!< -- Unit vector to the Sun in the spacecraft body frame*/
   int32_t OutputMsgID;     /*!< -- ID for the outgoing body estimate message*/
   int32_t InputMsgID;      /*!< -- ID for the incoming CSS sensor message*/
}CSSWLSConfig;

#ifdef __cplusplus
   extern "C" {
#endif

void SelfInit_cssWlsEst(CSSWLSConfig *ConfigData);
void CrossInit_cssWlsEst(CSSWLSConfig *ConfigData);
void Update_cssWlsEst(CSSWLSConfig *ConfigData, uint64_t callTime);
int computeWlsmn(int numActiveCss, double *H, double *W,
                 double *y, double x[3]);

#ifdef __cplusplus
   }
#endif

/*! @} */

#endif
