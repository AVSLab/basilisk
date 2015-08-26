
#ifndef _CSS_COMM_H_
#define _CSS_COMM_H_

#define MAX_NUM_CSS_SENSORS 32
#define MAX_NUM_CHEBY_POLYS 32

#include "messaging/static_messaging.h"

/*! \addtogroup ADCSAlgGroup
 *  This grouping contains the algorithms developed for the ADCS flight system
 * @{
 */

/*! @brief Output structure for CSS interface is only the current cosine for each sensor*/
typedef struct {
    double CosValue;   /*!< Current cosine value for a single sensor*/
}CSSOutputData;

/*! @brief Carrier structure for each coarse sun sensor name*/
typedef struct {
    char SensorMsgName[MAX_STAT_MSG_LENGTH]; /*!< We have to abstract this struct for SWIG interfacing*/
}SensorMsgNameCarrier;

/*! @brief Top level structure for the CSS sensor interface system.  Contains all parameters for the
 CSS interface*/
typedef struct {
    uint32_t  NumSensors;   /*!< The number of sensors we are processing*/
    SensorMsgNameCarrier SensorList[MAX_NUM_CSS_SENSORS]; /*!< The list of sensor names*/
    char OutputDataName[MAX_STAT_MSG_LENGTH]; /*!< The name of the output message*/
    int32_t SensorMsgIDs[MAX_NUM_CSS_SENSORS]; /*!< Sensor IDs tied to the list of names*/
    int32_t OutputMsgID; /*!< Message ID for the output port*/
    double InputValues[MAX_NUM_CSS_SENSORS]; /*!< Input values we took off the messaging system*/
    double MaxSensorValue; /*!< Scale factor to go from sensor values to cosine*/
    uint32_t ChebyCount; /*!< -- Count on the number of chebyshev polynominals we have*/
    double KellyCheby[MAX_NUM_CHEBY_POLYS]; /*!< -- Chebyshev polynominals to fit output to cosine*/
}CSSConfigData;

#ifdef __cplusplus
extern "C" {
#endif
    
    void SelfInit_cssProcessTelem(CSSConfigData *ConfigData);
    void CrossInit_cssProcessTelem(CSSConfigData *ConfigData);
    void Update_cssProcessTelem(CSSConfigData *ConfigData, uint64_t callTime);
    
#ifdef __cplusplus
}
#endif

/*! @} */

#endif
