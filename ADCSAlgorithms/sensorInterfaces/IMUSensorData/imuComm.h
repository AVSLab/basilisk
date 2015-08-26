
#ifndef _IMU_COMM_H_
#define _IMU_COMM_H_

#include "messaging/static_messaging.h"
#include "vehicleConfigData/vehicleConfigData.h"

/*! @brief Output structure for IMU structure in vehicle body frame*/
typedef struct {
    double DVFrameBody[3];      /*!< m/s Accumulated DVs in body*/
    double AccelBody[3];        /*!< m/s2 Apparent acceleration of the body*/
    double DRFrameBody[3];      /*!< r  Accumulated DRs in body*/
    double AngVelBody[3];       /*!< r/s Angular velocity in platform body*/
}IMUOutputData;

/*! @brief Top level structure for the CSS sensor interface system.  Contains all parameters for the
 CSS interface*/
typedef struct {
    double platform2StrDCM[9];                /*!< Row major platform 2 str DCM*/
    char InputDataName[MAX_STAT_MSG_LENGTH]; /*!< The name of the input message*/
    char InputPropsName[MAX_STAT_MSG_LENGTH]; /*!< The name of the ADCS config data message*/
    char OutputDataName[MAX_STAT_MSG_LENGTH]; /*!< The name of the output message*/
    int32_t SensorMsgID; /*!< Sensor IDs tied to the input name*/
    int32_t PropsMsgID;  /*!< Sensor ID tied to the ADCS config data message*/
    int32_t OutputMsgID; /*!< Message ID for the output port*/
    double platform2BdyDCM[9]; /*!< Row major platform 2 bdy DCM*/
    IMUOutputData LocalOutput; /*!< Output data structure*/
}IMUConfigData;

#ifdef __cplusplus
extern "C" {
#endif
    
    void SelfInit_imuProcessTelem(IMUConfigData *ConfigData);
    void CrossInit_imuProcessTelem(IMUConfigData *ConfigData);
    void Update_imuProcessTelem(IMUConfigData *ConfigData, uint64_t callTime);
    
#ifdef __cplusplus
}
#endif

/*! @} */

#endif
