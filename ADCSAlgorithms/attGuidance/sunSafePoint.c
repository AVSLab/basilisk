
#include "attGuidance/sunSafePoint.h"
#include "SimCode/utilities/linearAlgebra.h"
#include "SimCode/utilities/rigidBodyKinematics.h"
#include "sensorInterfaces/IMUSensorData/imuComm.h"
#include <string.h>
#include <math.h>

/*! This method initializes the ConfigData for the sun safe attitude guidance.
 It checks to ensure that the inputs are sane and then creates the
 output message
 @return void
 @param ConfigData The configuration data associated with the sun safe guidance
 */
void SelfInit_sunSafePoint(sunSafePointConfig *ConfigData)
{
    
    /*! Begin method steps */
    /*! - Create output message for module */
    ConfigData->outputMsgID = CreateNewMessage(ConfigData->outputDataName,
        sizeof(attGuidOut), "attGuidOut");
    memset(ConfigData->attOut.intsigma_BR, 0x0, 3*sizeof(double));
    memset(ConfigData->attOut.omega_rB, 0x0, 3*sizeof(double));
    memset(ConfigData->attOut.domega_rB, 0x0, 3*sizeof(double));
    
}

/*! This method performs the second stage of initialization for the sun safe attitude
 interface.  It's primary function is to link the input messages that were
 created elsewhere.
 @return void
 @param ConfigData The configuration data associated with the sun safe attitude guidance
 */
void CrossInit_sunSafePoint(sunSafePointConfig *ConfigData)
{
    /*! - Loop over the number of sensors and find IDs for each one */
    ConfigData->inputMsgID = FindMessageID(ConfigData->inputSunVecName);
    ConfigData->imuMsgID = FindMessageID(ConfigData->inputIMUDataName);
    
}

/*! This method takes the estimated body-observed sun vector and computes the
 current attitude/attitude rate errors to pass on to control.
 @return void
 @param ConfigData The configuration data associated with the sun safe attitude guidance
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void Update_sunSafePoint(sunSafePointConfig *ConfigData, uint64_t callTime)
{
    CSSWlsEstOut sunVecEst;
    uint64_t clockTime;
    uint32_t readSize;
    double ctSNormalized;
    double e_hat[3];
    double sigma_BR[3];
    IMUOutputData LocalIMUData;
    /*! Begin method steps*/
    /*! - Read the current sun body vector estimate*/
    ReadMessage(ConfigData->inputMsgID, &clockTime, &readSize,
                sizeof(CSSWlsEstOut), (void*) &(sunVecEst));
    ReadMessage(ConfigData->imuMsgID, &clockTime, &readSize,
                sizeof(IMUOutputData), (void*) &(LocalIMUData));
    
    /*! - Compute the current error vector if it is valid*/
    if(v3Norm(sunVecEst.sHatBdy) > ConfigData->minUnitMag)
    {
        ctSNormalized = v3Dot(ConfigData->sHatBdyCmd, sunVecEst.sHatBdy);
        ctSNormalized = fabs(ctSNormalized) > 1.0 ?
        ctSNormalized/fabs(ctSNormalized) : ctSNormalized;
        ConfigData->sunAngleErr = acos(ctSNormalized);
        v3Cross(sunVecEst.sHatBdy, ConfigData->sHatBdyCmd, e_hat);
        v3Normalize(e_hat, ConfigData->sunMnvrVec);
        v3Scale(tan(ConfigData->sunAngleErr*0.25), ConfigData->sunMnvrVec,
                sigma_BR);
        v3Copy(sigma_BR, ConfigData->attOut.sigma_BR);
    }
    v3Copy(LocalIMUData.AngVelBody, ConfigData->attOut.omega_BR);
    WriteMessage(ConfigData->outputMsgID, callTime, sizeof(attGuidOut),
                 (void*) &(ConfigData->attOut));
    
    return;
}
