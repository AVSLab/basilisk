
#include "attGuidance/singleAxisRot.h"
#include "SimCode/utilities/linearAlgebra.h"
#include "SimCode/utilities/rigidBodyKinematics.h"
#include "SimCode/environment/spice/spice_planet_state.h"
#include "sensorInterfaces/IMUSensorData/imuComm.h"
#include "attDetermination/CSSEst/navStateOut.h"
#include <string.h>
#include <math.h>

/*! This method initializes the ConfigData for the single axis rotation guidance.
 It checks to ensure that the inputs are sane and then creates the
 output message
 @return void
 @param ConfigData The configuration data associated with the single axis guidance
 */
void SelfInit_singleAxisRot(singleAxisRotConfig *ConfigData,
    uint64_t moduleID)
{
    
    /*! Begin method steps */
    /*! - Create output message for module */
    ConfigData->outputMsgID = CreateNewMessage(
        ConfigData->outputDataName, sizeof(attCmdOut), "attCmdOut", moduleID);
    return;
    
}

/*! This method performs the second stage of initialization for the single axis guidance
 interface.  It's primary function is to link the input messages that were
 created elsewhere.
 @return void
 @param ConfigData The configuration data associated with the single axis mnvr guidance
 */
void CrossInit_singleAxisRot(singleAxisRotConfig *ConfigData,
    uint64_t moduleID)
{
    ConfigData->inputAttID = subscribeToMessage(ConfigData->inputAttCmdName,
        sizeof(attCmdOut), moduleID);
    return;
    
}

/*! This method takes the nominal spacecraft pointing cmd (ex. NadirPoint), and then 
    rotates about a body axis.  The primary use of this function is to point an arbitrary 
	body axis at the ground and then rotate the spacecraft around that axis to image a disc.
 @return void
 @param ConfigData The configuration data associated with the single axis guidance
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void Update_singleAxisRot(singleAxisRotConfig *ConfigData,
    uint64_t callTime, uint64_t moduleID)
{
    uint64_t writeTime;
    uint32_t writeSize;
    
    return;
}

