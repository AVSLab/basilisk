#ifndef _SIM_ALGORITHMS_
#define _SIM_ALGORITHMS_

#include "imuComm.h"
#include "cssComm.h"
#include "cssWlsEst.h"
#include "sunSafePoint.h"
#include "simpleDeadband.h"
#include "MRP_PD.h"
#include "sunSafeACS.h"
#include "celestialBodyPoint.h"
#include "celestialBodyPoint.h"
#include "celestialBodyPoint.h"
#include "attRefGen.h"
#include "MRP_Steering.h"
#include "dvAttEffect.h"
#include "rwNullSpace.h"
#include "dvGuidance.h"
#include "dvGuidance.h"
#include "attRefGen.h"
#include "MRP_Steering.h"
#include "dvAttEffect.h"
#include "thrustRWDesat.h"
#include "cssComm.h"
#include "imuComm.h"
#include "stComm.h"
#include "inertial3D.h"
#include "hillPoint.h"
#include "velocityPoint.h"
#include "celestialTwoBodyPoint.h"
#include "rasterManager.h"
#include "eulerRotation.h"
#include "eulerRotation.h"
#include "inertial3DSpin.h"
#include "attTrackingError.h"
#include "errorDeadband.h"
#include "MRP_Steering.h"
#include "dvAttEffect.h"
#include "rwNullSpace.h"
#include "attTrackingError.h"
#include "MRP_Steering.h"
#include "dvAttEffect.h"
#include "rwNullSpace.h"
#include "attTrackingError.h"
#include "MRP_Feedback.h"
#include "dvAttEffect.h"
#include "rwNullSpace.h"
#include "attTrackingError.h"
#include "PRV_Steering.h"
#include "dvAttEffect.h"
#include "rwNullSpace.h"

#ifdef __cplusplus
extern "C" {
#endif
	void sunSafeFSWTask_Update(EMMConfigData *data);
	void sunSafeFSWTask_Reset(EMMConfigData *data);
	void sunPointTask_Update(EMMConfigData *data);
	void earthPointTask_Update(EMMConfigData *data);
	void marsPointTask_Update(EMMConfigData *data);
	void vehicleAttMnvrFSWTask_Update(EMMConfigData *data);
	void vehicleAttMnvrFSWTask_Reset(EMMConfigData *data);
	void vehicleDVPrepFSWTask_Update(EMMConfigData *data);
	void vehicleDVMnvrFSWTask_Update(EMMConfigData *data);
	void vehicleDVMnvrFSWTask_Reset(EMMConfigData *data);
	void RWADesatTask_Update(EMMConfigData *data);
	void RWADesatTask_Reset(EMMConfigData *data);
	void sensorProcessing_Update(EMMConfigData *data);
	void inertial3DPointTask_Update(EMMConfigData *data);
	void inertial3DPointTask_Reset(EMMConfigData *data);
	void hillPointTask_Update(EMMConfigData *data);
	void hillPointTask_Reset(EMMConfigData *data);
	void velocityPointTask_Update(EMMConfigData *data);
	void velocityPointTask_Reset(EMMConfigData *data);
	void celTwoBodyPointTask_Update(EMMConfigData *data);
	void rasterMnvrTask_Update(EMMConfigData *data);
	void rasterMnvrTask_Reset(EMMConfigData *data);
	void eulerRotationTask_Update(EMMConfigData *data);
	void eulerRotationTask_Reset(EMMConfigData *data);
	void inertial3DSpinTask_Update(EMMConfigData *data);
	void inertial3DSpinTask_Reset(EMMConfigData *data);
	void trackingErrorTask_Update(EMMConfigData *data);
	void trackingErrorTask_Reset(EMMConfigData *data);
	void controlTask_Update(EMMConfigData *data);
	void controlTask_Reset(EMMConfigData *data);
	void attitudeControlMnvrTask_Update(EMMConfigData *data);
	void attitudeControlMnvrTask_Reset(EMMConfigData *data);
	void feedbackControlMnvrTask_Update(EMMConfigData *data);
	void feedbackControlMnvrTask_Reset(EMMConfigData *data);
	void attitudePRVControlMnvrTask_Update(EMMConfigData *data);
	void attitudePRVControlMnvrTask_Reset(EMMConfigData *data);
	void allAlg_SelfInit(EMMConfigData *data);
	void allAlg_CrossInit(EMMConfigData *data);
#ifdef __cplusplus
}
#endif