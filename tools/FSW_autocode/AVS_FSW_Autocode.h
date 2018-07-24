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
#ifndef AVS_FSW_AUTOCODE_
#define AVS_FSW_AUTOCODE_

#include "../ADCSAlgorithms/vehicleConfigData/vehicleConfigData.h"
#include "../ADCSAlgorithms/rwConfigData/rwConfigData.h"
#include "../ADCSAlgorithms/sensorInterfaces/IMUSensorData/imuComm.h"
#include "../ADCSAlgorithms/sensorInterfaces/CSSSensorData/cssComm.h"
#include "../ADCSAlgorithms/attDetermination/CSSEst/cssWlsEst.h"
#include "../ADCSAlgorithms/attGuidance/sunSafePoint/sunSafePoint.h"
#include "../ADCSAlgorithms/attGuidance/simpleDeadband/simpleDeadband.h"
#include "../ADCSAlgorithms/attControl/MRP_PD/MRP_PD.h"
#include "../ADCSAlgorithms/attGuidance/celestialBodyPoint/celestialBodyPoint.h"
#include "../ADCSAlgorithms/attGuidance/attRefGen/attRefGen.h"
#include "../ADCSAlgorithms/attControl/MRP_Steering/MRP_Steering.h"
#include "../ADCSAlgorithms/effectorInterfaces/rwMotorTorque/rwMotorTorque.h"
#include "../ADCSAlgorithms/dvGuidance/dvAttGuidance/dvGuidance.h"
#include "../ADCSAlgorithms/effectorInterfaces/errorConversion/dvAttEffect.h"
#include "../ADCSAlgorithms/effectorInterfaces/thrustRWDesat/thrustRWDesat.h"
#include "../ADCSAlgorithms/effectorInterfaces/thrForceMapping/thrForceMapping.h"
#include "../ADCSAlgorithms/effectorInterfaces/thrFiringSchmitt/thrFiringSchmitt.h"
#include "../ADCSAlgorithms/sensorInterfaces/STSensorData/stComm.h"
#include "../ADCSAlgorithms/attGuidance/inertial3D/inertial3D.h"
#include "../ADCSAlgorithms/attGuidance/hillPoint/hillPoint.h"
#include "../ADCSAlgorithms/attGuidance/velocityPoint/velocityPoint.h"
#include "../ADCSAlgorithms/attGuidance/celestialTwoBodyPoint/celestialTwoBodyPoint.h"
#include "../ADCSAlgorithms/attGuidance/rasterManager/rasterManager.h"
#include "../ADCSAlgorithms/attGuidance/eulerRotation/eulerRotation.h"
#include "../ADCSAlgorithms/attGuidance/inertial3DSpin/inertial3DSpin.h"
#include "../ADCSAlgorithms/attGuidance/attTrackingError/attTrackingError.h"
#include "../ADCSAlgorithms/attControl/MRP_Feedback/MRP_Feedback.h"

typedef struct{
	VehConfigInputData vehConfigData;
	rwConfigData rwConfigData;
	IMUConfigData imuSensorDecode;
	CSSConfigData cssSensorDecode;
	CSSWLSConfig CSSWlsEst;
	sunSafePointConfig sunSafePoint;
	simpleDeadbandConfig simpleDeadband;
	MRP_PDConfig MRP_PD;
	celestialBodyPointConfig sunPoint;
	celestialBodyPointConfig earthPoint;
	celestialBodyPointConfig marsPoint;
	attRefGenConfig attMnvrPoint;
	MRP_SteeringConfig MRP_SteeringRWA;
	rwMotorTorqueConfig rwMotorTorque;
	dvGuidanceConfig dvGuidance;
	MRP_SteeringConfig MRP_SteeringMOI;
	dvAttEffectConfig dvAttEffect;
	thrustRWDesatConfig thrustRWDesat;
	thrForceMappingConfig thrForceMapping;
	thrFiringSchmittConfig thrFiringSchmitt;
	STConfigData stSensorDecode;
	inertial3DConfig inertial3D;
	hillPointConfig hillPoint;
	velocityPointConfig velocityPoint;
	celestialTwoBodyPointConfig celTwoBodyPoint;
	rasterManagerConfig rasterManager;
	eulerRotationConfig eulerRotation;
	inertial3DSpinConfig inertial3DSpin;
	attTrackingErrorConfig attTrackingError;
	MRP_FeedbackConfig MRP_FeedbackRWA;
	uint32_t sunSafeFSWTask_isActive;
	uint32_t velocityPointTask_isActive;
	uint32_t rasterMnvrTask_isActive;
	uint32_t earthPointTask_isActive;
	uint32_t thrFiringSchmittTask_isActive;
	uint32_t vehicleAttMnvrFSWTask_isActive;
	uint32_t inertial3DPointTask_isActive;
	uint32_t marsPointTask_isActive;
	uint32_t hillPointTask_isActive;
	uint32_t thrForceMappingTask_isActive;
	uint32_t celTwoBodyPointTask_isActive;
	uint32_t vehicleDVPrepFSWTask_isActive;
	uint32_t initOnlyTask_isActive;
	uint32_t vehicleDVMnvrFSWTask_isActive;
	uint32_t inertial3DSpinTask_isActive;
	uint32_t feedbackControlMnvrTask_isActive;
	uint32_t RWADesatTask_isActive;
	uint32_t sensorProcessing_isActive;
	uint32_t eulerRotationTask_isActive;
	uint32_t sunPointTask_isActive;
	uint32_t simpleRWControlTask_isActive;
}AVSConfigData;

#ifdef __cplusplus
extern "C" {
#endif
	void initOnlyTask_Update(AVSConfigData *data, uint64_t callTime);
	void initOnlyTask_Reset(AVSConfigData *data, uint64_t callTime);
	void sunSafeFSWTask_Update(AVSConfigData *data, uint64_t callTime);
	void sunSafeFSWTask_Reset(AVSConfigData *data, uint64_t callTime);
	void sunPointTask_Update(AVSConfigData *data, uint64_t callTime);
	void earthPointTask_Update(AVSConfigData *data, uint64_t callTime);
	void marsPointTask_Update(AVSConfigData *data, uint64_t callTime);
	void vehicleAttMnvrFSWTask_Update(AVSConfigData *data, uint64_t callTime);
	void vehicleAttMnvrFSWTask_Reset(AVSConfigData *data, uint64_t callTime);
	void vehicleDVPrepFSWTask_Update(AVSConfigData *data, uint64_t callTime);
	void vehicleDVMnvrFSWTask_Update(AVSConfigData *data, uint64_t callTime);
	void vehicleDVMnvrFSWTask_Reset(AVSConfigData *data, uint64_t callTime);
	void RWADesatTask_Update(AVSConfigData *data, uint64_t callTime);
	void RWADesatTask_Reset(AVSConfigData *data, uint64_t callTime);
	void thrForceMappingTask_Update(AVSConfigData *data, uint64_t callTime);
	void thrForceMappingTask_Reset(AVSConfigData *data, uint64_t callTime);
	void thrFiringSchmittTask_Update(AVSConfigData *data, uint64_t callTime);
	void thrFiringSchmittTask_Reset(AVSConfigData *data, uint64_t callTime);
	void sensorProcessing_Update(AVSConfigData *data, uint64_t callTime);
	void inertial3DPointTask_Update(AVSConfigData *data, uint64_t callTime);
	void inertial3DPointTask_Reset(AVSConfigData *data, uint64_t callTime);
	void hillPointTask_Update(AVSConfigData *data, uint64_t callTime);
	void hillPointTask_Reset(AVSConfigData *data, uint64_t callTime);
	void velocityPointTask_Update(AVSConfigData *data, uint64_t callTime);
	void velocityPointTask_Reset(AVSConfigData *data, uint64_t callTime);
	void celTwoBodyPointTask_Update(AVSConfigData *data, uint64_t callTime);
	void celTwoBodyPointTask_Reset(AVSConfigData *data, uint64_t callTime);
	void rasterMnvrTask_Update(AVSConfigData *data, uint64_t callTime);
	void rasterMnvrTask_Reset(AVSConfigData *data, uint64_t callTime);
	void eulerRotationTask_Update(AVSConfigData *data, uint64_t callTime);
	void eulerRotationTask_Reset(AVSConfigData *data, uint64_t callTime);
	void inertial3DSpinTask_Update(AVSConfigData *data, uint64_t callTime);
	void inertial3DSpinTask_Reset(AVSConfigData *data, uint64_t callTime);
	void feedbackControlMnvrTask_Update(AVSConfigData *data, uint64_t callTime);
	void feedbackControlMnvrTask_Reset(AVSConfigData *data, uint64_t callTime);
	void simpleRWControlTask_Update(AVSConfigData *data, uint64_t callTime);
	void simpleRWControlTask_Reset(AVSConfigData *data, uint64_t callTime);
	void AllAlg_SelfInit(AVSConfigData *data);
	void AllAlg_CrossInit(AVSConfigData *data);
	void AllAlg_Reset(AVSConfigData *data, uint64_t callTime);
	void AllTasks_Update(AVSConfigData *data, uint64_t callTime);
	void DataInit(AVSConfigData *data);
#ifdef __cplusplus
}
#endif

#endif