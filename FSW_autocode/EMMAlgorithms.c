#include EMMAlgorithms.h

void sunSafeFSWTask_Update(EMMConfigData *data)
{
	Update_imuProcessTelem(&(data->imuSensorDecode), 2);
	Update_cssProcessTelem(&(data->cssSensorDecode), 0);
	Update_cssWlsEst(&(data->CSSWlsEst), 1);
	Update_sunSafePoint(&(data->sunSafePoint), 28);
	Update_simpleDeadband(&(data->simpleDeadband), 25);
	Update_MRP_PD(&(data->MRP_PD), 4);
	Update_sunSafeACS(&(data->sunSafeACS), 27);
}
void sunSafeFSWTask_Reset(EMMConfigData *data)
{
	Reset_simpleDeadband(&(data->simpleDeadband), 25);
	Reset_MRP_PD(&(data->MRP_PD), 4);
}
void sunPointTask_Update(EMMConfigData *data)
{
	Update_celestialBodyPoint(&(data->sunPoint), 26);
}
void earthPointTask_Update(EMMConfigData *data)
{
	Update_celestialBodyPoint(&(data->earthPoint), 17);
}
void marsPointTask_Update(EMMConfigData *data)
{
	Update_celestialBodyPoint(&(data->marsPoint), 23);
}
void vehicleAttMnvrFSWTask_Update(EMMConfigData *data)
{
	Update_attRefGen(&(data->attMnvrPoint), 12);
	Update_MRP_Steering(&(data->MRP_SteeringRWA), 6);
	Update_dvAttEffect(&(data->RWAMappingData), 9);
	Update_rwNullSpace(&(data->RWNullSpace), 10);
}
void vehicleAttMnvrFSWTask_Reset(EMMConfigData *data)
{
	Reset_attRefGen(&(data->attMnvrPoint), 12);
	Reset_MRP_Steering(&(data->MRP_SteeringRWA), 6);
	Reset_dvAttEffect(&(data->RWAMappingData), 9);
	Reset_rwNullSpace(&(data->RWNullSpace), 10);
}
void vehicleDVPrepFSWTask_Update(EMMConfigData *data)
{
	Update_dvGuidance(&(data->dvGuidance), 16);
}
void vehicleDVMnvrFSWTask_Update(EMMConfigData *data)
{
	Update_dvGuidance(&(data->dvGuidance), 16);
	Update_attRefGen(&(data->attMnvrPoint), 12);
	Update_MRP_Steering(&(data->MRP_SteeringMOI), 5);
	Update_dvAttEffect(&(data->dvAttEffect), 15);
}
void vehicleDVMnvrFSWTask_Reset(EMMConfigData *data)
{
	Reset_attRefGen(&(data->attMnvrPoint), 12);
	Reset_MRP_Steering(&(data->MRP_SteeringMOI), 5);
	Reset_dvAttEffect(&(data->dvAttEffect), 15);
}
void RWADesatTask_Update(EMMConfigData *data)
{
	Update_thrustRWDesat(&(data->thrustRWDesat), 29);
}
void RWADesatTask_Reset(EMMConfigData *data)
{
	Reset_thrustRWDesat(&(data->thrustRWDesat), 29);
}
void sensorProcessing_Update(EMMConfigData *data)
{
	Update_cssProcessTelem(&(data->cssSensorDecode), 0);
	Update_imuProcessTelem(&(data->imuSensorDecode), 2);
	Update_stProcessTelem(&(data->stSensorDecode), 11);
}
void inertial3DPointTask_Update(EMMConfigData *data)
{
	Update_inertial3D(&(data->inertial3D), 22);
}
void inertial3DPointTask_Reset(EMMConfigData *data)
{
	Reset_inertial3D(&(data->inertial3D), 22);
}
void hillPointTask_Update(EMMConfigData *data)
{
	Update_hillPoint(&(data->hillPoint), 20);
}
void hillPointTask_Reset(EMMConfigData *data)
{
	Reset_hillPoint(&(data->hillPoint), 20);
}
void velocityPointTask_Update(EMMConfigData *data)
{
	Update_velocityPoint(&(data->velocityPoint), 30);
}
void velocityPointTask_Reset(EMMConfigData *data)
{
	Reset_velocityPoint(&(data->velocityPoint), 30);
}
void celTwoBodyPointTask_Update(EMMConfigData *data)
{
	Update_celestialTwoBodyPoint(&(data->celTwoBodyPoint), 14);
}
void rasterMnvrTask_Update(EMMConfigData *data)
{
	Update_rasterManager(&(data->rasterManager), 24);
	Update_eulerRotation(&(data->eulerRotation), 19);
}
void rasterMnvrTask_Reset(EMMConfigData *data)
{
	Reset_rasterManager(&(data->rasterManager), 24);
	Reset_eulerRotation(&(data->eulerRotation), 19);
}
void eulerRotationTask_Update(EMMConfigData *data)
{
	Update_eulerRotation(&(data->eulerRotation), 19);
}
void eulerRotationTask_Reset(EMMConfigData *data)
{
	Reset_eulerRotation(&(data->eulerRotation), 19);
}
void inertial3DSpinTask_Update(EMMConfigData *data)
{
	Update_inertial3DSpin(&(data->inertial3DSpin), 21);
}
void inertial3DSpinTask_Reset(EMMConfigData *data)
{
	Reset_inertial3DSpin(&(data->inertial3DSpin), 21);
}
void trackingErrorTask_Update(EMMConfigData *data)
{
	Update_attTrackingError(&(data->attTrackingError), 13);
	Update_errorDeadband(&(data->errorDeadband), 18);
}
void trackingErrorTask_Reset(EMMConfigData *data)
{
	Reset_attTrackingError(&(data->attTrackingError), 13);
	Reset_errorDeadband(&(data->errorDeadband), 18);
}
void controlTask_Update(EMMConfigData *data)
{
	Update_MRP_Steering(&(data->MRP_SteeringRWA), 6);
	Update_dvAttEffect(&(data->RWAMappingData), 9);
	Update_rwNullSpace(&(data->RWNullSpace), 10);
}
void controlTask_Reset(EMMConfigData *data)
{
	Reset_MRP_Steering(&(data->MRP_SteeringRWA), 6);
	Reset_dvAttEffect(&(data->RWAMappingData), 9);
	Reset_rwNullSpace(&(data->RWNullSpace), 10);
}
void attitudeControlMnvrTask_Update(EMMConfigData *data)
{
	Update_attTrackingError(&(data->attTrackingError), 13);
	Update_MRP_Steering(&(data->MRP_SteeringRWA), 6);
	Update_dvAttEffect(&(data->RWAMappingData), 9);
	Update_rwNullSpace(&(data->RWNullSpace), 10);
}
void attitudeControlMnvrTask_Reset(EMMConfigData *data)
{
	Reset_attTrackingError(&(data->attTrackingError), 13);
	Reset_MRP_Steering(&(data->MRP_SteeringRWA), 6);
	Reset_dvAttEffect(&(data->RWAMappingData), 9);
	Reset_rwNullSpace(&(data->RWNullSpace), 10);
}
void feedbackControlMnvrTask_Update(EMMConfigData *data)
{
	Update_attTrackingError(&(data->attTrackingError), 13);
	Update_MRP_Feedback(&(data->MRP_FeedbackRWA), 3);
	Update_dvAttEffect(&(data->RWAMappingData), 9);
	Update_rwNullSpace(&(data->RWNullSpace), 10);
}
void feedbackControlMnvrTask_Reset(EMMConfigData *data)
{
	Reset_attTrackingError(&(data->attTrackingError), 13);
	Reset_MRP_Feedback(&(data->MRP_FeedbackRWA), 3);
	Reset_dvAttEffect(&(data->RWAMappingData), 9);
	Reset_rwNullSpace(&(data->RWNullSpace), 10);
}
void attitudePRVControlMnvrTask_Update(EMMConfigData *data)
{
	Update_attTrackingError(&(data->attTrackingError), 13);
	Update_PRV_Steering(&(data->PRV_SteeringRWA), 8);
	Update_dvAttEffect(&(data->RWAMappingData), 9);
	Update_rwNullSpace(&(data->RWNullSpace), 10);
}
void attitudePRVControlMnvrTask_Reset(EMMConfigData *data)
{
	Reset_attTrackingError(&(data->attTrackingError), 13);
	Reset_PRV_Steering(&(data->PRV_SteeringRWA), 8);
	Reset_dvAttEffect(&(data->RWAMappingData), 9);
	Reset_rwNullSpace(&(data->RWNullSpace), 10);
}
void allAlg_SelfInit(EMMConfigData *data)
{
	SelfInit_imuProcessTelem(&(data->imuSensorDecode), 2);
	SelfInit_cssProcessTelem(&(data->cssSensorDecode), 0);
	SelfInit_cssWlsEst(&(data->CSSWlsEst), 1);
	SelfInit_sunSafePoint(&(data->sunSafePoint), 28);
	SelfInit_simpleDeadband(&(data->simpleDeadband), 25);
	SelfInit_MRP_PD(&(data->MRP_PD), 4);
	SelfInit_sunSafeACS(&(data->sunSafeACS), 27);
	SelfInit_celestialBodyPoint(&(data->sunPoint), 26);
	SelfInit_celestialBodyPoint(&(data->earthPoint), 17);
	SelfInit_celestialBodyPoint(&(data->marsPoint), 23);
	SelfInit_attRefGen(&(data->attMnvrPoint), 12);
	SelfInit_MRP_Steering(&(data->MRP_SteeringRWA), 6);
	SelfInit_dvAttEffect(&(data->RWAMappingData), 9);
	SelfInit_rwNullSpace(&(data->RWNullSpace), 10);
	SelfInit_dvGuidance(&(data->dvGuidance), 16);
	SelfInit_MRP_Steering(&(data->MRP_SteeringMOI), 5);
	SelfInit_dvAttEffect(&(data->dvAttEffect), 15);
	SelfInit_thrustRWDesat(&(data->thrustRWDesat), 29);
	SelfInit_stProcessTelem(&(data->stSensorDecode), 11);
	SelfInit_inertial3D(&(data->inertial3D), 22);
	SelfInit_hillPoint(&(data->hillPoint), 20);
	SelfInit_velocityPoint(&(data->velocityPoint), 30);
	SelfInit_celestialTwoBodyPoint(&(data->celTwoBodyPoint), 14);
	SelfInit_rasterManager(&(data->rasterManager), 24);
	SelfInit_eulerRotation(&(data->eulerRotation), 19);
	SelfInit_inertial3DSpin(&(data->inertial3DSpin), 21);
	SelfInit_attTrackingError(&(data->attTrackingError), 13);
	SelfInit_errorDeadband(&(data->errorDeadband), 18);
	SelfInit_MRP_Feedback(&(data->MRP_FeedbackRWA), 3);
	SelfInit_PRV_Steering(&(data->PRV_SteeringRWA), 8);
}
void allAlg_CrossInit(EMMConfigData *data)
{
	CrossInit_imuProcessTelem(&(data->imuSensorDecode), 2);
	CrossInit_cssProcessTelem(&(data->cssSensorDecode), 0);
	CrossInit_cssWlsEst(&(data->CSSWlsEst), 1);
	CrossInit_sunSafePoint(&(data->sunSafePoint), 28);
	CrossInit_simpleDeadband(&(data->simpleDeadband), 25);
	CrossInit_MRP_PD(&(data->MRP_PD), 4);
	CrossInit_sunSafeACS(&(data->sunSafeACS), 27);
	CrossInit_celestialBodyPoint(&(data->sunPoint), 26);
	CrossInit_celestialBodyPoint(&(data->earthPoint), 17);
	CrossInit_celestialBodyPoint(&(data->marsPoint), 23);
	CrossInit_attRefGen(&(data->attMnvrPoint), 12);
	CrossInit_MRP_Steering(&(data->MRP_SteeringRWA), 6);
	CrossInit_dvAttEffect(&(data->RWAMappingData), 9);
	CrossInit_rwNullSpace(&(data->RWNullSpace), 10);
	CrossInit_dvGuidance(&(data->dvGuidance), 16);
	CrossInit_MRP_Steering(&(data->MRP_SteeringMOI), 5);
	CrossInit_dvAttEffect(&(data->dvAttEffect), 15);
	CrossInit_thrustRWDesat(&(data->thrustRWDesat), 29);
	CrossInit_stProcessTelem(&(data->stSensorDecode), 11);
	CrossInit_inertial3D(&(data->inertial3D), 22);
	CrossInit_hillPoint(&(data->hillPoint), 20);
	CrossInit_velocityPoint(&(data->velocityPoint), 30);
	CrossInit_celestialTwoBodyPoint(&(data->celTwoBodyPoint), 14);
	CrossInit_rasterManager(&(data->rasterManager), 24);
	CrossInit_eulerRotation(&(data->eulerRotation), 19);
	CrossInit_inertial3DSpin(&(data->inertial3DSpin), 21);
	CrossInit_attTrackingError(&(data->attTrackingError), 13);
	CrossInit_errorDeadband(&(data->errorDeadband), 18);
	CrossInit_MRP_Feedback(&(data->MRP_FeedbackRWA), 3);
	CrossInit_PRV_Steering(&(data->PRV_SteeringRWA), 8);
}
