#include "sensors/imu/imu_sensor.h"
#include "architecture/messaging/system_messaging.h"
#include "utilities/rigidBodyKinematics.h"
#include "utilities/linearAlgebra.h"
#include <math.h>
#include <iostream>
#include <cstring>
#include <random>

ImuSensor::ImuSensor()
{
    CallCounts = 0;
    InputStateID = -1;
    InputStateMsg = "inertial_state_output";
    OutputDataMsg = "imu_meas_data";
    InputMassMsg = "spacecraft_mass_props";
    setStructureToPlatformDCM(0.0, 0.0, 0.0);
    this->OutputBufferCount = 2;
    memset(&StatePrevious, 0x0, sizeof(OutputStateData));
    memset(&StateCurrent, 0x0, sizeof(OutputStateData));
    PreviousTime = 0;
    NominalReady = false;
    memset(&senRotBias[0], 0x0, 3*sizeof(double));
    memset(&senRotNoiseStd[0], 0x0, 3*sizeof(double));
    memset(&senTransBias[0], 0x0, 3*sizeof(double));
    memset(&senTransNoiseStd[0], 0x0, 3*sizeof(double));
    
    return;
}

void ImuSensor::setStructureToPlatformDCM(double yaw, double pitch, double roll)
{
    double q[3] = {yaw, pitch, roll};
    Euler3212C(q, this->Str2Platform);
}

ImuSensor::~ImuSensor()
{
    return;
}

void ImuSensor::SelfInit()
{
    /// - Initialize randon number generators.  Note the cheat on seeds
    for(uint32_t i=0; i<3; i++)
    {
        std::normal_distribution<double>::param_type
        UpdateRotPair(senRotBias[i], senRotNoiseStd[i]);
        rot_rgen[i].seed(RNGSeed+i);
        rot_rnum[i].param(UpdateRotPair);
        std::normal_distribution<double>::param_type
        UpdateTransPair(senTransBias[i], senTransNoiseStd[i]);
        trans_rgen[i].seed(RNGSeed+i);
        trans_rnum[i].param(UpdateTransPair);
    }
    OutputDataID = SystemMessaging::GetInstance()->
        CreateNewMessage( OutputDataMsg, sizeof(ImuSensorOutput),
        OutputBufferCount, "ImuSensorOutput");
}

void ImuSensor::CrossInit()
{
    InputStateID = SystemMessaging::GetInstance()->FindMessageID(
                                                                 InputStateMsg);
    InputMassID = SystemMessaging::GetInstance()->FindMessageID(InputMassMsg);
    if(InputStateID < 0 || InputMassID < 0)
    {
        std::cerr << "WARNING: Failed to link an imu input message: ";
        std::cerr << std::endl << "State: "<<InputStateID;
        std::cerr << std::endl << "Mass: "<<InputMassID;
    }
    return;
}

void ImuSensor::ReadInputs()
{
    SingleMessageHeader LocalHeader;
    
    memset(&StateCurrent, 0x0, sizeof(OutputStateData));
    if(InputStateID >= 0)
    {
        SystemMessaging::GetInstance()->ReadMessage(InputStateID, &LocalHeader,
                                                    sizeof(OutputStateData), reinterpret_cast<uint8_t*> (&StateCurrent));
    }
    memset(&MassCurrent, 0x0, sizeof(MassPropsData));
    if(InputMassID >= 0)
    {
        SystemMessaging::GetInstance()->ReadMessage(InputMassID, &LocalHeader,
                                                    sizeof(MassPropsData), reinterpret_cast<uint8_t*> (&MassCurrent));
    }
}

void ImuSensor::WriteOutputs(uint64_t Clock)
{
    ImuSensorOutput LocalOutput;
    memcpy(LocalOutput.DVFramePlatform, DVFramePlatform, 3*sizeof(double));
    memcpy(LocalOutput.AccelPlatform, AccelPlatform, 3*sizeof(double));
    memcpy(LocalOutput.DRFramePlatform, DRFramePlatform, 3*sizeof(double));
    memcpy(LocalOutput.AngVelPlatform, AngVelPlatform, 3*sizeof(double));
    SystemMessaging::GetInstance()->WriteMessage(OutputDataID, Clock,
                                                 sizeof(ImuSensorOutput), reinterpret_cast<uint8_t*> (&LocalOutput));
}

void ImuSensor::ApplySensorErrors(uint64_t CurrentTime)
{
    double OmegaErrors[3];
    double AccelErrors[3];
    double dt;
    
    dt = (CurrentTime - PreviousTime)*1.0E-9;
    
    for(uint32_t i=0; i<3; i++)
    {
        OmegaErrors[i] = rot_rnum[i](rot_rgen[i]);
        AngVelPlatform[i] += OmegaErrors[i];
        DRFramePlatform[i] += OmegaErrors[i]*dt;
        AccelErrors[i] = trans_rnum[i](trans_rgen[i]);
        AccelPlatform[i] += AccelErrors[i];
        DVFramePlatform[i] += AccelErrors[i]*dt;
    }
    
}

void ImuSensor::ComputePlatformDR()
{
    
    double MRP_Bdy2Inrtl_Prev[3];
    double MRP_BdyPrev2BdyNow[3];
    double DRBodyFrame[3];
    double T_Bdy2Platform[3][3];
    
    m33MultM33t(Str2Platform, StateCurrent.T_str2Bdy, T_Bdy2Platform);
    v3Scale(-1.0, StatePrevious.sigma, MRP_Bdy2Inrtl_Prev);
    if(StateCurrent.MRPSwitchCount != StatePrevious.MRPSwitchCount)
    {
        for(uint32_t i=0; i<(StateCurrent.MRPSwitchCount -
                             StatePrevious.MRPSwitchCount); i++)
        {
            double Smag = v3Norm(MRP_Bdy2Inrtl_Prev);
            v3Scale(-1.0/Smag/Smag, MRP_Bdy2Inrtl_Prev, MRP_Bdy2Inrtl_Prev);
        }
    }
    addMRP(MRP_Bdy2Inrtl_Prev, StateCurrent.sigma, MRP_BdyPrev2BdyNow);
    MRP2PRV(MRP_BdyPrev2BdyNow, DRBodyFrame);
    m33MultV3(T_Bdy2Platform, DRBodyFrame, DRFramePlatform);
    m33MultV3(T_Bdy2Platform, StateCurrent.omega, AngVelPlatform);
}

void ImuSensor::ComputePlatformDV(uint64_t CurrentTime)
{
    
    double CmRelPos[3];
    double AlphaBodyRough[3];  /// -- Approximation but it shouldn't be too bad
    double omeg_x_omeg_x_r[3];
    double alpha_x_r[3];
    double RotForces[3];
    double InertialAccel[3];
    double dt;
    double T_Bdy2Platform[3][3];
    m33MultM33t(Str2Platform, StateCurrent.T_str2Bdy, T_Bdy2Platform);
    v3Subtract(SensorPosStr.data(), MassCurrent.CoM, CmRelPos);
    m33MultV3(StateCurrent.T_str2Bdy, CmRelPos, CmRelPos);
    dt = (CurrentTime - PreviousTime)*1.0E-9;
    v3Subtract(StateCurrent.omega, StatePrevious.omega, AlphaBodyRough);
    v3Scale(1.0/dt, AlphaBodyRough, AlphaBodyRough);
    v3Cross(AlphaBodyRough, CmRelPos, alpha_x_r);
    v3Cross(StateCurrent.omega, CmRelPos, omeg_x_omeg_x_r);
    v3Cross(StateCurrent.omega, omeg_x_omeg_x_r, omeg_x_omeg_x_r);
    v3Add(omeg_x_omeg_x_r, alpha_x_r, RotForces);
    v3Subtract(StateCurrent.TotalAccumDVBdy, StatePrevious.TotalAccumDVBdy,
               InertialAccel);
    v3Copy(InertialAccel, DVFramePlatform);
    v3Scale(1.0/dt, InertialAccel, InertialAccel);
    v3Subtract(InertialAccel, RotForces, InertialAccel);
    m33MultV3(T_Bdy2Platform, InertialAccel, AccelPlatform);
    v3Scale(dt, RotForces, RotForces);
    v3Subtract(DVFramePlatform, RotForces, DVFramePlatform);
    m33MultV3(T_Bdy2Platform, DVFramePlatform, DVFramePlatform);
    
}

void ImuSensor::UpdateState(uint64_t CurrentSimNanos)
{
    ReadInputs();
    if(NominalReady)
    {
        ComputePlatformDR();
        ComputePlatformDV(CurrentSimNanos);
        ApplySensorErrors(CurrentSimNanos);
        WriteOutputs(CurrentSimNanos);
    }
    memcpy(&StatePrevious, &StateCurrent, sizeof(OutputStateData));
    PreviousTime = CurrentSimNanos;
    NominalReady = true;
}
