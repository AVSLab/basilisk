/*
 Copyright (c) 2016, Autonomous Vehicle Systems Lab, Univeristy of Colorado at Boulder
 
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

#include "attDetermination/InertialUKF/attitude_ukf.h"
#include "messaging/static_messaging.h"
#include "SimCode/utilities/rigidBodyKinematics.h"
#include "SimCode/utilities/linearAlgebra.h"
#include "vehicleConfigData/vehicleConfigData.h"
#include "ADCSUtilities/ADCSAlgorithmMacros.h"
#include <math.h>
#include <string.h>

STInertialUKF::STInertialUKF()
{
    stInputName = "star_tracker_data";
    wheelSpeedsName = "RWASpeeds";
    InertialUKFStateName = "InertialStateEst";
    memset(CovarEst, 0x0, 6*6*sizeof(double));
    return;
}

STInertialUKF::~STInertialUKF()
{
    return;
}


void STInertialUKF::StateProp(MatrixOperations &StateCurr)
{
    //Assume zero torque acting on body
    double qDot[3];
    double torqueTotal[3];
    double torqueSingle[3];
    double angAccelTotal[3];
    double omegVecLocal[3];
    double BMatInv[3][3];
    double wheelAccel;
    v3Copy(&(StateCurr.vec_vals[3]), qDot);
    v3Scale(dt, qDot, qDot);
    v3Add(StateCurr.vec_vals, qDot, StateCurr.vec_vals);
    v3SetZero(torqueTotal);
    std::vector<RWConfigElement>::iterator it;
    for(it=rwData.begin(); it!=rwData.end(); it++)
    {
        wheelAccel = currentSpeeds.wheelSpeeds[it-rwData.begin()] -
           previousSpeeds.wheelSpeeds[it-rwData.begin()];
        wheelAccel /= dt/it->Js;
        v3Scale(wheelAccel, it->gsAxBdy, torqueSingle);
        v3Subtract(torqueTotal, torqueSingle, torqueTotal);
    }
    m33MultV3(RECAST3X3 IInv, torqueTotal, angAccelTotal);
    BinvMRP(StateCurr.vec_vals, BMatInv);
    m33MultV3(BMatInv, &(StateCurr.vec_vals[3]), omegVecLocal);
    v3Scale(dt, angAccelTotal, angAccelTotal);
    v3Add(omegVecLocal, angAccelTotal, omegVecLocal);
    BmatMRP(StateCurr.vec_vals, BMatInv);
    m33MultV3(BMatInv, omegVecLocal, &(StateCurr.vec_vals[3]));
    return;
}

void STInertialUKF::ComputeMeasModel()
{
	   int i;
	   MatrixOperations StateComp, ExpVec;
	   for(i=0; i<CountHalfSPs*2+1; i++)
       {
           YMeas[i].MatOps_init(NumObs, 1);
           YMeas[i].MatOps_VecSet(SP[i].vec_vals);
       }
	   //obs.MatOps_VecSet(stMeas.MRP_BdyInrtl);
    memcpy(obs.vec_vals, stMeas.MRP_BdyInrtl, 3*sizeof(double));
    if(NumObs > 3)
    {
        v3Subtract(stMeas.MRP_BdyInrtl, MRPPrevious, &(obs.vec_vals[3]));
        v3Scale(1.0/dt, &(obs.vec_vals[3]), &(obs.vec_vals[3]));
    }
	   
	   return;
}

MatrixOperations STInertialUKF::ComputeSPVector(
                                                MatrixOperations SPError,
                                                MatrixOperations StateIn)
{
    MatrixOperations StateSum, CompQuat, localMRP;
    MatrixOperations ErrorQuat(3, 1, SPError.vec_vals);
    MatrixOperations StateQuat(3, 1, StateIn.vec_vals);
    ErrorQuat.MatOps_MRP2Quat(ErrorQuat);
    StateQuat.MatOps_MRP2Quat(StateQuat);

    CompQuat.MatOps_QuatMult(ErrorQuat, StateQuat);
    localMRP.MatOps_Quat2MRP(CompQuat);
    StateSum.MatOps_init(NumStates, 1);
    memcpy(StateSum.vec_vals, localMRP.vec_vals, 3 * sizeof(double));
    v3Add(&(StateIn.vec_vals[3]), &(SPError.vec_vals[3]), &(StateSum.vec_vals[3]));
    return(StateSum);
}

void STInertialUKF::CheckForUpdate(
                                   double TimeLatest)
{
    if(TimeLatest > TimeTag)
    {
        TimeUpdateFilter(TimeLatest);
    }
    if(state.MatOps_twonorm_square() > 1.2) //Little extra margin
    {
        state.MatOps_ShadowMRP(state);
    }
    return;
}

MatrixOperations STInertialUKF::ComputeObsError(
                                                MatrixOperations SPError,
                                                MatrixOperations StateIn)
{
    //   MatrixOperations StateSum;
    //   StateSum.MatOps_add(SPError, StateIn);
    MatrixOperations StateSum, CompQuat, CompMRP;
    MatrixOperations ErrorQuat(3, 1, SPError.vec_vals);
    MatrixOperations StateQuat(3, 1, StateIn.vec_vals);
    
    ErrorQuat.MatOps_MRP2Quat(ErrorQuat);
    StateQuat.MatOps_MRP2Quat(StateQuat);
    //ErrorQuat.MatOps_QuatTrans(ErrorQuat);
    CompQuat.MatOps_QuatMult(ErrorQuat, StateQuat);
    CompMRP.MatOps_Quat2MRP(CompQuat);
    if (CompMRP.MatOps_twonorm_square() > 1.0)
    {
        CompMRP.MatOps_ShadowMRP(CompMRP);
    }
    StateSum.MatOps_init(NumStates, 1);
    memcpy(StateSum.vec_vals, CompMRP.vec_vals, 3*sizeof(double));
    if(NumStates > CompMRP.dim_array[0])
    {
        v3Add(&(SPError.vec_vals[3]), &(StateIn.vec_vals[3]),
              &(StateSum.vec_vals[3]));
    }
    return(StateSum);
}

void STInertialUKF::UpdateState(uint64_t callTime)
{
    uint64_t ClockTime;
    uint32_t ReadSize;
    double EPSum[4];
    double mrpSum[3];
    double quatTranspose[4];
    double quatMeas[4];
    double BMatInv[3][3];
    MatrixOperations inputMatrix;
    vehicleConfigData localConfig;
    
    ReadMessage(stInputID, &ClockTime, &ReadSize, sizeof(STOutputData), &stMeas, moduleID);
    ReadMessage(inputSpeedsID, &ClockTime, &ReadSize, sizeof(RWSpeedData), &currentSpeeds, moduleID);
    ReadMessage(inputVehicleConfigDataID, &ClockTime, &ReadSize,
        sizeof(vehicleConfigData), &localConfig, moduleID);
    m33Inverse(RECAST3X3 localConfig.ISCPntB_B, RECAST3X3 IInv);
    
    if (initToMeas && stMeas.timeTag != 0.0)
    {
        memcpy(state.vec_vals, stMeas.MRP_BdyInrtl, 3*sizeof(double));
        memcpy(MRPPrevious, stMeas.MRP_BdyInrtl, 3*sizeof(double));
        memset(&(state.vec_vals[3]), 0x0, 3 * sizeof(double));
        TimeTag = stMeas.timeTag;
        initToMeas = false;
        memcpy(&previousSpeeds, &currentSpeeds, sizeof(RWSpeedData));
        return;
    }
    MatrixOperations localMRP;
    localMRP.MatOps_init(3, 1);
    localMRP.MatOps_VecSet(state.vec_vals);
    
    if (localMRP.MatOps_twonorm_square() > 1.2) //Little extra margin
    {
        localMRP.MatOps_ShadowMRP(localMRP);
        memcpy(state.vec_vals, localMRP.vec_vals, 3*sizeof(double));
        double BMatrix[3][3];
        double localMRPDer[3];
        BmatMRP(state.vec_vals, BMatrix);
        m33MultV3(BMatrix, localOutput.omega_BN_B, localMRPDer);
        v3Scale(0.25, localMRPDer, &(state.vec_vals[3]));
    }
    this->TimeUpdateFilter(stMeas.timeTag);
    MRP2EP(state.vec_vals, quatTranspose);
    v3Scale(-1.0, &(quatTranspose[1]), &(quatTranspose[1]));
    MRP2EP(stMeas.MRP_BdyInrtl, quatMeas);
    addEP(quatTranspose, quatMeas, EPSum);
    EP2MRP(EPSum, mrpSum);
    if (v3Norm(mrpSum) > 1.0)
    {
        inputMatrix.MatOps_init(3, 1);
        inputMatrix.MatOps_VecSet(stMeas.MRP_BdyInrtl);
        inputMatrix.MatOps_ShadowMRP(inputMatrix);
        memcpy(stMeas.MRP_BdyInrtl, inputMatrix.vec_vals, 3 * (sizeof(double)));
    }
    MRP2EP(MRPPrevious, quatMeas);
    addEP(quatTranspose, quatMeas, EPSum);
    EP2MRP(EPSum, mrpSum);
    if (v3Norm(mrpSum) > 1.0)
    {
        inputMatrix.MatOps_init(3, 1);
        inputMatrix.MatOps_VecSet(MRPPrevious);
        inputMatrix.MatOps_ShadowMRP(inputMatrix);
        memcpy(MRPPrevious, inputMatrix.vec_vals, 3*sizeof(double));
    }
    
    MeasurementUpdate();
    memcpy(CovarEst, Covar.vec_vals, NumStates*NumStates*sizeof(double));
    memcpy(localOutput.sigma_BN, state.vec_vals, 3 * sizeof(double));
    localOutput.timeTag = TimeTag;
    BinvMRP(localOutput.sigma_BN, BMatInv);
    m33MultV3(BMatInv, &(state.vec_vals[3]), localOutput.omega_BN_B);
    v3Scale(4.0, localOutput.omega_BN_B, localOutput.omega_BN_B);
    WriteMessage(InertialUKFStateID, callTime, sizeof(NavStateOut), &localOutput,
                 moduleID);
    memcpy(MRPPrevious, stMeas.MRP_BdyInrtl, 3*sizeof(double));
    memcpy(&previousSpeeds, &currentSpeeds, sizeof(RWSpeedData));
    return;
}

void STInertialUKF::DetermineConvergence(
                                         double CovarNorm)
{
    if(CovarNorm < ConvThresh)
    {
        ConvCounter++;
        if(ConvCounter >= ConvTestCounter)
        {
            FilterConverged = true;
            ConvCounter = ConvTestCounter;
        }
    }
    else
    {
        FilterConverged = false;
        ConvCounter = 0;
    }
    return;
}

void STInertialUKF::SelfInit()
{
    if(ReInitFilter)
    {
        NumStates = 6;
        CountHalfSPs = NumStates;
        NumObs = 6;
        obs.MatOps_init(NumObs, 1);
        Q_obs.MatOps_init(NumObs,NumObs);
        Q_obs.MatOps_VecSet(&QStObs[0]);
        state.MatOps_init(NumStates,1);
        memcpy(state.vec_vals, MRP_BdyInrtl_Init, 3 * sizeof(double));
        memcpy(&(state.vec_vals[3]), w_BdyInrtl_Bdy, 3 * sizeof(double));
        Qnoise.MatOps_init(NumStates,NumStates);
        Qnoise.MatOps_VecSet(&QNoiseInit[0]);
        Covar.MatOps_init(NumStates,NumStates);
        Covar.MatOps_VecSet(&CovarInit[0]);
        lambdaVal = alpha*alpha*(NumStates+kappa)-NumStates;
        gamma = sqrt(NumStates+lambdaVal);
        YMeas = new MatrixOperations[CountHalfSPs * 2 + 1];
        for (int i = 0; i < CountHalfSPs * 2 + 1; i++)
        {
            YMeas[i].MatOps_init(NumObs, 1);
        }
        UKFInit();
        FilterConverged = false;
        ConvCounter = 0;
        ReInitFilter = false;
    }
    InertialUKFStateID = CreateNewMessage((char*) (InertialUKFStateName.c_str()), sizeof(NavStateOut),
                                          "NavStateOut", moduleID);
    return;
}
void STInertialUKF::CrossInit()
{
    stInputID = subscribeToMessage((char*)stInputName.c_str(), sizeof(STOutputData), moduleID);
    inputSpeedsID = subscribeToMessage((char *) inputRWSpeeds.c_str(),
                                                   sizeof(RWSpeedData), moduleID);
    inputVehicleConfigDataID = subscribeToMessage((char*)inputVehicleConfigDataName.c_str(),
                                                              sizeof(vehicleConfigData), moduleID);
    return;
}

