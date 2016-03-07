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
#include <math.h>
#include <string.h>

STInertialUKF::STInertialUKF() 
{
   stInputName = "star_tracker_data";
   wheelSpeedsName = "RWASpeeds";
   InertialUKFStateName = "InertialStateEst";
   return;
}

STInertialUKF::~STInertialUKF()
{
   return;
}


void STInertialUKF::StateProp(MatrixOperations &StateCurr)
{
    //Assume zero torque acting on body
    double BMatrix[3][3];
	double qDot[3];
	BmatMRP(StateCurr.vec_vals, BMatrix);
	m33MultV3(BMatrix, &(StateCurr.vec_vals[3]), qDot);
	v3Scale(0.25*dt, qDot, qDot);
	v3Add(StateCurr.vec_vals, qDot, StateCurr.vec_vals);
    return;
}

void STInertialUKF::ComputeMeasModel()
{
	   int i;
	   uint32_t index;
	   MatrixOperations StateComp, ExpVec;
	   for(i=0; i<CountHalfSPs*2+1; i++)
	   {
		   YMeas[i].MatOps_init(3, 1);
		   YMeas[i].MatOps_VecSet(SP[i].vec_vals);
	   }
	   obs.MatOps_VecSet(stMeas.MRP_BdyInrtl);
	   NumObs = 3;
	   return;
}

//MatrixOperations STInertialUKF::ComputeSPVector(
//	MatrixOperations SPError,
//	MatrixOperations StateIn)
//{
//	MatrixOperations StateSum, CompQuat, localMRP;
//	MatrixOperations ErrorQuat(3, 1, SPError.vec_vals);
//	MatrixOperations StateQuat(3, 1, StateIn.vec_vals);
//	ErrorQuat.MatOps_MRP2Quat(ErrorQuat);
//	StateQuat.MatOps_MRP2Quat(StateQuat);
//	//ErrorQuat.MatOps_QuatTrans(ErrorQuat);
//	int SignState = StateQuat.vec_vals[0] < 0.0 ? -1 : 1;
//	int SignErr = ErrorQuat.vec_vals[0] < 0.0 ? -1 : 1;
//	if (SignState != SignErr)
//	{
//		ErrorQuat.MatOps_scale(-1.0);
//	}
//	CompQuat.MatOps_QuatMult(ErrorQuat, StateQuat);
//	localMRP.MatOps_Quat2MRP(CompQuat);
//	StateSum.MatOps_init(6, 1);
//	memcpy(StateSum.vec_vals, localMRP.vec_vals, 3 * sizeof(double));
//	v3Add(&(StateIn.vec_vals[3]), &(SPError.vec_vals[3]), &(StateSum.vec_vals[3]));
//	return(StateSum);
//}

void STInertialUKF::CheckForUpdate(
   double TimeLatest)
{
   if(TimeLatest > TimeTag)
   {
      TimeUpdateFilter(TimeLatest);
   }
   if(state.MatOps_twonorm_square() > 1.5) //Little extra margin 
   {
      state.MatOps_ShadowMRP(state);
   }
   return;
}

//MatrixOperations STInertialUKF::ComputeObsError(
//	MatrixOperations SPError,
//	MatrixOperations StateIn)
//{
//	//   MatrixOperations StateSum;
//	//   StateSum.MatOps_add(SPError, StateIn);
//	MatrixOperations StateSum, CompQuat;
//	MatrixOperations ErrorQuat(3, 1, SPError.vec_vals);
//	MatrixOperations StateQuat(3, 1, StateIn.vec_vals);
//
//	ErrorQuat.MatOps_MRP2Quat(ErrorQuat);
//	StateQuat.MatOps_MRP2Quat(StateQuat);
//	//ErrorQuat.MatOps_QuatTrans(ErrorQuat);
//	CompQuat.MatOps_QuatMult(ErrorQuat, StateQuat);
//	StateSum.MatOps_Quat2MRP(CompQuat);
//	if (StateSum.MatOps_twonorm_square() > 1.0)
//	{
//		StateSum.MatOps_ShadowMRP(StateSum);
//	}
//	return(StateSum);
//}

void STInertialUKF::UpdateState(uint64_t callTime)
{
   bool MeasAvail, BVecNewest, Valid, Unread;
   double TimeTagArray[2];
   double **UpdTimeArray;
   double CovarNorm;
   uint32_t i;
   uint64_t ClockTime;
   uint32_t ReadSize;
   double EPSum[4];
   double mrpSum[3];
   double quatTranspose[4];
   double quatMeas[4];
   MatrixOperations inputMatrix;

   ReadMessage(stInputID, &ClockTime, &ReadSize, sizeof(STOutputData), &stMeas);

   if (initToMeas && stMeas.timeTag != 0.0)
   {
	   memcpy(state.vec_vals, stMeas.MRP_BdyInrtl, 3*sizeof(double));
	   memset(&(state.vec_vals[3]), 0x0, 3 * sizeof(double));
	   TimeTag = stMeas.timeTag;
	   initToMeas = false;
	   return;
   }
   MatrixOperations localMRP;
   localMRP.MatOps_init(3, 1);
   localMRP.MatOps_VecSet(state.vec_vals);

   if (localMRP.MatOps_twonorm_square() > 1.5) //Little extra margin
   {
	   localMRP.MatOps_ShadowMRP(localMRP);
	   memcpy(state.vec_vals, localMRP.vec_vals, 3*sizeof(double));
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
   MeasurementUpdate();
   memcpy(CovarEst, Covar.vec_vals, NumStates*NumStates*sizeof(double));
   memcpy(localOutput.MRP_BdyInrtl, state.vec_vals, 3 * sizeof(double));
   localOutput.TimeTag = TimeTag;
   memcpy(localOutput.w_BdyInrtl_Bdy, &(state.vec_vals[3]), 3 * sizeof(double));
   WriteMessage(InertialUKFStateID, callTime, sizeof(AttOutputStruct), &localOutput,
	   moduleID);
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
      CountHalfSPs = 6;
      NumObs = 3;
	  obs.MatOps_init(NumObs, 1);
      Q_obs.MatOps_init(3,3);
      Q_obs.MatOps_VecSet(&QStObs[0]);
      state.MatOps_init(6,1);
	  memcpy(state.vec_vals, MRP_BdyInrtl_Init, 3 * sizeof(double));
	  memcpy(&(state.vec_vals[3]), w_BdyInrtl_Bdy, 3 * sizeof(double));
      Qnoise.MatOps_init(6,6);
      Qnoise.MatOps_VecSet(&QNoiseInit[0]);
      Covar.MatOps_init(6,6);
      Covar.MatOps_VecSet(&CovarInit[0]);
      lambdaVal = alpha*alpha*(NumStates+kappa)-NumStates;
      gamma = sqrt(NumStates+lambdaVal);
	  YMeas = new MatrixOperations[CountHalfSPs * 2 + 1];
	  for (int i = 0; i < CountHalfSPs * 2 + 1; i++)
	  {
		  YMeas[i].MatOps_init(3, 1);
	  }
      UKFInit();
      FilterConverged = false;
      ConvCounter = 0;
      ReInitFilter = false;
   }
   InertialUKFStateID = CreateNewMessage((char*) (InertialUKFStateName.c_str()), sizeof(AttOutputStruct), 
	   "AttOutputStruct", moduleID);
   return;
}
void STInertialUKF::CrossInit()
{
	stInputID = subscribeToMessage((char*)stInputName.c_str(), sizeof(STOutputData), moduleID);
    return;
}

