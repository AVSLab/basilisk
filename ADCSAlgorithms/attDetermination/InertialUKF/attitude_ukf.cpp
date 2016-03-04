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
#include "sensorInterfaces/STSensorData/stComm.h"
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
   return;
}

void STInertialUKF::ComputeMeasModel()
{
//	   int i;
//	   uint32_t index;
//	   MatrixOperations StateComp, ExpVec;
//	   YMeas = new MatrixOperations[CountHalfSPs*2+1];
//	   index = CurrUpd == ACC_UPDATE ? 6 : 0;
//	   for(i=0; i<CountHalfSPs*2+1; i++)
//	   {
//	     StateComp.MatOps_MRP2Quat(SP[i]);
//	     StateComp.MatOps_Quat2DCM(StateComp);
//	     YMeas[i].MatOps_init(3, 1);
//	     if(CurrUpd == ACC_UPDATE)
//	     {
//	    	StateComp.MatOps_transpose(StateComp);
//	        YMeas[i].MatOps_VecSet(&StateComp.vec_vals[index]);
//	     }
//	     else
//	     {
//
//	        ExpVec.MatOps_init(3,1);
//	        ExpVec.MatOps_VecSet(BVecExpNED);
//	        //ExpVec.vec_vals[2] = 0.0;
//	        ExpVec.MatOps_mult(StateComp, ExpVec);
//	        ExpVec.MatOps_scale(1.0/ExpVec.MatOps_twonorm());
//	        YMeas[i] = ExpVec;
//	     }
//	   }
//	   obs.MatOps_init(3,1);
//	   if(CurrUpd == ACC_UPDATE)
//	   {
//		   StateComp.MatOps_MRP2Quat(SP[0]);
//		   	     StateComp.MatOps_Quat2DCM(StateComp);
//	      obs.MatOps_VecSet(AccVecObs);
//	      obs.MatOps_scale(-1.0/obs.MatOps_twonorm());
//	      StateComp.MatOps_transpose(StateComp);
//	     // __android_log_print(ANDROID_LOG_INFO, "Native", "NED Down = %f %f %f",
//	     // 	        	    			 StateComp.vec_vals[6], StateComp.vec_vals[7], StateComp.vec_vals[8]);
//	     // __android_log_print(ANDROID_LOG_INFO, "Native", "NED Accel = %f %f %f",
//	     // 	        	    			 obs.vec_vals[0], obs.vec_vals[1], obs.vec_vals[2]);
//	   }
//	   else
//	   {
//		   StateComp.MatOps_MRP2Quat(SP[0]);
//		   StateComp.MatOps_Quat2DCM(StateComp);
//		   StateComp.MatOps_transpose(StateComp);
//		   ExpVec.MatOps_init(3,1);
//		   ExpVec.MatOps_VecSet(&(StateComp.vec_vals[6]));
//
//	      obs.MatOps_VecSet(BVecObs);
//	      obs.MatOps_scale(1.0/obs.MatOps_twonorm());
//	      ExpVec.MatOps_scale(-obs.MatOps_dotprod(ExpVec));
//	      obs.MatOps_add(obs, ExpVec);
//
//	      obs.MatOps_VecSet(BVecObs);
//	      obs.MatOps_scale(1.0/obs.MatOps_twonorm());
//
//	      StateComp.MatOps_transpose(StateComp);
//	      ExpVec.MatOps_VecSet(BVecExpNED);
//	      //ExpVec.vec_vals[2] = 0.0;
//	      ExpVec.MatOps_mult(StateComp, ExpVec);
//	      		   ExpVec.MatOps_scale(1.0/ExpVec.MatOps_twonorm());
////	      __android_log_print(ANDROID_LOG_INFO, "Native", "NED BVEC = %f %f %f",
////	      	        	    			 obs.vec_vals[0], obs.vec_vals[1], obs.vec_vals[2]);
////	      __android_log_print(ANDROID_LOG_INFO, "Native", "NED North = %f %f %f",
////	     	      	        	    			 ExpVec.vec_vals[0], ExpVec.vec_vals[1], ExpVec.vec_vals[2]);
//	   }
	   return;
}



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

void STInertialUKF::UpdateState(uint64_t callTime)
{
//   bool MeasAvail, BVecNewest, Valid, Unread;
//   double TimeTagArray[2];
//   double **UpdTimeArray;
//   double CovarNorm;
//   uint32_t i;
//   ImuMeasEntry LastBVec;
//   MatrixOperations QCurrent, CovarMat, AccCurrent, MagCurrent, MatProd;
//   NEDUpdateSelector UpdList[2] = {NO_UPDATE, NO_UPDATE};
//   DerivOutputStruct *FiltAccelOutput;
//   SamplingPortSingleton::SamplingMessageReturn BlankPort, BlankPort2;
//   RingBuffer <ImuMeasEntry> BVecBuffer;
//   NEDOutputStruct NEDOut;
//
//   SamplingPortSingleton::GetInstance()->ReadSamplingMessage(
//      AccInputPortID, BlankPort, Valid, Unread);
//   if(!Valid)
//   {
//	   return;
//   }
//   FiltAccelOutput = reinterpret_cast<DerivOutputStruct *>
//      (BlankPort.SamplingData);
//   SamplingPortSingleton::GetInstance()->ReadSamplingMessage(
//         BVecInputPortID, BlankPort2, Valid, Unread);
//   if(!Valid)
//   {
//      return;
//   }
//   BVecBuffer.RingBufferUnPacketize(BlankPort2.SamplingData);
//   if(!BVecBuffer.BufferFull)
//   {
//	   return;
//   }
//
//   LastBVec = BVecBuffer.GetNewest();
//
//   AccCurrent.MatOps_init(3,1);
//   AccCurrent.MatOps_VecSet(FiltAccelOutput->CurrentDerivVec);
//   MagCurrent.MatOps_init(3,1);
//   MagCurrent.MatOps_VecSet(LastBVec.AccumulatedMeas);
//   if((LastBVec.TimeTag <= LastBVecTime || FiltAccelOutput->TimeTag<=LastAccTime) ||
//      (AccCurrent.MatOps_twonorm() < AccTolerance ||
//      MagCurrent.MatOps_twonorm() < MagTolerance))
//   {
//      return;
//   }
//   QCurrent.MatOps_init(3,3);
//   AccCurrent.MatOps_scale(-1.0/AccCurrent.MatOps_twonorm());
//   memcpy(&(QCurrent.vec_vals[6]), AccCurrent.vec_vals, 3*sizeof(double));
//   MagCurrent.MatOps_scale(1.0/MagCurrent.MatOps_twonorm());
//   MagCurrent.MatOps_crossprod(AccCurrent, MagCurrent);
//   MagCurrent.MatOps_scale(1.0/MagCurrent.MatOps_twonorm());
//   memcpy(&(QCurrent.vec_vals[3]), MagCurrent.vec_vals, 3*sizeof(double));
//   MagCurrent.MatOps_crossprod(MagCurrent, AccCurrent);
//   MagCurrent.MatOps_scale(1.0/MagCurrent.MatOps_twonorm());
//   memcpy(&(QCurrent.vec_vals[0]), MagCurrent.vec_vals, 3*sizeof(double));
//   QCurrent.MatOps_transpose(QCurrent);
//   QCurrent.MatOps_DCM2Quat(QCurrent);
//   TimeTag = LastBVec.TimeTag < FiltAccelOutput->TimeTag ?
//		   LastBVec.TimeTag : FiltAccelOutput->TimeTag;

//   BVecNewest = LastBVec.TimeTag > FiltAccelOutput->TimeTag ? true : false;
//
//   TimeTagArray[1] = BVecNewest ? LastBVec.TimeTag : FiltAccelOutput->TimeTag;
//   TimeTagArray[0] = !BVecNewest ? LastBVec.TimeTag : FiltAccelOutput->TimeTag;
//   //CheckForUpdate(TimeTagArray[1]);
//
//   UpdList[0] = BVecNewest ? ACC_UPDATE : BVEC_UPDATE;
//   UpdList[1] = BVecNewest ? BVEC_UPDATE : ACC_UPDATE;
//
//   UpdTimeArray = new double*[2];
//   UpdTimeArray[1] = BVecNewest ? &LastBVecTime : &LastAccTime;
//   UpdTimeArray[0] = !BVecNewest ? &LastBVecTime : &LastAccTime;
//   memcpy(BVecObs, LastBVec.AccumulatedMeas, 3*sizeof(double));
//   memcpy(AccVecObs, AccCurrent.vec_vals, 3*sizeof(double));
//   for(i=0; i<2; i++)
//   {
////	  if(UpdList[i] == BVEC_UPDATE)
////	  {
////		  continue;
////	  }
//      CurrUpd = UpdList[i];
//      if(TimeTagArray[i] > *(UpdTimeArray[i]))
//      {
//    	 TimeUpdateFilter(TimeTagArray[i]);
//         MeasurementUpdate();
//         *UpdTimeArray[i] = TimeTagArray[i];
//
//         delete [] YMeas;
//      }
//   }
//   QCurrent.MatOps_MRP2Quat(state);
//   memcpy(QNED2Bdy, QCurrent.vec_vals, 4*sizeof(double));
//   __android_log_print(ANDROID_LOG_INFO, "Native", "NED GROUND = %f %f %f %f",
//		   QNED2Bdy[0], QNED2Bdy[1], QNED2Bdy[2], QNED2Bdy[3]);
////		   LastBVecTime, LastAccTime, TimeTagArray[0], TimeTagArray[1]);
//   __android_log_print(ANDROID_LOG_INFO, "Native", "NED GROUND = %f %f %f",
//		   LastBVec.AccumulatedMeas[0], LastBVec.AccumulatedMeas[1], LastBVec.AccumulatedMeas[2]);
//   NEDOut.TimeTag = TimeTag;
//   memcpy(NEDOut.NEDOutputQuat, QNED2Bdy, 4*sizeof(double));
//   SamplingPortSingleton::GetInstance()->WriteSamplingMessage(NEDOutputPortID,
//		   sizeof(NEDOutputStruct), reinterpret_cast<uint8_t *>(&NEDOut));
//   memcpy(fsw_cast->AlignQNED2Bdy, QNED2Bdy, 4*sizeof(double));
//   CovarMat.MatOps_transpose(Sbar);
//   CovarMat.MatOps_mult(Sbar, CovarMat);
//   CovarNorm = CovarMat.MatOps_onenorm();
//   memcpy(CovarEst, CovarMat.vec_vals, 9*sizeof(double));
//   DetermineConvergence(CovarNorm);
//   fsw_cast->GroundCovarNorm = CovarNorm;
//   fsw_cast->AlignTimeTag = TimeTag;
//   fsw_cast->AlignmentConverged = FilterConverged;
   
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
      Q_obs.MatOps_init(3,3);
      Q_obs.MatOps_VecSet(&QStObs[0]);
      state.MatOps_init(6,1);
      /*state.MatOps_VecSet(&QNED2Bdy[0]);
      state.MatOps_Quat2MRP(state);*/
      Qnoise.MatOps_init(6,6);
      Qnoise.MatOps_VecSet(&QNoiseInit[0]);
      Covar.MatOps_init(6,6);
      Covar.MatOps_VecSet(&CovarInit[0]);
      lambdaVal = alpha*alpha*(NumStates+kappa)-NumStates;
      gamma = sqrt(NumStates+lambdaVal);
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

