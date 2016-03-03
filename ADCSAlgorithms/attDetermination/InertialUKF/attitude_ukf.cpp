

#include "attitude_ukf.h"
#include <math.h>
#include <string.h>


AttitudeUKF::AttitudeUKF() 
{
	InputNEDFiltName = "NEDOutputData";
	InputDRBufferName = "DRBufferData";
	InputGyroBiasName = "GyrFilterOut";
	OutputAttEstName = "AttFilterOut";
	OutputFiltDatName = "AttFiltInformation";
	memset(CurrGyroBias, '\0', 3*sizeof(double));
	UpdateCounter = 0;
   return;
}

AttitudeUKF::~AttitudeUKF()
{
   return;
}

void AttitudeUKF::CheckForUpdate(
      UKFQState TestState)
{
	MatrixOperations Q1, Q2;
	double EigAxis[3], EigAng;
	if(TestState.TimeTag > TimeTag)
	{
		Q1.MatOps_init(4,1);
		Q2.MatOps_init(3,1);
		Q1.MatOps_VecSet(TestState.Qstate);
		Q2.MatOps_VecSet(CurrGyroBias);
		Q2.MatOps_scale(TestState.TimeTag-TimeTag);
		EigAng = Q2.MatOps_twonorm();
		if(EigAng > 0.0)
		{
			Q2.MatOps_scale(-1.0/EigAng);
		}
		memcpy(EigAxis, Q2.vec_vals, 3*sizeof(double));
		Q2.MatOps_init(4,1);
		//OrbitalUtils::EigAxis2Quaternion(EigAxis, EigAng, Q2.vec_vals);
		Q2.MatOps_QuatMult(Q2, Q1);
		memcpy(this->Q_B1B2, Q2.vec_vals, 4*sizeof(double));
		this->TimeUpdateFilter(TestState.TimeTag);
		if(state.MatOps_twonorm_square() > 1.5) //Little extra margin
		{
			state.MatOps_ShadowMRP(state);
		}
	}
}


void AttitudeUKF::StateProp(MatrixOperations &StateCurr)
{
   double Q_state[4];
   MatrixOperations QMat;
   MatrixOperations Q_B1B2Mat = MatrixOperations(4, 1, Q_B1B2);
   
   QMat.MatOps_MRP2Quat(StateCurr);
   Q_B1B2Mat.MatOps_scale(1.0/Q_B1B2Mat.MatOps_twonorm());
   QMat.MatOps_QuatMult(Q_B1B2Mat, QMat);
   QMat.MatOps_scale(1.0/QMat.MatOps_twonorm());
   StateCurr.MatOps_Quat2MRP(QMat);

   return;
}

void AttitudeUKF::ComputeMeasModel()
{
   int i;
   YMeas = new MatrixOperations[CountHalfSPs*2+1];
  /* MatrixOperations QMeasMat = MatrixOperations(4,1,
		   PrevNEDEst.NEDOutputQuat);*/
   MatrixOperations QMeasMat;
   for(i=0; i<CountHalfSPs*2+1; i++)
   {
     YMeas[i].MatOps_init(3, 1);
     YMeas[i].MatOps_VecSet(SP[i].vec_vals);
   }
   obs.MatOps_Quat2MRP(QMeasMat);
   NumObs = 3;
   return;
}

MatrixOperations AttitudeUKF::ComputeSPVector(
        MatrixOperations SPError,
        MatrixOperations StateIn)
{
   MatrixOperations StateSum, ErrorQuat, StateQuat, CompQuat;

   ErrorQuat.MatOps_MRP2Quat(SPError);
   StateQuat.MatOps_MRP2Quat(StateIn);
   //ErrorQuat.MatOps_QuatTrans(ErrorQuat);
   int SignState = StateQuat.vec_vals[0] < 0.0 ? -1 : 1;
   int SignErr = ErrorQuat.vec_vals[0] < 0.0 ? -1 : 1;
   if(SignState != SignErr)
   {
	   ErrorQuat.MatOps_scale(-1.0);
   }
   CompQuat.MatOps_QuatMult(ErrorQuat, StateQuat);
   StateSum.MatOps_Quat2MRP(CompQuat);
   return(StateSum);
}

MatrixOperations AttitudeUKF::ComputeObsError(
        MatrixOperations SPError,
        MatrixOperations StateIn)
{
//   MatrixOperations StateSum;
//   StateSum.MatOps_add(SPError, StateIn);
   MatrixOperations StateSum, ErrorQuat, StateQuat, CompQuat;
   
   ErrorQuat.MatOps_MRP2Quat(SPError);
   StateQuat.MatOps_MRP2Quat(StateIn);
   //ErrorQuat.MatOps_QuatTrans(ErrorQuat);
   CompQuat.MatOps_QuatMult(ErrorQuat, StateQuat);
   StateSum.MatOps_Quat2MRP(CompQuat);
   if(StateSum.MatOps_twonorm_square() > 1.0)
   {
      StateSum.MatOps_ShadowMRP(StateSum);
   }
   return(StateSum);
}

std::vector<ImuMeasEntry> AttitudeUKF::SortDRs( 
   std::vector<ImuMeasEntry> DRBuffer)
{
   int i;
   std::vector<ImuMeasEntry> SortedDRs;
   std::vector<ImuMeasEntry>::iterator it;
   ImuMeasEntry CurrEntry;
   ImuMeasEntry TransEntry;
   SortedDRs.clear();
   for(i=0; i<DRBuffer.size(); i++)
   {
      CurrEntry = DRBuffer.at(i);
      if(CurrEntry.TimeTag <= QTimeTag)
      {
         continue; //Timetag is aged
      }
      for(it=SortedDRs.begin(); it != SortedDRs.end(); it++)
      {
         if(CurrEntry.TimeTag < it->TimeTag)
         {
            SortedDRs.insert(it, CurrEntry);
            break;
         }
      }
      if(it == SortedDRs.end())
      {
         SortedDRs.push_back(CurrEntry);
      }
   }
   return(SortedDRs);
}

void  AttitudeUKF::AdvanceState(
      MatrixOperations &QCurrent,
      MatrixOperations DRAdvance)
{
   double EigAng;
   MatrixOperations QAdvance;
   QAdvance.MatOps_init(4,1);
   QAdvance.vec_vals[0] = 1.0;
   EigAng = DRAdvance.MatOps_twonorm();
   if(EigAng > 0.0)
   {
      DRAdvance.MatOps_scale(1.0/EigAng);
      /*OrbitalUtils::EigAxis2Quaternion(DRAdvance.vec_vals, EigAng,
         QAdvance.vec_vals);*/
   }
   QCurrent.MatOps_QuatMult(QAdvance, QCurrent);
   
   return;
}

void AttitudeUKF::PropagateOutput(std::vector<ImuMeasEntry> &SortedDRs)
{
   /*int VecIndex;
   std::vector<ImuMeasEntry>::iterator it;
   MatrixOperations QCurrent = MatrixOperations(4,1,Q_ECI2Bdy);
   MatrixOperations DRVec[2];
   MatrixOperations DRDiff;
   MatrixOperations DRInit = MatrixOperations(3,1,DRLast);
   MatrixOperations QPrev;
   UKFQState BuffElem;
   if(SortedDRs.size() < 1)
   {
      return;
   }
   DRVec[0].MatOps_init(3,1);
   DRVec[1].MatOps_init(3,1);
   DRVec[0].MatOps_VecSet(SortedDRs.begin()->AccumulatedMeas);
   VecIndex = 0;
   for(it=SortedDRs.begin()+1; it!=SortedDRs.end(); it++)
   {
      DRVec[(VecIndex+1)%2].MatOps_VecSet(it->AccumulatedMeas);
      DRDiff = DRInit;
      DRDiff.MatOps_scale(-1.0);
      DRDiff.MatOps_add(DRDiff,  DRVec[(VecIndex+1)%2]);
      if(DRDiff.MatOps_twonorm() > SmallAngThresh)
      {
         DRDiff = DRInit;
         DRDiff.MatOps_scale(-1.0);
         DRDiff.MatOps_add(DRDiff,  DRVec[(VecIndex)%2]);
         QPrev = QCurrent;
         QPrev.MatOps_QuatTrans(QPrev); 
         AdvanceState(QCurrent, DRDiff);
         QPrev.MatOps_QuatMult(QCurrent, QPrev);
         memcpy(BuffElem.Qstate, QPrev.vec_vals, 4*sizeof(double));
         BuffElem.TimeTag = (it-1)->TimeTag;
   if(QBuffer.BufferFull)
   {
      CheckForUpdate(QBuffer.GetOldest());
   }
         QBuffer.Insert(BuffElem);
         DRInit =  DRVec[(VecIndex)%2];
      }
      VecIndex++;
      VecIndex = VecIndex%2;
   }
   it = SortedDRs.end() - 1;
   DRDiff = DRInit;
   DRDiff.MatOps_scale(-1.0);
   DRDiff.MatOps_add(DRDiff,  DRVec[(VecIndex)%2]);
   QPrev = QCurrent;
   QPrev.MatOps_QuatTrans(QPrev); 
   AdvanceState(QCurrent, DRDiff);
   QPrev.MatOps_QuatMult(QCurrent, QPrev);
   memcpy(BuffElem.Qstate, QPrev.vec_vals, 4*sizeof(double));
   BuffElem.TimeTag = (it)->TimeTag;
   if(QBuffer.BufferFull)
   {
      CheckForUpdate(QBuffer.GetOldest());
   }
   QBuffer.Insert(BuffElem);
   memcpy(Q_ECI2Bdy, QCurrent.vec_vals, 4*sizeof(double));
   memcpy(DRLast, it->AccumulatedMeas, 3*sizeof(double));
   QTimeTag = it->TimeTag;*/
   
   return;
}

void AttitudeUKF::UpdateStateToMeasurement()
{
  /* int i;
   double DtStates, DtTrack;
   UKFQState QData;
   MatrixOperations QVec, Qtotal;
   for(i=0; i<QBuffer.NumElements; i++)
   {
      QData = QBuffer.GetOldRel(i);
      if(QData.TimeTag > PrevNEDEst.TimeTag)
      {
         DtStates = QData.TimeTag - TimeTag;
         DtTrack = PrevNEDEst.TimeTag - TimeTag;
         QVec.MatOps_init(3,1);
         QVec.MatOps_VecSet(&QData.Qstate[1]);
         QVec.MatOps_scale(DtTrack/DtStates);
         Qtotal.MatOps_init(4,1);
         Qtotal.vec_vals[0] = QData.Qstate[0];
         memcpy(&Qtotal.vec_vals[1], QVec.vec_vals, 3*sizeof(double));
         Qtotal.MatOps_scale(1.0/Qtotal.MatOps_twonorm());
         CheckForUpdate(QData);
         break;
      }
      CheckForUpdate(QData);
   }*/
   return;
}

void AttitudeUKF::DetermineConvergence(
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

void AttitudeUKF::ApplyFilterState()
{
   int i;
   MatrixOperations StateCurrent, QCurrent;
   UKFQState QData;
   StateCurrent = state;
  /* for(i=0; i<QBuffer.NumElements; i++)
   {
      QData = QBuffer.GetOldRel(i);
      if(QData.TimeTag > TimeTag)
      {
         memcpy(&Q_B1B2[0], QData.Qstate, 4*sizeof(double));
         StateProp(StateCurrent);
      }
   }*/
   QCurrent.MatOps_MRP2Quat(StateCurrent);
   memcpy(Q_ECI2Bdy, QCurrent.vec_vals, 4*sizeof(double));

   return;
}

void AttitudeUKF::GNC_alg()
{
   int i;
   MatrixOperations CovarMat, QLocal, TLocal;
   double CovarNorm;
   bool Valid, Unread;
   std::vector<ImuMeasEntry> SortedDRs;
//   SamplingPortSingleton::SamplingMessageReturn BlankPort, BlankPort2, BlankPort3;
//   RingBuffer <ImuMeasEntry> DRBuffer;
//   NEDOutputStruct *NEDFiltOut;
//   UKFQState OutputState;
//   UKFAttFiltData OutputFiltData;
//   GyroBiasOutputStruct *GyroBiasOut;
//
//   SamplingPortSingleton::GetInstance()->ReadSamplingMessage(
//		   InputDRBufferID, BlankPort, Valid, Unread);
//   if(!Valid)
//   {
//	   return;
//   }
//   SamplingPortSingleton::GetInstance()->ReadSamplingMessage(
//   		   InputNEDFiltID, BlankPort2, Valid, Unread);
//   if(!Valid)
//   {
//	   return;
//   }
//   SamplingPortSingleton::GetInstance()->ReadSamplingMessage(
//		   InputGyroBiasID, BlankPort3, Valid, Unread);
//   if(Valid)
//   {
//	   GyroBiasOut = reinterpret_cast<GyroBiasOutputStruct *>
//	   (BlankPort3.SamplingData);
//	   memcpy(CurrGyroBias, GyroBiasOut->BodyBiasEst, 3*sizeof(double));
//   }
//
//   NEDFiltOut = reinterpret_cast<NEDOutputStruct *> (BlankPort2.SamplingData);
//   DRBuffer.RingBufferUnPacketize(BlankPort.SamplingData);
//
//   SortedDRs = SortDRs(DRBuffer);
//   PropagateOutput(SortedDRs);
//   if(PrevNEDEst.TimeTag != NEDFiltOut->TimeTag)
//   {
//      memcpy(&PrevNEDEst, NEDFiltOut, sizeof(NEDOutputStruct));
//      if(PrevNEDEst.NEDOutputQuat[0] < 0.0)
//      {
//    	  for(i=0; i<4; i++)
//    	  {
//    		  PrevNEDEst.NEDOutputQuat[i] = -PrevNEDEst.NEDOutputQuat[i];
//    	  }
//      }
//      UpdateStateToMeasurement();
//      MeasurementUpdate();
//      memcpy(QVecFilt, state.vec_vals, 3*sizeof(double));
//      UpdateCounter++;
////      MatrixOperations QuatFiltMat;
////            MatrixOperations QuatNEDMat;
////            QuatFiltMat.MatOps_MRP2Quat(state);
////            QuatNEDMat.MatOps_MRP2Quat(obs);
////            QuatNEDMat.MatOps_QuatTrans(QuatNEDMat);
////            QuatFiltMat.MatOps_QuatMult(QuatFiltMat, QuatNEDMat);
////            __android_log_print(ANDROID_LOG_INFO, "Native", "ATT_CONV = %f %f %f %f",
////               QuatFiltMat.vec_vals[0], QuatFiltMat.vec_vals[1], QuatFiltMat.vec_vals[2], QuatFiltMat.vec_vals[3]);
//   }
   QLocal.MatOps_MRP2Quat(state);
   memcpy(&Q_Filter[0], QLocal.vec_vals, 4*sizeof(double));
   CovarMat.MatOps_transpose(Sbar);
   CovarMat.MatOps_mult(Sbar, CovarMat);
   CovarNorm = CovarMat.MatOps_onenorm();
   DetermineConvergence(CovarNorm);
   if(FilterConverged)
   {
      ApplyFilterState();
   } 
  /* memcpy(CovarEst, CovarMat.vec_vals, 9*sizeof(double));
   QLocal.MatOps_VecSet(Q_ECI2Bdy);
   TLocal.MatOps_Quat2DCM(QLocal);
   QLocal.MatOps_DCM2Quat(TLocal);
   memcpy(OutputState.Qstate, Q_ECI2Bdy, 4*sizeof(double));
   OutputState.TimeTag = TimeTag;

   OutputFiltData.TimeTag = TimeTag;
   memcpy(&(OutputFiltData.CovarMat[0][0]),&(CovarMat.vec_vals[0]),
		   9*sizeof(double));
   memcpy(&(OutputFiltData.SBarMat[0][0]), &(Sbar.vec_vals[0]),
		   9*sizeof(double));
   OutputFiltData.UpdateCounter = UpdateCounter;
   memcpy(&(OutputFiltData.QFilter[0]), &(Q_Filter[0]), 4*sizeof(double));
   memcpy(&(OutputFiltData.StateValues[0]), state.vec_vals, 3*sizeof(double));
   SamplingPortSingleton::GetInstance()->WriteSamplingMessage(OutputAttEstID,
   		   sizeof(UKFQState), reinterpret_cast<uint8_t *>(&OutputState));
   SamplingPortSingleton::GetInstance()->WriteSamplingMessage(OutputFiltDatID,
         		   sizeof(UKFAttFiltData), reinterpret_cast<uint8_t *>(&OutputFiltData));
   __android_log_print(ANDROID_LOG_INFO, "Native", "ATT_CONV = %f", CovarNorm);*/
   //		   LastBVec.AccumulatedMeas[0], LastBVec.AccumulatedMeas[1], LastBVec.AccumulatedMeas[2]);
   //memcpy(fsw_cast->nav_out.T_inrtl_body, TLocal.vec_vals, 9*sizeof(double));
   //fsw_cast->NavTimeTag = this->QTimeTag;
   return;
}
void AttitudeUKF::GNC_config()
{
   /*MatrixOperations QLocal;
   if(RingLength != QBuffer.BufferSize)
   {
      QBuffer.ReSize(RingLength);
   }
   if(ReInitFilter)
   {
      NumStates = 3;
      CountHalfSPs = 3;
      NumObs = 3;
      Q_obs.MatOps_init(3,3);
      Q_obs.MatOps_VecSet(&QStrObs[0][0]);
      QLocal.MatOps_init(4,1);
      QLocal.MatOps_VecSet(&Q_ECI2Bdy[0]);
      state.MatOps_Quat2MRP(QLocal);
      if(state.MatOps_twonorm_square() > 1.5)
      {
         state.MatOps_ShadowMRP(state);
      }
      StateScalar = Q_ECI2Bdy[0];
      TimeTag = QTimeTag;
      Qnoise.MatOps_init(3,3);
      Qnoise.MatOps_VecSet(&QNoiseInit[0][0]);
      Covar.MatOps_init(3,3);
      Covar.MatOps_VecSet(&CovarInit[0][0]);
      lambda = alpha*alpha*(NumStates+kappa)-NumStates;
      gamma = sqrt(NumStates+lambda);
      UKFInit();
      FilterConverged = false;
      ConvCounter = 0;
      ReInitFilter = false;
   }*/
   return;
}
void AttitudeUKF::GNC_alg_init()
{
	/*InputDRBufferID = SamplingPortSingleton::GetInstance()->
			FindPortID(InputDRBufferName.c_str());
	InputNEDFiltID = SamplingPortSingleton::GetInstance()->
			FindPortID(InputNEDFiltName.c_str());
	InputGyroBiasID = SamplingPortSingleton::GetInstance()->CreateSamplingPort(
			sizeof(GyroBiasOutputStruct), 1E9, InputGyroBiasName.c_str());
	OutputAttEstID = SamplingPortSingleton::GetInstance()->CreateSamplingPort(
	   sizeof(UKFQState), 1E9, OutputAttEstName.c_str());
	OutputFiltDatID = SamplingPortSingleton::GetInstance()->CreateSamplingPort(
			sizeof(UKFAttFiltData), 1E9, OutputFiltDatName.c_str());*/
   return;
}

//bool AttitudeUKF::WriteCheckpoint()
//{
//   bool status = false;
//   bool TempInit;
//   status = checkpoint.StartCheckpoint(checkpointFile, algorithm_tag, checkpointName);
//   if( status )
//   {
//      TempInit = ReInitFilter;
//      ReInitFilter = true;
//      checkpoint.AddVariable("RingLength", (uint8_t*)this, &RingLength);
//      checkpoint.AddVariable("SmallAngThresh", (uint8_t*)this, &SmallAngThresh);
//      checkpoint.AddVariable("ReInitFilter", (uint8_t*)this, &ReInitFilter);
//      checkpoint.AddVariable("QNoiseInit[0][0]", (uint8_t*)this, &QNoiseInit[0][0]);
//      checkpoint.AddVariable("QNoiseInit[0][1]", (uint8_t*)this, &QNoiseInit[0][1]);
//      checkpoint.AddVariable("QNoiseInit[0][2]", (uint8_t*)this, &QNoiseInit[0][2]);
//      checkpoint.AddVariable("QNoiseInit[1][0]", (uint8_t*)this, &QNoiseInit[1][0]);
//      checkpoint.AddVariable("QNoiseInit[1][1]", (uint8_t*)this, &QNoiseInit[1][1]);
//      checkpoint.AddVariable("QNoiseInit[1][2]", (uint8_t*)this, &QNoiseInit[1][2]);
//      checkpoint.AddVariable("QNoiseInit[2][0]", (uint8_t*)this, &QNoiseInit[2][0]);
//      checkpoint.AddVariable("QNoiseInit[2][1]", (uint8_t*)this, &QNoiseInit[2][1]);
//      checkpoint.AddVariable("QNoiseInit[2][2]", (uint8_t*)this, &QNoiseInit[2][2]);
//      checkpoint.AddVariable("CovarInit[0][0]", (uint8_t*)this, &CovarInit[0][0]);
//      checkpoint.AddVariable("CovarInit[0][1]", (uint8_t*)this, &CovarInit[0][1]);
//      checkpoint.AddVariable("CovarInit[0][2]", (uint8_t*)this, &CovarInit[0][2]);
//      checkpoint.AddVariable("CovarInit[1][0]", (uint8_t*)this, &CovarInit[1][0]);
//      checkpoint.AddVariable("CovarInit[1][1]", (uint8_t*)this, &CovarInit[1][1]);
//      checkpoint.AddVariable("CovarInit[1][2]", (uint8_t*)this, &CovarInit[1][2]);
//      checkpoint.AddVariable("CovarInit[2][0]", (uint8_t*)this, &CovarInit[2][0]);
//      checkpoint.AddVariable("CovarInit[2][1]", (uint8_t*)this, &CovarInit[2][1]);
//      checkpoint.AddVariable("CovarInit[2][2]", (uint8_t*)this, &CovarInit[2][2]);
//      checkpoint.AddVariable("QStrObs[0][0]", (uint8_t*)this, &QStrObs[0][0]);
//      checkpoint.AddVariable("QStrObs[0][1]", (uint8_t*)this, &QStrObs[0][1]);
//      checkpoint.AddVariable("QStrObs[0][2]", (uint8_t*)this, &QStrObs[0][2]);
//      checkpoint.AddVariable("QStrObs[1][0]", (uint8_t*)this, &QStrObs[1][0]);
//      checkpoint.AddVariable("QStrObs[1][1]", (uint8_t*)this, &QStrObs[1][1]);
//      checkpoint.AddVariable("QStrObs[1][2]", (uint8_t*)this, &QStrObs[1][2]);
//      checkpoint.AddVariable("QStrObs[2][0]", (uint8_t*)this, &QStrObs[2][0]);
//      checkpoint.AddVariable("QStrObs[2][1]", (uint8_t*)this, &QStrObs[2][1]);
//      checkpoint.AddVariable("QStrObs[2][2]", (uint8_t*)this, &QStrObs[2][2]);
//      checkpoint.AddVariable("ConvThresh", (uint8_t*)this, &ConvThresh );
//      checkpoint.AddVariable("ConvTestCounter", (uint8_t*)this, &ConvTestCounter);
//
//      checkpoint.AddVariable("QTimeTag", (uint8_t*)this, &QTimeTag);
//      checkpoint.AddVariable("Q_ECI2Bdy[0]", (uint8_t*)this, &Q_ECI2Bdy[0]);
//      checkpoint.AddVariable("Q_ECI2Bdy[1]", (uint8_t*)this, &Q_ECI2Bdy[1]);
//      checkpoint.AddVariable("Q_ECI2Bdy[2]", (uint8_t*)this, &Q_ECI2Bdy[2]);
//      checkpoint.AddVariable("Q_ECI2Bdy[3]", (uint8_t*)this, &Q_ECI2Bdy[3]);
//      checkpoint.AddVariable("Q_Filter[0]", (uint8_t*)this, &Q_Filter[0]);
//      checkpoint.AddVariable("Q_Filter[1]", (uint8_t*)this, &Q_Filter[1]);
//      checkpoint.AddVariable("Q_Filter[2]", (uint8_t*)this, &Q_Filter[2]);
//      checkpoint.AddVariable("Q_Filter[3]", (uint8_t*)this, &Q_Filter[3]);
//      checkpoint.AddVariable("QVecFilt[0]", (uint8_t*)this, &QVecFilt[0]);
//      checkpoint.AddVariable("QVecFilt[1]", (uint8_t*)this, &QVecFilt[1]);
//      checkpoint.AddVariable("QVecFilt[2]", (uint8_t*)this, &QVecFilt[2]);
//      checkpoint.AddVariable("CovarEst[0][0]", (uint8_t*)this, &CovarEst[0][0]);
//      checkpoint.AddVariable("CovarEst[0][1]", (uint8_t*)this, &CovarEst[0][1]);
//      checkpoint.AddVariable("CovarEst[0][2]", (uint8_t*)this, &CovarEst[0][2]);
//      checkpoint.AddVariable("CovarEst[1][0]", (uint8_t*)this, &CovarEst[1][0]);
//      checkpoint.AddVariable("CovarEst[1][1]", (uint8_t*)this, &CovarEst[1][1]);
//      checkpoint.AddVariable("CovarEst[1][2]", (uint8_t*)this, &CovarEst[1][2]);
//      checkpoint.AddVariable("CovarEst[2][0]", (uint8_t*)this, &CovarEst[2][0]);
//      checkpoint.AddVariable("CovarEst[2][1]", (uint8_t*)this, &CovarEst[2][1]);
//      checkpoint.AddVariable("CovarEst[2][2]", (uint8_t*)this, &CovarEst[2][2]);
//
//      checkpoint.AddVariable("alpha", (uint8_t*)this, &alpha);
//      checkpoint.AddVariable("beta", (uint8_t*)this, &beta);
//      checkpoint.AddVariable("kappa", (uint8_t*)this, &kappa);
//
//      status = checkpoint.FinishCheckpoint();
//      ReInitFilter = TempInit;
//   }
//
//   return status;
//}
//
//
//bool AttitudeUKF::DisplayCheckpoint()
//{
//   bool status = false;
//   status = checkpoint.PrintCheckpointToScreen( checkpointFile );
//   return status;
//}
//
//
//bool AttitudeUKF::ResetFromCheckpoint()
//{
//   bool status = false;
//   status = checkpoint.ResetFromCheckpoint( checkpointFile, algorithm_tag,
//      (uint8_t*) this );
//   return status;
//}
