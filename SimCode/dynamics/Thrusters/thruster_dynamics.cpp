#include "dynamics/Thrusters/thruster_dynamics.h"
#include "architecture/messaging/system_messaging.h"
#include "utilities/linearAlgebra.h"
#include <cstring>
#include <iostream>

ThrusterDynamics::ThrusterDynamics() 
{
   CallCounts = 0;
   InputCmds = "acs_thruster_cmds";
   OutputDataString = "acs_thruster_output";
   OutputBufferCount = 2;
   CmdsInMsgID = -1;
   StateOutMsgID = -1;
   IncomingCmdBuffer = NULL;
   return;
}

ThrusterDynamics::~ThrusterDynamics()
{
   return;
}

void ThrusterDynamics::SelfInit()
{

   NewThrustCmds.clear();
   NewThrustCmds.insert(NewThrustCmds.begin(), ThrusterData.size(), 0.0);
   if(IncomingCmdBuffer != NULL)
   {
      delete [] IncomingCmdBuffer;
   }
   IncomingCmdBuffer = new ThrustCmdStruct[ThrusterData.size()];

}

void ThrusterDynamics::CrossInit()
{
   CmdsInMsgID = SystemMessaging::GetInstance()->FindMessageID(InputCmds);
   if(CmdsInMsgID < 0)
   {
      std::cerr << "WARNING: Did not find a valid message with name: ";
      std::cerr << InputCmds << "  :" << __FILE__ << std::endl;
   }
}

void ThrusterDynamics::WriteOutputMessages(uint64_t CurrentClock)
{

   //OutputStateData LocalState;
   //if(Elements2Cart)
   //{
   //   memset(&LocalState, 0x0, sizeof(OutputStateData));
   //   memcpy(LocalState.r_N, r_N, 3*sizeof(double));
   //   memcpy(LocalState.v_N, v_N, 3*sizeof(double));
   //   SystemMessaging::GetInstance()->WriteMessage(StateOutMsgID, CurrentClock,
   //      sizeof(OutputStateData), reinterpret_cast<uint8_t*> (&LocalState));
   //}
   //else
   //{
   //   SystemMessaging::GetInstance()->WriteMessage(StateOutMsgID, CurrentClock,
   //      sizeof(classicElements), reinterpret_cast<uint8_t*> (&CurrentElem));
   //}

}

void ThrusterDynamics::ReadInputs()
{

   std::vector<double>::iterator CmdIt;
   uint64_t i;

   if(CmdsInMsgID < 0)
   {
      return;
   }

   SingleMessageHeader LocalHeader;
   memset(IncomingCmdBuffer, 0x0, ThrusterData.size()*sizeof(ThrustCmdStruct));
   SystemMessaging::GetInstance()->ReadMessage(CmdsInMsgID, &LocalHeader,
      ThrusterData.size()*sizeof(ThrustCmdStruct), 
      reinterpret_cast<uint8_t*> (IncomingCmdBuffer)); 

   double *CmdPtr; //= NewThrustCmds.begin().data();
   for(i=0, CmdPtr = NewThrustCmds.data(); i<ThrusterData.size(); 
      CmdPtr++, i++)
   {
      *CmdPtr = IncomingCmdBuffer[i].OnTimeRequest;
   }

}

void ThrusterDynamics::ConfigureThrustRequests(double CurrentTime)
{
   std::vector<ThrusterConfigData>::iterator it;
   std::vector<double>::iterator CmdIt;
   std::vector<ThrusterTimePair>::iterator PairIt;
   for(CmdIt=NewThrustCmds.begin(), it=ThrusterData.begin(); 
      it != ThrusterData.end(); it++, CmdIt++)
   {
      if(*CmdIt > it->MinOnTime) /// - Check to see if we have met minimum
      {
         it->ThrustOps.ThrusterStartTime = CurrentTime;
         it->ThrustOps.ThrustOnCmd = *CmdIt;
         it->ThrustOps.ThrustOnRampTime = 0.0;
         it->ThrustOps.ThrustOnSteadyTime = 0.0;
         it->ThrustOps.ThrustOffRampTime = 0.0;
         it->ThrustOps.PreviousIterTime = CurrentTime;
         if(it->ThrustOps.ThrustFactor > 0.0) /// - Check to see if we are already firing
         {
            for(PairIt = it->ThrusterOnRamp.begin(); 
               PairIt != it->ThrusterOnRamp.end(); PairIt++)
            {
               /// - Find the point in the thrust curve to start with
               if(PairIt->ThrustFactor <= PairIt->ThrustFactor)
               {
                  it->ThrustOps.ThrustOnRampTime = PairIt->TimeDelta;
                  break;
               }
            }
         }
      }
      *CmdIt = 0.0;
   }

}

void ThrusterDynamics::ComputeThrusterFire(ThrusterConfigData *CurrentThruster, 
   double CurrentTime)
{

   std::vector<ThrusterTimePair>::iterator it;
   ThrusterOperationData *ops = &(CurrentThruster->ThrustOps);

   double LocalOnRamp = (CurrentTime - ops->PreviousIterTime) + 
      ops->ThrustOnRampTime;

   for(it = CurrentThruster->ThrusterOnRamp.begin(); 
      it != CurrentThruster->ThrusterOnRamp.end(); it++)
   {
      if(LocalOnRamp < it->TimeDelta)
      {
         ops->ThrustFactor = it->ThrustFactor;
         ops->ThrustOnRampTime = LocalOnRamp;
         ops->PreviousIterTime = CurrentTime;
         return;
      }
   }
   ops->ThrustOnSteadyTime += (CurrentTime - ops->PreviousIterTime);
   ops->PreviousIterTime = CurrentTime;
   ops->ThrustFactor = 1.0;
}

void ThrusterDynamics::ComputeThrusterShut(ThrusterConfigData *CurrentThruster,
   double CurrentTime)
{
   std::vector<ThrusterTimePair>::iterator it;
   ThrusterOperationData *ops = &(CurrentThruster->ThrustOps);

   double LocalOffRamp = (CurrentTime - ops->PreviousIterTime) + 
      ops->ThrustOffRampTime;
   for(it = CurrentThruster->ThrusterOffRamp.begin();
      it != CurrentThruster->ThrusterOffRamp.end(); it++)
   {
      if(LocalOffRamp < it->TimeDelta)
      {
         ops->ThrustFactor = it->ThrustFactor;
         ops->ThrustOffRampTime = LocalOffRamp;
         return;
      }
   }
   ops->ThrustFactor = 0.0;
}

void ThrusterDynamics::ComputeDynamics(MassPropsData *Props, 
   OutputStateData *Bstate, double CurrentTime)
{
   std::vector<ThrusterConfigData>::iterator it;
   ThrusterOperationData *ops;  
   double SingleThrusterForce[3];
   double SingleThrusterTorque[3];
   double CoMRelPos[3];

   memset(StrForce, 0x0, 3*sizeof(double));
   memset(StrTorque, 0x0, 3*sizeof(double));


   for(it=ThrusterData.begin(); it != ThrusterData.end(); it++)
   {
      ops = &it->ThrustOps;
      if((ops->ThrustOnCmd + ops->ThrusterStartTime  - CurrentTime) >= 0.0 && 
         ops->ThrustOnCmd > 0.0)
      {
         ComputeThrusterFire(&(*it), CurrentTime);
      }
      else if(ops->ThrustFactor > 0.0)
      {
         ComputeThrusterShut(&(*it), CurrentTime);
      }
      v3Scale(it->MaxThrust*ops->ThrustFactor, it->ThrusterDirection.data(),
         SingleThrusterForce);
      v3Add(StrForce, SingleThrusterForce, StrForce);
      v3Subtract(it->ThrusterLocation.data(), Props->CoM, CoMRelPos);
      v3Cross(CoMRelPos, SingleThrusterForce, SingleThrusterTorque);
      v3Add(StrTorque, SingleThrusterTorque, StrTorque);
   }

   m33MultV3(Props->T_str2Bdy, StrForce, BodyForce);
   m33MultV3(Props->T_str2Bdy, StrTorque, BodyTorque);

}

void ThrusterDynamics::UpdateState(uint64_t CurrentSimNanos)
{
   ReadInputs();
   ConfigureThrustRequests(CurrentSimNanos*1.0E-9);
   //if(Elements2Cart)
   //{
   //   Elements2Cartesian();
   //}
   //else
   //{
   //   Cartesian2Elements();
   //}
   //WriteOutputMessages(CurrentSimNanos);
}
