#include "dynamics/SixDofEOM/six_dof_eom.h"
#include "dynamics/DynOutput/orb_elem_convert.h"
#include "architecture/messaging/system_messaging.h"
#include <cstring>
#include <iostream>

OrbElemConvert::OrbElemConvert() 
{
   CallCounts = 0;
   StateString = "inertial_state_output";
   OutputDataString = "OrbitalElements";
   OutputBufferCount = 2;
   StateInMsgID = -1;
   StateOutMsgID = -1;
   Elements2Cart = false;
   ReinitSelf = false;
   return;
}

OrbElemConvert::~OrbElemConvert()
{
   return;
}

void OrbElemConvert::SelfInit()
{

   uint64_t OutputSize = Elements2Cart ? sizeof(OutputStateData) : 
      sizeof(classicElements);

   StateOutMsgID = SystemMessaging::GetInstance()->CreateNewMessage(
      OutputDataString, OutputSize, OutputBufferCount);

}

void OrbElemConvert::CrossInit()
{
   StateInMsgID = SystemMessaging::GetInstance()->FindMessageID(StateString);
   if(StateInMsgID < 0)
   {
      std::cerr << "WARNING: Did not find a valid message with name: ";
      std::cerr << StateString << "  :" << __FILE__ << std::endl;
   }
}

void OrbElemConvert::WriteOutputMessages(uint64_t CurrentClock)
{

   OutputStateData LocalState;
   if(Elements2Cart)
   {
      memset(&LocalState, 0x0, sizeof(OutputStateData));
      memcpy(LocalState.r_N, r_N, 3*sizeof(double));
      memcpy(LocalState.v_N, v_N, 3*sizeof(double));
      SystemMessaging::GetInstance()->WriteMessage(StateOutMsgID, CurrentClock,
         sizeof(OutputStateData), reinterpret_cast<uint8_t*> (&LocalState));
   }
   else
   {
      SystemMessaging::GetInstance()->WriteMessage(StateOutMsgID, CurrentClock,
         sizeof(classicElements), reinterpret_cast<uint8_t*> (&CurrentElem));
   }

}

void OrbElemConvert::Elements2Cartesian()
{
   elem2rv(mu, &CurrentElem, r_N, v_N); 
}

void OrbElemConvert::Cartesian2Elements()
{
   rv2elem(mu, r_N, v_N, &CurrentElem);
}

void OrbElemConvert::ReadInputs()
{
   if(StateInMsgID < 0)
   {
      return;
   }
   classicElements LocalElements;
   OutputStateData LocalState;
   SingleMessageHeader LocalHeader;

   uint8_t *InputPtr = Elements2Cart ? reinterpret_cast<uint8_t *> 
      (&LocalElements) : reinterpret_cast<uint8_t *> (&LocalState);

   uint64_t InputSize = Elements2Cart ? sizeof(classicElements) : 
      sizeof(OutputStateData);
   SystemMessaging::GetInstance()->ReadMessage(StateInMsgID, &LocalHeader,
         InputSize, InputPtr);
   if(Elements2Cart)
   {
      memcpy(&CurrentElem, &LocalElements, sizeof(classicElements));
   }
   else
   {
      memcpy(r_N, LocalState.r_N, 3*sizeof(double));
      memcpy(v_N, LocalState.v_N, 3*sizeof(double));
   }
   
}

void OrbElemConvert::UpdateState(uint64_t CurrentSimNanos)
{
   if(ReinitSelf)
   {
      SelfInit();
      CrossInit();
      ReinitSelf = false;
   }
   ReadInputs();
   if(Elements2Cart)
   {
      Elements2Cartesian();
   }
   else
   {
      Cartesian2Elements();
   }
   WriteOutputMessages(CurrentSimNanos);
}
