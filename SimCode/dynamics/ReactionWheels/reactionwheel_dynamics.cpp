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
#include "dynamics/ReactionWheels/reactionwheel_dynamics.h"
#include "architecture/messaging/system_messaging.h"
#include "utilities/linearAlgebra.h"
#include <cstring>
#include <iostream>
#include <cmath>

/*! This is the constructor.  It sets some default initializers that can be
 overriden by the user.*/
ReactionWheelDynamics::ReactionWheelDynamics()
{
    this->ConfigDataOutMsgName = "reactionwheel_config_data_output";
    this->ConfigDataOutMsgID = -1;
    CallCounts = 0;
    InputCmds = "reactionwheel_cmds";
    OutputDataString = "reactionwheel_output_states";
    OutputBufferCount = 2;
    CmdsInMsgID = -1;
    StateOutMsgID = -1;
    IncomingCmdBuffer = NULL;
    prevCommandTime = 0xFFFFFFFFFFFFFFFF;
    memset(F_B, 0x0, 3*sizeof(double));
    memset(tau_B, 0x0, 3*sizeof(double));
    return;
}

/*! The destructor.  Nothing of note is performed here*/
ReactionWheelDynamics::~ReactionWheelDynamics()
{
    return;
}

/*! This method is used to clear out the current RW states and make sure
 that the overall model is ready
 @return void
 */
void ReactionWheelDynamics::SelfInit()
{

    RWCmdStruct RWCmdInitializer;
    RWCmdInitializer.u_cmd = 0.0;

    //! Begin method steps
    //! - Clear out any currently firing RWs and re-init cmd array
    NewRWCmds.clear();
    NewRWCmds.insert(NewRWCmds.begin(), ReactionWheelData.size(), RWCmdInitializer );
//    ! - Clear out the incoming command buffer and resize to max RWs
    if(IncomingCmdBuffer != NULL)
    {
        delete [] IncomingCmdBuffer;
    }
    IncomingCmdBuffer = new RWCmdStruct[ReactionWheelData.size()];

	StateOutMsgID = SystemMessaging::GetInstance()->
		CreateNewMessage(OutputDataString, sizeof(RWSpeedData), 
			OutputBufferCount, "RWSpeedData", moduleID);
    
//    ConfigDataOutMsgID = SystemMessaging::GetInstance()->
//        CreateNewMessage(ConfigDataOutMsgName, sizeof(ReactionWheelData),
//                         OutputBufferCount, "RWConfigData", moduleID);

}

/*! This method is used to connect the input command message to the RWs.
 It sets the message ID based on what it finds for the input string.  If the
 message is not successfully linked, it will warn the user.
 @return void
 */
void ReactionWheelDynamics::CrossInit()
{
    //! Begin method steps
    //! - Find the message ID associated with the InputCmds string.
    //! - Warn the user if the message is not successfully linked.
    CmdsInMsgID = SystemMessaging::GetInstance()->subscribeToMessage(InputCmds,
		sizeof(RWCmdStruct)*MAX_EFF_CNT, moduleID);
    if(CmdsInMsgID < 0)
    {
        std::cerr << "WARNING: Did not find a valid message with name: ";
		std::cerr << InputCmds << "  :" << std::endl<< __FILE__ << std::endl;
    }
}

/*! This method is here to write the output message structure into the specified
 message.  It is currently blank but we will certainly have an output message
 soon.  If it is already here, bludgeon whoever added it and didn't fix the
 comment.
 @param CurrentClock The current time used for time-stamping the message
 @return void
 */
void ReactionWheelDynamics::WriteOutputMessages(uint64_t CurrentClock)
{
	std::vector<ReactionWheelConfigData>::iterator it;

	for (it = ReactionWheelData.begin(); it != ReactionWheelData.end(); it++)
	{
		outputStates.wheelSpeeds[it - ReactionWheelData.begin()] = it->Omega;
	}

	SystemMessaging::GetInstance()->WriteMessage(StateOutMsgID, CurrentClock,
		sizeof(RWSpeedData), reinterpret_cast<uint8_t*> (&outputStates), moduleID);
    
//    std::vector<ReactionWheelConfigData> localOutput;
//    for (it = ReactionWheelData.begin(); it != ReactionWheelData.end(); it++)
//    {
//        memcpy(&localOutput[it - ReactionWheelData.begin()], &it, (4*3+10)*sizeof(double)+sizeof(bool));
//    }
//    
//    SystemMessaging::GetInstance()->WriteMessage(ConfigDataOutMsgID, CurrentClock,
//        sizeof(localOutput), reinterpret_cast<uint8_t*> (&localOutput), moduleID);
}

/*! This method is used to read the incoming command message and set the
 associated command structure for operating the RWs.
 @return void
 */
void ReactionWheelDynamics::ReadInputs()
{
    
    std::vector<double>::iterator CmdIt;
    uint64_t i;
    //! Begin method steps
    //! - If the input message ID is invalid, return without touching states
    if(CmdsInMsgID < 0)
    {
        return;
    }
    
    //! - Zero the command buffer and read the incoming command array
    SingleMessageHeader LocalHeader;
    memset(IncomingCmdBuffer, 0x0, ReactionWheelData.size()*sizeof(RWCmdStruct));
    SystemMessaging::GetInstance()->ReadMessage(CmdsInMsgID, &LocalHeader,
                                                ReactionWheelData.size()*sizeof(RWCmdStruct),
                                                reinterpret_cast<uint8_t*> (IncomingCmdBuffer), moduleID);

    //! - Check if message has already been read, if stale return
//    if(prevCommandTime==LocalHeader.WriteClockNanos) {
//        return;
//    }
    prevCommandTime = LocalHeader.WriteClockNanos;
    
    //! - Set the NewRWCmds vector.  Using the data() method for raw speed
    RWCmdStruct *CmdPtr;
    for(i=0, CmdPtr = NewRWCmds.data(); i<ReactionWheelData.size();
        CmdPtr++, i++)
    {
        CmdPtr->u_cmd = IncomingCmdBuffer[i].u_cmd;
    }

}

///*! This method is used to read the new commands vector and set the RW
// firings appropriately.  It assumes that the ReadInputs method has already been
// run successfully.
// @return void
// @param CurrentTime The current simulation time converted to a double
// */
void ReactionWheelDynamics::ConfigureRWRequests(double CurrentTime)
{
 //! Begin method steps
 std::vector<RWCmdStruct>::iterator CmdIt;
 int RWIter = 0;
 double tau_B_temp[3];
 double u_s;
 double cosw;
 double sinw;
 double ggHat_B[3];
 double temp1[3];
 double temp2[3];
 double temp3[3];
 double Fi[3];
 double Li[3];

 // zero previous torque vector
 v3Set(0,0,0,tau_B);

 // loop through commands
 for(CmdIt=NewRWCmds.begin(); CmdIt!=NewRWCmds.end(); CmdIt++)
 {

  // saturation
  if(CmdIt->u_cmd > ReactionWheelData[RWIter].u_max) {
   CmdIt->u_cmd = ReactionWheelData[RWIter].u_max;
  } else if(CmdIt->u_cmd < -ReactionWheelData[RWIter].u_max) {
   CmdIt->u_cmd = -ReactionWheelData[RWIter].u_max;
  }

  // minimum torque
  if( std::abs(CmdIt->u_cmd) < ReactionWheelData[RWIter].u_min) {
   CmdIt->u_cmd = 0;
  }

  // coulomb friction
  if(CmdIt->u_cmd > ReactionWheelData[RWIter].u_f) {
   u_s = CmdIt->u_cmd - ReactionWheelData[RWIter].u_f;
  } else if(CmdIt->u_cmd < -ReactionWheelData[RWIter].u_f) {
   u_s = CmdIt->u_cmd + ReactionWheelData[RWIter].u_f;
  } else {
   u_s = 0;
  }

  ReactionWheelData[RWIter].u_current = u_s; // save actual torque for reaction wheel motor



   v3Set(0,0,0,tau_B_temp); // zero torque vector for current RW
   v3Scale(u_s, ReactionWheelData[RWIter].gsHat_B, tau_B_temp); // torque vector for current RW
   v3Add(tau_B,tau_B_temp,tau_B); // sum with other RW torque vectors

  // imbalance torque
  if (ReactionWheelData[RWIter].usingRWJitter) {
   cosw = cos(ReactionWheelData[RWIter].theta);
   sinw = sin(ReactionWheelData[RWIter].theta);
   
   v3Scale(cosw, ReactionWheelData[RWIter].gtHat0_B, temp1);
   v3Scale(sinw, ReactionWheelData[RWIter].ggHat0_B, temp2);
   v3Add(temp1, temp2, ggHat_B); // current ggHat axis vector represented in body frame

   /* Fs = Us * Omega^2 */ // calculate static imbalance force
   v3Scale(ReactionWheelData[RWIter].U_s*pow(ReactionWheelData[RWIter].Omega,2),ggHat_B,Fi);
   v3Cross(ReactionWheelData[RWIter].r_B, Fi, Li); /* tau_s = cross(r_B,Fs) */ // calculate static imbalance torque
   v3Add(Li, temp3, temp3); // add in static imbalance torque
   v3Scale(ReactionWheelData[RWIter].U_d*pow(ReactionWheelData[RWIter].Omega,2),ggHat_B, Li); /* tau_d = Ud * Omega^2 */ // calculate dynamic imbalance torque
   v3Add(Li, temp3, temp3); // add in dynamic imbalance torque

   v3Add(tau_B, temp3, tau_B);
  }
  
  RWIter++;

 }



}

/*! This method is used to compute all the dynamical effects for the RW set.
 It is an inherited method from the DynEffector class and is designed to be called
 by the dynamics plant for the simulation.
 @return void
 @param Props Current mass properties of the vehicle (using center of mass and str2bdy transformation
 @param Bstate Current state of the vehicle (not used by RW dynamics)
 @param CurrentTime Current simulation time converted to double precision
 */
void ReactionWheelDynamics::ComputeDynamics(MassPropsData *Props,
                                       OutputStateData *Bstate, double CurrentTime)
{}

/*! This method is the main cyclical call for the scheduled part of the RW
 dynamics model.  It reads the current commands array and sets the RW
 configuration data based on that incoming command set.  Note that the main
 dynamical method (ComputeDynamics()) is not called here and is intended to be
 called from the dynamics plant in the system
 @return void
 @param CurrentSimNanos The current simulation time in nanoseconds
 */
void ReactionWheelDynamics::UpdateState(uint64_t CurrentSimNanos)
{
    //! Begin method steps
    //! - Read the inputs and then call ConfigureRWRequests to set up dynamics
    ReadInputs();
    ConfigureRWRequests(CurrentSimNanos*NANO2SEC);
	WriteOutputMessages(CurrentSimNanos);

}
