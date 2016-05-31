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
    memset(this->sumF_B, 0x0, 3*sizeof(double));
    memset(this->sumTau_B, 0x0, 3*sizeof(double));
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
    double u_s;
    double cosTheta;
    double sinTheta;
    double gtHat_B[3];
    double temp1[3];
    double temp2[3];

    // zero the sum vectors of RW jitter torque and force
    v3SetZero(this->sumTau_B);
    v3SetZero(this->sumF_B);

    // loop through commands
    for(CmdIt=NewRWCmds.begin(); CmdIt!=NewRWCmds.end(); CmdIt++)
    {
        // saturation
        if (this->ReactionWheelData[RWIter].u_max > 0) {
            if(CmdIt->u_cmd > this->ReactionWheelData[RWIter].u_max) {
                CmdIt->u_cmd = this->ReactionWheelData[RWIter].u_max;
            } else if(CmdIt->u_cmd < -this->ReactionWheelData[RWIter].u_max) {
                CmdIt->u_cmd = -this->ReactionWheelData[RWIter].u_max;
            }
        }

        // minimum torque
        if( std::abs(CmdIt->u_cmd) < this->ReactionWheelData[RWIter].u_min) {
            CmdIt->u_cmd = 0;
        }

        // Coulomb friction
        if(this->ReactionWheelData[RWIter].Omega > 0.0) {
            u_s = CmdIt->u_cmd - this->ReactionWheelData[RWIter].u_f;
        } else if(this->ReactionWheelData[RWIter].Omega < 0.0) {
            u_s = CmdIt->u_cmd + this->ReactionWheelData[RWIter].u_f;
        } else {
            if (fabs(CmdIt->u_cmd) < this->ReactionWheelData[RWIter].u_f) {
                // stickage
                u_s = 0;
            } else {
                u_s = CmdIt->u_cmd;
            }
        }

        this->ReactionWheelData[RWIter].u_current = u_s; // save actual torque for reaction wheel motor

        // zero previous RW jitter torque/force vector
        v3SetZero(this->ReactionWheelData[RWIter].tau_B);
        v3SetZero(this->ReactionWheelData[RWIter].F_B);
        // imbalance torque
        if (this->ReactionWheelData[RWIter].usingRWJitter) {

            cosTheta = cos(this->ReactionWheelData[RWIter].theta);
            sinTheta = sin(this->ReactionWheelData[RWIter].theta);

            v3Scale(cosTheta, this->ReactionWheelData[RWIter].gtHat0_B, temp1);
            v3Scale(sinTheta, this->ReactionWheelData[RWIter].ggHat0_B, temp2);
            v3Add(temp1, temp2, gtHat_B); // current gtHat axis vector represented in body frame

            /* Fs = Us * Omega^2 */ // calculate static imbalance force
            v3Scale(this->ReactionWheelData[RWIter].U_s*pow(this->ReactionWheelData[RWIter].Omega,2),
                    gtHat_B,
                    this->ReactionWheelData[RWIter].F_B);
            v3Add(this->ReactionWheelData[RWIter].F_B, this->sumF_B, this->sumF_B);

            /* tau_s = cross(r_B,Fs) */ // calculate static imbalance torque
            v3Cross(this->ReactionWheelData[RWIter].r_B,
                    this->ReactionWheelData[RWIter].F_B,
                    this->ReactionWheelData[RWIter].tau_B);
            /* tau_d = Ud * Omega^2 */ // calculate dynamic imbalance torque
            v3Scale(this->ReactionWheelData[RWIter].U_d*pow(this->ReactionWheelData[RWIter].Omega,2),
                    gtHat_B,
                    temp2);
            // add in dynamic imbalance torque
            v3Add(this->ReactionWheelData[RWIter].tau_B, temp2, this->ReactionWheelData[RWIter].tau_B);
            v3Add(this->ReactionWheelData[RWIter].tau_B, this->sumTau_B, this->sumTau_B);
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
                                            OutputStateData *Bstate,
                                            double CurrentTime)
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
