#include "dynamics/ReactionWheels/reactionwheel_dynamics.h"
#include "architecture/messaging/system_messaging.h"
#include "utilities/linearAlgebra.h"
#include <cstring>
#include <iostream>

/*! This is the constructor.  It sets some defaul initializers that can be
 overriden by the user.*/
ReactionWheelDynamics::ReactionWheelDynamics()
{
    CallCounts = 0;
    InputCmds = "reactionwheel_cmds";
    OutputDataString = "reactionwheel_output";
    OutputBufferCount = 2;
    CmdsInMsgID = -1;
    StateOutMsgID = -1;
    IncomingCmdBuffer = NULL;
    prevCommandTime = 0xFFFFFFFFFFFFFFFF;
    memset(StrForce, 0x0, 3*sizeof(double));
    memset(StrTorque, 0x0, 3*sizeof(double));
    return;
}

/*! The destructor.  Nothing of note is performed here*/
ReactionWheelDynamics::~ReactionWheelDynamics()
{
    return;
}

/*! This method is used to clear out the current thruster states and make sure
 that the overall model is ready for firings
 @return void
 */
void ReactionWheelDynamics::SelfInit()
{

    RWCmdStruct RWCmdInitializer;
    RWCmdInitializer.TorqueRequest = 0.0;

    //! Begin method steps
    //! - Clear out any currently firing thrusters and re-init cmd array
    NewRWCmds.clear();
    NewRWCmds.insert(NewRWCmds.begin(), ReactionWheelData.size(), RWCmdInitializer );
//    ! - Clear out the incoming command buffer and resize to max thrusters
    if(IncomingCmdBuffer != NULL)
    {
        delete [] IncomingCmdBuffer;
    }
    IncomingCmdBuffer = new RWCmdStruct[ReactionWheelData.size()];

}

/*! This method is used to connect the input command message to the thrusters.
 It sets the message ID based on what it finds for the input string.  If the
 message is not successfully linked, it will warn the user.
 @return void
 */
void ReactionWheelDynamics::CrossInit()
{
    //! Begin method steps
    //! - Find the message ID associated with the InputCmds string.
    //! - Warn the user if the message is not successfully linked.
    CmdsInMsgID = SystemMessaging::GetInstance()->FindMessageID(InputCmds);
    if(CmdsInMsgID < 0)
    {
        std::cerr << "WARNING: Did not find a valid message with name: ";
        std::cerr << InputCmds << "  :" << __FILE__ << std::endl;
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
    
}

/*! This method is used to read the incoming command message and set the
 associated command structure for operating the thrusters.
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
                                                reinterpret_cast<uint8_t*> (IncomingCmdBuffer));

    //! - Check if message has already been read, if stale return
    if(prevCommandTime==LocalHeader.WriteClockNanos) {
        return;
    }
    prevCommandTime = LocalHeader.WriteClockNanos;
    
    //! - Set the NewRWCmds vector.  Using the data() method for raw speed
    RWCmdStruct *CmdPtr;
    for(i=0, CmdPtr = NewRWCmds.data(); i<ReactionWheelData.size();
        CmdPtr++, i++)
    {
        CmdPtr->TorqueRequest = IncomingCmdBuffer[i].TorqueRequest;
    }

}

///*! This method is used to read the new commands vector and set the thruster
// firings appropriately.  It assumes that the ReadInputs method has already been
// run successfully.  It honors all previous thruster firings if they are still
// active.  Note that for unit testing purposes you can insert firings directly
// into NewRWCmds.
// @return void
// @param CurrentTime The current simulation time converted to a double
// */
void ReactionWheelDynamics::ConfigureRWRequests(double CurrentTime)
{
 //! Begin method steps
 std::vector<RWCmdStruct>::iterator CmdIt;
 std::vector<ReactionWheelConfigData>::iterator it;

 for(it = ReactionWheelData.begin(), CmdIt=NewRWCmds.begin();
     CmdIt!=NewRWCmds.end(); CmdIt++, it++)
 {
	 //! - Just set the motor torque equal to the torque request for now
	 it->currentTorque = CmdIt->TorqueRequest;
	 v3Scale(CmdIt->TorqueRequest, &(ReactionWheelData[0].ReactionWheelDirection[0]), StrTorque);

 }


}

/*! This method is used to compute all the dynamical effects for the thruster set.
 It is an inherited method from the DynEffector class and is designed to be called
 by the dynamics plant for the simulation.  It uses the thruster force magnitude
 computed for the current time as well as the current vehicle state and mass
 properties to get the current body force/torque which serve as the API to
 dynamics
 @return void
 @param Props Current mass properties of the vehicle (using center of mass and str2bdy transformation
 @param Bstate Current state of the vehicle (not used by thruster dynamics)
 @param CurrentTime Current simulation time converted to double precision
 */
void ReactionWheelDynamics::ComputeDynamics(MassPropsData *Props,
                                       OutputStateData *Bstate, double CurrentTime)
{}

/*! This method is the main cyclical call for the scheduled part of the thruster
 dynamics model.  It reads the current commands array and sets the thruster
 configuration data based on that incoming command set.  Note that the main
 dynamical method (ComputeDynamics()) is not called here and is intended to be
 called from the dynamics plant in the system
 @return void
 @param CurrentSimNanos The current simulation time in nanoseconds
 */
void ReactionWheelDynamics::UpdateState(uint64_t CurrentSimNanos)
{
    //! Begin method steps
    //! - Read the inputs and then call ConfigureThrustRequests to set up dynamics
    ReadInputs();
    ConfigureRWRequests(CurrentSimNanos*1.0E-9);

}
