#include "dynamics/Thrusters/thruster_dynamics.h"
#include "architecture/messaging/system_messaging.h"
#include "utilities/linearAlgebra.h"
#include <cstring>
#include <iostream>

/*! This is the constructor.  It sets some defaul initializers that can be
 overriden by the user.*/
ThrusterDynamics::ThrusterDynamics()
{
    CallCounts = 0;
    InputCmds = "acs_thruster_cmds";
    OutputDataString = "acs_thruster_output";
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
ThrusterDynamics::~ThrusterDynamics()
{
    return;
}

/*! This method is used to clear out the current thruster states and make sure
 that the overall model is ready for firings
 @return void
 */
void ThrusterDynamics::SelfInit()
{
    //! Begin method steps
    //! - Clear out any currently firing thrusters and re-init cmd array
    NewThrustCmds.clear();
    NewThrustCmds.insert(NewThrustCmds.begin(), ThrusterData.size(), 0.0);
    //! - Clear out the incoming command buffer and resize to max thrusters
    if(IncomingCmdBuffer != NULL)
    {
        delete [] IncomingCmdBuffer;
    }
    IncomingCmdBuffer = new ThrustCmdStruct[ThrusterData.size()];
    
}

/*! This method is used to connect the input command message to the thrusters.
 It sets the message ID based on what it finds for the input string.  If the
 message is not successfully linked, it will warn the user.
 @return void
 */
void ThrusterDynamics::CrossInit()
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
void ThrusterDynamics::WriteOutputMessages(uint64_t CurrentClock)
{
    
}

/*! This method is used to read the incoming command message and set the
 associated command structure for operating the thrusters.
 @return void
 */
void ThrusterDynamics::ReadInputs()
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
    memset(IncomingCmdBuffer, 0x0, ThrusterData.size()*sizeof(ThrustCmdStruct));
    SystemMessaging::GetInstance()->ReadMessage(CmdsInMsgID, &LocalHeader,
                                                ThrusterData.size()*sizeof(ThrustCmdStruct),
                                                reinterpret_cast<uint8_t*> (IncomingCmdBuffer));

    //! - Check if message has already been read, if stale return
    if(prevCommandTime==LocalHeader.WriteClockNanos) {
        return;
    }
    prevCommandTime = LocalHeader.WriteClockNanos;
    
    //! - Set the NewThrustCmds vector.  Using the data() method for raw speed
    double *CmdPtr;
    for(i=0, CmdPtr = NewThrustCmds.data(); i<ThrusterData.size();
        CmdPtr++, i++)
    {
        *CmdPtr = IncomingCmdBuffer[i].OnTimeRequest;
    }
    
}

/*! This method is used to read the new commands vector and set the thruster
 firings appropriately.  It assumes that the ReadInputs method has already been
 run successfully.  It honors all previous thruster firings if they are still
 active.  Note that for unit testing purposes you can insert firings directly
 into NewThrustCmds.
 @return void
 @param CurrentTime The current simulation time converted to a double
 */
void ThrusterDynamics::ConfigureThrustRequests(double CurrentTime)
{
    //! Begin method steps
    std::vector<ThrusterConfigData>::iterator it;
    std::vector<double>::iterator CmdIt;
    std::vector<ThrusterTimePair>::iterator PairIt;
    //! - Iterate through the list of thruster commands that we read in.
    for(CmdIt=NewThrustCmds.begin(), it=ThrusterData.begin();
        it != ThrusterData.end(); it++, CmdIt++)
    {
        if(*CmdIt > it->MinOnTime) /// - Check to see if we have met minimum for each thruster
        {
            //! - For each case where we are above the minimum firing request, reset the thruster
            it->ThrustOps.ThrusterStartTime = CurrentTime;
            it->ThrustOps.ThrustOnCmd = *CmdIt;
            it->ThrustOps.ThrustOnRampTime = 0.0;
            it->ThrustOps.ThrustOnSteadyTime = 0.0;
            it->ThrustOps.ThrustOffRampTime = 0.0;
            it->ThrustOps.PreviousIterTime = CurrentTime;
            if(it->ThrustOps.ThrustFactor > 0.0) /// - Check to see if we are already firing for active thrusters
            {
                for(PairIt = it->ThrusterOnRamp.begin();
                    PairIt != it->ThrusterOnRamp.end(); PairIt++)
                {
                    //! - Find the point in the thrust curve to start with if we are already in the middle of a firing
                    if(PairIt->ThrustFactor <= PairIt->ThrustFactor)
                    {
                        it->ThrustOps.ThrustOnRampTime = PairIt->TimeDelta;
                        break;
                    }
                }
            }
        }
        //! After we have assigned the firing to the internal thruster, zero the command request.
        *CmdIt = 0.0;
    }
    
}

/*! This method is used to get the current force for a thruster firing.  It uses
 the configuration data associated with a given thruster and the current clock
 time to determine what state and force the thruster should be in.
 @return void
 @param CurrentThruster Pointer to the configuration data for a given thruster
 @param CurrentTime The current simulation clock time converted to a double
 */
void ThrusterDynamics::ComputeThrusterFire(ThrusterConfigData *CurrentThruster,
                                           double CurrentTime)
{
    //! Begin method steps
    std::vector<ThrusterTimePair>::iterator it;
    ThrusterOperationData *ops = &(CurrentThruster->ThrustOps);
    //! - Set the current ramp time for the thruster firing
    double LocalOnRamp = (CurrentTime - ops->PreviousIterTime) +
    ops->ThrustOnRampTime;
    
    //! - Iterate through the on-ramp for the thruster data to find where we are in ramp
    for(it = CurrentThruster->ThrusterOnRamp.begin();
        it != CurrentThruster->ThrusterOnRamp.end(); it++)
    {
        //! - If the current on-time is less than the ramp delta, set that ramp thrust factor
        if(LocalOnRamp < it->TimeDelta)
        {
            ops->ThrustFactor = it->ThrustFactor;
            ops->ThrustOnRampTime = LocalOnRamp;
            ops->PreviousIterTime = CurrentTime;
            return;
        }
    }
    //! - If we did not find the current time in the on-ramp, then we are at steady-state
    ops->ThrustOnSteadyTime += (CurrentTime - ops->PreviousIterTime);
    ops->PreviousIterTime = CurrentTime;
    ops->ThrustFactor = 1.0;
}

/*! This method is used to go through the process of shutting down a thruster
 once it has been commanded off.  It uses the configuration data associated with
 a given thruster and the current clock time to turn off the thruster according
 to the ramp profile.
 @return void
 @param CurrentThruster Pointer to the configuration data for a given thruster
 @param CurrentTime The current simulation clock time converted to a double
 */
void ThrusterDynamics::ComputeThrusterShut(ThrusterConfigData *CurrentThruster,
                                           double CurrentTime)
{
    //! Begin method steps
    std::vector<ThrusterTimePair>::iterator it;
    ThrusterOperationData *ops = &(CurrentThruster->ThrustOps);
    
    //! - Set the current off-ramp time based on the previous clock time and now
    double LocalOffRamp = (CurrentTime - ops->PreviousIterTime) +
    ops->ThrustOffRampTime;
    //! - Iterate through the off-ramp to find the place where we are in the shutdown ramp
    for(it = CurrentThruster->ThrusterOffRamp.begin();
        it != CurrentThruster->ThrusterOffRamp.end(); it++)
    {
        //! - Once we find the location in the off-ramp, set that thrust factor to current
        if(LocalOffRamp < it->TimeDelta)
        {
            ops->ThrustFactor = it->ThrustFactor;
            ops->ThrustOffRampTime = LocalOffRamp;
            return;
        }
    }
    //! - If we did not find the location in the off-ramp, we've reached the end state and zero thrust
    ops->ThrustFactor = 0.0;
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
void ThrusterDynamics::ComputeDynamics(MassPropsData *Props,
                                       OutputStateData *Bstate, double CurrentTime)
{
    std::vector<ThrusterConfigData>::iterator it;
    ThrusterOperationData *ops;
    double SingleThrusterForce[3];
    double SingleThrusterTorque[3];
    double CoMRelPos[3];
    
    //! Begin method steps
    //! - Zero out the structure force/torque for the thruster set
    memset(StrForce, 0x0, 3*sizeof(double));
    memset(StrTorque, 0x0, 3*sizeof(double));
    
    //! - Iterate through all of the thrusters to aggregate the force/torque in the system
    for(it=ThrusterData.begin(); it != ThrusterData.end(); it++)
    {
        ops = &it->ThrustOps;
        //! - For each thruster see if the on-time is still valid and if so, call ComputeThrusterFire()
        if((ops->ThrustOnCmd + ops->ThrusterStartTime  - CurrentTime) >= 0.0 &&
           ops->ThrustOnCmd > 0.0)
        {
            ComputeThrusterFire(&(*it), CurrentTime);
        }
        //! - If we are not actively firing, continue shutdown process for active thrusters
        else if(ops->ThrustFactor > 0.0)
        {
            ComputeThrusterShut(&(*it), CurrentTime);
        }
        //! - For each thruster, aggregate the current thrust direction into composite structural force
        v3Scale(it->MaxThrust*ops->ThrustFactor, it->ThrusterDirection.data(),
                SingleThrusterForce);
        v3Add(StrForce, SingleThrusterForce, StrForce);
        //! - Compute the center-of-mass relative torque and aggregate into the composite structural torque
        v3Subtract(it->ThrusterLocation.data(), Props->CoM, CoMRelPos);
        v3Cross(CoMRelPos, SingleThrusterForce, SingleThrusterTorque);
        v3Add(StrTorque, SingleThrusterTorque, StrTorque);
    }
    //! - Once all thrusters have been checked, convert the structural force/torque to body for API
    m33MultV3(Props->T_str2Bdy, StrForce, BodyForce);
    m33MultV3(Props->T_str2Bdy, StrTorque, BodyTorque);
    
}

/*! This method is the main cyclical call for the scheduled part of the thruster
 dynamics model.  It reads the current commands array and sets the thruster
 configuration data based on that incoming command set.  Note that the main
 dynamical method (ComputeDynamics()) is not called here and is intended to be
 called from the dynamics plant in the system
 @return void
 @param CurrentSimNanos The current simulation time in nanoseconds
 */
void ThrusterDynamics::UpdateState(uint64_t CurrentSimNanos)
{
    //! Begin method steps
    //! - Read the inputs and then call ConfigureThrustRequests to set up dynamics
    ReadInputs();
    ConfigureThrustRequests(CurrentSimNanos*1.0E-9);
    
}
