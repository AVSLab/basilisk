
#include "dynamics/DynOutput/bore_ang_calc.h"
#include "architecture/messaging/system_messaging.h"
#include "utilities/linearAlgebra.h"
#include "utilities/rigidBodyKinematics.h"
#include <cstring>
#include <iostream>

//! The constructor.  Note that you have to overwrite the message names.
BoreAngCalc::BoreAngCalc()
{
    CallCounts = 0;
    StateString = "";
    celBodyString = "";
    OutputDataString = "";
    OutputBufferCount = 2;
    StateInMsgID = -1;
    AngOutMsgID = -1;
    ReinitSelf = false;
    memset(&localPlanet, 0x0, sizeof(SpicePlanetState));
    memset(&localState, 0x0, sizeof(OutputStateData));
    return;
}

//! The destructor.  So tired of typing this.
BoreAngCalc::~BoreAngCalc()
{
    return;
}

/*! This method initializes the messages that are associated with the object.
 @return void
 */
void BoreAngCalc::SelfInit()
{
    
    //! Begin method steps
    //! - Determine what the size of the output should be and create the message
    
    AngOutMsgID = SystemMessaging::GetInstance()->
        CreateNewMessage( OutputDataString, sizeof(AngOffValues), OutputBufferCount,
        "AngOffValues", moduleID);
    
}

/*! This method links up the desired input messages with whoever created them.
 @return void
 */
void BoreAngCalc::CrossInit()
{
    StateInMsgID = SystemMessaging::GetInstance()->subscribeToMessage(
                            StateString, sizeof(OutputStateData), moduleID);
    celInMsgID = SystemMessaging::GetInstance()->subscribeToMessage(celBodyString,
                            sizeof(SpicePlanetState), moduleID);
}

/*! This method writes the output data out into the messaging system.
 @return void
 @param CurrentClock The current time in the system for output stamping
 */
void BoreAngCalc::WriteOutputMessages(uint64_t CurrentClock)
{
    
    //! Begin method steps
    
    SystemMessaging::GetInstance()->WriteMessage(AngOutMsgID, CurrentClock,
        sizeof(AngOffValues), reinterpret_cast<uint8_t*> (&boresightAng),
        moduleID);
    
}

/*! This method reads the input messages in from the system and sets the
 appropriate parameters
 @return void
 */
void BoreAngCalc::ReadInputs()
{
    //! Begin method steps
    SingleMessageHeader localHeader;
    
    //! - Set the input pointer and size appropriately based on input type
    //! - Read the input message into the correct pointer
    SystemMessaging::GetInstance()->ReadMessage(StateInMsgID, &localHeader,
        sizeof(OutputStateData), reinterpret_cast<uint8_t*> (&localState));
    SystemMessaging::GetInstance()->ReadMessage(celInMsgID, &localHeader,
        sizeof(SpicePlanetState), reinterpret_cast<uint8_t*> (&localPlanet));
    
}

/*! This method computes the vector specified in the input file in the LVLH 
    reference frame of the spacecraft above the target celestial body.  This 
    is used later to compute how far off that vector is in an angular sense.
    @return void
*/
void BoreAngCalc::computeAxisPoint()
{
    double T_inrtl2Bdy[3][3];
    double T_Inrtl2Point[3][3];
    double T_Point2Bdy[3][3];
    double bdyBoreVec[3];
    double relPosVector[3];
    double relVelVector[3];
    double secPointVector[3];
    double primPointVector[3];
    
    MRP2C(localState.sigma, T_inrtl2Bdy);
    v3Subtract(localPlanet.PositionVector, localState.r_N, relPosVector);
    v3Subtract(localPlanet.VelocityVector, localState.v_N, relVelVector);
    v3Cross(relPosVector, relVelVector, secPointVector);
    v3Normalize(secPointVector, secPointVector);
    v3Normalize(relPosVector, primPointVector);
    v3Copy(primPointVector, &(T_Inrtl2Point[0][0]));
    v3Cross(primPointVector, secPointVector, &(T_Inrtl2Point[2][0]));
    v3Normalize(&(T_Inrtl2Point[2][0]), &(T_Inrtl2Point[2][0]));
    v3Cross(&(T_Inrtl2Point[2][0]), &(T_Inrtl2Point[0][0]),
            &(T_Inrtl2Point[1][0]));
    m33MultM33t(T_inrtl2Bdy, T_Inrtl2Point, T_Point2Bdy);
    m33MultV3(localState.T_str2Bdy, strBoreVec, bdyBoreVec);
    m33tMultV3(T_Point2Bdy, bdyBoreVec, boreVecPoint);
    
}

/*! This method is the main carrier for the boresight calculation routine.  If it detects
 that it needs to re-init (direction change maybe) it will re-init itself.
 Then it will compute the angles away that the boresight is from the celestial target.
 @return void
 @param CurrentSimNanos The current simulation time for system
 */
void BoreAngCalc::UpdateState(uint64_t CurrentSimNanos)
{
    //! Begin method steps
    //! - If we need to reinit, call SelfInit, CrossInit, and set flag false
    if(ReinitSelf)
    {
        SelfInit();
        CrossInit();
        ReinitSelf = false;
    }
    //! - Read the input message and convert it over appropriately depending on switch
    ReadInputs();
    
    computeAxisPoint();
    
    //! Write out the current output for current time
    WriteOutputMessages(CurrentSimNanos);
}
