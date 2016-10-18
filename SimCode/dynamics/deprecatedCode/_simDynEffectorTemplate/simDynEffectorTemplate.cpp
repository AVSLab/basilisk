#include "simDynEffectorTemplate.h"
#include "architecture/messaging/system_messaging.h"
#include "utilities/linearAlgebra.h"
#include "utilities/astroConstants.h"
#include <cstring>
#include <cmath>
#include <iostream>

/*! This is the constructor.  It sets some defaul initializers that can be
overriden by the user.*/
simDynEffectorTemplate::simDynEffectorTemplate():
    sunEphmInMsgName("sun_planet_data")
    ,stateInMsgName("inertial_state_output")
    ,sunEphmInMsgID(-1)
    ,stateInMsgID(-1)
    ,stateInBuffer(OutputStateData())
{
    CallCounts = 0;
    memset(this->extForce_N, 0x0, 3*sizeof(double));
    memset(this->extForce_B, 0x0, 3*sizeof(double));
    memset(this->extTorquePntB_B, 0x0, 3*sizeof(double));
    return;
}

/*! The destructor.  Nothing of note is performed here*/
simDynEffectorTemplate::~simDynEffectorTemplate()
{
    return;
}

/*! This method is used to setup the module 
@return void
*/
void simDynEffectorTemplate::SelfInit()
{
    //! Begin method steps
    this->exampleOutMsgID = SystemMessaging::GetInstance()->
    CreateNewMessage(this->exampleOutMsgName, sizeof(double[3]),
                     2, "double", this->moduleID);
    
}

/*! This method is used to connect the input command message to the thrusters.
It sets the message ID based on what it finds for the input string.  If the
message is not successfully linked, it will warn the user.
@return void
*/
void simDynEffectorTemplate::CrossInit()
{
    //! Begin method steps
    //! - Find the message ID associated with the ephmInMsgID string.
    //! - Warn the user if the message is not successfully linked.
    this->sunEphmInMsgID = SystemMessaging::GetInstance()->subscribeToMessage(this->sunEphmInMsgName,
                                                                        sizeof(SpicePlanetState), this->moduleID);
    
    this->stateInMsgID = SystemMessaging::GetInstance()->subscribeToMessage(this->stateInMsgName, sizeof(OutputStateData), this->moduleID);
    
    // Delete: It is good practice to check that the module
    // successfully subscribed to it's incoming messages.
    if(this->sunEphmInMsgID < 0)
    {
        std::cerr << "WARNING: Did not find a valid message with name: ";
        std::cerr << sunEphmInMsgName << "  :" << __FILE__ << std::endl;
    }
    if(this->stateInMsgID < 0)
    {
        std::cerr << "WARNING: Did not find a valid message with name: ";
        std::cerr << stateInMsgName << "  :" << __FILE__ << std::endl;
    }
}

/*! This method is here to write the output message structure into the specified
message.  It is currently blank but we will certainly have an output message
soon.  If it is already here, bludgeon whoever added it and didn't fix the
comment.
@param CurrentClock The current time used for time-stamping the message
@return void
*/
void simDynEffectorTemplate::writeOutputMessages(uint64_t CurrentClock)
{

}

/*! This method is used to read the incoming ephmeris message and set the
associated buffer structure.
@return void
*/
void simDynEffectorTemplate::readInputs()
{
    bool dataGood;
    //! Begin method steps
    //! - Zero the command buffer and read the incoming command array
    SingleMessageHeader localHeader;
    
    memset(&localHeader, 0x0, sizeof(localHeader));
    if(sunEphmInMsgID >= 0)
    {
        memset(&sunEphmInBuffer, 0x0, sizeof(SpicePlanetState));
        dataGood = SystemMessaging::GetInstance()->ReadMessage(sunEphmInMsgID, &localHeader,
                                                               sizeof(SpicePlanetState),
                                                               reinterpret_cast<uint8_t*> (&sunEphmInBuffer));
        std::cout << &sunEphmInBuffer << std::endl;
    }
    if (stateInMsgID >= 0) {
        memset(&stateInBuffer, 0x0, sizeof(OutputStateData));
        dataGood = SystemMessaging::GetInstance()->ReadMessage(stateInMsgID, &localHeader,
                                                               sizeof(OutputStateData),
                                                               reinterpret_cast<uint8_t*> (&stateInBuffer));
        
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
void simDynEffectorTemplate::ComputeDynamics(MassPropsData *massPropsData, OutputStateData *bodyState, double currentTime)
{
    //! Begin method steps
    //! - Zero out the structure force/torque for the thruster set

    

    v3Copy(this->extForce_B, dynEffectorForce_B);
    v3Copy(this->extTorquePntB_B, dynEffectorTorquePntB_B);
}

void simDynEffectorTemplate::UpdateState(uint64_t CurrentSimNanos)
{
    this->readInputs();
}

/*! Computes the solar radiation force vector
*   using a lookup table given the current spacecraft attitude
*   and the position vector of the spacecraft to the sun.
*   Note: It is assumed that the solar radiation pressure decreases
*   quadratically with distance from sun (in AU)
*
@return void
@param s_B (m) Position vector to the Sun relative to the body frame
*/
void simDynEffectorTemplate::computeDummyData(double *s_B)
{
    /* Magnitude of position vector */
    double sunDist = v3Norm(s_B);
    sunDist += 0.0;

}
