#include "dynamics/SixDofEOM/six_dof_eom.h"
#include "dynamics/DynOutput/orb_elem_convert.h"
#include "architecture/messaging/system_messaging.h"
#include <cstring>
#include <iostream>

//! The constructor.  Note that you may want to overwrite the message names.
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

//! The destructor.  So tired of typing this.
OrbElemConvert::~OrbElemConvert()
{
    return;
}

/*! This method initializes the messages that are associated with the object.
 It does detect which "direction" it is and sets the sizing appropriately.
 @return void
 */
void OrbElemConvert::SelfInit()
{
    
    //! Begin method steps
    //! - Determine what the size of the output should be and create the message
    uint64_t OutputSize = Elements2Cart ? sizeof(OutputStateData) :
    sizeof(classicElements);
    std::string messageType = Elements2Cart ? "OutputStateData" :
        "classicElements";
    
    StateOutMsgID = SystemMessaging::GetInstance()->
        CreateNewMessage( OutputDataString, OutputSize, OutputBufferCount,
        messageType, moduleID);
    
}

/*! This method links up the desired input method with whoever created it.
 @return void
 */
void OrbElemConvert::CrossInit()
{
    StateInMsgID = SystemMessaging::GetInstance()->FindMessageID(StateString);
    if(StateInMsgID < 0)
    {
        std::cerr << "WARNING: Did not find a valid message with name: ";
        std::cerr << StateString << "  :" << __FILE__ << std::endl;
    }
}

/*! This method writes the output data out into the messaging system.  It does
 switch depending on whether it is outputting cartesian position/velocity or
 orbital elements.
 @return void
 @param CurrentClock The current time in the system for output stamping
 */
void OrbElemConvert::WriteOutputMessages(uint64_t CurrentClock)
{
    
    OutputStateData LocalState;
    //! Begin method steps
    //! - If it is outputting cartesian, create a StateData struct and write out.
    //! - If it is outputting elements, just write the current elements out
    if(Elements2Cart)
    {
        memset(&LocalState, 0x0, sizeof(OutputStateData));
        memcpy(LocalState.r_N, r_N, 3*sizeof(double));
        memcpy(LocalState.v_N, v_N, 3*sizeof(double));
        SystemMessaging::GetInstance()->WriteMessage(StateOutMsgID, CurrentClock,
                                                     sizeof(OutputStateData), reinterpret_cast<uint8_t*> (&LocalState), moduleID);
    }
    else
    {
        SystemMessaging::GetInstance()->WriteMessage(StateOutMsgID, CurrentClock,
                                                     sizeof(classicElements), reinterpret_cast<uint8_t*> (&CurrentElem), moduleID);
    }
    
}

/*! The name kind of says it all right?  Converts CurrentElem to pos/vel.
 @return void
 */
void OrbElemConvert::Elements2Cartesian()
{
    elem2rv(mu, &CurrentElem, r_N, v_N);
}

/*! The name kind of says it all right?  Converts pos/vel to CurrentElem.
 @return void
 */
void OrbElemConvert::Cartesian2Elements()
{
    rv2elem(mu, r_N, v_N, &CurrentElem);
}

/*! This method reads the input message in from the system and sets the
 appropriate parameters based on which direction the module is running
 @return void
 */
void OrbElemConvert::ReadInputs()
{
    //! Begin method steps
    //! - Quit if we don't have a valid input ID.
    if(StateInMsgID < 0)
    {
        return;
    }
    classicElements LocalElements;
    OutputStateData LocalState;
    SingleMessageHeader LocalHeader;
    
    //! - Set the input pointer and size appropriately based on input type
    uint8_t *InputPtr = Elements2Cart ? reinterpret_cast<uint8_t *>
    (&LocalElements) : reinterpret_cast<uint8_t *> (&LocalState);
    uint64_t InputSize = Elements2Cart ? sizeof(classicElements) :
    sizeof(OutputStateData);
    //! - Read the input message into the correct pointer
    SystemMessaging::GetInstance()->ReadMessage(StateInMsgID, &LocalHeader,
                                                InputSize, InputPtr);
    //! - Depending on switch, either write into CurrentElem or pos/vel
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

/*! This method is the main carrier for the conversion routine.  If it detects
 that it needs to re-init (direction change maybe) it will re-init itself.
 The it either converts elements to cartesian or cartesian to elements.
 @return void
 @param CurrentSimNanos The current simulation time for system
 */
void OrbElemConvert::UpdateState(uint64_t CurrentSimNanos)
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
    if(Elements2Cart)
    {
        Elements2Cartesian();
    }
    else
    {
        Cartesian2Elements();
    }
    //! Write out the current output for current time
    WriteOutputMessages(CurrentSimNanos);
}
