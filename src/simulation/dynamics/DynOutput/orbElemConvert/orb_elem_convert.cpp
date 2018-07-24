/*
 ISC License

 Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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
#include "dynamics/DynOutput/orbElemConvert/orb_elem_convert.h"
#include "architecture/messaging/system_messaging.h"
#include <cstring>
#include <iostream>
#include "simMessages/spicePlanetStateSimMsg.h"
#include "utilities/bsk_Print.h"

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
    inputsGood = false;
	useEphemFormat = false;
    r_N[0] = r_N[1] = r_N[2] = 0.0;
    v_N[0] = v_N[1] = v_N[2] = 0.0;
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
	stateMsgSize = useEphemFormat ? sizeof(SpicePlanetStateSimMsg) :
		sizeof(SCPlusStatesSimMsg);
	std::string stateMsgType = useEphemFormat ? "SpicePlanetStateSimMsg" :
		"SCPlusStatesSimMsg";
    uint64_t OutputSize = Elements2Cart ? stateMsgSize :
    sizeof(classicElements);
    std::string messageType = Elements2Cart ? useEphemFormat ? "SpicePlanetStateSimMsg" : "SCPlusStatesSimMsg" : "classicElements";
    
    StateOutMsgID = SystemMessaging::GetInstance()->
        CreateNewMessage( OutputDataString, OutputSize, OutputBufferCount,
        messageType, moduleID);
    
}

/*! This method links up the desired input method with whoever created it.
 @return void
 */
void OrbElemConvert::CrossInit()
{
    uint64_t inputSize = Elements2Cart ? sizeof(classicElements) :
        stateMsgSize;
    StateInMsgID = SystemMessaging::GetInstance()->subscribeToMessage(
                                                                      StateString, inputSize, moduleID);
    if(StateInMsgID < 0)
    {
        BSK_PRINT(MSG_WARNING, "Did not find a valid message with name: %s", StateString.c_str());
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
    
    SCPlusStatesSimMsg statesIn;
	SpicePlanetStateSimMsg planetIn;
	uint8_t *msgPtr = useEphemFormat ? reinterpret_cast<uint8_t *> (&planetIn) :
		reinterpret_cast<uint8_t *> (&statesIn);
    //! Begin method steps
    //! - If it is outputting cartesian, create a StateData struct and write out.
    //! - If it is outputting elements, just write the current elements out
    if(Elements2Cart)
    {
		if (useEphemFormat)
		{
			memset(&planetIn, 0x0, sizeof(SpicePlanetStateSimMsg));
			memcpy(planetIn.PositionVector, r_N, 3 * sizeof(double));
			memcpy(planetIn.VelocityVector, v_N, 3 * sizeof(double));
		}
		else
		{
			memset(&statesIn, 0x0, sizeof(SCPlusStatesSimMsg));
			memcpy(statesIn.r_BN_N, r_N, 3 * sizeof(double));
			memcpy(statesIn.v_BN_N, v_N, 3 * sizeof(double));
		}
        SystemMessaging::GetInstance()->WriteMessage(StateOutMsgID, CurrentClock,
                                                     stateMsgSize, msgPtr, moduleID);
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
    SCPlusStatesSimMsg LocalState;
	SpicePlanetStateSimMsg localPlanet;
    SingleMessageHeader LocalHeader;

	uint8_t *msgPtr = useEphemFormat ? reinterpret_cast<uint8_t *> (&localPlanet) :
		reinterpret_cast<uint8_t *> (&LocalState);
    
    //! - Set the input pointer and size appropriately based on input type
    uint8_t *InputPtr = Elements2Cart ? reinterpret_cast<uint8_t *>
    (&LocalElements) : msgPtr;
    uint64_t InputSize = Elements2Cart ? sizeof(classicElements) :
    stateMsgSize;
    //! - Read the input message into the correct pointer
    inputsGood = SystemMessaging::GetInstance()->ReadMessage(StateInMsgID, &LocalHeader,
                                                InputSize, InputPtr, moduleID);
    //! - Depending on switch, either write into CurrentElem or pos/vel
    if(Elements2Cart)
    {
        memcpy(&CurrentElem, &LocalElements, sizeof(classicElements));
    }
    else
    {
		if (useEphemFormat)
		{
			memcpy(r_N, localPlanet.PositionVector, 3 * sizeof(double));
			memcpy(v_N, localPlanet.VelocityVector, 3 * sizeof(double));
		}
		else
		{
			memcpy(r_N, LocalState.r_BN_N, 3 * sizeof(double));
			memcpy(v_N, LocalState.v_BN_N, 3 * sizeof(double));
		}
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
    if(Elements2Cart && inputsGood)
    {
        Elements2Cartesian();
    }
    else if(inputsGood)
    {
        Cartesian2Elements();
    }
    //! Write out the current output for current time
    WriteOutputMessages(CurrentSimNanos);
}
