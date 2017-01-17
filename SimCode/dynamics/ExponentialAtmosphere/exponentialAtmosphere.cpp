/*
 ISC License

 Copyright (c) 2016-2017, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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


#include "exponentialAtmosphere.h"
#include "architecture/messaging/system_messaging.h"
#include "utilities/linearAlgebra.h"
#include "utilities/astroConstants.h"
#include "../ADCSAlgorithms/effectorInterfaces/errorConversion/vehEffectorOut.h"
#include "../ADCSAlgorithms/ADCSUtilities/ADCSAlgorithmMacros.h"
#include <cstring>
#include <iostream>
#include <cmath>

exponentialAtmosphere::exponentialAtmosphere()
{
    OutputDataString = "atmosphere_output_states";
    OutputBufferCount = 2;
    StateOutMsgID = -1;
    //! Set the default atmospheric properties to those of Earth
    this->atmosphereProps.baseDensity = 1.207;
    this->atmosphereProps.scaleHeight = 8500.0;

    this->localAtmoDens = this->atmosphereProps.baseDensity;
    this->localAtmoTemp = 293.0; // Placeholder value from http://nssdc.gsfc.nasa.gov/planetary/factsheet/earthfact.html

    this->currentPosition.fill(0.0);
    this->tmpAtmo.neutralDensity = this->localAtmoDens;
    this->tmpAtmo.localTemp = this->localAtmoTemp;
    return;
}

/*! The destructor.*/
exponentialAtmosphere::~exponentialAtmosphere()
{
    return;
}

/*! This method is used to clear out the current thruster states and make sure
 that the overall model is ready for firing
 @return void
 */
void exponentialAtmosphere::SelfInit()
{
    //! Begin method steps
    SystemMessaging *messageSys = SystemMessaging::GetInstance(); // Gets a pointer to the messaging system

  	// Reserve a message ID for each reaction wheel config output message
  	uint64_t tmpAtmoMsgId;
  	std::string tmpAtmoMsgName;

  	tmpAtmoMsgName = "std_atmosphere_data";
  	tmpAtmoMsgId = messageSys->CreateNewMessage(tmpAtmoMsgName, sizeof(AtmoOutputData), OutputBufferCount, "AtmosphereProperties", moduleID);

    return;
}

/*! This method is used to connect the input command message to the thrusters.
 It sets the message ID based on what it finds for the input string.  If the
 message is not successfully linked, it will warn the user.
 @return void
 */
void exponentialAtmosphere::CrossInit()
{
}


/*! This method is here to write the output message structure into the specified
 message.  It is currently blank but we will certainly have an output message
 soon.  If it is already here, bludgeon whoever added it and didn't fix the
 comment.sizeof(ThrusterOutputData)
 @param CurrentClock The current time used for time-stamping the message
 @return void
 */
void exponentialAtmosphere::WriteOutputMessages(uint64_t CurrentClock)
{
    int idx = 0;

    AtmoOutputData tmpAtmo;
    tmpAtmo.neutralDensity = this->localAtmoDens;
    tmpAtmo.localTemp = this->localAtmoTemp;

    SystemMessaging::GetInstance()->WriteMessage(this->StateOutMsgID,
                                                CurrentClock,
                                                sizeof(AtmoOutputData),
                                                reinterpret_cast<uint8_t*>(&tmpAtmo),
                                                moduleID);
}


/*! This method is used to read the incoming command message and set the
 associated command structure for operating the thrusters.
 @return void
 */
bool exponentialAtmosphere::ReadInputs()
{
  this->currentPosition = this->hubPos->getState();
  return(true);

}


void exponentialAtmosphere::linkInStates(DynParamManager& states)
{
    this->hubPos = states.getStateObject("hubPosition");
}

/*! This method is used to get the current force for a thruster firing.  It uses
 the configuration data associated with a given thruster and the current clock
 time to determine what state and force the thruster should be in.
 @return void
 @param CurrentThruster Pointer to the configuration data for a given thruster
 @param CurrentTime The current simulation clock time converted to a double
 */
void exponentialAtmosphere::ComputeLocalAtmo(double currentTime)
{
    tmpPosMag = this->currentPosition.norm();
    this->currentAlt = tmpPosMag - this->atmosphereProps.planetRadius;
    this->localAtmoDens = this->atmosphereProps.baseDensity *
    exp(this->currentAlt / this->atmosphereProps.scaleHeight);
    return;
}

/*! This method is the main cyclical call for the scheduled part of the thruster
 dynamics model.  It reads the current commands array and sets the thruster
 configuration data based on that incoming command set.  Note that the main
 dynamical method (ComputeDynamics()) is not called here and is intended to be
 called from the dynamics plant in the system
 @return void
 @param CurrentSimNanos The current simulation time in nanoseconds
 */
void exponentialAtmosphere::UpdateState(uint64_t CurrentSimNanos)
{
    //! Begin method steps
    //! - Read the inputs and then call ConfigureThrustRequests to set up dynamics
    if(ReadInputs())
    {
        ComputeLocalAtmo(CurrentSimNanos*1.0E-9);
    }
    WriteOutputMessages(CurrentSimNanos);

}
