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
#include "environment/spice/spice_interface.h"
#include <cstring>
#include <iostream>
#include <cmath>
#include "../dynamics/spacecraftPlus/spacecraftPlusMsg.h"

ExponentialAtmosphere::ExponentialAtmosphere()
{
    this->planetName = "Earth"
    this->atmoDensOutMsgName = "base_atmo_props"
    this->scStateInMsgName = "inertial_state_output";
    this->planetPosInMsgName = "spice_planet_output_data";
    //! Set the default atmospheric properties to those of Earth
    this->atmosphereProps.baseDensity = 1.207;
    this->atmosphereProps.scaleHeight = 8500.0;
    this->atmosphereProps.planetRadius = 6.371 * 1000000.0;
    this->localAtmoDens = this->atmosphereProps.baseDensity;
    this->localAtmoTemp = 293.0; // Placeholder value from http://nssdc.gsfc.nasa.gov/planetary/factsheet/earthfact.html

    this->relativePos.fill(0.0);
    this->tmpAtmo.neutralDensity = this->localAtmoDens;
    this->tmpAtmo.localTemp = this->localAtmoTemp;
    return;
}

/*! The destructor.*/
ExponentialAtmosphere::~ExponentialAtmosphere()
{
    return;
}

/*! This method is used to clear out the current thruster states and make sure
 that the overall model is ready for firing
 @return void
 */
void ExponentialAtmosphere::SelfInit()
{
    //! Begin method steps
    this->atmoOutMsgId = SystemMessaging::GetInstance()->CreateNewMessage(this->atmoOutMsgName, sizeof(AtmoOutputData), this->OutputBufferCount, this->atmoOutMsgName, moduleID);
    return;
}

/*! This method is used to connect the input command message to the thrusters.
 It sets the message ID based on what it finds for the input string.  If the
 message is not successfully linked, it will warn the user.
 @return void
 */
void ExponentialAtmosphere::CrossInit()
{
  this->planetPosInMsgId = SystemMessaging::GetInstance()->subscribeToMessage(
      this->planetPosInMsgName, sizeof(SpicePlanetState), moduleID);
  this->scStateInMsgId = SystemMessaging::GetInstance()->subscribeToMessage(
      this->scStateInMsgName, sizeof(SCPlusOutputStateData), moduleID);

  return;
}


/*! This method is here to write the output message structure into the specified
 message.  It is currently blank but we will certainly have an output message
 soon.  If it is already here, bludgeon whoever added it and didn't fix the
 comment.sizeof(ThrusterOutputData)
 @param CurrentClock The current time used for time-stamping the message
 @return void
 */
void ExponentialAtmosphere::WriteOutputMessages(uint64_t CurrentClock)
{
    AtmoOutputData tmpAtmo;
    tmpAtmo.neutralDensity = this->localAtmoDens;
    tmpAtmo.localTemp = this->localAtmoTemp;
    //std::cout<<this->atmoOutMsgIds.at(0)<<std::endl;
    SystemMessaging::GetInstance()->WriteMessage(this->atmoOutMsgId.at(0),
                                                CurrentClock,
                                                sizeof(AtmoOutputData),
                                                reinterpret_cast<uint8_t*>(&tmpAtmo),
                                                moduleID);
    /*SystemMessaging::GetInstance()->WriteMessage(this->thrusterOutMsgIds.at(idx),
                                                CurrentClock,
                                                sizeof(ThrusterOutputData),
                                                reinterpret_cast<uint8_t*>(&tmpThruster),
                                                moduleID);*/
}


/*! This method is used to read the incoming command message and set the
 associated command structure for operating the thrusters.
 @return void
 */
bool ExponentialAtmosphere::ReadInputs()
{
  //! Begin method steps
  SingleMessageHeader localHeader;
  memset(&this->sunState, 0x0, sizeof(SpicePlanetState));
  memset(&this->inertialState, 0x0, sizeof(SCPlusOutputStateData));
  if(inputStateID >= 0)
  {
      SystemMessaging::GetInstance()->ReadMessage(inputStateID, &localHeader,
                                                  sizeof(SCPlusOutputStateData), reinterpret_cast<uint8_t*>(&this->scState), modueleID);
  }
  if(inputSunID >= 0)
  {
      SystemMessaging::GetInstance()->ReadMessage(inputSunID, &localHeader,
                                                  sizeof(SpicePlanetState), reinterpret_cast<uint8_t*>(&this->bodyState), moduleID);
  }
  return(true);

}

/*! This method is used to get the current force for a thruster firing.  It uses
 the configuration data associated with a given thruster and the current clock
 time to determine what state and force the thruster should be in.
 @return void
 @param CurrentThruster Pointer to the configuration data for a given thruster
 @param CurrentTime The current simulation clock time converted to a double
 */
void ExponentialAtmosphere::ComputeLocalAtmo(double currentTime)
{
    this->ComputeRelativePos( this->scState.r_BN_N, this->bodyState.PositionVector);
    tmpPosMag = this->currentPosition.norm();
    this->currentAlt = tmpPosMag - this->atmosphereProps.planetRadius;
    this->localAtmoDens = this->atmosphereProps.baseDensity * exp(this->currentAlt / this->atmosphereProps.scaleHeight);
    return;
}

void ExponentialAtmosphere::ComputeRelativePos(SpicePlanetState planetState, SCPlusOutputStateData scState)
{
    uint64_t iter = 0;
    for(iterator <= 3; iterator++)
    {
      this->relativePos[iterator] = scState.r_BN_N[iterator] - planetState.PositionVector[iterator];
    }
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
void ExponentialAtmosphere::UpdateState(uint64_t CurrentSimNanos)
{
    //! Begin method steps
    //! - Read the inputs and then call ConfigureThrustRequests to set up dynamics
    if(this->ReadInputs())
    {
        ComputeLocalAtmo(CurrentSimNanos*1.0E-9);
    }
    WriteOutputMessages(CurrentSimNanos);

}
