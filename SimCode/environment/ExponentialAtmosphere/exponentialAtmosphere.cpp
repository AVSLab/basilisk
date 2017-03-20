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
#include "../../dynamics/_GeneralModuleFiles/stateData.h"
#include "../../_GeneralModuleFiles/sys_model.h"
#include "../spice/spice_planet_state.h"
#include "../../dynamics/spacecraftPlus/spacecraftPlus.h"
#include "../../dynamics/spacecraftPlus/spacecraftPlusMsg.h"
#include <Eigen/Dense>
#include <vector>

#include "environment/spice/spice_interface.h"
#include <cstring>
#include <iostream>
#include <cmath>

ExponentialAtmosphere::ExponentialAtmosphere()
{
    this->planetName = "Earth";
    this->planetPosInMsgName = "spice_planet_output_data";
    this->OutputBufferCount = 2;
    //! Set the default atmospheric properties to those of Earth
    this->atmosphereProps.baseDensity = 1.217;
    this->atmosphereProps.scaleHeight = 8500.0; //  Meters
    this->atmosphereProps.planetRadius = 6371.008 * 1000.0;
    this->localAtmoTemp = 293.0; // Placeholder value from http://nssdc.gsfc.nasa.gov/planetary/factsheet/earthfact.html
    this->relativePos.fill(0.0);
    this->scStateInMsgNames.clear();
    this->tmpAtmo.neutralDensity = this->atmosphereProps.baseDensity;
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
void ExponentialAtmosphere::AddSpacecraftToModel(std::string tmpScMsgName){
  std::string tmpAtmoMsgName;
  this->scStateInMsgNames.push_back(tmpScMsgName);
  tmpAtmoMsgName = "atmo_dens"+ std::to_string(this->scStateInMsgNames.size()-1)+"_data";
  this->atmoDensOutMsgNames.push_back(tmpAtmoMsgName);
  return;
}

void ExponentialAtmosphere::SelfInit()
{
    std::string tmpAtmoMsgName;
    uint64_t tmpAtmoMsgId;
    //! Begin method steps
    std::vector<std::string>::iterator it;
    std::vector<std::string>::iterator nameIt;

    for(it = this->atmoDensOutMsgNames.begin(); it!=this->atmoDensOutMsgNames.end(); it++){
      tmpAtmoMsgId = SystemMessaging::GetInstance()->CreateNewMessage(*it, sizeof(AtmoOutputData), this->OutputBufferCount, "AtmoOutputData", moduleID);

      this->atmoDensOutMsgIds.push_back(tmpAtmoMsgId);
    }

    return;
}

void ExponentialAtmosphere::SetBaseDensity(double newBaseDens){
  this->atmosphereProps.baseDensity = newBaseDens;
  return;
}

void ExponentialAtmosphere::SetScaleHeight(double newScaleHeight){
  this->atmosphereProps.scaleHeight = newScaleHeight;
}

void ExponentialAtmosphere::SetPlanetRadius(double newPlanetRadius){
  this->atmosphereProps.planetRadius = newPlanetRadius;
}

void ExponentialAtmosphere::SetPlanet(std::string newPlanetName){
  this->planetName = newPlanetName;
  double newBaseDens = 0; // In kg/m^3
  double newScaleHeight = 0; // In meters

  if(newPlanetName.compare("Earth") == 0){
    newBaseDens = 1.217;
    newScaleHeight = 8500.0;
    SetBaseDensity(newBaseDens);
    SetScaleHeight(newScaleHeight);
  } else if(newPlanetName.compare("Mars")==0){
    newBaseDens = 0.020;
    newScaleHeight = 11000.0;
    SetPlanetRadius(3389.5 * 1000.0);
    SetBaseDensity(newBaseDens);
    SetScaleHeight(newScaleHeight);
  } else if (newPlanetName.compare("Venus")==0){
    SetPlanetRadius(6051.8 * 1000.0);
    newBaseDens = 65.0;
    newScaleHeight = 15900.0;
    SetBaseDensity(newBaseDens);
    SetScaleHeight(newScaleHeight);
  } else{
    std::cout<<"Error: Planet "<< newPlanetName<<" not found. Either undefined or non-atmospheric."<<std::endl;
  }
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

  std::vector<std::string>::iterator it;
  for(it = scStateInMsgNames.begin(); it!=scStateInMsgNames.end(); it++){
    std::cout<<"CrossInit: "<<*it<<std::endl;
    this->scStateInMsgIds.push_back(SystemMessaging::GetInstance()->subscribeToMessage(*it, sizeof(SCPlusOutputStateData), moduleID));
  }
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
    std::vector<uint64_t>::iterator it;
    std::vector<AtmoOutputData>::iterator atmoIt;
    atmoIt = atmoOutBuffer.begin();
    for(it = atmoDensOutMsgIds.begin(); it!= atmoDensOutMsgIds.end(); it++, atmoIt++){
      tmpAtmo = *atmoIt;
      //std::cout<<"WriteMsg: "<<tmpAtmo.neutralDensity<<std::endl;
      SystemMessaging::GetInstance()->WriteMessage(*it,
                                                  CurrentClock,
                                                  sizeof(AtmoOutputData),
                                                  reinterpret_cast<uint8_t*>(&tmpAtmo),
                                                  moduleID);
    }
}
    /*SystemMessaging::GetInstance()->WriteMessage(this->thrusterOutMsgIds.at(idx),
                                                CurrentClock,
                                                sizeof(ThrusterOutputData),
                                                reinterpret_cast<uint8_t*>(&tmpThruster),
                                                moduleID);*/


/*! This method is used to read the incoming command message and set the
 associated command structure for operating the thrusters.
 @return void
 */
bool ExponentialAtmosphere::ReadInputs()
{
  SCPlusOutputStateData tmpState;
  //! Begin method steps
  SingleMessageHeader localHeader;
  memset(&this->bodyState, 0x0, sizeof(SpicePlanetState));
  memset(&tmpState, 0x0, sizeof(SCPlusOutputStateData));
  scStates.clear();
  if(scStateInMsgIds[0] >= 0)
  {
    //! Iterate over spacecraft message ids
    std::vector<uint64_t>::iterator it;
    for(it = scStateInMsgIds.begin(); it!= scStateInMsgIds.end(); it++){
      SystemMessaging::GetInstance()->ReadMessage(*it, &localHeader,
                                                  sizeof(SCPlusOutputStateData),
                                                  reinterpret_cast<uint8_t*>(&tmpState),
                                                  moduleID);
    this->scStates.push_back(tmpState);
      }
  }
  if(planetPosInMsgId >= 0)
  {
      SystemMessaging::GetInstance()->ReadMessage(this->planetPosInMsgId , &localHeader,
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
void ExponentialAtmosphere::updateLocalAtmo(double currentTime)
{
    double tmpDensity = 0.0;
    double tmpAltitude = 0.0;
    std::vector<SCPlusOutputStateData>::iterator it;
    AtmoOutputData tmpData;
    uint64_t atmoInd = 0;
    this->atmoOutBuffer.clear();
    for(it = scStates.begin(); it != scStates.end(); it++, atmoInd++){
      this->updateRelativePos(this->bodyState, *it);
      tmpPosMag = this->relativePos.norm();
      tmpAltitude = tmpPosMag - this->atmosphereProps.planetRadius;

      tmpDensity = this->atmosphereProps.baseDensity * exp(-1.0 * tmpAltitude / this->atmosphereProps.scaleHeight);
      //std::cout<<tmpPosMag<<","<<this->currentAlt<<","<<tmpDensity<<","<<std::endl;
      tmpData.neutralDensity = tmpDensity;
      tmpData.localTemp = 300.0;

      this->atmoOutBuffer.push_back(tmpData);
    }
      return;
}

void ExponentialAtmosphere::updateRelativePos(SpicePlanetState& planetState, SCPlusOutputStateData& scState)
{
    uint64_t iter = 0;
    if(planetPosInMsgId >= 0)
    {
      for(iter = 0; iter < 3; iter++)
      {
        //std::cout<<"Planet State: "<<planetState.PositionVector[iter]<<std::endl;
        //std::cout<<"SC State:"<<scState.r_BN_N[iter]<<std::endl;
        this->relativePos(iter,0) = scState.r_BN_N[iter] - planetState.PositionVector[iter];

        //std::cout<<relativePos[iter,0]<<std::endl;
      }
    }
    else{
      this->relativePos[iter,0] = scState.r_BN_N[iter];
    }
    //std::cout<<"Relative Pos: "<<this->relativePos <<std::endl;
    return;
}

/*! This method is the main cyclical call for the scheduled part of the thruster
 dynamics model.  It reads the current commands array and sets the thruster
 configuration data based on that incoming command set.  Note that the main
 dynamical method (updateDynamics()) is not called here and is intended to be
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
        updateLocalAtmo(CurrentSimNanos*1.0E-9);
    }
    WriteOutputMessages(CurrentSimNanos);

}
