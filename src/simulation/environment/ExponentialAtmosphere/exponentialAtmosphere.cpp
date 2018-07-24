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
#include <Eigen/Dense>
#include <vector>
#include "environment/spice/spice_interface.h"
#include <cstring>
#include <iostream>
#include <cmath>

#include "exponentialAtmosphere.h"
#include "architecture/messaging/system_messaging.h"
#include "utilities/linearAlgebra.h"
#include "utilities/astroConstants.h"
#include "utilities/bsk_Print.h"
#include "../../dynamics/_GeneralModuleFiles/stateData.h"
#include "../../_GeneralModuleFiles/sys_model.h"

/*! Initializer - sets the default properties to those of Earth.
*/
ExponentialAtmosphere::ExponentialAtmosphere()
{
    this->planetName = "Earth";
    this->planetPosInMsgName = "spice_planet_output_data";
    this->OutputBufferCount = 2;
    //! Set the default atmospheric properties to those of Earth
    this->atmosphereProps.baseDensity = 1.217;
    this->atmosphereProps.scaleHeight = 8500.0;
    this->atmosphereProps.planetRadius = 6371.008 * 1000.0;
    this->localAtmoTemp = 293.0; //! Placeholder value from http://nssdc.gsfc.nasa.gov/planetary/factsheet/earthfact.html
    this->relativePos.fill(0.0);
    this->scStateInMsgNames.clear();
    this->tmpAtmo.neutralDensity = this->atmosphereProps.baseDensity;
    this->tmpAtmo.localTemp = this->localAtmoTemp;
    this->planetPosInMsgId = -1;
    memset(&this->bodyState, 0x0, sizeof(SpicePlanetStateSimMsg));

    return;
}

/*! The destructor.*/
ExponentialAtmosphere::~ExponentialAtmosphere()
{
    return;
}

/*! This method is used to add additional spacecraft to the model density update. This allows
the use of a single exponentialAtmosphere instance for multiple spacecraft about a planet.
 @return void
 */
void ExponentialAtmosphere::addSpacecraftToModel(std::string tmpScMsgName){
  std::string tmpAtmoMsgName;
  this->scStateInMsgNames.push_back(tmpScMsgName);
  tmpAtmoMsgName = "atmo_dens"+ std::to_string(this->scStateInMsgNames.size()-1)+"_data";
  this->atmoDensOutMsgNames.push_back(tmpAtmoMsgName);
  return;
}

/*! SelfInit for this method creates a seperate density message for each of the spacecraft
that were added using AddSpacecraftToModel.
*/
void ExponentialAtmosphere::SelfInit()
{
    std::string tmpAtmoMsgName;
    uint64_t tmpAtmoMsgId;
    //! Begin method steps
    std::vector<std::string>::iterator it;
    std::vector<std::string>::iterator nameIt;

    for(it = this->atmoDensOutMsgNames.begin(); it!=this->atmoDensOutMsgNames.end(); it++){
      tmpAtmoMsgId = SystemMessaging::GetInstance()->CreateNewMessage(*it, sizeof(atmoPropsSimMsg), this->OutputBufferCount, "atmoPropsSimMsg", moduleID);

      this->atmoDensOutMsgIds.push_back(tmpAtmoMsgId);
    }

    return;
}

/*! Private "setter" method for changing the base density.*/
void ExponentialAtmosphere::setBaseDensity(double BaseDens){
  this->atmosphereProps.baseDensity = BaseDens;
  return;
}

/*! Private "setter" method for changing the scale height.*/
void ExponentialAtmosphere::setScaleHeight(double ScaleHeight){
  this->atmosphereProps.scaleHeight = ScaleHeight;
}

/*! Private "setter" method for changing the planet radius.*/
void ExponentialAtmosphere::setPlanetRadius(double PlanetRadius){
  this->atmosphereProps.planetRadius = PlanetRadius;
}

/*! Public "setter" method to change the planet referenced by the model.*/
void ExponentialAtmosphere::setPlanet(std::string PlanetName){
  this->planetName = PlanetName;
  double BaseDens = 0; // In kg/m^3
  double ScaleHeight = 0; // In meters

  if(PlanetName.compare("earth") == 0){
    BaseDens = 1.217;
    ScaleHeight = 8500.0;
    setBaseDensity(BaseDens);
    setScaleHeight(ScaleHeight);
} else if(PlanetName.compare("mars")==0){
    BaseDens = 0.020;
    ScaleHeight = 11100.0;
    setPlanetRadius(3389.5 * 1000.0);
    setBaseDensity(BaseDens);
    setScaleHeight(ScaleHeight);
} else if (PlanetName.compare("venus")==0){
    setPlanetRadius(6051.8 * 1000.0);
    BaseDens = 65.0;
    ScaleHeight = 15900.0;
    setBaseDensity(BaseDens);
    setScaleHeight(ScaleHeight);
  } else{
      BSK_PRINT(MSG_WARNING, "Planet %s not found. Either undefined or non-atmospheric. Please define other atmospheric parameters.", PlanetName.c_str());
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
  this->planetPosInMsgName, sizeof(SpicePlanetStateSimMsg), moduleID);

  std::vector<std::string>::iterator it;
  for(it = scStateInMsgNames.begin(); it!=scStateInMsgNames.end(); it++){
    this->scStateInMsgIds.push_back(SystemMessaging::GetInstance()->subscribeToMessage(*it, sizeof(SCPlusStatesSimMsg), moduleID));
  }
  return;
}


/*! This method is used to write the output densities whose names are established in AddSpacecraftToModel.
 @param CurrentClock The current time used for time-stamping the message
 @return void
 */
void ExponentialAtmosphere::WriteOutputMessages(uint64_t CurrentClock)
{
    atmoPropsSimMsg tmpAtmo;
    std::vector<int64_t>::iterator it;
    std::vector<atmoPropsSimMsg>::iterator atmoIt;
    atmoIt = atmoOutBuffer.begin();
    for(it = atmoDensOutMsgIds.begin(); it!= atmoDensOutMsgIds.end(); it++, atmoIt++){
        tmpAtmo = *atmoIt;
//        BSK_PRINT(MSG_DEBUG, "Neutral Density %f",tmpAtmo.neutralDensity);
      SystemMessaging::GetInstance()->WriteMessage(*it,
                                                  CurrentClock,
                                                  sizeof(atmoPropsSimMsg),
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
 associated spacecraft positions for computing the atmosphere.
 */
bool ExponentialAtmosphere::ReadInputs()
{
  SCPlusStatesSimMsg tmpState;
  //! Begin method steps
  SingleMessageHeader localHeader;
  memset(&this->bodyState, 0x0, sizeof(SpicePlanetStateSimMsg));
  memset(&tmpState, 0x0, sizeof(SCPlusStatesSimMsg));
  scStates.clear();
  if(this->scStateInMsgIds[0] >= 0)
  {
    //! Iterate over spacecraft message ids
    std::vector<int64_t>::iterator it;
    for(it = scStateInMsgIds.begin(); it!= scStateInMsgIds.end(); it++){
      SystemMessaging::GetInstance()->ReadMessage(*it, &localHeader,
                                                  sizeof(SCPlusStatesSimMsg),
                                                  reinterpret_cast<uint8_t*>(&tmpState),
                                                  moduleID);
    this->scStates.push_back(tmpState);
      }
  }
  if(planetPosInMsgId >= 0)
  {
      SystemMessaging::GetInstance()->ReadMessage(this->planetPosInMsgId , &localHeader,
                                                  sizeof(SpicePlanetStateSimMsg), reinterpret_cast<uint8_t*>(&this->bodyState), moduleID);
  }
  return(true);

}

/*! This method is used to update the internal density variables based on each spacecraft's position
and the pre-set atmospheric density properties. 
 */
void ExponentialAtmosphere::updateLocalAtmo(double currentTime)
{
    double tmpDensity = 0.0;
    double tmpAltitude = 0.0;
    std::vector<SCPlusStatesSimMsg>::iterator it;
    atmoPropsSimMsg tmpData;
    uint64_t atmoInd = 0;
    this->atmoOutBuffer.clear();
    for(it = scStates.begin(); it != scStates.end(); it++, atmoInd++){
      this->updateRelativePos(this->bodyState, *it);
      tmpPosMag = this->relativePos.norm();
      tmpAltitude = tmpPosMag - this->atmosphereProps.planetRadius;

      tmpDensity = this->atmosphereProps.baseDensity * exp(-1.0 * tmpAltitude / this->atmosphereProps.scaleHeight);
//        BSK_PRINT(MSG_DEBUG, "Altitude %f, Density %f", tmpAltitude, tmpDensity);
      tmpData.neutralDensity = tmpDensity;
      tmpData.localTemp = 300.0;

      this->atmoOutBuffer.push_back(tmpData);
    }
      return;
}

void ExponentialAtmosphere::updateRelativePos(SpicePlanetStateSimMsg& planetState, SCPlusStatesSimMsg& scState)
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
      this->relativePos(iter,0) = scState.r_BN_N[iter];
    }
    //std::cout<<"Relative Pos: "<<this->relativePos <<std::endl;
    return;
}

/*! This method prompts the exponential atmosphere model to update the state message as part of the cyclic simulation run.
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
