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

#include "atmosphere.h"
#include "architecture/messaging/system_messaging.h"
#include "utilities/linearAlgebra.h"
#include "utilities/astroConstants.h"
#include "utilities/bsk_Print.h"
#include "../../dynamics/_GeneralModuleFiles/stateData.h"
#include "../../_GeneralModuleFiles/sys_model.h"

/*! Initializer - sets the default properties to those of Earth.
*/
Atmosphere::Atmosphere()
{
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
Atmosphere::~Atmosphere()
{
    return;
}

void Atmosphere::setEnvType(std::string inputType)
{
    this->envType = inputType;
    return;
}

void Atmosphere::setEpoch(double julianDate)
{
    this->epochDate = julianDate;
    return;
}

void Atmosphere::addSpacecraftToModel(std::string tmpScMsgName){
    std::string tmpEnvMsgName;
    this->scStateInMsgNames.push_back(tmpScMsgName);
    tmpEnvMsgName = this->envType + std::to_string(this->scStateInMsgNames.size()-1)+"_data";
    this->envOutMsgNames.push_back(tmpEnvMsgName);
    return;
}

/*! SelfInit for this method creates a seperate density message for each of the spacecraft
that were added using AddSpacecraftToModel.
*/
void Atmosphere::SelfInit()
{
    std::string expString ("exponential");
    std::string msisString ("nrlmsise-00");

    if(this->envType.compare(expString))
    {
        std::string tmpAtmoMsgName;
        uint64_t tmpAtmoMsgId;
        //! Begin method steps
        std::vector<std::string>::iterator it;
        std::vector<std::string>::iterator nameIt;

        for (it = this->envOutMsgNames.begin(); it!=this->envOutMsgNames.end(); it++) {
            tmpAtmoMsgId = SystemMessaging::GetInstance()->CreateNewMessage(*it, sizeof(atmoPropsSimMsg),
                    this->OutputBufferCount, "atmoPropsSimMsg", moduleID);

            this->envOutMsgIds.push_back(tmpAtmoMsgId);
        }
    }
    return;
}

/*! This method is used to connect the input position method from the spacecraft. .
 @return void
 */
void Atmosphere::CrossInit()
{
    this->planetPosInMsgId = SystemMessaging::GetInstance()->subscribeToMessage(
    this->planetPosInMsgName, sizeof(SpicePlanetStateSimMsg), moduleID);

    std::vector<std::string>::iterator it;
    for(it = this->scStateInMsgNames.begin(); it!=this->scStateInMsgNames.end(); it++){
        this->scStateInMsgIds.push_back(SystemMessaging::GetInstance()->subscribeToMessage(*it, sizeof(SCPlusStatesSimMsg), moduleID));
    }
  return;
}


/*! This method is used to write the output densities whose names are established in AddSpacecraftToModel.
 @param CurrentClock The current time used for time-stamping the message
 @return void
 */
void Atmosphere::WriteOutputMessages(uint64_t CurrentClock)
{
    atmoPropsSimMsg tmpAtmo;
    std::vector<int64_t>::iterator it;
    std::vector<atmoPropsSimMsg>::iterator atmoIt;
    atmoIt = atmoOutBuffer.begin();
    for(it = this->envOutMsgIds.begin(); it!= this->envOutMsgIds.end(); it++, atmoIt++){
        tmpAtmo = *atmoIt;
        SystemMessaging::GetInstance()->WriteMessage(*it,
                                                  CurrentClock,
                                                  sizeof(atmoPropsSimMsg),
                                                  reinterpret_cast<uint8_t*>(&tmpAtmo),
                                                  moduleID);
    }
}


/*! This method is used to read the incoming command message and set the
 associated spacecraft positions for computing the atmosphere.
 */
bool Atmosphere::ReadInputs()
    {
    SCPlusStatesSimMsg tmpState;
    //! Begin method steps
    SingleMessageHeader localHeader;
    memset(&this->bodyState, 0x0, sizeof(SpicePlanetStateSimMsg));
    memset(&tmpState, 0x0, sizeof(SCPlusStatesSimMsg));
    scStates.clear();

    bool scReads = false;
    bool planetRead = false;
    if(planetPosInMsgId >= 0) {
        bool planetRead = false; // If the message HAS been set, make sure we read it before updating.
    }
    else{
        bool planetRead = true; // if the message hasn't been initialized, assume the planet is the origin and a failed read is okay.
    }

    //SC message reads
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
        scReads = true;
    }

    // Planet message read
    if(planetPosInMsgId >= 0)
        {
        SystemMessaging::GetInstance()->ReadMessage(this->planetPosInMsgId , &localHeader,
                                              sizeof(SpicePlanetStateSimMsg), reinterpret_cast<uint8_t*>(&this->bodyState), moduleID);
        planetRead = true;
        }
    return(scReads && planetRead);
    }

/*! This method is used to update the internal density variables based on each spacecraft's position
and the pre-set atmospheric density properties. 
 */
void Atmosphere::updateLocalAtmo(double currentTime)
{

    if(this->envType.compare("exponential")){

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
            tmpData.neutralDensity = tmpDensity;
            tmpData.localTemp = 300.0;

            this->atmoOutBuffer.push_back(tmpData);
        }

    }
    else{
        BSK_PRINT(MSG_WARNING, "Atmospheric model not set. Skipping computation.")
    }
    return;
}

void Atmosphere::updateRelativePos(SpicePlanetStateSimMsg& planetState, SCPlusStatesSimMsg& scState)
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
void Atmosphere::UpdateState(uint64_t CurrentSimNanos)
{
    //! Begin method steps
    //! - Read the inputs and then call ConfigureThrustRequests to set up dynamics
    if(this->ReadInputs())
    {
        updateLocalAtmo(CurrentSimNanos*1.0E-9);
    }
    WriteOutputMessages(CurrentSimNanos);
    return;
}
