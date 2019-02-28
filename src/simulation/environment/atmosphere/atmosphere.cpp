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
#include "simFswInterfaceMessages/macroDefinitions.h"

/*! This method initializes some basic parameters for the module.
 @return void
 */
Atmosphere::Atmosphere()
{
    this->planetPosInMsgName = "spice_planet_output_data";
    this->OutputBufferCount = 2;
    //! - Set the default atmospheric properties to those of Earth
    this->exponentialParams.baseDensity = 1.217;
    this->exponentialParams.scaleHeight = 8500.0;
    this->exponentialParams.planetRadius = 6371.008 * 1000.0;
    this->localAtmoTemp = 293.0; //! - Placeholder temperature value from http://nssdc.gsfc.nasa.gov/planetary/factsheet/earthfact.html
    this->relativePos.fill(0.0);
    this->scStateInMsgNames.clear();
    this->tmpAtmo.neutralDensity = this->exponentialParams.baseDensity;
    this->tmpAtmo.localTemp = this->localAtmoTemp;
    this->planetPosInMsgId = -1;
    memset(&this->bodyState, 0x0, sizeof(SpicePlanetStateSimMsg));

    return;
}

/*! Destructor.
 @return void
 */
Atmosphere::~Atmosphere()
{
    return;
}

/*! Sets the model used to compute atmospheric density/temperature; must be set before init.
 @return void
 @param inputType The desired model type.
 */
void Atmosphere::setEnvType(std::string inputType)
{
    this->envType = inputType;
    return;
}

/*! Sets the epoch date used by some models. This is converted automatically to the desired units.
 @return void
 @param julianDate The specified epoch date in JD2000.
 */
void Atmosphere::setEpoch(double julianDate)
{
    this->epochDate = julianDate;
    return;
}

/*! Adds the spacecraft message name to a vector of sc message names and automatically creates an output message name.
    Must be called after ``setEnvType''.
 @return void
 @param tmpScMsgName A spacecraft state message name.
 */
void Atmosphere::addSpacecraftToModel(std::string tmpScMsgName){
    std::string tmpEnvMsgName;
    this->scStateInMsgNames.push_back(tmpScMsgName);
    tmpEnvMsgName = this->envType + std::to_string(this->scStateInMsgNames.size()-1)+"_data";
    this->envOutMsgNames.push_back(tmpEnvMsgName);
    return;
}

/*! SelfInit for this method creates a seperate density message for each of the spacecraft
that were added using AddSpacecraftToModel. Additional model outputs are also initialized per-spacecraft.
 @return void
*/
void Atmosphere::SelfInit()
{
    std::string expString ("exponential");
    std::string msisString ("nrlmsise-00");

    std::string tmpAtmoMsgName;
    uint64_t tmpAtmoMsgId;

    std::vector<std::string>::iterator it;
    std::vector<std::string>::iterator nameIt;

    for (it = this->envOutMsgNames.begin(); it!=this->envOutMsgNames.end(); it++) {
        tmpAtmoMsgId = SystemMessaging::GetInstance()->CreateNewMessage(*it, sizeof(AtmoPropsSimMsg),
                this->OutputBufferCount, "AtmoPropsSimMsg", moduleID);

        this->envOutMsgIds.push_back(tmpAtmoMsgId);
        if(this->envType.compare(msisString)==0){
            BSK_PRINT(MSG_WARNING, "NRLMSISE-00 is not implemented. Skipping message init.")
        }
    }

    return;
}

/*! This method is used to connect the input position message from the spacecraft. Additonal model-specific cross inits are also conducted.
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

    if(this->envType.compare("nrlmsise-00")==0){
        //* [WIP] Also do MSISE messaging setup*//
        BSK_PRINT(MSG_WARNING, "NRLMSISE-00 is not implemented. Skipping message init.")
    }
  return;
}


/*! This method is used to write the output densities whose names are established in AddSpacecraftToModel.
 @param CurrentClock The current time used for time-stamping the message
 @return void
 */
void Atmosphere::WriteOutputMessages(uint64_t CurrentClock)
{
    AtmoPropsSimMsg tmpAtmo;
    std::vector<int64_t>::iterator it;
    std::vector<AtmoPropsSimMsg>::iterator atmoIt;
    atmoIt = atmoOutBuffer.begin();
    for(it = this->envOutMsgIds.begin(); it!= this->envOutMsgIds.end(); it++, atmoIt++){
        tmpAtmo = *atmoIt;
        SystemMessaging::GetInstance()->WriteMessage(*it,
                                                  CurrentClock,
                                                  sizeof(AtmoPropsSimMsg),
                                                  reinterpret_cast<uint8_t*>(&tmpAtmo),
                                                  moduleID);
    }

    if(this->envType.compare("nrlmsise-00")==0){
        /* [WIP] - Include additional outputs for other MSISE outputs (species count, etc.)*/
        BSK_PRINT(MSG_WARNING, "NRLMSISE-00 is not implemented. Skipping message write.")
    }
}


/*! This method is used to read the incoming command message and set the
 associated spacecraft positions for computing the atmosphere.
 @return void
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

    if(this->envType.compare("nrlmsise-00")==0){
        /* WIP - Also read in all the MSISE inputs.*/
        BSK_PRINT(MSG_WARNING, "NRLMSISE-00 is not implemented. Skipping message read.")
    }
    return(scReads && planetRead);
    }

/*! This method is used to update the internal density variables based on each spacecraft's position
and the pre-set atmospheric density properties.
  @return void
 */
void Atmosphere::updateLocalAtmo(double currentTime)
{
    double tmpAltitude = 0.0;
    std::vector<SCPlusStatesSimMsg>::iterator it;
    AtmoPropsSimMsg tmpData;
    uint64_t atmoInd = 0;
    this->atmoOutBuffer.clear();


    //! - loop over all the spacecraft
    for(it = scStates.begin(); it != scStates.end(); it++, atmoInd++){
        this->updateRelativePos(this->bodyState, *it); //! - Computes planet relative state vector
        tmpPosMag = this->relativePos.norm();
        tmpAltitude = tmpPosMag - this->exponentialParams.planetRadius; //! - computes the altitude above the planet radius

        //! - zero the output message for each spacecraft by default
        memset(&tmpData, 0x0, sizeof(AtmoPropsSimMsg));

        //! - check if radius is in permissible range
        if(tmpAltitude > this->envMinReach &&
           (tmpAltitude < this->envMaxReach || this->envMaxReach < 0)) {
            //! - check for exponential atmosphere model case
            if(this->envType.compare("exponential")==0){
                tmpData.neutralDensity = this->exponentialParams.baseDensity * exp(-1.0 * tmpAltitude / this->exponentialParams.scaleHeight);
                tmpData.localTemp = this->localAtmoTemp;

            } else {
                BSK_PRINT(MSG_WARNING, "Atmospheric model not set. Skipping computation.")
            }
        }

        //! - store the evaluated atmospheric neutral density info for each spacecraft
        this->atmoOutBuffer.push_back(tmpData);
    }

    return;
}

/*! This method is used to write the output densities whose names are established in AddSpacecraftToModel.
 @param planetState A space planetstate message struct.
 @param scState A spacecraftPlusStates message struct.
 @return void
 */
void Atmosphere::updateRelativePos(SpicePlanetStateSimMsg& planetState, SCPlusStatesSimMsg& scState)
{
    /*! This loop iterates over each spacecraft with respect to one planet if the planet message has been set.*/
    uint64_t iter = 0;
    if(planetPosInMsgId >= 0)
    {
        for(iter = 0; iter < 3; iter++)
        {
            /*! determine spacecrat position relative to plant */
            this->relativePos(iter,0) = scState.r_BN_N[iter] - planetState.PositionVector[iter];
        }
    }
    /*! If no planet message is set then the spacecraft position is assumed to be relative to planet center */
    else{
        this->relativePos(iter,0) = scState.r_BN_N[iter];
    }
    return;
}


/*! Computes the current atmospheric parameters for each spacecraft and writes their respective messages.
 @return void
 @param CurrentSimNanos The current simulation time in nanoseconds
 */
void Atmosphere::UpdateState(uint64_t CurrentSimNanos)
{
    //! - update local neutral density information
    if(this->ReadInputs())
    {
        updateLocalAtmo(CurrentSimNanos*NANO2SEC);
    }

    //! - write out neutral density message
    WriteOutputMessages(CurrentSimNanos);
    return;
}
