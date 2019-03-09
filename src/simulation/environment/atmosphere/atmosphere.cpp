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

#include "atmosphere.h"
#include "architecture/messaging/system_messaging.h"
#include "utilities/astroConstants.h"
#include "utilities/bsk_Print.h"
#include "simFswInterfaceMessages/macroDefinitions.h"

/*! This method initializes some basic parameters for the module.
 @return void
 */
Atmosphere::Atmosphere()
{
    this->planetPosInMsgName = "";
    this->OutputBufferCount = 2;
    //! - Set the default atmospheric properties to those of Earth
    this->envType = EXPONENTIAL_MODEL;  // - atmospheric environment label
    this->exponentialParams.baseDensity = 1.217;  // [kg/m^3] exponential atmosphere model base density
    this->exponentialParams.scaleHeight = 8500.0; // [m] exponential atmosphere model scale height
    this->planetRadius = REQ_EARTH*1000; // [m] Earth equatorial radius
    this->localAtmoTemp = 293.0; // Placeholder temperature value from http://nssdc.gsfc.nasa.gov/planetary/factsheet/earthfact.html
    this->relativePos_N.fill(0.0);
    this->scStateInMsgNames.clear();
    this->planetPosInMsgId = -1;

    //!< zero the planet message, and set the DCM to an identity matrix
    memset(&this->planetState, 0x0, sizeof(SpicePlanetStateSimMsg));
    this->planetState.J20002Pfix[0][0] = 1.0;
    this->planetState.J20002Pfix[1][1] = 1.0;
    this->planetState.J20002Pfix[2][2] = 1.0;

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
    tmpEnvMsgName = this->envType + "_" + std::to_string(this->scStateInMsgNames.size()-1)+"_data";
    this->envOutMsgNames.push_back(tmpEnvMsgName);
    return;
}

/*! SelfInit for this method creates a seperate density message for each of the spacecraft
that were added using AddSpacecraftToModel. Additional model outputs are also initialized per-spacecraft.
 @return void
*/
void Atmosphere::SelfInit()
{
    uint64_t tmpAtmoMsgId;
    std::vector<std::string>::iterator it;

    //! - create all the environment output messages for each spacecraft
    for (it = this->envOutMsgNames.begin(); it!=this->envOutMsgNames.end(); it++) {
        tmpAtmoMsgId = SystemMessaging::GetInstance()->CreateNewMessage(*it, sizeof(AtmoPropsSimMsg),
                this->OutputBufferCount, "AtmoPropsSimMsg", moduleID);
        this->envOutMsgIds.push_back(tmpAtmoMsgId);

        if(this->envType.compare(MSISE_MODEL)==0){
            BSK_PRINT(MSG_ERROR, "NRLMSISE-00 is not implemented. Skipping message init.\n")
        }
    }

    return;
}

/*! This method is used to connect the input position message from the spacecraft. Additonal model-specific cross inits are also conducted.
 @return void
 */
void Atmosphere::CrossInit()
{
    //! - if a planet message name is specified, subscribe to this message. If not, then a zero planet position and orientation is assumed
    if (this->planetPosInMsgName.length() > 0) {
        this->planetPosInMsgId = SystemMessaging::GetInstance()->subscribeToMessage(this->planetPosInMsgName,
                                                                                    sizeof(SpicePlanetStateSimMsg),
                                                                                    moduleID);
    }

    //! - subscribe to the spacecraft messages and create associated output message buffer
    std::vector<std::string>::iterator it;
    this->atmoOutBuffer.clear();
    AtmoPropsSimMsg tmpAtmo;
    memset(&tmpAtmo, 0x0, sizeof(AtmoPropsSimMsg));
    for(it = this->scStateInMsgNames.begin(); it!=this->scStateInMsgNames.end(); it++){
        this->scStateInMsgIds.push_back(SystemMessaging::GetInstance()->subscribeToMessage(*it, sizeof(SCPlusStatesSimMsg), moduleID));
        this->atmoOutBuffer.push_back(tmpAtmo);
    }

    if(this->envType.compare(MSISE_MODEL)==0){
        //* [WIP] Also do MSISE messaging setup*//
        BSK_PRINT(MSG_ERROR, "NRLMSISE-00 is not implemented. Skipping message init.\n")
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
    atmoIt = this->atmoOutBuffer.begin();
    for(it = this->envOutMsgIds.begin(); it!= this->envOutMsgIds.end(); it++, atmoIt++){
        tmpAtmo = *atmoIt;
        SystemMessaging::GetInstance()->WriteMessage(*it,
                                                  CurrentClock,
                                                  sizeof(AtmoPropsSimMsg),
                                                  reinterpret_cast<uint8_t*>(&tmpAtmo),
                                                  moduleID);
    }

    if(this->envType.compare(MSISE_MODEL)==0){
        /* [WIP] - Include additional outputs for other MSISE outputs (species count, etc.)*/
        BSK_PRINT(MSG_ERROR, "NRLMSISE-00 is not implemented. Skipping message write.\n")
    }
}


/*! This method is used to read the incoming command message and set the
 associated spacecraft positions for computing the atmosphere.
 @return void
 */
bool Atmosphere::ReadInputs()
{
    SCPlusStatesSimMsg scMsg;
    SingleMessageHeader localHeader;

    this->scStates.clear();

    //SC message reads
    bool scRead;
    if(this->scStateInMsgIds.size() > 0)
    {
        scRead = true;
        //! Iterate over spacecraft message ids
        std::vector<int64_t>::iterator it;
            for(it = scStateInMsgIds.begin(); it!= scStateInMsgIds.end(); it++){
                bool tmpScRead;
                memset(&scMsg, 0x0, sizeof(SCPlusStatesSimMsg));
                tmpScRead = SystemMessaging::GetInstance()->ReadMessage(*it, &localHeader,
                                                      sizeof(SCPlusStatesSimMsg),
                                                      reinterpret_cast<uint8_t*>(&scMsg),
                                                      moduleID);
                scRead = scRead && tmpScRead;

                this->scStates.push_back(scMsg);
            }
    } else {
        BSK_PRINT(MSG_ERROR, "Atmosphere model has no spacecraft added to it.\n");
        scRead = false;
    }

    // Planet message read
    bool planetRead = true;     // if no planet message is set, then a zero planet position, velocity and orientation is assumed
    if(planetPosInMsgId >= 0)
    {
        planetRead = SystemMessaging::GetInstance()->ReadMessage(this->planetPosInMsgId , &localHeader,
                                              sizeof(SpicePlanetStateSimMsg), reinterpret_cast<uint8_t*>(&this->planetState), moduleID);
    }

    if(this->envType.compare(MSISE_MODEL)==0){
        /* WIP - Also read in all the MSISE inputs.*/
        BSK_PRINT(MSG_ERROR, "NRLMSISE-00 is not implemented. Skipping message read.\n")
    }

    return(planetRead && scRead);
}

/*! This method is used to update the internal density variables based on each spacecraft's position
and the pre-set atmospheric density properties.
  @return void
 */
void Atmosphere::updateLocalAtmo(double currentTime)
{
    double tmpAltitude = 0.0;   // [m] spacecraft altitude above planet radius
    double tmpPosMag;           // [m/s] Magnitude of the spacecraft's current position
    std::vector<SCPlusStatesSimMsg>::iterator it;


    //! - loop over all the spacecraft
    std::vector<AtmoPropsSimMsg>::iterator atmoMsgIt;
    atmoMsgIt = this->atmoOutBuffer.begin();
    for(it = scStates.begin(); it != scStates.end(); it++, atmoMsgIt++){
        //! - Computes planet relative state vector
        this->updateRelativePos(&(this->planetState), &(*it));

        //! - compute spacecraft altitude above planet radius
        tmpPosMag = this->relativePos_N.norm();
        tmpAltitude = tmpPosMag - this->planetRadius; //! - computes the altitude above the planet radius

        //! - zero the output message for each spacecraft by default
        memset(&(*atmoMsgIt), 0x0, sizeof(AtmoPropsSimMsg));

        //! - check if radius is in permissible range
        if(tmpAltitude > this->envMinReach &&
           (tmpAltitude < this->envMaxReach || this->envMaxReach < 0)) {
            //! - check for exponential atmosphere model case
            if(this->envType.compare(EXPONENTIAL_MODEL)==0){
                (*atmoMsgIt).neutralDensity = this->exponentialParams.baseDensity * exp(-1.0 * tmpAltitude / this->exponentialParams.scaleHeight);
                (*atmoMsgIt).localTemp = this->localAtmoTemp;

            } else {
                BSK_PRINT(MSG_WARNING, "Atmospheric model not set. Skipping computation.\n")
            }
        }
    }

    return;
}

/*! This method is used to determine the spacecraft position vector relative to the planet.
 @param planetState A space planetstate message struct.
 @param scState A spacecraftPlusStates message struct.
 @return void
 */
void Atmosphere::updateRelativePos(SpicePlanetStateSimMsg *planetState, SCPlusStatesSimMsg *scState)
{
    int iter;
    /*! determine spacecraft position relative to planet */
    for(iter = 0; iter < 3; iter++)
    {
        this->relativePos_N[iter] = scState->r_BN_N[iter] - planetState->PositionVector[iter];
    }

    return;
}


/*! Computes the current atmospheric parameters for each spacecraft and writes their respective messages.
 @return void
 @param CurrentSimNanos The current simulation time in nanoseconds
 */
void Atmosphere::UpdateState(uint64_t CurrentSimNanos)
{
    //! - clear the output buffer
    std::vector<AtmoPropsSimMsg>::iterator it;
    for(it = this->atmoOutBuffer.begin(); it!= this->atmoOutBuffer.end(); it++){
        memset(&(*it), 0x0, sizeof(AtmoPropsSimMsg));
    }

    //! - update local neutral density information
    if(this->ReadInputs())
    {
        updateLocalAtmo(CurrentSimNanos*NANO2SEC);
    }

    //! - write out neutral density message
    WriteOutputMessages(CurrentSimNanos);

    return;
}
