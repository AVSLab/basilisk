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

#include "magneticFieldBase.h"
#include "architecture/messaging/system_messaging.h"
#include "utilities/astroConstants.h"
#include "utilities/bsk_Print.h"
#include "utilities/linearAlgebra.h"
#include "simFswInterfaceMessages/macroDefinitions.h"

/*! This method initializes some basic parameters for the module.
 @return void
 */
MagneticFieldBase::MagneticFieldBase()
{
    this->planetPosInMsgName = "";
    this->OutputBufferCount = 2;

    //! - zero class variables
    this->planetRadius = 0.0; // [m] Earth magnetic spherical reference radius (see p. 404 in doi:10.1007/978-1-4939-0802-8)
    this->r_BP_N.fill(0.0);
    this->r_BP_P.fill(0.0);
    this->scStateInMsgNames.clear();
    this->planetPosInMsgId = -1;

    //! - turn off minimum and maximum reach features
    this->envMinReach = -1;
    this->envMaxReach = -1;

    //! - zero the planet message, and set the DCM to an identity matrix
    memset(&this->planetState, 0x0, sizeof(SpicePlanetStateSimMsg));
    m33SetIdentity(this->planetState.J20002Pfix);

    return;
}

/*! Destructor.
 @return void
 */
MagneticFieldBase::~MagneticFieldBase()
{
    return;
}

/*! Sets the epoch date used by some models. The default is setting a julian date.  If other date formats are required by
    the environmental model, then these setEpoch() method should be redefined in the child class.
 @return void
 @param julianDate The specified epoch date in JD2000.
 */
void MagneticFieldBase::setEpoch(double julianDate)
{
    this->epochDate = julianDate;
    return;
}

/*! Adds the spacecraft message name to a vector of sc message names and automatically creates an output message name.
 @return void
 @param tmpScMsgName A spacecraft state message name.
 */
void MagneticFieldBase::addSpacecraftToModel(std::string tmpScMsgName){
    std::string tmpEnvMsgName;
    this->scStateInMsgNames.push_back(tmpScMsgName);
        tmpEnvMsgName = this->ModelTag + "_" + std::to_string(this->scStateInMsgNames.size()-1) + "_data";
    this->envOutMsgNames.push_back(tmpEnvMsgName);
    return;
}

/*! SelfInit for this method creates a seperate magnetic field message for each of the spacecraft
that were added using AddSpacecraftToModel. Additional model outputs are also initialized per-spacecraft.
 @return void
*/
void MagneticFieldBase::SelfInit()
{
    uint64_t tmpMagFieldMsgId;
    std::vector<std::string>::iterator it;

    //! - create all the environment output messages for each spacecraft
    for (it = this->envOutMsgNames.begin(); it!=this->envOutMsgNames.end(); it++) {
        tmpMagFieldMsgId = SystemMessaging::GetInstance()->CreateNewMessage(*it,
                                                                            sizeof(MagneticFieldSimMsg),
                                                                            this->OutputBufferCount,
                                                                            "MagneticFieldSimMsg",
                                                                            moduleID);
        this->envOutMsgIds.push_back(tmpMagFieldMsgId);
    }

    //! - call the custom SelfInit() method to add addtional self initialization steps
    customSelfInit();

    return;
}

/*! This method is used to connect the input position message from the spacecraft. Additonal model-specific cross inits are also conducted.
 @return void
 */
void MagneticFieldBase::CrossInit()
{
    //! - if a planet message name is specified, subscribe to this message. If not, then a zero planet position and orientation is assumed
    if (this->planetPosInMsgName.length() > 0) {
        this->planetPosInMsgId = SystemMessaging::GetInstance()->subscribeToMessage(this->planetPosInMsgName, sizeof(SpicePlanetStateSimMsg), moduleID);
    }

    //! - subscribe to the spacecraft messages and create associated output message buffer
    std::vector<std::string>::iterator it;
    this->magFieldOutBuffer.clear();
    MagneticFieldSimMsg tmpMagField;
    memset(&tmpMagField, 0x0, sizeof(MagneticFieldSimMsg));
    for(it = this->scStateInMsgNames.begin(); it != this->scStateInMsgNames.end(); it++){
        this->scStateInMsgIds.push_back(SystemMessaging::GetInstance()->subscribeToMessage(*it, sizeof(SCPlusStatesSimMsg), moduleID));
        this->magFieldOutBuffer.push_back(tmpMagField);
    }

    //!- call the custom CrossInit() method to all additional cross initialization steps
    customCrossInit();

    return;
}

/*! This method is used to reset the module.
 @return void
 */
void MagneticFieldBase::Reset(uint64_t CurrentSimNanos)
{
    //! - call the custom environment module reset method
    customReset(CurrentSimNanos);

    return;
}



/*! Custom SelfInit() method.  This allows a child class to add additional functionality to the SelfInit() method
 @return void
 */
void MagneticFieldBase::customSelfInit()
{
    return;
}

/*! Custom CrossInit() method.  This allows a child class to add additional functionality to the CrossInit() method
 @return void
 */
void MagneticFieldBase::customCrossInit()
{
    return;
}

/*! Custom Reset() method.  This allows a child class to add additional functionality to the Reset() method
 @return void
 */
void MagneticFieldBase::customReset(uint64_t CurrentClock)
{
    return;
}

/*! This method is used to write the output magnetic field messages whose names are established in AddSpacecraftToModel.
 @param CurrentClock The current time used for time-stamping the message
 @return void
 */
void MagneticFieldBase::writeMessages(uint64_t CurrentClock)
{
    MagneticFieldSimMsg tmpMagFieldOutMsg;
    std::vector<int64_t>::iterator it;
    std::vector<MagneticFieldSimMsg>::iterator magFieldIt;
    magFieldIt = this->magFieldOutBuffer.begin();
    //! - write magnetic field output messages for each spacecaft's locations
    for(it = this->envOutMsgIds.begin(); it != this->envOutMsgIds.end(); it++, magFieldIt++){
        tmpMagFieldOutMsg = *magFieldIt;
        SystemMessaging::GetInstance()->WriteMessage(*it,
                                                  CurrentClock,
                                                  sizeof(MagneticFieldSimMsg),
                                                  reinterpret_cast<uint8_t*>(&tmpMagFieldOutMsg),
                                                  moduleID);
    }

    //! - call the custom method to perform additional output message writing
    customWriteMessages(CurrentClock);

    return;
}

/*! Custom output message writing method.  This allows a child class to add additional functionality.
 @return void
 */
void MagneticFieldBase::customWriteMessages(uint64_t CurrentClock)
{
    return;
}

/*! This method is used to read the incoming command message and set the
 associated spacecraft positions for computing the atmosphere.
 @return void
 */
bool MagneticFieldBase::readMessages()
{
    SCPlusStatesSimMsg scMsg;
    SingleMessageHeader localHeader;

    this->scStates.clear();

    //! - read in the spacecraft state messages
    bool scRead;
    if(this->scStateInMsgIds.size() > 0)
    {
        scRead = true;
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
        BSK_PRINT(MSG_ERROR, "Atmosphere model has no spacecraft added to it.");
        scRead = false;
    }

    //! - Read in the optional planet message.  if no planet message is set, then a zero planet position, velocity and orientation is assumed
    bool planetRead = true;
    if(planetPosInMsgId >= 0)
    {
        planetRead = SystemMessaging::GetInstance()->ReadMessage(this->planetPosInMsgId , &localHeader,
                                                                 sizeof(SpicePlanetStateSimMsg),
                                                                 reinterpret_cast<uint8_t*>(&this->planetState),
                                                                 moduleID);
    }

    //! - call the custom method to perform additional input reading
    bool customRead = customReadMessages();

    return(planetRead && scRead && customRead);
}


/*! Custom output input reading method.  This allows a child class to add additional functionality.
 @return void
 */
bool MagneticFieldBase::customReadMessages()
{
    return true;
}

/*! This method is used to update the local magnetic field based on each spacecraft's position.
  @return void
 */
void MagneticFieldBase::updateLocalMagField(double currentTime)
{
    std::vector<SCPlusStatesSimMsg>::iterator it;
    uint64_t atmoInd = 0;

    //! - loop over all the spacecraft
    std::vector<MagneticFieldSimMsg>::iterator magMsgIt;
    magMsgIt = this->magFieldOutBuffer.begin();
    for(it = scStates.begin(); it != scStates.end(); it++, atmoInd++, magMsgIt++){
        //! - Computes planet relative state vector
        this->updateRelativePos(&(this->planetState), &(*it));

        //! - zero the output message for each spacecraft by default
        memset(&(*magMsgIt), 0x0, sizeof(MagneticFieldSimMsg));

        //! - check if radius is in permissible range
        if(this->orbitRadius > this->envMinReach &&
           (this->orbitRadius < this->envMaxReach || this->envMaxReach < 0)) {
            //! - compute the local magnetic field.  The evaluateMageticFieldModel() method must be implement for each model
            evaluateMagneticFieldModel(&(*magMsgIt));
        }
    }

    return;
}

/*! This method is used to determine the spacecraft position vector relative to the planet.
 @param planetState A space planetstate message struct.
 @param scState A spacecraftPlusStates message struct.
 @return void
 */
void MagneticFieldBase::updateRelativePos(SpicePlanetStateSimMsg *planetState, SCPlusStatesSimMsg *scState)
{
    //! - compute spacecraft position vector relative to planet
    v3Subtract(scState->r_BN_N, planetState->PositionVector, this->r_BP_N.data());

    //! - convert spacecraft position vector in Earth-fixed vector components
    m33MultV3(this->planetState.J20002Pfix, this->r_BP_N.data(), this->r_BP_P.data());

    //! - compute orbit radius
    this->orbitRadius = this->r_BP_N.norm();

    return;
}

/*! Computes the current local magnetic field for each spacecraft and writes their respective messages.
 @return void
 @param CurrentSimNanos The current simulation time in nanoseconds
 */
void MagneticFieldBase::UpdateState(uint64_t CurrentSimNanos)
{
    //! - clear the output buffer
    std::vector<MagneticFieldSimMsg>::iterator it;
    for(it = this->magFieldOutBuffer.begin(); it!= this->magFieldOutBuffer.end(); it++){
        memset(&(*it), 0x0, sizeof(MagneticFieldSimMsg));
    }
    //! - update local neutral density information
    if(this->readMessages())
    {
        updateLocalMagField(CurrentSimNanos*NANO2SEC);
    }

    //! - write out neutral density message
    this->writeMessages(CurrentSimNanos);

    return;
}
