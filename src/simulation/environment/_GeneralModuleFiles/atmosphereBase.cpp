
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

#include "atmosphereBase.h"
#include "architecture/utilities/linearAlgebra.h"
#include "architecture/utilities/macroDefinitions.h"
#include "architecture/utilities/simDefinitions.h"

/*! This method initializes some basic parameters for the module.

 */
AtmosphereBase::AtmosphereBase()
{
    //! - zero class variables
    this->planetRadius = 0.0; // [m] Earth magnetic spherical reference radius (see p. 404 in doi:10.1007/978-1-4939-0802-8)
    this->r_BP_N.fill(0.0);
    this->r_BP_P.fill(0.0);
    this->scStateInMsgs.clear();
    this->envOutMsgs.clear();

    //! - set the default epoch information
    this->epochDateTime.tm_year = EPOCH_YEAR - 1900;
    this->epochDateTime.tm_mon = EPOCH_MONTH - 1;
    this->epochDateTime.tm_mday = EPOCH_DAY;
    this->epochDateTime.tm_hour = EPOCH_HOUR;
    this->epochDateTime.tm_min = EPOCH_MIN;
    this->epochDateTime.tm_sec = (int) round(EPOCH_SEC);
    this->epochDateTime.tm_isdst = -1;

    //! - turn off minimum and maximum reach features
    this->envMinReach = -1;
    this->envMaxReach = -1;

    //! - zero the planet message, and set the DCM to an identity matrix
    this->planetState = planetPosInMsg.zeroMsgPayload;
    m33SetIdentity(this->planetState.J20002Pfix);

    this->scStateInMsgs.clear();
    this->envOutMsgs.clear();
    this->envOutBuffer.clear();

    return;
}

/*! Destructor.

 */
AtmosphereBase::~AtmosphereBase()
{
    for (long unsigned int c=0; c<this->envOutMsgs.size(); c++) {
        delete this->envOutMsgs.at(c);
    }
    return;
}

/*! Adds the spacecraft message to a vector of sc messages and automatically creates the corresponding output message.

 @param tmpScMsg A spacecraft state message name.
 */
void AtmosphereBase::addSpacecraftToModel(Message<SCStatesMsgPayload> *tmpScMsg){

    /* add input message */
    this->scStateInMsgs.push_back(tmpScMsg->addSubscriber());

    /* create output message */
    Message<AtmoPropsMsgPayload> *msg;
    msg = new Message<AtmoPropsMsgPayload>;
    this->envOutMsgs.push_back(msg);

    /* create buffer message copies*/
    AtmoPropsMsgPayload msgAtmoBuffer;
    this->envOutBuffer.push_back(msgAtmoBuffer);


    return;
}



/*! This method is used to reset the module.

 */
void AtmosphereBase::Reset(uint64_t CurrentSimNanos)
{
    //! - call the custom environment module reset method
    customReset(CurrentSimNanos);

    /* set epoch information.  If provided, then the epoch message information should be used.  */
    if (this->epochInMsg.isLinked()) {
        // Read in the epoch message and set the internal time structure
        EpochMsgPayload epochMsg;
        if (!this->epochInMsg.isWritten()) {
            bskLogger.bskLog(BSK_ERROR, "An un-written epoch msg was linked in!");
        }
        epochMsg = this->epochInMsg();
        this->epochDateTime.tm_year = epochMsg.year - 1900;
        this->epochDateTime.tm_mon = epochMsg.month - 1;
        this->epochDateTime.tm_mday = epochMsg.day;
        this->epochDateTime.tm_hour = epochMsg.hours;
        this->epochDateTime.tm_min = epochMsg.minutes;
        this->epochDateTime.tm_sec = (int) round(epochMsg.seconds);
        mktime(&this->epochDateTime);
    } else {
        customSetEpochFromVariable();
    }

    return;
}

/*! Custom customSetEpochFromVariable() method.  This allows a child class to specify how the module epoch information
 is set by a module variable.

 */
void AtmosphereBase::customSetEpochFromVariable()
{
    return;
}


/*! Custom Reset() method.  This allows a child class to add additional functionality to the Reset() method

 */
void AtmosphereBase::customReset(uint64_t CurrentClock)
{
    return;
}

/*! This method is used to write the output magnetic field messages whose names are established in AddSpacecraftToModel.
 @param CurrentClock The current time used for time-stamping the message

 */
void AtmosphereBase::writeMessages(uint64_t CurrentClock)
{
    //! - write density output messages for each spacecaft's locations
    for(long unsigned int c = 0; c < this->envOutMsgs.size(); c++){
        this->envOutMsgs.at(c)->write(&this->envOutBuffer.at(c), this->moduleID, CurrentClock);
    }

    //! - call the custom method to perform additional output message writing
    customWriteMessages(CurrentClock);

    return;
}

/*! Custom output message writing method.  This allows a child class to add additional functionality.

 */
void AtmosphereBase::customWriteMessages(uint64_t CurrentClock)
{
    return;
}

/*! This method is used to read the incoming command message and set the
 associated spacecraft positions for computing the atmosphere.

 */
bool AtmosphereBase::readMessages()
{
    SCStatesMsgPayload scMsg;

    this->scStates.clear();

    //! - read in the spacecraft state messages
    bool scRead;
    if(this->scStateInMsgs.size() > 0)
    {
        scRead = true;
        for(long unsigned int c = 0; c<this->scStateInMsgs.size(); c++){
            bool tmpScRead;
            scMsg = this->scStateInMsgs.at(c)();
            tmpScRead = this->scStateInMsgs.at(c).isWritten();
            scRead = scRead && tmpScRead;

            this->scStates.push_back(scMsg);
        }
    } else {
        bskLogger.bskLog(BSK_ERROR, "Atmosphere model has no spacecraft added to it.");
        scRead = false;
    }

    //! - Read in the optional planet message.  if no planet message is set, then a zero planet position, velocity and orientation is assumed
    bool planetRead = true;
    if(this->planetPosInMsg.isLinked())
    {
        this->planetState = this->planetPosInMsg();
        planetRead = this->planetPosInMsg.isWritten();
    }

    //! - call the custom method to perform additional input reading
    bool customRead = customReadMessages();

    return(planetRead && scRead && customRead);
}


/*! Custom output input reading method.  This allows a child class to add additional functionality.

 */
bool AtmosphereBase::customReadMessages()
{
    return true;
}

/*! This method is used to determine the spacecraft position vector relative to the planet.
 @param planetState A space planetstate message struct.
 @param scState A spacecraft states message struct.

 */
void AtmosphereBase::updateRelativePos(SpicePlanetStateMsgPayload *planetState, SCStatesMsgPayload *scState)
{
    //! - compute spacecraft position vector relative to planet
    v3Subtract(scState->r_BN_N, planetState->PositionVector, this->r_BP_N.data());

    //! - convert spacecraft position vector in Earth-fixed vector components
    m33MultV3(this->planetState.J20002Pfix, this->r_BP_N.data(), this->r_BP_P.data());

    //! - compute orbit radius
    this->orbitRadius = this->r_BP_N.norm();
    this->orbitAltitude = this->orbitRadius - this->planetRadius;

    return;
}

/*! This method is used to update the local magnetic field based on each spacecraft's position.

 */
void AtmosphereBase::updateLocalAtmosphere(double currentTime)
{
    std::vector<SCStatesMsgPayload>::iterator scIt;

    //! - loop over all the spacecraft
    std::vector<AtmoPropsMsgPayload>::iterator envMsgIt;
    envMsgIt = this->envOutBuffer.begin();
    for(scIt = scStates.begin(); scIt != scStates.end(); scIt++, envMsgIt++){
        //! - Computes planet relative state vector
        this->updateRelativePos(&(this->planetState), &(*scIt));

        //! - zero the output message for each spacecraft by default
        *envMsgIt = this->envOutMsgs[0]->zeroMsgPayload;

        //! - check if radius is in permissible range
        if(this->orbitAltitude > this->envMinReach &&
           (this->orbitAltitude < this->envMaxReach || this->envMaxReach < 0)) {
            //! - compute the local atmosphere data.  The evaluateMageticFieldModel() method must be implement for each model
            evaluateAtmosphereModel(&(*envMsgIt), currentTime);
        }
    }

    return;
}


/*! Computes the current local magnetic field for each spacecraft and writes their respective messages.

 @param CurrentSimNanos The current simulation time in nanoseconds
 */
void AtmosphereBase::UpdateState(uint64_t CurrentSimNanos)
{
    //! - clear the output buffer
    std::vector<AtmoPropsMsgPayload>::iterator it;
    for(it = this->envOutBuffer.begin(); it!= this->envOutBuffer.end(); it++){
        *it = this->envOutMsgs[0]->zeroMsgPayload;
    }
    //! - update local neutral density information
    if(this->readMessages())
    {
        this->updateLocalAtmosphere(CurrentSimNanos*NANO2SEC);
    }

    //! - write out neutral density message
    this->writeMessages(CurrentSimNanos);

    return;
}
