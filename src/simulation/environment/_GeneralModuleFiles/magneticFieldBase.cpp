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
#include "architecture/utilities/linearAlgebra.h"
#include "architecture/utilities/macroDefinitions.h"
#include "architecture/utilities/simDefinitions.h"

/*! This method initializes some basic parameters for the module.

 */
MagneticFieldBase::MagneticFieldBase()
{
    //! - zero class variables
    this->planetRadius = 0.0;
    this->r_BP_N.fill(0.0);
    this->r_BP_P.fill(0.0);
    this->scStateInMsgs.clear();

    //! - turn off minimum and maximum reach features
    this->envMinReach = -1;
    this->envMaxReach = -1;

    //! - set the default epoch information
    this->epochDateTime.tm_year = EPOCH_YEAR - 1900;
    this->epochDateTime.tm_mon = EPOCH_MONTH - 1;
    this->epochDateTime.tm_mday = EPOCH_DAY;
    this->epochDateTime.tm_hour = EPOCH_HOUR;
    this->epochDateTime.tm_min = EPOCH_MIN;
    this->epochDateTime.tm_sec = (int) round(EPOCH_SEC);
    this->epochDateTime.tm_isdst = -1;


    //! - zero the planet message, and set the DCM to an identity matrix
    this->planetState = planetPosInMsg.zeroMsgPayload;
    m33SetIdentity(this->planetState.J20002Pfix);

    return;
}

/*! Destructor.

 */
MagneticFieldBase::~MagneticFieldBase()
{
    for (long unsigned int c=0; c<this->envOutMsgs.size(); c++) {
        delete this->envOutMsgs.at(c);
    }
    return;
}

/*! Adds the spacecraft message name to a vector of sc message names and automatically creates an output message name.

 @param tmpScMsg A spacecraft state message name.
 */
void MagneticFieldBase::addSpacecraftToModel(Message<SCStatesMsgPayload> *tmpScMsg){

    /* add input message */
    this->scStateInMsgs.push_back(tmpScMsg->addSubscriber());

    /* create output message */
    Message<MagneticFieldMsgPayload> *msg;
    msg = new Message<MagneticFieldMsgPayload>;
    this->envOutMsgs.push_back(msg);

    /* create buffer message copies*/
    MagneticFieldMsgPayload msgMagBuffer;
    this->magFieldOutBuffer.push_back(msgMagBuffer);

    return;
}


/*! This method is used to reset the module.

 */
void MagneticFieldBase::Reset(uint64_t CurrentSimNanos)
{
    //! - call the custom environment module reset method
    customReset(CurrentSimNanos);

    /* set epoch information.  If provided, then the epoch message information should be used.  */
    if (this->epochInMsg.isLinked()) {
        // Read in the epoch message and set the internal time structure
        EpochMsgPayload epochMsg;
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


/*! Custom Reset() method.  This allows a child class to add additional functionality to the Reset() method

 */
void MagneticFieldBase::customReset(uint64_t CurrentClock)
{
    return;
}

/*! Custom customSetEpochFromVariable() method.  This allows a child class to specify how the module epoch information
 is set by a module variable.

 */
void MagneticFieldBase::customSetEpochFromVariable()
{
    return;
}

/*! This method is used to write the output magnetic field messages whose names are established in AddSpacecraftToModel.
 @param CurrentClock The current time used for time-stamping the message

 */
void MagneticFieldBase::writeMessages(uint64_t CurrentClock)
{
    for (long unsigned int c=0; c<this->envOutMsgs.size(); c++) {
        this->envOutMsgs.at(c)->write(&this->magFieldOutBuffer.at(c), this->moduleID, CurrentClock);
    }

    //! - call the custom method to perform additional output message writing
    customWriteMessages(CurrentClock);

    return;
}

/*! Custom output message writing method.  This allows a child class to add additional functionality.

 */
void MagneticFieldBase::customWriteMessages(uint64_t CurrentClock)
{
    return;
}

/*! This method is used to read the incoming command message and set the
 associated spacecraft positions for computing the atmosphere.

 */
bool MagneticFieldBase::readMessages()
{
    SCStatesMsgPayload scMsg;

    this->scStates.clear();

    //! - read in the spacecraft state messages
    bool scRead;
    if(this->scStateInMsgs.size() > 0)
    {
        scRead = true;
        for (long unsigned int c=0; c<this->scStateInMsgs.size(); c++) {
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
bool MagneticFieldBase::customReadMessages()
{
    return true;
}

/*! This method is used to update the local magnetic field based on each spacecraft's position.

 */
void MagneticFieldBase::updateLocalMagField(double currentTime)
{
    std::vector<SCStatesMsgPayload>::iterator it;
    uint64_t atmoInd = 0;

    //! - loop over all the spacecraft
    std::vector<MagneticFieldMsgPayload>::iterator magMsgIt;
    magMsgIt = this->magFieldOutBuffer.begin();
    for(it = scStates.begin(); it != scStates.end(); it++, atmoInd++, magMsgIt++){
        //! - Computes planet relative state vector
        this->updateRelativePos(&(this->planetState), &(*it));

        //! - zero the output message for each spacecraft by default
        *magMsgIt = this->envOutMsgs[0]->zeroMsgPayload;

        //! - check if radius is in permissible range
        if(this->orbitRadius > this->envMinReach &&
           (this->orbitRadius < this->envMaxReach || this->envMaxReach < 0)) {
            //! - compute the local magnetic field.  The evaluateMageticFieldModel() method must be implement for each model
            evaluateMagneticFieldModel(&(*magMsgIt), currentTime);
        }
    }

    return;
}

/*! This method is used to determine the spacecraft position vector relative to the planet.
 @param planetState A space planetstate message struct.
 @param scState A spacecraft states message struct.

 */
void MagneticFieldBase::updateRelativePos(SpicePlanetStateMsgPayload *planetState, SCStatesMsgPayload *scState)
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

 @param CurrentSimNanos The current simulation time in nanoseconds
 */
void MagneticFieldBase::UpdateState(uint64_t CurrentSimNanos)
{
    //! - clear the output buffer
    std::vector<MagneticFieldMsgPayload>::iterator it;
    for(it = this->magFieldOutBuffer.begin(); it!= this->magFieldOutBuffer.end(); it++){
        memset(&(*it), 0x0, sizeof(MagneticFieldMsgPayload));
        *it = this->envOutMsgs[0]->zeroMsgPayload;
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
