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

#include "groundLocation.h"

/*! The constructor method initializes the dipole parameters to zero, resuling in a zero magnetic field result by default.
 @return void
 */
GroundLocation::GroundLocation()
{
    //! - Set the default atmospheric properties to yield a zero response
    this->minimumElevation = 10.; //! [deg] minimum elevation above the local horizon needed to see a spacecraft; defaults to 10 degrees
    this->planetInMsgName = "";
    this->planetInMsgId = -1;

    this->initialPosition_P.fill(0.0);
    this->currentPosition_N.fill(0.0);

};

    return;
}

/*! Empty destructor method.
 @return void
 */
GroundLocation::~GroundLocation()
{
    return;
}

/*! Adds a scState message name to the vector of names to be subscribed to. Also creates a corresponding access message output name.
*/
GroundLocation::addSpacecraftToModel()
{
    std::string tmpAccessMsgName;
    this->scStateInMsgNames.push_back(tmpScMsgName);
        tmpAccessMsgName = this->ModelTag + "_" + std::to_string(this->scStateInMsgNames.size()-1) + "_access";
    this->accessOutMsgNames.push_back(tmpAccessMsgName);
    return;
}

GroundLocation::SelfInit()
{
    uint64_t tmpMagFieldMsgId;
    std::vector<std::string>::iterator it;

    //! - create all the environment output messages for each spacecraft
    for (it = this->accessOutMsgNames.begin(); it!=this->accessOutMsgNames.end(); it++) {
        tmpMagFieldMsgId = SystemMessaging::GetInstance()->CreateNewMessage(*it,
                                                                            sizeof(AccessSimMsg),
                                                                            this->OutputBufferCount,
                                                                            "AccessSimMsg",
                                                                            moduleID);
        this->envOutMsgIds.push_back(tmpMagFieldMsgId);
    }

    //! - call the custom SelfInit() method to add addtional self initialization steps
    customSelfInit();

    return;
    return;
}

GroundLocation::CrossInit()
{
      //! - if a planet message name is specified, subscribe to this message. If not, then a zero planet position and orientation is assumed
    if (this->planetPosInMsgName.length() > 0) {
        this->planetPosInMsgId = SystemMessaging::GetInstance()->subscribeToMessage(this->planetPosInMsgName, sizeof(SpicePlanetStateSimMsg), moduleID);
    }
    //! - subscribe to the spacecraft messages and create associated output message buffer
    std::vector<std::string>::iterator it;
    for(it = this->scStateInMsgNames.begin(); it != this->scStateInMsgNames.end(); it++){
        this->scStateInMsgIds.push_back(SystemMessaging::GetInstance()->subscribeToMessage(*it, sizeof(SCPlusStatesSimMsg), moduleID));
    }

    return;
}

GroundLocation::ReadMessages()
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
        BSK_PRINT(MSG_ERROR, "Ground location has no spacecraft to track.");
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
    return;
}

GroundLocation::WriteMessages()
{

    AccessSimMsg tmpAccessSimMsg;
    std::vector<int64_t>::iterator it;
    std::vector<AccessSimMsg>::iterator accessIt;
    accessIt = this->accessBuffer.begin();
    //! - write magnetic field output messages for each spacecaft's locations
    for(it = this->accessOutMsgIds.begin(); it != this->accessOutMsgIds.end(); it++, accessIt++){
        tmpAccessSimMsg = *accessIt;
        SystemMessaging::GetInstance()->WriteMessage(*it,
                                                  CurrentClock,
                                                  sizeof(AccessSimMsg),
                                                  reinterpret_cast<uint8_t*>(&tmpAccessSimMsg),
                                                  moduleID);
    }

    //! - call the custom method to perform additional output message writing
    customWriteMessages(CurrentClock);
    return;
}

GroundLocation::updateInertialPositions()
{
    // Update the planet inertial position:
    this->r_PN_N = this->planetState.positionVector
    this->r_

    return;
}

GroundLocation::computeAccess()
{
    // Update the groundLocation's inertial position
    this->updateInertialPosition();
    

    // Iterate over spacecraft position messages and compute the access for each one
    std::vector<AccessSimMsg>::iterator accessMsgIt;
    std::vector<SCPlusStatesSimMsg>::iterator scStatesMsgIt;
    for(scStatesMsgIt = scStates.begin(); scStatesMsgIt != scStates.end(); scStatesMsgIt++; accessMsgIt)
    {
        // View angle calculation
        Eigen::Vector3d relativePosition_N = (*scStatesMsgIt->r_BN_N - this->r_PN_N) - this->r_BP_N;
        relativeHeading_N = relativePosition_N / relativePosition_N.norm();

        viewAngle = acos(this->localNormal_N.dot(relativeHeading_N));

        
        if(viewAngle > this->minimumElevation){
            *accessMsgIt.hasAccess = True;
            *accessMsgIt.slantRange = relativePosition_N.norm();
            *accessMsgIt.elevationAngle = 90.-viewAngle;
        }
        else
        {
            *accessMsgIt.hasAccess = False;
            *accessMsgIt.slantRange = 0.0
            *accessMsgIt.elevationAngle = 0.0
        }
    }


    return
}

GroundLocation::UpdateState(uint64_t CurrentSimNanos)
{
    this->ReadMessages();
    
    this->computeAccess();

    this->WriteMessages();

    return
}