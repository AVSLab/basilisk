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
#include "architecture/messaging/system_messaging.h"
#include "../utilities/avsEigenSupport.h"


/*! @brief Creates an instance of the GroundLocation class with a minimum elevation of 10 degrees,
 @return void
 */
GroundLocation::GroundLocation()
{
    //! - Set some default initial conditions:
    this->minimumElevation = 0.; //! [deg] minimum elevation above the local horizon needed to see a spacecraft; defaults to 0 degrees
    this->maximumRange = -1; //! [m] Maximum range for the groundLocation to compute access.
    this->planetInMsgName = "";
    this->planetInMsgId = -1;

    this->planetRadius = REQ_EARTH;

    this->r_LP_P.fill(0.0);
    this->r_LP_P_Init.fill(0.0);

    this->planetState.J20002Pfix[0][0] = 1;
    this->planetState.J20002Pfix[1][1] = 1;
    this->planetState.J20002Pfix[2][2] = 1;

    this->r_North_N << 0, 0, 1;
}

/*! Empty destructor method.
 @return void
 */
GroundLocation::~GroundLocation()
{
    return;
}

/*! Resets the internal position to the specified initial position.*/
void GroundLocation::Reset()
{
    this->r_LP_P = this->r_LP_P_Init;
}

/*! Specifies the ground location from planet-centered latitude, longitude, altitude position.
 *
 * @param lat
 * @param longitude
 * @param alt
 * @return
 */
void GroundLocation::specifyLocation(double lat, double longitude, double alt)
{
    Eigen::Vector3d tmpLLAPosition(lat, longitude, alt);
    this->r_LP_P_Init = LLA2PCPF(tmpLLAPosition, this->planetRadius);

    return;
}


/*! Adds a scState message name to the vector of names to be subscribed to. Also creates a corresponding access message output name.
*/
void GroundLocation::addSpacecraftToModel(std::string tmpScMsgName)
{
    std::string tmpAccessMsgName;
    AccessSimMsg tmpAccessMsg;
    this->scStateInMsgNames.push_back(tmpScMsgName);
        tmpAccessMsgName = this->ModelTag + "_" + std::to_string(this->scStateInMsgNames.size()-1) + "_access";
    this->accessOutMsgNames.push_back(tmpAccessMsgName);
    this->accessMsgBuffer.push_back(tmpAccessMsg);

    return;
}

void GroundLocation::SelfInit()
{
    uint64_t tmpAccessMsgId;
    std::vector<std::string>::iterator it;

    //! - create all the environment output messages for each spacecraft
    for (it = this->accessOutMsgNames.begin(); it!=this->accessOutMsgNames.end(); it++) {
        tmpAccessMsgId = SystemMessaging::GetInstance()->CreateNewMessage(*it,
                                                                            sizeof(AccessSimMsg),
                                                                            this->OutputBufferCount,
                                                                            "AccessSimMsg",
                                                                            moduleID);
        this->accessOutMsgIds.push_back(tmpAccessMsgId);
    }

    this->r_LP_P = this->r_LP_P_Init;
    return;
}

void GroundLocation::CrossInit()
{
      //! - if a planet message name is specified, subscribe to this message. If not, then a zero planet position and orientation is assumed
    if (this->planetInMsgName.length() > 0) {
        this->planetInMsgId = SystemMessaging::GetInstance()->subscribeToMessage(this->planetInMsgName, sizeof(SpicePlanetStateSimMsg), moduleID);
    }
    //! - subscribe to the spacecraft messages and create associated output message buffer
    std::vector<std::string>::iterator it;
    for(it = this->scStateInMsgNames.begin(); it != this->scStateInMsgNames.end(); it++){
        this->scStateInMsgIds.push_back(SystemMessaging::GetInstance()->subscribeToMessage(*it, sizeof(SCPlusStatesSimMsg), moduleID));
    }

    return;
}

bool GroundLocation::ReadMessages()
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
    if(planetInMsgId >= 0)
    {
        planetRead = SystemMessaging::GetInstance()->ReadMessage(this->planetInMsgId , &localHeader,
                                                                 sizeof(SpicePlanetStateSimMsg),
                                                                 reinterpret_cast<uint8_t*>(&this->planetState),
                                                                 moduleID);
    }

    return(planetRead && scRead);
}

void GroundLocation::WriteMessages(uint64_t CurrentClock)
{

    AccessSimMsg tmpAccessSimMsg;
    std::vector<int64_t>::iterator it;
    std::vector<AccessSimMsg>::iterator accessIt;
    accessIt = this->accessMsgBuffer.begin();
    //! - write access message for each spacecraft
    for(it = this->accessOutMsgIds.begin(); it != this->accessOutMsgIds.end(); it++, accessIt++){
        tmpAccessSimMsg = *accessIt;
        SystemMessaging::GetInstance()->WriteMessage(*it,
                                                  CurrentClock,
                                                  sizeof(AccessSimMsg),
                                                  reinterpret_cast<uint8_t*>(&tmpAccessSimMsg),
                                                  moduleID);
    }

    return;
}

void GroundLocation::updateInertialPositions()
{
    // Update the planet inertial position:
    this->r_PN_N = cArray2EigenVector3d(this->planetState.PositionVector);
    this->r_LP_N = cArray2EigenMatrix3d(*this->planetState.J20002Pfix) * this->r_LP_P_Init; // Need to make sure J20002 is I_3x3
    this->rhat_LP_N = this->r_LP_N/this->r_LP_N.norm();
    this->r_LN_N = this->r_PN_N + this->r_LP_N;


    return;
}

void GroundLocation::computeAccess()
{
    // Update the groundLocation's inertial position
    this->updateInertialPositions();

    // Iterate over spacecraft position messages and compute the access for each one
    std::vector<AccessSimMsg>::iterator accessMsgIt;
    std::vector<SCPlusStatesSimMsg>::iterator scStatesMsgIt;
    for(scStatesMsgIt = scStates.begin(), accessMsgIt = accessMsgBuffer.begin(); scStatesMsgIt != scStates.end(); scStatesMsgIt++, accessMsgIt++){
        //! Compute the relative position of each spacecraft to the site in the planet-centered inertial frame
        Eigen::Vector3d r_BL_N = (cArray2EigenVector3d(scStatesMsgIt->r_BN_N) - this->r_PN_N) - this->r_LP_N;
        auto r_BL_mag = r_BL_N.norm();
        Eigen::Vector3d relativeHeading_N = r_BL_N / r_BL_mag;
        double viewAngle = D2R*(90.-R2D*acos(this->rhat_LP_N.dot(relativeHeading_N)));

        if( (viewAngle > this->minimumElevation) && (r_BL_mag <= this->maximumRange)){
            accessMsgIt->hasAccess = 1;
            accessMsgIt->slantRange = r_BL_N.norm();
            accessMsgIt->elevation = viewAngle;
            accessMsgIt->azimuth = acos(this->r_North_N.dot(relativeHeading_N));
        }
        else
        {
            accessMsgIt->hasAccess = 0;
            accessMsgIt->slantRange = 0.0;
            accessMsgIt->elevation = 0.0;
        }
    }
}

void GroundLocation::UpdateState(uint64_t CurrentSimNanos)
{
    this->ReadMessages();
    this->computeAccess();
    this->WriteMessages(CurrentSimNanos);

}