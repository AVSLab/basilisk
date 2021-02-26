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

#include "simulation/environment/groundLocation/groundLocation.h"
#include "architecture/utilities/avsEigenSupport.h"
#include "architecture/utilities/linearAlgebra.h"


/*! @brief Creates an instance of the GroundLocation class with a minimum elevation of 10 degrees,
 @return void
 */
GroundLocation::GroundLocation()
{
    //! - Set some default initial conditions:
    this->minimumElevation = 10.*D2R; // [rad] minimum elevation above the local horizon needed to see a spacecraft; defaults to 10 degrees
    this->maximumRange = -1; // [m] Maximum range for the groundLocation to compute access.

    this->currentGroundStateBuffer = this->currentGroundStateOutMsg.zeroMsgPayload;

    this->planetRadius = REQ_EARTH*1e3;

    this->r_LP_P.fill(0.0);
    this->r_LP_P_Init.fill(0.0);

    this->planetState = this->planetInMsg.zeroMsgPayload;
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
    for (int c=0; c<this->accessOutMsgs.size(); c++) {
        delete this->accessOutMsgs.at(c);
    }
    return;
}

/*! Resets the internal position to the specified initial position.*/
void GroundLocation::Reset(uint64_t CurrentSimNanos)
{
    this->r_LP_P = this->r_LP_P_Init;
    
    if (this->planetRadius < 0) {
        bskLogger.bskLog(BSK_ERROR, "GroundLocation module must have planetRadius set.");
    }
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
    this->dcm_LP = C_PCPF2SEZ(lat, longitude);
}


/*! Adds a scState message name to the vector of names to be subscribed to. Also creates a corresponding access message output name.
*/
void GroundLocation::addSpacecraftToModel(Message<SCStatesMsgPayload> *tmpScMsg)
{
    this->scStateInMsgs.push_back(tmpScMsg->addSubscriber());

    /* create output message */
    Message<AccessMsgPayload> *msg;
    msg = new Message<AccessMsgPayload>;
    this->accessOutMsgs.push_back(msg);

    /* expand the buffer vector */
    AccessMsgPayload accMsg;
    this->accessMsgBuffer.push_back(accMsg);
}


/*! Read module messages
*/
bool GroundLocation::ReadMessages()
{
    SCStatesMsgPayload scMsg;

    /* clear out the vector of spacecraft states.  This is created freshly below. */
    this->scStatesBuffer.clear();

    //! - read in the spacecraft state messages
    bool scRead;
    if(!this->scStateInMsgs.empty())
    {
        scRead = true;
        for (int c = 0; c < this->scStateInMsgs.size(); c++) {
            scMsg = this->scStateInMsgs.at(c)();
            scRead = scRead && this->scStateInMsgs.at(c).isWritten();
            this->scStatesBuffer.push_back(scMsg);
        }
    } else {
        bskLogger.bskLog(BSK_ERROR, "Ground location has no spacecraft to track.");
        scRead = false;
    }
    //! - Read in the optional planet message.  if no planet message is set, then a zero planet position, velocity and orientation is assumed
    bool planetRead = true;
    if(this->planetInMsg.isLinked())
    {
        planetRead = this->planetInMsg.isWritten();
        this->planetState = this->planetInMsg();
    }

    return(planetRead && scRead);
}

/*! write module messages
*/
void GroundLocation::WriteMessages(uint64_t CurrentClock)
{
    //! - write access message for each spacecraft
    for (int c=0; c< this->accessMsgBuffer.size(); c++) {
        this->accessOutMsgs.at(c)->write(&this->accessMsgBuffer.at(c), this->moduleID, CurrentClock);
    }
    this->currentGroundStateOutMsg.write(&this->currentGroundStateBuffer, this->moduleID, CurrentClock);
}

void GroundLocation::updateInertialPositions()
{
    // First, get the rotation matrix from the inertial to planet frame from SPICE:
    this->dcm_PN = cArray2EigenMatrix3d(*this->planetState.J20002Pfix);
    this->r_PN_N = cArray2EigenVector3d(this->planetState.PositionVector);
    // Then, transpose it to get the planet to inertial frame
    this->r_LP_N = this->dcm_PN.transpose() * this->r_LP_P_Init;
    this->rhat_LP_N = this->r_LP_N/this->r_LP_N.norm();
    this->r_LN_N = this->r_PN_N + this->r_LP_N;
    //  Stash updated position in the groundState message
    eigenVector3d2CArray(this->r_LN_N, this->currentGroundStateBuffer.r_LN_N);
    eigenVector3d2CArray(this->r_LP_N, this->currentGroundStateBuffer.r_LP_N);
}

void GroundLocation::computeAccess()
{
    // Update the groundLocation's inertial position
    this->updateInertialPositions();

    // Iterate over spacecraft position messages and compute the access for each one
    std::vector<AccessMsgPayload>::iterator accessMsgIt;
    std::vector<SCStatesMsgPayload>::iterator scStatesMsgIt;
    for(scStatesMsgIt = this->scStatesBuffer.begin(), accessMsgIt = accessMsgBuffer.begin(); scStatesMsgIt != scStatesBuffer.end(); scStatesMsgIt++, accessMsgIt++){
        //! Compute the relative position of each spacecraft to the site in the planet-centered inertial frame
        Eigen::Vector3d r_BP_N = cArray2EigenVector3d(scStatesMsgIt->r_BN_N) - this->r_PN_N;
        Eigen::Vector3d r_BL_N = r_BP_N - this->r_LP_N;
        auto r_BL_mag = r_BL_N.norm();
        Eigen::Vector3d relativeHeading_N = r_BL_N / r_BL_mag;

        double viewAngle = (M_PI_2-acos(this->rhat_LP_N.dot(relativeHeading_N)));

        if( (viewAngle > this->minimumElevation) && (r_BL_mag <= this->maximumRange || this->maximumRange < 0)){
            accessMsgIt->hasAccess = 1;
            accessMsgIt->slantRange = r_BL_N.norm();
            accessMsgIt->elevation = viewAngle;

            Eigen::Vector3d sezPosition = this->dcm_LP * this->dcm_PN * r_BL_N;
            double cos_az = -sezPosition[0]/(sqrt(pow(sezPosition[0],2) + pow(sezPosition[1],2)));
            double sin_az = sezPosition[1]/(sqrt(pow(sezPosition[0],2) + pow(sezPosition[1],2)));
            accessMsgIt->azimuth = atan2(sin_az, cos_az);
        }
        else
        {
            accessMsgIt->hasAccess = 0;
            accessMsgIt->slantRange = 0.0;
            accessMsgIt->elevation = 0.0;
            accessMsgIt->azimuth = 0.0;
        }
    }
}

/*!
 update module 
 @param CurrentSimNanos
 */
void GroundLocation::UpdateState(uint64_t CurrentSimNanos)
{
    this->ReadMessages();
    this->computeAccess();
    this->WriteMessages(CurrentSimNanos);

}
