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
#include <iostream>

/*! @brief Creates an instance of the GroundLocation class with a minimum elevation of 10 degrees,

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

 */
GroundLocation::~GroundLocation()
{
    for (long unsigned int c=0; c<this->accessOutMsgs.size(); c++) {
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
//
//    if (!this->sunInMsg.isLinked()) {
//        bskLogger.bskLog(BSK_ERROR, "Eclipse: sunInMsg must be linked to sun Spice state message.");
//    }
}

/*! Specifies the ground location from planet-centered latitude, longitude, altitude position.
 *
 * @param lat
 * @param longitude
 * @param alt
 */
void GroundLocation::specifyLocation(double lat, double longitude, double alt)
{
    Eigen::Vector3d tmpLLAPosition(lat, longitude, alt);
    this->r_LP_P_Init = LLA2PCPF(tmpLLAPosition, this->planetRadius);
    this->dcm_LP = C_PCPF2SEZ(lat, longitude);
}

/*! Specifies the ground location from planet-centered, planet-fixed coordinates
 * @param r_LP_P_Loc
 */
void GroundLocation::specifyLocationPCPF(Eigen::Vector3d& r_LP_P_Loc){
    /* Assign to r_LP_P_Init */
    this->r_LP_P_Init = r_LP_P_Loc;

    /* Convert to LLA */
    Eigen::Vector3d tmpLLAPosition = PCPF2LLA(this->r_LP_P_Init, this->planetRadius);

    /* Compute dcm_LP */
    this->dcm_LP = C_PCPF2SEZ(tmpLLAPosition[0], tmpLLAPosition[1]);
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
        for (long unsigned int c = 0; c < this->scStateInMsgs.size(); c++) {
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

//    bool sunRead = true;
//    if (this->sunVectorInMsg.isLinked())
//    {
//        sunRead = this->sunVectorInMsg.isWritten();
//        this->sunInMsgState = this->sunInMsg();
//    } else {
//        sunRead = false;
//        this->sunVector_N.setZero();
//    }

//    return (planetRead && scRead && sunRead);
    return(planetRead && scRead);
}

/*! write module messages
*/
void GroundLocation::WriteMessages(uint64_t CurrentClock)
{
    //! - write access message for each spacecraft
    for (long unsigned int c=0; c< this->accessMsgBuffer.size(); c++) {
        this->accessOutMsgs.at(c)->write(&this->accessMsgBuffer.at(c), this->moduleID, CurrentClock);
    }
    this->currentGroundStateOutMsg.write(&this->currentGroundStateBuffer, this->moduleID, CurrentClock);
}

void GroundLocation::updateInertialPositions()
{
    // First, get the rotation matrix from the inertial to planet frame from SPICE:
    this->dcm_PN = cArray2EigenMatrix3d(*this->planetState.J20002Pfix);
    this->dcm_PN_dot = cArray2EigenMatrix3d(*this->planetState.J20002Pfix_dot);
    this->r_PN_N = cArray2EigenVector3d(this->planetState.PositionVector);
    // Then, transpose it to get the planet to inertial frame
    this->r_LP_N = this->dcm_PN.transpose() * this->r_LP_P_Init;
    this->rhat_LP_N = this->r_LP_N/this->r_LP_N.norm();
    this->r_LN_N = this->r_PN_N + this->r_LP_N;
    // Get planet frame angular velocity vector
    Eigen::Matrix3d w_tilde_PN = - this->dcm_PN_dot * this->dcm_PN.transpose();
    this->w_PN << w_tilde_PN(2,1), w_tilde_PN(0,2), w_tilde_PN(1,0);
    //  Stash updated position in the groundState message
    eigenVector3d2CArray(this->r_LN_N, this->currentGroundStateBuffer.r_LN_N);
    eigenVector3d2CArray(this->r_LP_N, this->currentGroundStateBuffer.r_LP_N);
}

void GroundLocation::computeAccess()
{
    // Update the groundLocation's inertial position
    this->updateInertialPositions();
//
//    // get sun position in inertial frame from sunInMsg
//    Eigen::Vector3d r_HN_N(this->sunInMsgState.PositionVector); // r_sun

    // Iterate over spacecraft position messages and compute the access for each one
    std::vector<AccessMsgPayload>::iterator accessMsgIt;
    std::vector<SCStatesMsgPayload>::iterator scStatesMsgIt;
    for(scStatesMsgIt = this->scStatesBuffer.begin(), accessMsgIt = accessMsgBuffer.begin(); scStatesMsgIt != scStatesBuffer.end(); scStatesMsgIt++, accessMsgIt++){
        //! Compute the relative position of each spacecraft to the site in the planet-centered inertial frame
        Eigen::Vector3d r_BP_N = cArray2EigenVector3d(scStatesMsgIt->r_BN_N) - this->r_PN_N;
        Eigen::Vector3d r_BL_N = r_BP_N - this->r_LP_N;
        auto r_BL_mag = r_BL_N.norm();
        Eigen::Vector3d relativeHeading_N = r_BL_N / r_BL_mag;

        double viewAngle = (M_PI_2-safeAcos(this->rhat_LP_N.dot(relativeHeading_N)));

        accessMsgIt->slantRange = r_BL_mag;
        accessMsgIt->elevation = viewAngle;
        Eigen::Vector3d r_BL_L = this->dcm_LP * this->dcm_PN * r_BL_N;
        eigenVector3d2CArray(r_BL_L, accessMsgIt->r_BL_L);
        double cos_az = -r_BL_L[0]/(sqrt(pow(r_BL_L[0],2) + pow(r_BL_L[1],2)));
        double sin_az = r_BL_L[1]/(sqrt(pow(r_BL_L[0],2) + pow(r_BL_L[1],2)));
        accessMsgIt->azimuth = atan2(sin_az, cos_az);

        Eigen::Vector3d v_BL_L = this->dcm_LP * this->dcm_PN * (cArray2EigenVector3d(scStatesMsgIt->v_BN_N) - this->w_PN.cross(r_BP_N)); // V observed from gL wrt P frame, expressed in L frame coords (SEZ)
        eigenVector3d2CArray(v_BL_L, accessMsgIt->v_BL_L);
        accessMsgIt->range_dot = v_BL_L.dot(r_BL_L)/r_BL_mag;
        double xy_norm = sqrt(pow(r_BL_L[0],2)+pow(r_BL_L[1],2));
        accessMsgIt->az_dot = (-r_BL_L[0]*v_BL_L[1] + r_BL_L[1]*v_BL_L[0])/pow(xy_norm,2);
        accessMsgIt->el_dot = (v_BL_L[2]/xy_norm - r_BL_L[2]*(r_BL_L[0]*v_BL_L[0] + r_BL_L[1]*v_BL_L[1])/pow(xy_norm,3))/(1+pow(r_BL_L[2]/xy_norm,2));

        if( (viewAngle > this->minimumElevation) && (r_BL_mag <= this->maximumRange || this->maximumRange < 0)){
            accessMsgIt->hasAccess = 1;
        }
        else
        {
            accessMsgIt->hasAccess = 0;
        }
    }
    //

//    // calculating the sun-incidence-angle and the deputy-view-angle
//    double sunIncidenceAngle = safeAcos(r_LP_N.dot(r_HN_N) / (r_LP_N.norm() * r_HN_N.norm()));
//    double scViewAngle = viewAngle //safeAcos(r_LP_N.dot(r_BL_N) / (r_LP_N.norm() * r_BL_N.norm()));
//
//    //storing the two angles in the output butter
//    this->accessMsgBuffer.at(c).sunIncidenceAngle = sunIncidenceAngle;
//    this->accessMsgBuffer.at(c).scViewAngle = scViewAngle;
//
//    // Compute scalar triple product
//    r_HN_N_normalized = r_HN_N/r_HN_N.norm()
//    r_BL_N_normalized = r_BL_N/r_BL_N.norm()
//    double scalarTripleProduct = (r_HN_N_normalized.cross(r_BL_N_normalized)).dot(r_LP_N);
//
//    // Check coplanarity & glare condition
//    bool isCoplanar = fabs(scalarTripleProduct) < 1e-3;  // Tolerance for numerical precision // TODO: we should think about slightly off-coplanar... it still has some glare
//    double epsilon = 10.0 * M_PI / 180.0; // [rad] if the two angles are within epsilon degrees then it will be glared
//    bool glare = isCoplanar && (fabs(sunIncidenceAngle - scViewAngle) * 180.0 / M_PI <= 10.0);
//
//    // GLARE AS A boolean (Float)
//    // Initialize glare variable as 0 (default: no glare)
//    double glare = 0.0;
//
//    // Check if either angle exceeds 90 degrees (π/2 radians)
//    if (sunIncidenceAngle > M_PI_2 || scViewAngle > M_PI_2)
//    {
//        glare = -1.0;  // Invalid case
//    }
//    // Check glare condition: angles within epsilon & coplanar
//    else if (isCoplanar && fabs(sunIncidenceAngle - scViewAngle) <= epsilon)
//    {
//        glare = 1.0;  // Glare detected
//    }

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
