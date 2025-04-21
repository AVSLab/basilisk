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

#include "simulation/environment/spacecraftLocation/spacecraftLocation.h"
#include "architecture/utilities/avsEigenSupport.h"
#include "architecture/utilities/linearAlgebra.h"

#include <iostream>

/*! @brief Creates an instance of the SpacecraftLocation class

 */
SpacecraftLocation::SpacecraftLocation()
{
    this->rEquator = -1.0;
    this->rPolar = -1.0;
    this->maximumRange = -1.0;

    this->r_LB_B.fill(0.0);
    this->aHat_B.fill(0.0);
    this->theta = -1.0;
    this->theta_solar = -1.0;
    this->shadow_factor_limit = 1.0;

    this->planetState = this->planetInMsg.zeroMsgPayload;
    this->planetState.J20002Pfix[0][0] = 1;
    this->planetState.J20002Pfix[1][1] = 1;
    this->planetState.J20002Pfix[2][2] = 1;

}

/*! Empty destructor method.

 */
SpacecraftLocation::~SpacecraftLocation()
{
    for (long unsigned int c=0; c<this->accessOutMsgs.size(); c++) {
        delete this->accessOutMsgs.at(c);
        delete this->illuminationOutMsgs.at(c);
    }
    return;
}

/*! Resets the internal position to the specified initial position.*/
void SpacecraftLocation::Reset(uint64_t CurrentSimNanos)
{
    if (this->scStateInMsgs.size() == 0) {
        bskLogger.bskLog(BSK_ERROR, "SpacecraftLocation module must have at least one spacecraft added through `addSpacecraftToModel`");
    }

    if (!this->primaryScStateInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "SpacecraftLocation module must have `primaryScStateInMsg` connected.");
    }

    if (this->rEquator < 0.0) {
        bskLogger.bskLog(BSK_ERROR, "SpacecraftLocation rEquator must be set to the planet equatorial radius");
    }
    /* if the polar radius is not specified, then it is set equal to the equatorial radius */
    if (this->rEquator > 0.0 && this->rPolar<0.0) {
        this->rPolar = rEquator;
    }
    this->zScale = this->rEquator / this->rPolar;

    if (this->aHat_B.norm() > 0.1 ) {
        if (this->theta < 0.0) {
            bskLogger.bskLog(BSK_ERROR, "SpacecraftLocation must set theta if you specify aHat_B");
        }
        this->aHat_B.normalize();
    }

    if (!this->sunInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "Eclipse: sunInMsg must be linked to sun Spice state message.");
    }

    if (!this->eclipseInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "No ECLIPSE.");
    }

}


/*! Adds a scState message name to the vector of names to be subscribed to. Also creates a corresponding access message output name.
*/
void SpacecraftLocation::addSpacecraftToModel(Message<SCStatesMsgPayload> *tmpScMsg)
{
    this->scStateInMsgs.push_back(tmpScMsg->addSubscriber());

    /* create output message */
    Message<AccessMsgPayload> *msg;
    msg = new Message<AccessMsgPayload>;
    this->accessOutMsgs.push_back(msg);
    Message<AccessMsgPayload> *ilmsg;
    ilmsg = new Message<AccessMsgPayload>;
    this->illuminationOutMsgs.push_back(ilmsg);

    /* expand the buffer vector */
    AccessMsgPayload accMsg;
    this->accessMsgBuffer.push_back(accMsg);
    AccessMsgPayload illMsg;
    this->illuminationMsgBuffer.push_back(illMsg);
}


/*! Read module messages
*/
bool SpacecraftLocation::ReadMessages()
{
    SCStatesMsgPayload scMsg;

    /* clear out the vector of spacecraft states.  This is created freshly below. */
    this->scStatesBuffer.clear();

    // read in primary spacecraft states
    this->primaryScStatesBuffer = this->primaryScStateInMsg();
    this->r_BN_N = cArray2EigenVector3d(this->primaryScStatesBuffer.r_BN_N);

    // read in the spacecraft state messages
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
        bskLogger.bskLog(BSK_ERROR, "Spacecraft location has no other spacecraft to track.");
        scRead = false;
    }
    //! - Read in the optional planet message.  if no planet message is set, then a zero planet position, velocity and orientation is assumed
    bool planetRead = true;
    if(this->planetInMsg.isLinked())
    {
        planetRead = this->planetInMsg.isWritten();
        this->planetState = this->planetInMsg();
    }

    bool sunRead = true;
    if (this->sunInMsg.isLinked())
    {
        sunRead = this->sunInMsg.isWritten();
        //this->sunVector_N = cArray2EigenVector3d(this->sunVectorInMsg().sunVector);
        this->sunInMsgState = this->sunInMsg();
    } else {
        sunRead = false;
//        this->sunInMsgState.setZero();
    }

    bool eclipseRead = true;
    if (this->eclipseInMsg.isLinked())
    {
        eclipseRead = this->eclipseInMsg.isWritten();
        //this->sunVector_N = cArray2EigenVector3d(this->sunVectorInMsg().sunVector);
        this->eclipseInMsgState = this->eclipseInMsg();
    } else {
        eclipseRead = false;
//        this->sunInMsgState.setZero();
    }

    return (planetRead && scRead && sunRead && eclipseRead);
//    return(planetRead && scRead);

}

/*! write module messages
*/
void SpacecraftLocation::WriteMessages(uint64_t CurrentClock)
{
    //! - write access message for each spacecraft
    for (long unsigned int c=0; c< this->accessMsgBuffer.size(); c++) {
        this->accessOutMsgs.at(c)->write(&this->accessMsgBuffer.at(c), this->moduleID, CurrentClock);
        this->illuminationOutMsgs.at(c)->write(&this->illuminationMsgBuffer.at(c), this->moduleID, CurrentClock);
    }
}

/*! compute the spacecraft to spacecraft access messages
 */
void SpacecraftLocation::computeAccess()
{
    Eigen::Vector3d r_LP_P; //!< [m] spacecraft Location relative to planet origin vector

    // get planet position and orientation relative to inertial frame
    this->dcm_PN = cArray2EigenMatrix3d(*this->planetState.J20002Pfix);
    this->r_PN_N = cArray2EigenVector3d(this->planetState.PositionVector);

    // get sun position in inertial frame from sunInMsg
    Eigen::Vector3d r_HN_N(this->sunInMsgState.PositionVector); // r_sun

    // compute primary spacecraft relative to planet
    Eigen::MRPd sigma_BN = cArray2EigenMRPd(this->primaryScStatesBuffer.sigma_BN);
    Eigen::Matrix3d dcm_NB = sigma_BN.toRotationMatrix();
    r_LP_P = this->dcm_PN * (this->r_BN_N + dcm_NB * this->r_LB_B - this->r_PN_N);

    // do affine scaling to map ellipsoid to sphere
    r_LP_P[2] = r_LP_P[2] * this->zScale;

    // compute other spacecraft positions relative to planet
    for (long unsigned int c=0; c < this->scStateInMsgs.size(); c++) {
        Eigen::Vector3d r_SN_N;     // other satellite position relative to inertial
        Eigen::Vector3d r_SP_P;     // other satellite position relative to planet
        Eigen::Vector3d r_SL_P;     // other satellite position relative to primary spacecraft location L

        r_SN_N = cArray2EigenVector3d(this->scStatesBuffer.at(c).r_BN_N);
        r_SP_P = this->dcm_PN * (r_SN_N - this->r_PN_N);

        // do affine scaling to map ellipsoid to sphere
        r_SP_P[2] = r_SP_P[2] * this->zScale;

        r_SL_P = r_SP_P - r_LP_P;

        // compute point of closest approach
        double param;               // line scaling parameter
        param = - r_LP_P.dot(r_SL_P)/r_SL_P.dot(r_SL_P);

        // check for out of bounds condition.
        param = std::min(param, 1.0); // If param > 1, the closest point on segment is the other satellite
        param = std::max(param, 0.0); // If param < 0, the closest point on segment is the primary satellite

        Eigen::Vector3d rClose = r_LP_P + param * r_SL_P;

        // determine access output message
        this->accessMsgBuffer.at(c) = this->accessOutMsgs.at(c)->zeroMsgPayload;
        this->illuminationMsgBuffer.at(c) = this->illuminationOutMsgs.at(c)->zeroMsgPayload;
        if (rClose.norm() > this->rEquator) {
            r_SL_P[2] = r_SL_P[2] / this->zScale;
            double range = r_SL_P.norm();
            this->accessMsgBuffer.at(c).slantRange = range;
            this->accessMsgBuffer.at(c).hasAccess = 1;

            // check for out of range condition
            if (this->maximumRange > 0 && range > this->maximumRange) {
                this->accessMsgBuffer.at(c).hasAccess = 0;
            }

            // check if other spacecraft is within sensor/communication boresight axis
            if (this->theta > 0.0) {
                Eigen::Vector3d aHat_P;     // sensor axis in planet frame components
                double phi;                 // angle between relative positin vector and aHat
                aHat_P = this->dcm_PN * dcm_NB * this->aHat_B;
                phi = safeAcos(r_SL_P.dot(aHat_P) / range);
                this->accessMsgBuffer.at(c).elevation = M_PI_2 - phi;
                if (phi > this->theta) {
                    // other spacecraft is outside the cone field of view
                    this->accessMsgBuffer.at(c).hasAccess = 0;
                }
            }
        }

        // aHat vector in inertial frame
        Eigen::Vector3d aHat_N = dcm_NB * this->aHat_B;

        Eigen::Vector3d r_SL_N = r_SN_N - this->r_BN_N;

        // calculating the sun-incidence-angle and the deputy-view-angle
        double sunIncidenceAngle = safeAcos(aHat_N.dot(r_HN_N) / (aHat_N.norm() * r_HN_N.norm()));
        double scViewAngle = safeAcos(aHat_N.dot(r_SL_N) / (aHat_N.norm() * r_SL_N.norm()));


        this->illuminationMsgBuffer.at(c).hasAccess = 1;
        if (this->theta_solar > 0.0) {
            // check if the sun is within the solar cone
            if (sunIncidenceAngle > this->theta_solar) {
                this->accessMsgBuffer.at(c).hasAccess = 0;
                this->illuminationMsgBuffer.at(c).hasAccess = 0;
            }
        }

        if (this->shadow_factor_limit < 1.0) {
            // check if the shadow factor is within the limit
            // std::cout << "shadow factor a: " << this->eclipseInMsgState.shadowFactor << std::endl;
            if (eclipseInMsgState.shadowFactor >= this->shadow_factor_limit) {
                this->accessMsgBuffer.at(c).hasAccess = 0;
                this->illuminationMsgBuffer.at(c).hasAccess = 0;
            }
        }

        //storing the two angles in the output butter
        this->accessMsgBuffer.at(c).sunIncidenceAngle = sunIncidenceAngle;
        this->accessMsgBuffer.at(c).scViewAngle = scViewAngle;

        // Compute scalar triple product
        Eigen::Vector3d r_HN_N_normalized = r_HN_N/r_HN_N.norm();
        Eigen::Vector3d r_SL_N_normalized = r_SL_N/r_SL_N.norm();
        double scalarTripleProduct = (r_HN_N_normalized.cross(r_SL_N_normalized)).dot(aHat_N);

        // Check coplanarity & glare condition
        bool isCoplanar = fabs(scalarTripleProduct) < 1e-3;  // Tolerance for numerical precision // TODO: we should think about slightly off-coplanar... it still has some glare
        double epsilon = 10.0 * M_PI / 180.0; // [rad] if the two angles are within epsilon degrees then it will be glared
//        bool glare = isCoplanar && (fabs(sunIncidenceAngle - scViewAngle) * 180.0 / M_PI <= 10.0);

//        // GLARE AS A BOOLEAN
//        // Glare occurs if angles are within epsilon and vectors are coplanar
//        bool glare = isCoplanar && (std::abs(sunIncidenceAngle - scViewAngle) < 10.0 * M_PI / 180.0);

        // GLARE AS A DOUBLe
        // Initialize glare variable as 0 (default: no glare)
        double glare = 0.0;

        // Check if either angle exceeds 90 degrees (π/2 radians)
        if (sunIncidenceAngle > M_PI_2 || scViewAngle > M_PI_2)
        {
            glare = -1.0;  // Invalid case
        }
        // Check glare condition: angles within epsilon & coplanar
        else if (isCoplanar && fabs(sunIncidenceAngle - scViewAngle) <= epsilon)
        {
            glare = 1.0;  // Glare detected
        }

//        ////GLARE AS A FLOAT with linear transition between 'glare' and 'no glare'////
//        // Compute cosine similarity (normalized dot product) for glare intensity
//        double sunAlignment = fabs(aHat_N.dot(this->sunVector_N) / (aHat_N.norm() * this->sunVector_N.norm())); // Range [0,1]
//        double deputyAlignment = fabs(aHat_N.dot(r_SL_N) / (aHat_N.norm() * r_SL_N.norm())); // Range [0,1]
//
//        // Compute glare as a continuous value between 0 and 1
//        double glare = 0.0;  // Default: no glare
//
//        // If either angle is greater than 90° (π/2), set glare to -1
//        if (sunIncidenceAngle > M_PI_2 || scViewAngle > M_PI_2)
//        {
//            glare = -1.0;
//        }
//        // If coplanar and within epsilon, compute glare intensity
//        else if (isCoplanar && fabs(sunIncidenceAngle - scViewAngle) <= epsilon)
//        {
//            glare = 1.0 - fabs(sunAlignment - deputyAlignment);  // Higher similarity → Higher glare
//        }
//
//        // Store results in output message
        this->accessMsgBuffer.at(c).glare = glare;  // Store glare result
    }
}

/*!
 update module
 @param CurrentSimNanos
 */
void SpacecraftLocation::UpdateState(uint64_t CurrentSimNanos)
{
    this->ReadMessages();
    this->computeAccess();
    this->WriteMessages(CurrentSimNanos);

}
