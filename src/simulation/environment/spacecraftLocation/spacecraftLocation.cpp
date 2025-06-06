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
    this->min_shadow_factor = -1.0;

    this->planetState = this->planetInMsg.zeroMsgPayload;
    this->planetState.J20002Pfix[0][0] = 1;
    this->planetState.J20002Pfix[1][1] = 1;
    this->planetState.J20002Pfix[2][2] = 1;
}

/*! Empty destructor method.

 */
SpacecraftLocation::~SpacecraftLocation()
{
    for (long unsigned int c = 0; c < this->accessOutMsgs.size(); c++) {
        delete this->accessOutMsgs.at(c);
    }
    return;
}

/*! Resets the internal position to the specified initial position.*/
void
SpacecraftLocation::Reset(uint64_t CurrentSimNanos)
{
    if (this->scStateInMsgs.size() == 0) {
        bskLogger.bskLog(
          BSK_ERROR,
          "SpacecraftLocation module must have at least one spacecraft added through `addSpacecraftToModel`");
    }

    if (!this->primaryScStateInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "SpacecraftLocation module must have `primaryScStateInMsg` connected.");
    }

    if (this->rEquator < 0.0) {
        bskLogger.bskLog(BSK_ERROR, "SpacecraftLocation rEquator must be set to the planet equatorial radius");
    }
    /* if the polar radius is not specified, then it is set equal to the equatorial radius */
    if (this->rEquator > 0.0 && this->rPolar < 0.0) {
        this->rPolar = rEquator;
    }
    this->zScale = this->rEquator / this->rPolar;

    if (this->aHat_B.norm() > 0.1) {
        if (this->theta < 0.0) {
            bskLogger.bskLog(BSK_ERROR, "SpacecraftLocation must set theta if you specify aHat_B");
        }
        this->aHat_B.normalize();
    }

    if (this->theta_solar >= 0.0) {
        if (this->aHat_B.norm() < 0.001) {
            bskLogger.bskLog(BSK_ERROR, "SpacecraftLocation must set aHat_B if you specify theta_solar");
        }
    }
}

/*! Adds a scState message name to the vector of names to be subscribed to. Also creates a corresponding access message
 * output name.
 */
void
SpacecraftLocation::addSpacecraftToModel(Message<SCStatesMsgPayload>* tmpScMsg)
{
    this->scStateInMsgs.push_back(tmpScMsg->addSubscriber());

    /* create output message */
    Message<AccessMsgPayload>* msg;
    msg = new Message<AccessMsgPayload>;
    this->accessOutMsgs.push_back(msg);

    /* expand the buffer vector */
    AccessMsgPayload accMsg;
    this->accessMsgBuffer.push_back(accMsg);
}

/*! Read module messages
 */
bool
SpacecraftLocation::ReadMessages()
{
    SCStatesMsgPayload scMsg;

    /* clear out the vector of spacecraft states.  This is created freshly below. */
    this->scStatesBuffer.clear();

    // read in primary spacecraft states
    this->primaryScStatesBuffer = this->primaryScStateInMsg();
    this->r_BN_N = cArray2EigenVector3d(this->primaryScStatesBuffer.r_BN_N);

    // read in the spacecraft state messages
    bool scRead;
    if (!this->scStateInMsgs.empty()) {
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
    //! - Read in the optional planet message.  if no planet message is set, then a zero planet position, velocity and
    //! orientation is assumed
    bool planetRead = true;
    if (this->planetInMsg.isLinked()) {
        planetRead = this->planetInMsg.isWritten();
        this->planetState = this->planetInMsg();
    }

    //! - Zero ephemeris information
    this->sunData = sunInMsg.zeroMsgPayload;

    bool sunRead = true;
    if (this->sunInMsg.isLinked()) {
        sunRead = this->sunInMsg.isWritten();
        this->sunData = this->sunInMsg();
    } else {
        this->r_HN_N.setZero();
    }

    //! - Zero eclipse information
    this->eclipseInMsgData = eclipseInMsg.zeroMsgPayload;

    bool eclipseRead = true;
    if (this->eclipseInMsg.isLinked()) {
        eclipseRead = this->eclipseInMsg.isWritten();
        this->eclipseInMsgData = this->eclipseInMsg();
    }


    return (planetRead && scRead && sunRead && eclipseRead);
}

/*! write module messages
 */
void
SpacecraftLocation::WriteMessages(uint64_t CurrentClock)
{
    //! - write access message for each spacecraft
    for (long unsigned int c = 0; c < this->accessMsgBuffer.size(); c++) {
        this->accessOutMsgs.at(c)->write(&this->accessMsgBuffer.at(c), this->moduleID, CurrentClock);
    }
}

/*! compute the spacecraft to spacecraft access messages
 */
void
SpacecraftLocation::computeAccess()
{
    Eigen::Vector3d r_LP_P; //!< [m] spacecraft Location relative to planet origin vector

    // get planet position and orientation relative to inertial frame
    this->dcm_PN = cArray2EigenMatrix3d(*this->planetState.J20002Pfix);
    this->r_PN_N = cArray2EigenVector3d(this->planetState.PositionVector);

    // get sun position in inertial frame from sunInMsg
    this->r_HN_N = cArray2EigenVector3d(this->sunData.PositionVector);

    // compute primary spacecraft relative to planet
    Eigen::MRPd sigma_BN = cArray2EigenMRPd(this->primaryScStatesBuffer.sigma_BN);
    Eigen::Matrix3d dcm_NB = sigma_BN.toRotationMatrix();
    r_LP_P = this->dcm_PN * (this->r_BN_N + dcm_NB * this->r_LB_B - this->r_PN_N);

    // do affine scaling to map ellipsoid to sphere
    r_LP_P[2] = r_LP_P[2] * this->zScale;

    // compute other spacecraft positions relative to planet
    for (long unsigned int c = 0; c < this->scStateInMsgs.size(); c++) {
        Eigen::Vector3d r_SN_N; // other satellite position relative to inertial
        Eigen::Vector3d r_SP_P; // other satellite position relative to planet
        Eigen::Vector3d r_SL_P; // other satellite position relative to primary spacecraft location L

        r_SN_N = cArray2EigenVector3d(this->scStatesBuffer.at(c).r_BN_N);
        r_SP_P = this->dcm_PN * (r_SN_N - this->r_PN_N);

        // do affine scaling to map ellipsoid to sphere
        r_SP_P[2] = r_SP_P[2] * this->zScale;

        r_SL_P = r_SP_P - r_LP_P;

        // compute point of closest approach
        double param; // line scaling parameter
        param = -r_LP_P.dot(r_SL_P) / r_SL_P.dot(r_SL_P);

        // check for out of bounds condition.
        param = std::min(param, 1.0); // If param > 1, the closest point on segment is the other satellite
        param = std::max(param, 0.0); // If param < 0, the closest point on segment is the primary satellite

        Eigen::Vector3d rClose = r_LP_P + param * r_SL_P;

        // determine access output message
        this->accessMsgBuffer.at(c) = this->accessOutMsgs.at(c)->zeroMsgPayload;
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
                Eigen::Vector3d aHat_P; // sensor axis in planet frame components
                double phi;             // angle between relative position vector and aHat
                aHat_P = this->dcm_PN * dcm_NB * this->aHat_B;
                phi = safeAcos(r_SL_P.dot(aHat_P) / range);
                this->accessMsgBuffer.at(c).elevation = M_PI_2 - phi;
                if (phi > this->theta) {
                    // other spacecraft is outside the cone field of view
                    this->accessMsgBuffer.at(c).hasAccess = 0;
                }
            }
        }

        this->accessMsgBuffer.at(c).hasIllumination = 0; // default to no illumination

        // Check illumination if sun message is present
        if (this->sunInMsg.isLinked()) {
            // Assume illumination conditions are met; then check for unmet conditions
            this->accessMsgBuffer.at(c).hasIllumination = 1;

            // aHat vector in inertial frame
            Eigen::Vector3d aHat_N = dcm_NB * this->aHat_B;

            // Vector from spacecraft to Sun
            Eigen::Vector3d r_SL_N = r_SN_N - this->r_BN_N;

            // Calculate the sun-incidence angle and spacecraft-view angle
            double sunIncidenceAngle = safeAcos(aHat_N.dot(this->r_HN_N) / (aHat_N.norm() * this->r_HN_N.norm()));
            double scViewAngle = safeAcos(aHat_N.dot(r_SL_N) / (aHat_N.norm() * r_SL_N.norm()));

            // Store the angles in the output buffer
            this->accessMsgBuffer.at(c).sunIncidenceAngle = sunIncidenceAngle;
            this->accessMsgBuffer.at(c).scViewAngle = scViewAngle;

            // Check if location is illuminated if threshold is set
            if (this->theta_solar >= 0.0) {
                if (sunIncidenceAngle > this->theta_solar) {
                    this->accessMsgBuffer.at(c).hasAccess = 0; // outside solar cone
                    this->accessMsgBuffer.at(c).hasIllumination = 0;
                }
            }

            // Check if eclipse is valid
            if (this->eclipseInMsg.isLinked() && this->min_shadow_factor > 0.0) {
                if (eclipseInMsgData.shadowFactor < this->min_shadow_factor) {
                    this->accessMsgBuffer.at(c).hasAccess = 0;
                    this->accessMsgBuffer.at(c).hasIllumination = 0;
                }
            }
        }
    }
}

/*!
 update module
 @param CurrentSimNanos
 */
void
SpacecraftLocation::UpdateState(uint64_t CurrentSimNanos)
{
    this->ReadMessages();
    this->computeAccess();
    this->WriteMessages(CurrentSimNanos);
}
