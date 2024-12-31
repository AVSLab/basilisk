/*
 ISC License

 Copyright (c) 2022, Autonomous Vehicle Systems Lab, University of Colorado Boulder

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


#include "simulation/environment//groundMapping/groundMapping.h"
#include "architecture/utilities/avsEigenSupport.h"
#include "architecture/utilities/linearAlgebra.h"
#include "architecture/utilities/rigidBodyKinematics.h"
#include <iostream>
#include <math.h>

/*! This is the constructor for the module class.  It sets default variable
    values and initializes the various parts of the model */
GroundMapping::GroundMapping()
{
    //! - Set some default initial conditions:
    this->minimumElevation = 0.*D2R; // [rad] minimum elevation above the local horizon needed to see a spacecraft; defaults to 0 degrees
    this->maximumRange = -1; // [m] Maximum range for the groundLocation to compute access.
    this->halfFieldOfView = 10.*D2R;  // [rad] half-angle field-of-view of the instrument
    this->cameraPos_B.setZero(3);  // Default to zero
    this->nHat_B.setZero(3);  // Default to zero

    this->planetInMsgBuffer = this->planetInMsg.zeroMsgPayload;
    this->planetInMsgBuffer.J20002Pfix[0][0] = 1;
    this->planetInMsgBuffer.J20002Pfix[1][1] = 1;
    this->planetInMsgBuffer.J20002Pfix[2][2] = 1;

    this->scStateInMsgBuffer = this->scStateInMsg.zeroMsgPayload;
}

/*! Module Destructor */
GroundMapping::~GroundMapping()
{
    for (long unsigned int c = 0; c < this->accessOutMsgs.size(); c++) {
        delete this->accessOutMsgs.at(c);
        delete this->currentGroundStateOutMsgs.at(c);
    }
}

/*! This method is used to reset the module and checks that required input messages are connect.

*/
void GroundMapping::Reset(uint64_t CurrentSimNanos)
{
    // check that required input messages are connected
    if (!this->scStateInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "GroundMapping.scStateInMsg was not linked.");
    }

    // check that the direction of the camera is provided
    if (this->nHat_B.isZero()){
        bskLogger.bskLog(BSK_ERROR, "GroundMapping.nHat_B vector not set.");
    }
}

/*! Read module messages
*/
void GroundMapping::ReadMessages(){

    if(this->planetInMsg.isLinked())
    {
        this->planetInMsgBuffer = this->planetInMsg();
    }

    this->scStateInMsgBuffer = this->scStateInMsg();
}

/*! Method to add map points
 * @param r_LP_P_init: mapping point in planet-fixed frame
 */
void GroundMapping::addPointToModel(Eigen::Vector3d& r_LP_P_init){
    /* Add the mapping point */
    this->mappingPoints.push_back(r_LP_P_init);

    /* Create buffer output messages */
    Message<AccessMsgPayload> *msg;
    msg = new Message<AccessMsgPayload>;
    this->accessOutMsgs.push_back(msg);

    /* Expand the access buffer vectors */
    AccessMsgPayload accMsg;
    this->accessMsgBuffer.push_back(accMsg);

    /* Create ground state output message */
    Message<GroundStateMsgPayload> *msg_2;
    msg_2 = new Message<GroundStateMsgPayload>;
    this->currentGroundStateOutMsgs.push_back(msg_2);

    /* Expand the ground state buffer vectors */
    GroundStateMsgPayload groundMsg;
    this->currentGroundStateMsgBuffer.push_back(groundMsg);
}

/*! Method to compute whether or not the spacecraft has access to the location
 * @param c: index of the given location
 */
void GroundMapping::computeAccess(uint64_t c){
    //! Zero the output message buffers
    this->accessMsgBuffer.at(c) = this->accessOutMsgs.at(c)->zeroMsgPayload;
    this->currentGroundStateMsgBuffer.at(c) = this->currentGroundStateOutMsgs.at(c)->zeroMsgPayload;

    //! Compute the planet to inertial frame location position
    this->r_LP_N = this->dcm_PN.transpose() * this->mappingPoints[c];
    this->rhat_LP_N = this->r_LP_N/this->r_LP_N.norm();
    this->r_LN_N = this->r_PN_N + this->r_LP_N;

    //!  Stash updated position in the groundState message
    eigenVector3d2CArray(this->r_LN_N, this->currentGroundStateMsgBuffer.at(c).r_LN_N);
    eigenVector3d2CArray(this->r_LP_N, this->currentGroundStateMsgBuffer.at(c).r_LP_N);

    //! Compute the relative position of each spacecraft to the site in the planet-centered inertial frame
    Eigen::Vector3d r_BL_N = r_BP_N - this->r_LP_N;
    auto r_BL_mag = r_BL_N.norm();
    Eigen::Vector3d relativeHeading_N = r_BL_N / r_BL_mag;

    double viewAngle = (M_PI_2-safeAcos(this->rhat_LP_N.dot(relativeHeading_N)));

    this->accessMsgBuffer.at(c).slantRange = r_BL_mag;
    this->accessMsgBuffer.at(c).elevation = viewAngle;
    Eigen::Vector3d r_BL_L = this->dcm_LP * this->dcm_PN * r_BL_N;
    eigenVector3d2CArray(r_BL_L, this->accessMsgBuffer.at(c).r_BL_L);
    double cos_az = -r_BL_L[0]/(sqrt(pow(r_BL_L[0],2) + pow(r_BL_L[1],2)));
    double sin_az = r_BL_L[1]/(sqrt(pow(r_BL_L[0],2) + pow(r_BL_L[1],2)));
    this->accessMsgBuffer.at(c).azimuth = atan2(sin_az, cos_az);

    Eigen::Vector3d v_BL_L = this->dcm_LP * this->dcm_PN * (cArray2EigenVector3d(scStateInMsgBuffer.v_BN_N) - this->w_PN.cross(r_BP_N)); // V observed from gL wrt P frame, expressed in L frame coords (SEZ)
    eigenVector3d2CArray(v_BL_L, this->accessMsgBuffer.at(c).v_BL_L);
    this->accessMsgBuffer.at(c).range_dot = v_BL_L.dot(r_BL_L)/r_BL_mag;
    double xy_norm = sqrt(pow(r_BL_L[0],2)+pow(r_BL_L[1],2));
    this->accessMsgBuffer.at(c).az_dot = (-r_BL_L[0]*v_BL_L[1] + r_BL_L[1]*v_BL_L[0])/pow(xy_norm,2);
    this->accessMsgBuffer.at(c).el_dot = (v_BL_L[2]/xy_norm - r_BL_L[2]*(r_BL_L[0]*v_BL_L[0] + r_BL_L[1]*v_BL_L[1])/pow(xy_norm,3))/(1+pow(r_BL_L[2]/xy_norm,2));

    uint64_t within_view = this->checkInstrumentFOV();

    if( (viewAngle > this->minimumElevation) && (r_BL_mag <= this->maximumRange || this->maximumRange < 0) && within_view){
        this->accessMsgBuffer.at(c).hasAccess = 1;
    }
    else
    {
        this->accessMsgBuffer.at(c).hasAccess = 0;
    }

}

/*! Method to update the relevant rotations matrices
 */
void GroundMapping::updateInertialPositions()
{
    // First, get the rotation matrix from the inertial to planet frame from SPICE:
    this->dcm_PN = cArray2EigenMatrix3d(*this->planetInMsgBuffer.J20002Pfix);
    this->dcm_PN_dot = cArray2EigenMatrix3d(*this->planetInMsgBuffer.J20002Pfix_dot);
    this->r_PN_N = cArray2EigenVector3d(this->planetInMsgBuffer.PositionVector);

    // Compute the position of the body frame to the planet in the inertial frame
    this->r_BP_N = cArray2EigenVector3d(scStateInMsgBuffer.r_BN_N) - this->r_PN_N;
    double dcm_BN[3][3];
    MRP2C(scStateInMsgBuffer.sigma_BN, dcm_BN);
    this->dcm_NB = cArray2EigenMatrixXd(*dcm_BN, 3, 3);

    // Get planet frame angular velocity vector
    Eigen::Matrix3d w_tilde_PN = - this->dcm_PN_dot * this->dcm_PN.transpose();
    this->w_PN << w_tilde_PN(2,1), w_tilde_PN(0,2), w_tilde_PN(1,0);
}

/*! Method to compute whether or not the mapping point is in the instrument's FOV
 */
uint64_t GroundMapping::checkInstrumentFOV(){
    /* Compute the projection of the mapping point along the instrument's boresight */
    double boresightNormalProj = ((this->r_LP_N - (this->r_BP_N + this->dcm_NB*cameraPos_B)).transpose())*(dcm_NB*this->nHat_B);

    /* Check that the normal projection is within the maximum range*/
    if ((boresightNormalProj >= 0) && (boresightNormalProj <= this->maximumRange || this->maximumRange < 0)){
        /* Compute the radius of the instrument's cone at the projection distance */
        double coneRadius = boresightNormalProj*tan(this->halfFieldOfView);
        double orthDistance = (this->r_LP_N - (this->r_BP_N + this->dcm_NB*cameraPos_B)  - boresightNormalProj*dcm_NB*this->nHat_B).norm();
        /* Check that the orthogonal distance is less than the cone radius */
        if (orthDistance < coneRadius) {
            return 1;
        } else {
            return 0;
        }
    }
    /* If the first condition does not pass, return 0 */
    else{
        return 0;
    }
}

/*! write module messages
*/
void GroundMapping::WriteMessages(uint64_t CurrentClock)
{
    //! - write access message for each spacecraft
    for (long unsigned int c=0; c< this->accessMsgBuffer.size(); c++) {
        this->accessOutMsgs.at(c)->write(&this->accessMsgBuffer.at(c), this->moduleID, CurrentClock);
        this->currentGroundStateOutMsgs.at(c)->write(&this->currentGroundStateMsgBuffer.at(c), this->moduleID, CurrentClock);
    }

}

/*! This is the main method that gets called every time the module is updated.  Provide an appropriate description.

*/
void GroundMapping::UpdateState(uint64_t CurrentSimNanos)
{
    // Read messages
    this->ReadMessages();

    // Update the inertial positions
    this->updateInertialPositions();

    // Loop through each mapping point and perform computations
    for (long unsigned int c = 0; c < this->mappingPoints.size(); c++) {
        this->computeAccess(c);
    }

    // Write output messages
    this->WriteMessages(CurrentSimNanos);
}
