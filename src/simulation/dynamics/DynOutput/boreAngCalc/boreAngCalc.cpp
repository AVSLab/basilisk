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

#include "simulation/dynamics/DynOutput/boreAngCalc/boreAngCalc.h"
#include "architecture/utilities/linearAlgebra.h"

//! The constructor.  Note that you have to overwrite the message names.
BoreAngCalc::BoreAngCalc()
{
    CallCounts = 0;

    // Initialize the pointing vectors
    this->boreVec_Po.setZero();
    this->inertialHeadingVec_N.setZero();

    // Zero the payloads
    this->localPlanet = this->celBodyInMsg.zeroMsgPayload;
    this->localState = this->scStateInMsg.zeroMsgPayload;
}

//! The destructor.
BoreAngCalc::~BoreAngCalc() = default;


/*! This method is used to reset the module.

 */
void BoreAngCalc::Reset(uint64_t CurrentSimNanos)
{
    // check if required input messages have not been included
    if (!this->scStateInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "boreAngCalc.scStateInMsg was not linked.");
    }

    if (this->celBodyInMsg.isLinked()) {
        this->useCelestialHeading = true;
    }
    else if (this->inertialHeadingVec_N.norm() > 1e-8) {
        this->useInertialHeading = true;
    }
    else {
        bskLogger.bskLog(BSK_ERROR, "Either boreAngCalc.celBodyInMsg was not linked or boreAngCalc.inertialHeadingVec_N was not set.");
    }

}

/*! This method writes the output data out into the messaging system.

 @param CurrentClock The current time in the system for output stamping
 */
void BoreAngCalc::WriteOutputMessages(uint64_t CurrentClock)
{
    this->angOutMsg.write(&this->boresightAng, this->moduleID, CurrentClock);
}

/*! This method reads the input messages in from the system and sets the
 appropriate parameters

 */
void BoreAngCalc::ReadInputs()
{
    //! - Read the input message into the correct pointer
    this->localState = this->scStateInMsg();
    bool celBodyMsgGood = false;
    if (this->useCelestialHeading) {
        this->localPlanet = this->celBodyInMsg();
        celBodyMsgGood = this->celBodyInMsg.isWritten();
    }

    this->inputsGood = this->scStateInMsg.isWritten() && (celBodyMsgGood || this->useInertialHeading);
}

/*! This method computes the vector specified in the input file in the LVLH
    reference frame of the spacecraft above the target celestial body.  This
    is used later to compute how far off that vector is in an angular sense.

*/
void BoreAngCalc::computeCelestialAxisPoint()
{
    // Convert planet and body data to Eigen variables
    Eigen::Vector3d r_BN_N = cArray2EigenVector3d(this->localState.r_BN_N); // spacecraft's inertial position
    Eigen::Vector3d v_BN_N = cArray2EigenVector3d(this->localState.v_BN_N); // spacecraft''s inertial velocity
    Eigen::Vector3d r_PN_N = cArray2EigenVector3d(this->localPlanet.PositionVector);    // planet's inertial position
    Eigen::Vector3d v_PN_N = cArray2EigenVector3d(this->localPlanet.VelocityVector);    // planet's inertial velocity

    // Compute the relative vectors
    Eigen::Vector3d r_PB_N = r_PN_N - r_BN_N;   // relative position vector
    Eigen::Vector3d rHat_PB_N = r_PB_N.normalized();    // unit vector for relative position
    Eigen::Vector3d v_PB_N = v_PN_N - v_BN_N;   // relative velocity vector
    Eigen::Vector3d secHat_N = r_PB_N.cross(v_PB_N).normalized();   // perpendicular unit vector

    // Calculate the inertial to point DCM
    Eigen::Matrix3d dcm_PoN;  // dcm, inertial to point frame
    dcm_PoN.row(0) = rHat_PB_N.transpose();
    dcm_PoN.row(2) = rHat_PB_N.cross(secHat_N).normalized();
    dcm_PoN.row(1) = dcm_PoN.row(2).cross(dcm_PoN.row(0));

    // Compute the point to body frame DCM and convert the boresight vector to the Po frame
    Eigen::MRPd sigma_BN = cArray2EigenMRPd(this->localState.sigma_BN); // mrp, inertial to body frame
    Eigen::Matrix3d dcm_BN = sigma_BN.toRotationMatrix().transpose();   // dcm, inertial to body frame
    Eigen::Matrix3d dcm_BPo = dcm_BN * dcm_PoN.transpose(); // dcm, point to body frame
    this->boreVec_Po = dcm_BPo.transpose() * this->boreVec_B;
}

/*! This method computes the output structure for messaging. The miss angle is
    absolute distance between the desired body point and the specified structural
    vector.  The aximuth angle is the angle between the y pointing axis and the
    desired pointing vector projected into the y/z plane.

*/
void BoreAngCalc::computeCelestialOutputData()
{
    // Define epsilon that will avoid atan2 giving a NaN.
    double eps = 1e-10;

    Eigen::Vector3d baselinePoint(1.0, 0.0, 0.0);
    double dotValue = this->boreVec_Po.dot(baselinePoint);
    this->boresightAng.missAngle = fabs(safeAcos(dotValue));
    if (fabs(this->boreVec_Po(1)) < eps) {
        this->boresightAng.azimuth = 0.0;
    }
    else {
        this->boresightAng.azimuth = atan2(this->boreVec_Po(2), this->boreVec_Po(1));
    }
}

/*! This method computes the output structure for messaging. The miss angle is
    computed using the body heading and the provided inertial heading

*/
void BoreAngCalc::computeInertialOutputData()
{
    // Compute the DCM from inertial do body frame
    Eigen::MRPd sigma_BN = cArray2EigenMRPd(this->localState.sigma_BN); // mrp, inertial to body frame
    Eigen::Matrix3d dcm_BN = sigma_BN.toRotationMatrix().transpose();   // dcm, inertial to body frame

    // Calculate the inertial heading vector
    Eigen::Vector3d inertialHeadingVec_B = dcm_BN * this->inertialHeadingVec_N; // inertial heading written in the body frame
    double dotValue = this->boreVec_B.dot(inertialHeadingVec_B);
    this->boresightAng.missAngle = fabs(safeAcos(dotValue));

    // Azimuth is undefined, so we set it to 0
    this->boresightAng.azimuth = 0.0;
}

/*! This method is the main carrier for the boresight calculation routine.  If it detects
 that it needs to re-init (direction change maybe) it will re-init itself.
 Then it will compute the angles away that the boresight is from the celestial target.

 @param CurrentSimNanos The current simulation time for system
 */
void BoreAngCalc::UpdateState(uint64_t CurrentSimNanos)
{
    //! - Read the input message and convert it over appropriately depending on switch
    ReadInputs();

    if(this->inputsGood)
    {
        if (this->useCelestialHeading)
        {
            this->computeCelestialAxisPoint();
            this->computeCelestialOutputData();
        }
        else {
            this->computeInertialOutputData();
        }
    }

    //! Write out the current output for current time
    WriteOutputMessages(CurrentSimNanos);
}
