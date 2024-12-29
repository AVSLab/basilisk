/*
 ISC License

 Copyright (c) 2023, Autonomous Vehicle Systems Lab, University of Colorado Boulder

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

#include "fswAlgorithms/orbitControl/lambertSurfaceRelativeVelocity/lambertSurfaceRelativeVelocity.h"
#include "architecture/utilities/linearAlgebra.h"
#include <cmath>

/*! This is the constructor for the module class.  It sets default variable
    values and initializes the various parts of the model */
LambertSurfaceRelativeVelocity::LambertSurfaceRelativeVelocity() = default;

/*! Module Destructor */
LambertSurfaceRelativeVelocity::~LambertSurfaceRelativeVelocity() = default;

/*! This method is used to reset the module and checks that required input messages are connected.
    @param currentSimNanos current simulation time in nano-seconds

*/
void LambertSurfaceRelativeVelocity::Reset(uint64_t currentSimNanos)
{
    // check that required input messages are connected
    if (!this->lambertProblemInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "lambertSurfaceRelativeVelocity.lambertProblemInMsg was not linked.");
    }
    if (!this->ephemerisInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "lambertSurfaceRelativeVelocity.ephemerisInMsg was not linked.");
    }
}

/*! This is the main method that gets called every time the module is updated.
    It computes the solution of Lambert's problem.
    @param currentSimNanos current simulation time in nano-seconds

*/
void LambertSurfaceRelativeVelocity::UpdateState(uint64_t currentSimNanos)
{
    // read messages
    this->readMessages();

    // surface frame: s1 in east direction, s2 in north direction, s3 in up direction
    Eigen::Vector3d s1Hat_N;
    if (omega_PN_N.norm() > 1e-6){
        s1Hat_N = (this->omega_PN_N.cross(this->r_BN_N)).normalized();
    } else {
        Eigen::Vector3d z_N = {0.0, 0.0, 1.0};
        s1Hat_N = (z_N.cross(this->r_BN_N)).normalized();
    }
    Eigen::Vector3d s3Hat_N = this->r_BN_N.normalized();
    Eigen::Vector3d s2Hat_N = (s3Hat_N.cross(s1Hat_N)).normalized();
    // DCM from inertial frame N to surface frame S
    Eigen::Matrix3d dcm_SN;
    dcm_SN << s1Hat_N.transpose(), s2Hat_N.transpose(), s3Hat_N.transpose();

    this->v_BN_N = this->omega_PN_N.cross(this->r_BN_N) + dcm_SN.transpose()*this->vRelativeDesired_S;

    // write messages
    this->writeMessages(currentSimNanos);
}

/*! This method reads the input messages each call of updateState.

*/
void LambertSurfaceRelativeVelocity::readMessages(){
    LambertProblemMsgPayload lambertProblemInMsgBuffer = this->lambertProblemInMsg();
    EphemerisMsgPayload ephemerisInMsgBuffer = this->ephemerisInMsg();

    this->r_BN_N = cArray2EigenVector3d(lambertProblemInMsgBuffer.r2_N);

    Eigen::MRPd sigma_PN = cArray2EigenMRPd(ephemerisInMsgBuffer.sigma_BN);
    this->dcm_PN = sigma_PN.toRotationMatrix().transpose();
    Eigen::Vector3d omega_PN_P = cArray2EigenVector3d(ephemerisInMsgBuffer.omega_BN_B);
    this->omega_PN_N = this->dcm_PN.transpose()*omega_PN_P;
}

/*! This method writes the output messages each call of updateState
    @param currentSimNanos current simulation time in nano-seconds

*/
void LambertSurfaceRelativeVelocity::writeMessages(uint64_t currentSimNanos){
    DesiredVelocityMsgPayload desiredVelocityOutMsgBuffer;
    desiredVelocityOutMsgBuffer = this->desiredVelocityOutMsg.zeroMsgPayload;

    eigenVector3d2CArray(this->v_BN_N, desiredVelocityOutMsgBuffer.vDesired_N);
    desiredVelocityOutMsgBuffer.maneuverTime = this->time;

    // Write to the output messages
    this->desiredVelocityOutMsg.write(&desiredVelocityOutMsgBuffer, this->moduleID, currentSimNanos);
}

void LambertSurfaceRelativeVelocity::setVRelativeDesired_S(const Eigen::Vector3d value)
{
    this->vRelativeDesired_S = value;
}

void LambertSurfaceRelativeVelocity::setTime(const double value){
    this->time = value;
}
