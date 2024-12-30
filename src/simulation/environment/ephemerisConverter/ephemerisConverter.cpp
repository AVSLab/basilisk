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
#include "simulation/environment/ephemerisConverter/ephemerisConverter.h"
#include "architecture/utilities/linearAlgebra.h"
#include "architecture/utilities/avsEigenSupport.h"
#include "architecture/utilities/macroDefinitions.h"

EphemerisConverter::EphemerisConverter()
{
}

EphemerisConverter::~EphemerisConverter()
{
    for (long unsigned int c=0; c<this->ephemOutMsgs.size(); c++) {
        free(this->ephemOutMsgs.at(c));
    }
}

/*! Reset the module to origina configuration values.

 */
void EphemerisConverter::Reset(uint64_t CurrenSimNanos)
{
    // check if the spiceInMsgs is empty or not
    if (this->spiceInMsgs.size() == 0) {
        bskLogger.bskLog(BSK_ERROR, "ephemerisConverter.spiceInMsgs is empty.");
    }
}

/*!
 add a planet spice input message
 */
void EphemerisConverter::addSpiceInputMsg(Message<SpicePlanetStateMsgPayload> *tmpMsg)
{
    this->spiceInMsgs.push_back(tmpMsg->addSubscriber());

    /* setup output corresponding message */
    Message<EphemerisMsgPayload> *msg;
    msg = new Message<EphemerisMsgPayload>;
    this->ephemOutMsgs.push_back(msg);


    /* update input and output buffers*/
    SpicePlanetStateMsgPayload tmpSpice = {};
    this->spiceInBuffers.push_back(tmpSpice);

    EphemerisMsgPayload tmpEphem = {};
    this->ephemOutBuffers.push_back(tmpEphem);
}

/*!
    convert ephemeris data
    @param clockNow
 */
void EphemerisConverter::convertEphemData(uint64_t clockNow)
{
    Eigen::Matrix3d dcm_BN;
    Eigen::Vector3d sigma_BN;
    Eigen::Matrix3d dcm_BN_dot;
    Eigen::Matrix3d omega_tilde_BN_B_eigen;
    double omega_tilde_BN_B[3][3];
    double omega_tilde_BN_B_array[9];

    for (long unsigned int c=0; c < this->spiceInMsgs.size(); c++) {
        v3Copy(this->spiceInBuffers.at(c).PositionVector,
               this->ephemOutBuffers.at(c).r_BdyZero_N);

        v3Copy(this->spiceInBuffers.at(c).VelocityVector,
               this->ephemOutBuffers.at(c).v_BdyZero_N);

        this->ephemOutBuffers.at(c).timeTag = this->spiceInMsgs.at(c).timeWritten()*1.0E-9;

        /* Compute sigma_BN */
        dcm_BN = cArray2EigenMatrix3d(*this->spiceInBuffers.at(c).J20002Pfix);
        sigma_BN = eigenMRPd2Vector3d(eigenC2MRP(dcm_BN));
        eigenVector3d2CArray(sigma_BN, this->ephemOutBuffers.at(c).sigma_BN); //sigma_BN

        /* Compute omega_BN_B */
        dcm_BN_dot = cArray2EigenMatrix3d(*this->spiceInBuffers.at(c).J20002Pfix_dot);
        omega_tilde_BN_B_eigen = -dcm_BN_dot*dcm_BN.transpose();
        eigenMatrix3d2CArray(omega_tilde_BN_B_eigen, omega_tilde_BN_B_array);
        m33Copy(RECAST3X3 omega_tilde_BN_B_array, omega_tilde_BN_B);
        this->ephemOutBuffers.at(c).omega_BN_B[0] = omega_tilde_BN_B[2][1];
        this->ephemOutBuffers.at(c).omega_BN_B[1] = omega_tilde_BN_B[0][2];
        this->ephemOutBuffers.at(c).omega_BN_B[2] = omega_tilde_BN_B[1][0];
    }
}

void EphemerisConverter::readInputMessages()
{
    for (long unsigned int c=0; c < this->spiceInMsgs.size(); c++) {
        this->spiceInBuffers.at(c) = this->spiceInMsgs.at(c)();
    }
}

/*!
    write output message
    @param CurrentSimNanos time in nano-seconds
 */
void EphemerisConverter::writeOutputMessages(uint64_t CurrentSimNanos)
{
    for (long unsigned int c=0; c < this->ephemOutMsgs.size(); c++) {
        this->ephemOutMsgs.at(c)->write(&this->ephemOutBuffers.at(c), this->moduleID, CurrentSimNanos);
    }
}


/*!
    update module states
    @param CurrentSimNanos time in nano-seconds
 */
void EphemerisConverter::UpdateState(uint64_t CurrentSimNanos)
{
    readInputMessages();
    convertEphemData(CurrentSimNanos);
    writeOutputMessages(CurrentSimNanos);
}
