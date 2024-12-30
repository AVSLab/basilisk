/*
 ISC License

 Copyright (c) 2021, Autonomous Vehicle Systems Lab, University of Colorado Boulder

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


#include "simulation/dynamics/msmForceTorque/msmForceTorque.h"
#include <iostream>

/*! This is the constructor for the module class.  It sets default variable
    values and initializes the various parts of the model */
MsmForceTorque::MsmForceTorque()
{
}

/*! Module Destructor */
MsmForceTorque::~MsmForceTorque()
{
    /* free up output message objects */
    for (long unsigned int c=0; c<this->eTorqueOutMsgs.size(); c++) {
        delete this->eTorqueOutMsgs.at(c);
    }
    for (long unsigned int c=0; c<this->eForceOutMsgs.size(); c++) {
        delete this->eForceOutMsgs.at(c);
    }
    for (long unsigned int c=0; c<this->chargeMsmOutMsgs.size(); c++) {
        delete this->chargeMsmOutMsgs.at(c);
    }
}

/*! This method is used to reset the module and checks that required input messages are connect.

*/
void MsmForceTorque::Reset(uint64_t CurrentSimNanos)
{
    // check that required input messages are connected
    for (long unsigned int c=0; c < this->scStateInMsgs.size(); c++ ){
        if (!this->scStateInMsgs.at(c).isLinked()) {
            bskLogger.bskLog(BSK_ERROR, "MsmForceTorque.scStateInMsgs[%d] was not linked.", c);
        }
    }

    for (long unsigned int c=0; c < this->voltInMsgs.size(); c++) {
        if (!this->voltInMsgs.at(c).isLinked()) {
            bskLogger.bskLog(BSK_ERROR, "MsmForceTorque.voltInMsgs[%d] was not linked.", c);
        }
    }

    this->numSat = (uint32_t) this->scStateInMsgs.size();
    if (this->numSat < 2) {
        bskLogger.bskLog(BSK_ERROR, "MsmForceTorque must have 2 or more spacecraft components added. You added %lu.", this->numSat);
    }

    /* determine number of spheres being modeled */
    this->numSpheres = 0;
    for (long unsigned int c=0; c < this->numSat; c++) {
        this->numSpheres += this->radiiList.at(c).size();
    }
    if (this->numSpheres == 0) {
        bskLogger.bskLog(BSK_ERROR, "MsmForceTorque does not have any spheres added?");
    }

    return;
}

/*!   Subscribe to the spacecraft state message and store the corresponding MSM radii and sphere positions
 */
void MsmForceTorque::addSpacecraftToModel(Message<SCStatesMsgPayload> *tmpScMsg
                                          , std::vector<double> radii
                                          , std::vector<Eigen::Vector3d> r_SB_B)
{
    /* add the message reader to the vector of input spacecraft state messages */
    this->scStateInMsgs.push_back(tmpScMsg->addSubscriber());

    /* increase the vector of voltage input message readers */
    ReadFunctor<VoltMsgPayload> inVoltMsg;
    this->voltInMsgs.push_back(inVoltMsg);

    /* store MSM sphere radii and location information */
    if (radii.size() != r_SB_B.size()) {
        bskLogger.bskLog(BSK_ERROR, "MsmForceTorque:addSpacecraftToModel() The vector of MSM radii and positions must have the same size, they have sizes %lu and %lu.", radii.size(), r_SB_B.size());
    }
    this->radiiList.push_back(radii);
    this->r_SB_BList.push_back(r_SB_B);

    this->volt.push_back(0);
    Eigen::Vector3d zero;
    zero << 0.0, 0.0, 0.0;
    this->r_BN_NList.push_back(zero);
    Eigen::MRPd zeroMRP;
    zeroMRP = zero;
    this->sigma_BNList.push_back(zeroMRP);

    /* create output message objects */
    Message<CmdTorqueBodyMsgPayload> *msgTorque;
    msgTorque = new Message<CmdTorqueBodyMsgPayload>;
    this->eTorqueOutMsgs.push_back(msgTorque);

    Message<CmdForceInertialMsgPayload> *msgForce;
    msgForce = new Message<CmdForceInertialMsgPayload>;
    this->eForceOutMsgs.push_back(msgForce);

    Message<ChargeMsmMsgPayload> *msmCharge;
    msmCharge = new Message<ChargeMsmMsgPayload>;
    this->chargeMsmOutMsgs.push_back(msmCharge);

}

/*!  Read in the input messages
 */
void MsmForceTorque::readMessages()
{
    VoltMsgPayload voltInMsgBuffer;          //!< local copy of voltage input message buffer
    SCStatesMsgPayload scStateInMsgsBuffer;     //!< local copy of spacecraft state input message buffer
    long unsigned int c;                        //!< spacecraft loop counter

    for (c = 0; c < this->numSat; c++) {
        voltInMsgBuffer = this->voltInMsgs.at(c)();
        this->volt.at(c) = voltInMsgBuffer.voltage;

        scStateInMsgsBuffer = this->scStateInMsgs.at(c)();
        this->r_BN_NList.at(c) = cArray2EigenVector3d(scStateInMsgsBuffer.r_BN_N);
        this->sigma_BNList.at(c) = cArray2EigenVector3d(scStateInMsgsBuffer.sigma_BN);
    }
}


/*! This is the main method that gets called every time the module is updated.  Provide an appropriate description.

*/
void MsmForceTorque::UpdateState(uint64_t CurrentSimNanos)
{
    // read the input messages
    this->readMessages();

    // compute the electrostatic forces and torques
    Eigen::MatrixXd S;                          //!< [1/m] Elastance matrix divided by kc
    Eigen::VectorXd V;                          //!< [V] vector of sphere voltages
    Eigen::VectorXd q;                          //!< [C] vector of sphere charges
    double kc;                                  //!< [Nm^2/C^2] Coulomb's constant
    std::vector<Eigen::Vector3d> r_SN_NList;    //!< [m] list of inertial sphere locations
    Eigen::Matrix3d dcm_NB;                     //!< [] DCM from body B to inertial frame N
    Eigen::Vector3d r_BN_N;                     //!< [m] spacecraft inertial position vector
    Eigen::Vector3d r_ij_N;                     //!< [m] relative position vector between ith and jth spheres
    double r_ij;                                //!< [m] norm of r_ij_N
    long unsigned int counter;                  //!< [] loop counter
    CmdForceInertialMsgPayload forceMsgBuffer;  //!< [] force out message buffer
    CmdTorqueBodyMsgPayload torqueMsgBuffer;    //!< [] torque out message buffer
    ChargeMsmMsgPayload chargeMsmMsgBuffer;     //!< [] MSM charge message buffer

    kc = 8.99e9;

    /* size matrices */
    S.resize(this->numSpheres, this->numSpheres);
    V.resize(this->numSpheres);
    q.resize(this->numSpheres);

    /* determine inertial sphere locations */
    for (long unsigned int c=0; c < this->numSat; c++) {
        dcm_NB = this->sigma_BNList.at(c).toRotationMatrix();
        r_BN_N = this->r_BN_NList.at(c);
        for (long unsigned int k=0; k < this->radiiList.at(c).size(); k++) {
            r_SN_NList.push_back(r_BN_N + dcm_NB * this->r_SB_BList.at(c).at(k));
        }
    }

    /*
     setup elastance matrix
     */
    /* setup diagonal S matrix and voltage components */
    counter = 0;
    for (long unsigned int c=0; c < this->numSat; c++) {
        for (long unsigned int k=0; k < this->radiiList.at(c).size(); k++) {
            S(counter, counter) = kc/this->radiiList.at(c).at(k);
            V(counter) = this->volt.at(c);
            counter++;
        }
    }
    /* setup off-diagonal components */
    for (long unsigned int i=0; i < this->numSpheres; i++) {
        for (long unsigned int j=0; j < this->numSpheres; j++) {
            if (i != j) {
                r_ij_N = r_SN_NList.at(i) - r_SN_NList.at(j);
                S(i,j) = kc / r_ij_N.norm();
                S(j,i) = S(i,j);
            }
        }
    }

    /* solve for sphere charges */
    q = S.llt().solve(V);

    /* find forces and torques acting on each space object */
    counter = 0;
    long unsigned int i0 = 0;       // counter where the MSM sphere charges start in the q vector
    long unsigned int i1;           // counter where the next spacecraft MSM sphere charges start
    // loop over all satellites
    for (long unsigned int c=0; c < this->numSat; c++) {
        Eigen::Vector3d netForce_N;     // net force acting on spacecraft
        Eigen::Vector3d netTorque_B;    // net torque acting on spacecraft in B frame
        Eigen::Vector3d force_N;        // force acting on a sphere
        Eigen::Matrix3d dcm_BN;         // DCM from inertial frame N to body B

        netForce_N.setZero();
        netTorque_B.setZero();

        dcm_BN = this->sigma_BNList.at(c).toRotationMatrix().transpose();

        // set the body sphere end counter
        i1 = i0 + this->radiiList.at(c).size();

        // zero output message buffer
        forceMsgBuffer = this->eForceOutMsgs.at(c)->zeroMsgPayload;
        torqueMsgBuffer = this->eTorqueOutMsgs.at(c)->zeroMsgPayload;
        chargeMsmMsgBuffer = this->chargeMsmOutMsgs.at(c)->zeroMsgPayload;

        // loop over current body spheres
        for (long unsigned int j=i0; j<i1; j++) {
            force_N.setZero();
            // loop over all other spheres
            for (long unsigned int i=0; i<this->numSpheres; i++) {
                if (i<i0 || i>=i1) {
                    r_ij_N = r_SN_NList.at(i) - r_SN_NList.at(j);
                    r_ij = r_ij_N.norm();
                    // check if separation is larger then current MSM sphere radius
                    if (r_ij > this->radiiList.at(c).at(j-i0)) {
                        force_N -= kc * q(j)*q(i) * r_ij_N/r_ij/r_ij/r_ij;
                    } else {
                        bskLogger.bskLog(BSK_WARNING, "MsmForceTorque: spacecraft %lu, sphere %lu is too close to another sphere.", c, j-i0);
                    }
                }
            }
            // add to total force acting on spacecraft
            netForce_N += force_N;

            // add to total torque acting on spacecraft
            netTorque_B += this->r_SB_BList.at(c).at(j-i0).cross(dcm_BN * force_N);
        }

        // store net force and torque acting on body
        eigenVector3d2CArray(netForce_N, forceMsgBuffer.forceRequestInertial);
        this->eForceOutMsgs.at(c)->write(&forceMsgBuffer, this->moduleID, CurrentSimNanos);
        eigenVector3d2CArray(netTorque_B, torqueMsgBuffer.torqueRequestBody);
        this->eTorqueOutMsgs.at(c)->write(&torqueMsgBuffer, this->moduleID, CurrentSimNanos);

        // store MSM charges to output message
        chargeMsmMsgBuffer.q = q.segment(i0, i1-i0);
        this->chargeMsmOutMsgs.at(c)->write(&chargeMsmMsgBuffer, this->moduleID, CurrentSimNanos);

        // set the body sphere start counter
        i0 = i1;
    }

}
