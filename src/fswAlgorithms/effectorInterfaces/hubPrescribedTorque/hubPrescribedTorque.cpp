/*
 ISC License

 Copyright (c) 2025, Autonomous Vehicle Systems Lab, University of Colorado Boulder

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


#include "fswAlgorithms/effectorInterfaces/hubPrescribedTorque/hubPrescribedTorque.h"
#include <iostream>
#include <cstring>

/*! Initialize C-wrapped output messages */
void
HubPrescribedTorque::SelfInit(){
    CmdTorqueBodyMsg_C_init(&this->cmdTorqueOutMsgC);
}

/*! This is the constructor for the module class.  It sets default variable
    values and initializes the various parts of the model */
HubPrescribedTorque::HubPrescribedTorque()
{
}

/*! This method is used to reset the module and checks that required input messages are connect.
*/
void HubPrescribedTorque::Reset(uint64_t CurrentSimNanos)
{
    // check that required input messages are connected
    if (!this->massMatrixInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "HubPrescribedTorque.massMatrixInMsg was not linked.");
    }
    if (!this->nonActForceInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "HubPrescribedTorque.nonActForceInMsg was not linked.");
    }
    if (!this->jointTorqueInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "HubPrescribedTorque.jointTorqueInMsg was not linked.");
    }

}


/*! This is the main method that gets called every time the module is updated.  Provide an appropriate description.
*/
void HubPrescribedTorque::UpdateState(uint64_t CurrentSimNanos)
{
    int nbase;                              //< [-] number of base DOFs (6 if free base, else 0)
    int nj;                                 //< [-] number of joint DOFs
    int nDOF;                               //< [-] total number of DOFs
    MJSysMassMatrixMsgPayload massMatrix;
    MJNonActuatorForcesMsgPayload nonActForce;
    ArrayMotorTorqueMsgPayload jointTorque;
    CmdTorqueBodyMsgPayload cmdTorque;

    // always zero the output message buffers before assigning values
    cmdTorque = this->cmdTorqueOutMsg.zeroMsgPayload;

    // read in the input messages
    massMatrix = this->massMatrixInMsg();
    nonActForce = this->nonActForceInMsg();
    jointTorque = this->jointTorqueInMsg();

    // Extract the DOFs
    nbase = massMatrix.nbase;
    nj = massMatrix.nj;
    nDOF = nbase + nj;

    // Handle fixed base
    if (nbase != 6){
        cmdTorque.torqueRequestBody[0] = 0.0;
        cmdTorque.torqueRequestBody[1] = 0.0;
        cmdTorque.torqueRequestBody[2] = 0.0;
    }

    // Do math for free base
    if (nbase == 6){
        Eigen::Vector3d baseTransBias;
        Eigen::Vector3d baseRotBias;
        Eigen::Vector3d temp1;
        Eigen::Vector3d tau_prescribed;
        Eigen::Vector3d r_ddot;
        Eigen::VectorXd jointBias(nj);
        Eigen::VectorXd u_H(nj);
        Eigen::VectorXd theta_ddot(nj);
        Eigen::Matrix3d Mtt;
        Eigen::Matrix3d Mrt;
        Eigen::MatrixXd Mtth(3, nj);
        Eigen::MatrixXd Mrth(3, nj);
        Eigen::MatrixXd Mtht(nj, 3);
        Eigen::MatrixXd Mthth(nj, nj);
        Eigen::MatrixXd M(nDOF, nDOF);
        Eigen::MatrixXd temp2(3,nj);
        Eigen::MatrixXd temp3(nj,nj);

        baseTransBias.setZero();
        baseRotBias.setZero();
        temp1.setZero();
        tau_prescribed.setZero();
        r_ddot.setZero();
        jointBias.setZero();
        u_H.setZero();
        theta_ddot.setZero();
        Mtt.setZero();
        Mrt.setZero();
        Mtth.setZero();
        Mrth.setZero();
        Mtht.setZero();
        Mthth.setZero();
        M.setZero();
        temp2.setZero();
        temp3.setZero();

        for (int r=0; r<nDOF; ++r)
            for (int c=0; c<nDOF; ++c)
                M(r,c) = massMatrix.MassMatrix[r][c];

        Mtt = M.block<3,3>(0,0);
        Mrt = M.block<3,3>(3,0);
        Mtth = M.block(0,nbase,3,nj);
        Mrth = M.block(3,nbase,3,nj);
        Mtht = M.block(nbase,0,nj,3);
        Mthth = M.block(nbase,nbase,nj,nj);

        for (int i=0; i<3; ++i) {
            baseTransBias(i) = nonActForce.baseTransForces[i];
            baseRotBias(i) = nonActForce.baseRotForces[i];
        }
        for (int i=0; i<nj; ++i) {
            jointBias(i) = nonActForce.jointForces[i];
            u_H(i) = jointTorque.motorTorque[i];
        }

        Eigen::LDLT<Eigen::Matrix3d> ldltMtt(Mtt);
        temp1 = ldltMtt.solve(baseTransBias);
        temp2 = ldltMtt.solve(Mtth);
        temp3 = Mthth - Mtht*temp2;
        theta_ddot = temp3.ldlt().solve(jointBias + u_H - Mtht*temp1);
        r_ddot = temp1 - temp2*theta_ddot;

        tau_prescribed = -baseRotBias + Mrt*r_ddot + Mrth*theta_ddot;

        cmdTorque.torqueRequestBody[0] = tau_prescribed(0);
        cmdTorque.torqueRequestBody[1] = tau_prescribed(1);
        cmdTorque.torqueRequestBody[2] = tau_prescribed(2);

    }

    // write to the output messages
    this->cmdTorqueOutMsg.write(&cmdTorque, this->moduleID, CurrentSimNanos);
    // Write the C-wrapped output message
    CmdTorqueBodyMsg_C_write(&cmdTorque, &this->cmdTorqueOutMsgC, this->moduleID, CurrentSimNanos);
}
