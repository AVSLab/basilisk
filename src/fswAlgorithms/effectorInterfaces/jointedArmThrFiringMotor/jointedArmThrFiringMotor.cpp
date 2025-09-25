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


#include "fswAlgorithms/effectorInterfaces/jointedArmThrFiringMotor/jointedArmThrFiringMotor.h"
#include <iostream>
#include <cstring>

/*! Initialize C-wrapped output messages */
void
JointedArmThrFiringMotor::SelfInit(){
    ArrayMotorTorqueMsg_C_init(&this->jointTorqueOutMsgC);
}

/*! This is the constructor for the module class.  It sets default variable
    values and initializes the various parts of the model */
JointedArmThrFiringMotor::JointedArmThrFiringMotor()
{
    // initialize module variables
    this->u_max = std::vector<double>(MAX_EFF_CNT, -1.0);
}

/*! This method is used to reset the module and checks that required input messages are connect.
*/
void JointedArmThrFiringMotor::Reset(uint64_t CurrentSimNanos)
{
    // check that required input messages are connected
    if (!this->massMatrixInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "JointedArmThrFiringMotor.massMatrixInMsg was not linked.");
    }
    if (!this->nonActForceInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "JointedArmThrFiringMotor.nonActForceInMsg was not linked.");
    }
    if (!this->bodyForceInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "JointedArmThrFiringMotor.bodyForceInMsg was not linked.");
    }
    if (!this->bodyTorqueInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "JointedArmThrFiringMotor.bodyTorqueInMsg was not linked.");
    }
    if (!this->jointTorqueInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "JointedArmThrFiringMotor.jointTorqueInMsg was not linked.");
    }

}


/*! This is the main method that gets called every time the module is updated.  Provide an appropriate description.
*/
void JointedArmThrFiringMotor::UpdateState(uint64_t CurrentSimNanos)
{
    int nbase;                             //< [-] number of base DOFs (6 if free base, else 0)
    int nj;                                //< [-] number of joint DOFs populated in mass matrix
    int nDOF;                              //< [-] total number of DOFs
    MJSysMassMatrixMsgPayload massMatrix;
    MJNonActuatorForcesMsgPayload nonActForce;
    CmdForceBodyMsgPayload bodyForce;
    CmdTorqueBodyMsgPayload bodyTorque;
    ArrayMotorTorqueMsgPayload jointTorqueIn;
    ArrayMotorTorqueMsgPayload jointTorqueOut;

    // always zero the output message buffers before assigning values
    jointTorqueOut = this->jointTorqueOutMsg.zeroMsgPayload;

    // read in the input messages
    massMatrix = this->massMatrixInMsg();
    nonActForce = this->nonActForceInMsg();
    bodyForce = this->bodyForceInMsg();
    bodyTorque = this->bodyTorqueInMsg();
    jointTorqueIn = this->jointTorqueInMsg();

    // Pull out DoFs
    nbase = massMatrix.nbase;
    nj = massMatrix.nj;
    nDOF = nbase + nj;
    Eigen::VectorXd u_H(nj);
    u_H.setZero();

    // Find joint torques if using a fixed base
    if (nbase == 0){
        for (int i = 0; i < nj; i++) {
            u_H(i) = -jointTorqueIn.motorTorque[i] - nonActForce.jointForces[i];
        }
    }

    // find joint torques if using a free-floating base
    if (nbase == 6){
        Eigen::Vector3d r_ddot;
        Eigen::Vector3d omega_dot;
        Eigen::VectorXd baseThr(nbase);
        Eigen::VectorXd baseBias(nbase);
        Eigen::VectorXd baseAccel(nbase);
        Eigen::VectorXd jointThr(nj);
        Eigen::VectorXd jointBias(nj);
        Eigen::MatrixXd Mbase(nbase, nbase);
        Eigen::MatrixXd Mtht(nj,3);
        Eigen::MatrixXd Mthr(nj,3);
        Eigen::MatrixXd Mfull(nDOF, nDOF);

        r_ddot.setZero();
        omega_dot.setZero();
        baseThr.setZero();
        baseBias.setZero();
        baseAccel.setZero();
        jointThr.setZero();
        jointBias.setZero();
        Mbase.setZero();
        Mtht.setZero();
        Mthr.setZero();
        Mfull.setZero();

        for (int r = 0; r < nDOF; r++) {
            for (int c = 0; c < nbase + nj; c++) {
                Mfull(r, c) = massMatrix.MassMatrix[r][c];
            }
        }
        Mbase = Mfull.block(0, 0, nbase, nbase);
        Mtht = Mfull.block(nbase, 0, nj, 3);
        Mthr = Mfull.block(nbase, 3, nj, 3);
        for (int i = 0; i < 3; i++) {
            baseThr(i) = bodyForce.forceRequestBody[i];
            baseThr(i + 3) = bodyTorque.torqueRequestBody[i];
            baseBias(i) = nonActForce.baseTransForces[i];
            baseBias(i + 3) = nonActForce.baseRotForces[i];

        }
        for (int i = 0; i < nj; i++) {
            jointThr(i) = jointTorqueIn.motorTorque[i];
            jointBias(i) = nonActForce.jointForces[i];
        }

        baseAccel = Mbase.ldlt().solve(baseThr + baseBias);
        r_ddot = baseAccel.segment(0, 3);
        omega_dot = baseAccel.segment(3, 3);

        u_H = -(Mtht*r_ddot + Mthr*omega_dot) - jointBias - jointThr;
    }

    for (int i = 0; i < nj; i++) {
        // enforce torque limits if specified
        if (this->u_max[i] >= 0.0) {
            const double umax = this->u_max[i];
            u_H(i) = std::max(-umax, std::min(u_H(i), umax));
        }
        jointTorqueOut.motorTorque[i] = u_H(i);
    }

    // write to the output messages
    this->jointTorqueOutMsg.write(&jointTorqueOut, this->moduleID, CurrentSimNanos);
    // Write the C-wrapped output message
    ArrayMotorTorqueMsg_C_write(&jointTorqueOut, &this->jointTorqueOutMsgC, this->moduleID, CurrentSimNanos);
}

void JointedArmThrFiringMotor::setU_max(std::vector<double> var)
{
    std::copy(var.begin(), var.end(), this->u_max.begin());
}
