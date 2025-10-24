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


#include "fswAlgorithms/effectorInterfaces/jointedArmPDMotor/jointedArmPDMotor.h"
#include <iostream>
#include <cstring>
#include <algorithm>

// namespace {
// inline double wrapToPi(double a) {
//     // returns angle in (-pi, pi]
//     return std::atan2(std::sin(a), std::cos(a));
// }
// template<typename DerivedA, typename DerivedB>
// inline void shortestAngleError(const Eigen::MatrixBase<DerivedA>& theta,
//                                const Eigen::MatrixBase<DerivedB>& thetaDes,
//                                Eigen::VectorXd& deltaOut) {
//     const int n = static_cast<int>(theta.size());
//     deltaOut.resize(n);
//     for (int i = 0; i < n; ++i) {
//         const double e = theta(i) - thetaDes(i);
//         deltaOut(i) = wrapToPi(e);
//     }
// }
// }

/*! Initialize C-wrapped output messages */
void
JointedArmPDMotor::SelfInit(){
    ArrayMotorTorqueMsg_C_init(&this->jointTorqueOutMsgC);
}

/*! This is the constructor for the module class.  It sets default variable
    values and initializes the various parts of the model */
JointedArmPDMotor::JointedArmPDMotor()
{
    // initialize module variables
    this->Ktheta = Eigen::MatrixXd::Zero(MAX_EFF_CNT, MAX_EFF_CNT);
    this->Ptheta = Eigen::MatrixXd::Zero(MAX_EFF_CNT, MAX_EFF_CNT);
    this->u_max = std::vector<double>(MAX_EFF_CNT, -1.0);
}

/*! This method is used to reset the module and checks that required input messages are connect.
*/
void JointedArmPDMotor::Reset(uint64_t CurrentSimNanos)
{
    // check that required input messages are connected
    if (!this->massMatrixInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "JointedArmPDMotor.massMatrixInMsg was not linked.");
    }
    if (!this->jointStateInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "JointedArmPDMotor.jointStateInMsg was not linked.");
    }
    if (!this->desJointStateInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "JointedArmPDMotor.desJointStateInMsg was not linked.");
    }
    if (!this->nonActForceInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "JointedArmPDMotor.nonActForceInMsg was not linked.");
    }

}


/*! This is the main method that gets called every time the module is updated.  Provide an appropriate description.
*/
void JointedArmPDMotor::UpdateState(uint64_t CurrentSimNanos)
{
    int nbase;                             //< [-] number of base DOFs (6 if free base, else 0)
    int nj;                                //< [-] number of joint DOFs
    MJSysMassMatrixMsgPayload massMatrix;
    JointArrayStateMsgPayload jointStates;
    JointArrayStateMsgPayload desJointStates;
    MJNonActuatorForcesMsgPayload nonActForce;
    ArrayMotorTorqueMsgPayload TorqueOut;

    // always zero the output message buffers before assigning values
    TorqueOut = this->jointTorqueOutMsg.zeroMsgPayload;

    // read in the input messages
    massMatrix = this->massMatrixInMsg();
    jointStates = this->jointStateInMsg();
    desJointStates = this->desJointStateInMsg();
    nonActForce = this->nonActForceInMsg();

    // Pull out DoFs
    nbase = massMatrix.nbase;
    nj = massMatrix.nj;
    Eigen::VectorXd u_H(nj);
    u_H.setZero();

    // Calculate the errors
    Eigen::VectorXd deltaTheta(nj);
    Eigen::VectorXd deltaThetaDot(nj);
    Eigen::VectorXd theta_ddot_des(nj);
    deltaTheta.setZero();
    deltaThetaDot.setZero();
    theta_ddot_des.setZero();

    // Map inputs to Eigen vectors
    Eigen::VectorXd theta(nj), thetaDes(nj), thetaDot(nj), thetaDotDes(nj);\
    for (int i = 0; i < nj; i++){
        theta(i) = jointStates.thetas[i];
        thetaDes(i) = desJointStates.thetas[i];
        thetaDot(i) = jointStates.thetaDots[i];
        thetaDotDes(i) = desJointStates.thetaDots[i];
    }

    // Find the shortest angle error
    // shortestAngleError(theta, thetaDes, deltaTheta);
    // deltaThetaDot = thetaDot - thetaDotDes;
    // theta_ddot_des = -Ktheta.topLeftCorner(nj, nj) * deltaTheta - Ptheta.topLeftCorner(nj, nj) * deltaThetaDot;

    auto wrap = [](double a){ return std::atan2(std::sin(a), std::cos(a)); };

    Eigen::VectorXd e(nj);
    for (int i = 0; i < nj; ++i)
        e(i) = wrap(theta(i) - thetaDes(i));   // desired - measured, wrapped
    theta_ddot_des = - Ktheta.topLeftCorner(nj,nj) * e
                - Ptheta.topLeftCorner(nj,nj) * thetaDot;

    // if (CurrentSimNanos*1e-9 > 99.9 and CurrentSimNanos*1e-9 < 100.05) {
    //     // std::cout << "desired joint 6 accelerations at time t= " <<CurrentSimNanos*1e-9<< "s:" << theta_ddot_des(5) << std::endl;
    //     // std::cout << "desired joint 8 accelerations at time t= " <<CurrentSimNanos*1e-9<< "s:" << theta_ddot_des(7) << std::endl;
    //     std::cout << "desired joint accelerations at time t= " <<CurrentSimNanos*1e-9<< "s:" << theta_ddot_des.transpose() << std::endl;
    // }


    // Find joint torques if using a fixed base
    if (nbase == 0){
        Eigen::MatrixXd Mtheta(nj, nj);
        Eigen::VectorXd jointBias(nj);

        Mtheta.setZero();
        jointBias.setZero();

        for (int i = 0; i < nj; i++) {
            jointBias(i) = nonActForce.jointForces[i];
            for (int j = 0; j < nj; j++) {
                Mtheta(i, j) = massMatrix.MassMatrix[nbase + i][nbase + j];
            }
        }

        u_H = Mtheta * theta_ddot_des - jointBias;
    }

    // Find joint torques if using a free-floating base
    if (nbase == 6){
        Eigen::MatrixXd Mfull(nbase + nj, nbase + nj);
        Eigen::MatrixXd Mtth(3, nj);
        Eigen::MatrixXd Mtht(nj, 3);
        Eigen::MatrixXd Mthth(nj, nj);
        Eigen::Matrix3d Mtt;
        Eigen::VectorXd jointBias(nj);
        Eigen::Vector3d baseTransBias;
        Eigen::Vector3d rhs;
        Eigen::Vector3d temp1;

        Mfull.setZero();
        Mtth.setZero();
        Mtht.setZero();
        Mthth.setZero();
        Mtt.setZero();
        jointBias.setZero();
        baseTransBias.setZero();
        rhs.setZero();
        temp1.setZero();

        for (int r = 0; r < nbase + nj; r++) {
            for (int c = 0; c < nbase + nj; c++) {
                Mfull(r, c) = massMatrix.MassMatrix[r][c];
            }
        }
        Mtt = Mfull.block<3, 3>(0, 0);
        Mtth = Mfull.block(0,nbase,3,nj);
        Mtht = Mfull.block(nbase,0,nj,3);
        Mthth = Mfull.block(nbase,nbase,nj,nj);
        for (int i = 0; i < 3; i++) {
            baseTransBias(i) = nonActForce.baseTransForces[i];
        }
        for (int i = 0; i < nj; i++) {
            jointBias(i) = nonActForce.jointForces[i];
        }

        // Eigen::LDLT<Eigen::Matrix3d> ldlt(Mtt);
        // Eigen::MatrixXd MttInvMtth = ldlt.solve(Mtth);
        // Eigen::Vector3d MttInv_Ct = ldlt.solve(-baseTransBias);

        // Eigen::MatrixXd Mbar = Mthth - Mtht * MttInvMtth;
        // Eigen::VectorXd Cbar = -jointBias - Mtht * MttInv_Ct;

        // u_H = Mbar * theta_ddot_des + Cbar;

        rhs = -Mtth*theta_ddot_des + baseTransBias;
        Eigen::LDLT<Eigen::Matrix3d> ldlt(Mtt);
        temp1 = ldlt.solve(rhs);

        u_H = (Mthth * theta_ddot_des) + (Mtht*temp1) - jointBias;

        // Eigen::VectorXd check1(nj);
        // Eigen::VectorXd check2(nj);
        // check1 = Mthth*theta_ddot_des;
        // check2 = Mtht*temp1;

        // if (CurrentSimNanos*1e-9 > 99.9 and CurrentSimNanos*1e-9 < 100.05) {
        //     // std::cout << "joint 8 Mthth*theta_ddot_des at time t= " <<CurrentSimNanos*1e-9<< "s:" << check1(7) << std::endl;
        //     // std::cout << "joint 8 Mtht*temp1 at time t= " <<CurrentSimNanos*1e-9<< "s:" << check2(7) << std::endl;
        //     // std::cout << "joint 8 bias at time t= " <<CurrentSimNanos*1e-9<< "s:" << jointBias(7) << std::endl;
        //     // for (int k=0; k<nj; ++k) {
        //     //     double contrib = Mthth(7,k) * theta_ddot_des(k);
        //     //     std::cout << "k="<<k<<" : M(8,"<<k+1<<")*ddot(k) = "<< contrib << "\n";
        //     // }
        //     std::cout << "joint 8 torque at time t= " <<CurrentSimNanos*1e-9<< "s:" << u_H(7) << std::endl;
        // }
    }

    for (int i = 0; i < nj; i++) {
        // enforce torque limits if specified
        if (this->u_max[i] >= 0.0) {
            const double umax = this->u_max[i];
            u_H(i) = std::max(-umax, std::min(u_H(i), umax));
        }
        TorqueOut.motorTorque[i] = u_H(i);
    }

    // if (CurrentSimNanos*1e-9 > 99.9 and CurrentSimNanos*1e-9 < 100.05) {
    //     // std::cout << "joint 6 torque at time t= " <<CurrentSimNanos*1e-9<< "s:" << u_H(5) << std::endl;
    //     // std::cout << "joint 8 torque at time t= " <<CurrentSimNanos*1e-9<< "s:" << u_H(7) << std::endl;
    //     std::cout << "joint torques at time t= " <<CurrentSimNanos*1e-9<< "s:" << u_H.transpose() << std::endl;
    // }

    // write to the output messages
    this->jointTorqueOutMsg.write(&TorqueOut, this->moduleID, CurrentSimNanos);
    // Write the C-wrapped output message
    ArrayMotorTorqueMsg_C_write(&TorqueOut, &this->jointTorqueOutMsgC, this->moduleID, CurrentSimNanos);
}

void JointedArmPDMotor::setKtheta(std::vector<double> var)
{
    const size_t N = var.size();
    const int m = static_cast<int>(std::llround(std::sqrt(static_cast<double>(N))));

    using MatMap = Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>;
    MatMap Kmap(var.data(), m, m);

    this->Ktheta.block(0, 0, Kmap.rows(), Kmap.cols()) = Kmap;
}

void JointedArmPDMotor::setPtheta(std::vector<double> var)
{
    const size_t N = var.size();
    const int m = static_cast<int>(std::llround(std::sqrt(static_cast<double>(N))));

    using MatMap = Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>;
    MatMap Pmap(var.data(), m, m);

    this->Ptheta.block(0, 0, Pmap.rows(), Pmap.cols()) = Pmap;
}

void JointedArmPDMotor::setU_max(std::vector<double> var)
{
    std::copy(var.begin(), var.end(), this->u_max.begin());
}
