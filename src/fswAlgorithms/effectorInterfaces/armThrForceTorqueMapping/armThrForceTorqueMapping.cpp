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


#include "fswAlgorithms/effectorInterfaces/armThrForceTorqueMapping/armThrForceTorqueMapping.h"
#include <iostream>
#include <cstring>
#include <cmath>

inline Eigen::Matrix3d M1(double q) {
    const double c = std::cos(q), s = std::sin(q);
    Eigen::Matrix3d R;
    R << 1,  0,  0,
         0,  c, s,
         0,  -s,  c;
    return R;
}

inline Eigen::Matrix3d M2(double q) {
    const double c = std::cos(q), s = std::sin(q);
    Eigen::Matrix3d R;
    R <<  c, 0,  -s,
          0, 1,  0,
          s, 0,  c;
    return R;
}

inline Eigen::Matrix3d M3(double q) {
    const double c = std::cos(q), s = std::sin(q);
    Eigen::Matrix3d R;
    R <<  c, s,  0,
          -s, c,  0,
          0, 0,  1;
    return R;
}

/*! Initialize C-wrapped output messages */
void
ArmThrForceTorqueMapping::SelfInit(){
    CmdForceBodyMsg_C_init(&this->bodyForceOutMsgC);
    CmdTorqueBodyMsg_C_init(&this->bodyTorqueOutMsgC);
    ArrayMotorTorqueMsg_C_init(&this->jointTorqueOutMsgC);
}

/*! This is the constructor for the module class.  It sets default variable
    values and initializes the various parts of the model */
ArmThrForceTorqueMapping::ArmThrForceTorqueMapping()
{
}

/*! This method is used to reset the module and checks that required input messages are connect.
*/
void ArmThrForceTorqueMapping::Reset(uint64_t CurrentSimNanos)
{
    // check that required input messages are connected
    if (!this->configInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "ArmThrForceTorqueMapping.configInMsg was not linked.");
    }
    if (!this->jointStatesInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "ArmThrForceTorqueMapping.jointStatesInMsg was not linked.");
    }
    if (!this->thrForceInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "ArmThrForceTorqueMapping.thrForceInMsg was not linked.");
    }

    // read in the configuration message
    MJSCConfigMsgPayload config;
    config = this->configInMsg();

    // store the configuration information
    for (int i=0; i<3; i++) {
        for (int j=0; j<3; j++) {
            this->dcm_H1B(i,j) = config.dcm_HB[i][j];
            this->dcm_H2B(i,j) = config.dcm_HB[i][j+3];
        }
        for (int j=0; j<2; j++) {
            this->thr_F_T(i,j) = config.thr_F_T[i][j];
        }
        for (int j=0; j<10; j++) {
            this->r_S2S1_S1(i,j) = config.r_S2S1_S1[i][j];
        }
    }

}


/*! This is the main method that gets called every time the module is updated.  Provide an appropriate description.
*/
void ArmThrForceTorqueMapping::UpdateState(uint64_t CurrentSimNanos)
{
    Eigen::Vector3d s1_B;         //< [-] first joint spin axis in body frame
    Eigen::Vector3d s2_B;         //< [-] second joint spin axis in body frame
    Eigen::Vector3d s3_B;         //< [-] third joint spin axis in body frame
    Eigen::Vector3d s4_B;         //< [-] fourth joint spin axis in body frame
    Eigen::Vector3d s5_B;         //< [-] fifth joint spin axis in body frame
    Eigen::Vector3d s6_B;         //< [-] sixth joint spin axis in body frame
    Eigen::Vector3d s7_B;         //< [-] seventh joint spin axis in body frame
    Eigen::Vector3d s8_B;         //< [-] eighth joint spin axis in body frame
    Eigen::VectorXd jointTorques(8);   //< [Nm] joint torques
    Eigen::Matrix3d dcm_J1B;      //< [-] DCM from body frame to joint 1 frame
    Eigen::Matrix3d dcm_J2B;      //< [-] DCM from body frame to joint 2 frame
    Eigen::Matrix3d dcm_J3B;      //< [-] DCM from body frame to joint 3 frame
    Eigen::Matrix3d dcm_J4B;      //< [-] DCM from body frame to joint 4 frame
    Eigen::Matrix3d dcm_J5B;      //< [-] DCM from body frame to joint 5 frame
    Eigen::Matrix3d dcm_J6B;      //< [-] DCM from body frame to joint 6 frame
    Eigen::Matrix3d dcm_J7B;      //< [-] DCM from body frame to joint 7 frame
    Eigen::Matrix3d dcm_J8B;      //< [-] DCM from body frame to joint 8 frame
    Eigen::Matrix3d dcm_T1B;      //< [-] DCM from body frame to thruster 1 frame
    Eigen::Matrix3d dcm_T2B;      //< [-] DCM from body frame to thruster 2 frame
    Eigen::Matrix<double,3,2> thr_F_B;       //< [N] thruster force vectors in body frame
    Eigen::Matrix<double,3,2> thr_Hub_T_B;       //< [Nm] thruster hub torques in body frame
    Eigen::Matrix<double,3,10> r_SB_B;       //< [m] position vectors of each site relative to hub CoM in body frame
    JointArrayStateMsgPayload jointStates;
    THRArrayCmdForceMsgPayload thrForcer;
    CmdForceBodyMsgPayload bodyForceOut;
    CmdTorqueBodyMsgPayload bodyTorqueOut;
    ArrayMotorTorqueMsgPayload jointTorqueOut;

    s1_B.setZero();
    s2_B.setZero();
    s3_B.setZero();
    s4_B.setZero();
    s5_B.setZero();
    s6_B.setZero();
    s7_B.setZero();
    s8_B.setZero();
    dcm_J1B.setZero();
    dcm_J2B.setZero();
    dcm_J3B.setZero();
    dcm_J4B.setZero();
    dcm_J5B.setZero();
    dcm_J6B.setZero();
    dcm_J7B.setZero();
    dcm_J8B.setZero();
    dcm_T1B.setZero();
    dcm_T2B.setZero();
    r_SB_B.setZero();

    // always zero the output message buffers before assigning values
    bodyForceOut = this->bodyForceOutMsg.zeroMsgPayload;
    bodyTorqueOut = this->bodyTorqueOutMsg.zeroMsgPayload;
    jointTorqueOut = this->jointTorqueOutMsg.zeroMsgPayload;

    // read in the input messages
    jointStates = this->jointStatesInMsg();
    thrForcer = this->thrForceInMsg();

    // calculate all DCMs
    dcm_J1B = M1(jointStates.thetas[0])*this->dcm_H1B;
    dcm_J2B = M2(jointStates.thetas[1])*dcm_J1B;
    dcm_J3B = M3(jointStates.thetas[2])*dcm_J2B;
    dcm_J4B = M2(jointStates.thetas[3])*dcm_J3B;
    dcm_T1B = dcm_J4B;
    dcm_J5B = M1(jointStates.thetas[4])*this->dcm_H2B;
    dcm_J6B = M2(jointStates.thetas[5])*dcm_J5B;
    dcm_J7B = M3(jointStates.thetas[6])*dcm_J6B;
    dcm_J8B = M2(jointStates.thetas[7])*dcm_J7B;
    dcm_T2B = dcm_J8B;

    // calculate all the vectors to each site from the hub CoM
    r_SB_B.col(0) = this->r_S2S1_S1.col(0);
    r_SB_B.col(1) = dcm_J1B.transpose() * this->r_S2S1_S1.col(1) + r_SB_B.col(0);
    r_SB_B.col(2) = dcm_J2B.transpose() * this->r_S2S1_S1.col(2) + r_SB_B.col(1);
    r_SB_B.col(3) = dcm_J3B.transpose() * this->r_S2S1_S1.col(3) + r_SB_B.col(2);
    r_SB_B.col(4) = dcm_J4B.transpose() * this->r_S2S1_S1.col(4) + r_SB_B.col(3);
    r_SB_B.col(5) = this->r_S2S1_S1.col(5);
    r_SB_B.col(6) = dcm_J5B.transpose() * this->r_S2S1_S1.col(6) + r_SB_B.col(5);
    r_SB_B.col(7) = dcm_J6B.transpose() * this->r_S2S1_S1.col(7) + r_SB_B.col(6);
    r_SB_B.col(8) = dcm_J7B.transpose() * this->r_S2S1_S1.col(8) + r_SB_B.col(7);
    r_SB_B.col(9) = dcm_J8B.transpose() * this->r_S2S1_S1.col(9) + r_SB_B.col(8);

    // calculate all the spin axes in body frame
    s1_B = dcm_J1B.transpose() * Eigen::Vector3d(1,0,0);
    s2_B = dcm_J2B.transpose() * Eigen::Vector3d(0,1,0);
    s3_B = dcm_J3B.transpose() * Eigen::Vector3d(0,0,1);
    s4_B = dcm_J4B.transpose() * Eigen::Vector3d(0,1,0);
    s5_B = dcm_J5B.transpose() * Eigen::Vector3d(1,0,0);
    s6_B = dcm_J6B.transpose() * Eigen::Vector3d(0,1,0);
    s7_B = dcm_J7B.transpose() * Eigen::Vector3d(0,0,1);
    s8_B = dcm_J8B.transpose() * Eigen::Vector3d(0,1,0);

    // calculate all the thruster force vectors in body frame
    thr_F_B.col(0) = dcm_T1B.transpose() * this->thr_F_T.col(0) * thrForcer.thrForce[0];
    thr_F_B.col(1) = dcm_T2B.transpose() * this->thr_F_T.col(1) * thrForcer.thrForce[1];

    // calculate all the thruster hub torques in body frame
    thr_Hub_T_B.col(0) = r_SB_B.col(4).cross(thr_F_B.col(0));
    thr_Hub_T_B.col(1) = r_SB_B.col(9).cross(thr_F_B.col(1));

    // calculate all the joint torques
    jointTorques(0) = s1_B.dot((r_SB_B.col(4)-r_SB_B.col(0)).cross(thr_F_B.col(0)));
    jointTorques(1) = s2_B.dot((r_SB_B.col(4)-r_SB_B.col(1)).cross(thr_F_B.col(0)));
    jointTorques(2) = s3_B.dot((r_SB_B.col(4)-r_SB_B.col(2)).cross(thr_F_B.col(0)));
    jointTorques(3) = s4_B.dot((r_SB_B.col(4)-r_SB_B.col(3)).cross(thr_F_B.col(0)));
    jointTorques(4) = s5_B.dot((r_SB_B.col(9)-r_SB_B.col(5)).cross(thr_F_B.col(1)));
    jointTorques(5) = s6_B.dot((r_SB_B.col(9)-r_SB_B.col(6)).cross(thr_F_B.col(1)));
    jointTorques(6) = s7_B.dot((r_SB_B.col(9)-r_SB_B.col(7)).cross(thr_F_B.col(1)));
    jointTorques(7) = s8_B.dot((r_SB_B.col(9)-r_SB_B.col(8)).cross(thr_F_B.col(1)));

    // Build the output messages
    for (int i=0; i<3; i++) {
        bodyForceOut.forceRequestBody[i] = thr_F_B.row(i).sum();
        bodyTorqueOut.torqueRequestBody[i] = thr_Hub_T_B.row(i).sum();
    }
    for (int i=0; i<8; i++) {
        jointTorqueOut.motorTorque[i] = jointTorques(i);
    }


    // std::cout<< "Body Force: " << bodyForceOut.forceRequestBody[0] << " "
    //          << bodyForceOut.forceRequestBody[1] << " "
    //          << bodyForceOut.forceRequestBody[2] << std::endl;
    // std::cout<< "Body Torque: " << bodyTorqueOut.torqueRequestBody[0] << " "
    //          << bodyTorqueOut.torqueRequestBody[1] << " "
    //          << bodyTorqueOut.torqueRequestBody[2] << std::endl;

    // write to the output messages
    this->bodyForceOutMsg.write(&bodyForceOut, this->moduleID, CurrentSimNanos);
    this->bodyTorqueOutMsg.write(&bodyTorqueOut, this->moduleID, CurrentSimNanos);
    this->jointTorqueOutMsg.write(&jointTorqueOut, this->moduleID, CurrentSimNanos);
    // Write the C-wrapped output message
    CmdForceBodyMsg_C_write(&bodyForceOut, &this->bodyForceOutMsgC, this->moduleID, CurrentSimNanos);
    CmdTorqueBodyMsg_C_write(&bodyTorqueOut, &this->bodyTorqueOutMsgC, this->moduleID, CurrentSimNanos);
    ArrayMotorTorqueMsg_C_write(&jointTorqueOut, &this->jointTorqueOutMsgC, this->moduleID, CurrentSimNanos);
}
