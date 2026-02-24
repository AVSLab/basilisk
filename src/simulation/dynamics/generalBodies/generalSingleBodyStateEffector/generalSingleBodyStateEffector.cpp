/*
 ISC License

 Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#include "generalSingleBodyStateEffector.h"
#include "architecture/utilities/avsEigenSupport.h"
#include "architecture/utilities/macroDefinitions.h"
#include "architecture/utilities/rigidBodyKinematics.h"
#include <string>
#include <Eigen/Dense>

const Eigen::Matrix<double, 3, 6> transMap =
        (Eigen::Matrix<double, 3, 6>() <<
                1, 0, 0,  0, 0, 0,
                0, 1, 0,  0, 0, 0,
                0, 0, 1,  0, 0, 0
        ).finished();

const Eigen::Matrix<double, 3, 6> rotMap =
        (Eigen::Matrix<double, 3, 6>() <<
                0, 0, 0,  1, 0, 0,
                0, 0, 0,  0, 1, 0,
                0, 0, 0,  0, 0, 1
        ).finished();

/*! This is the constructor, setting variables to default values. */
GeneralSingleBodyStateEffector::GeneralSingleBodyStateEffector()
{
    this->effProps.mEff = 0.0;
    this->effProps.rEff_CB_B.fill(0.0);
    this->effProps.IEffPntB_B.fill(0.0);
    this->effProps.rEffPrime_CB_B.fill(0.0);
    this->effProps.IEffPrimePntB_B.fill(0.0);

    this->nameOfBetaState = "generalBodyBeta" + std::to_string(GeneralSingleBodyStateEffector::effectorID);
    this->nameOfBetaDotState = "generalBodyBetaDot" + std::to_string(GeneralSingleBodyStateEffector::effectorID);
    this->nameOfInertialPositionProperty = "generalBodyInertialPosition" + std::to_string(GeneralSingleBodyStateEffector::effectorID);
    this->nameOfInertialVelocityProperty = "generalBodyInertialVelocity" + std::to_string(GeneralSingleBodyStateEffector::effectorID);
    this->nameOfInertialAttitudeProperty = "generalBodyInertialAttitude" + std::to_string(GeneralSingleBodyStateEffector::effectorID);
    this->nameOfInertialAngVelocityProperty = "generalBodyInertialAngVelocity" + std::to_string(GeneralSingleBodyStateEffector::effectorID);

    GeneralSingleBodyStateEffector::effectorID++;
}

uint64_t GeneralSingleBodyStateEffector::effectorID = 1;

/*! This is the destructor. */
GeneralSingleBodyStateEffector::~GeneralSingleBodyStateEffector()
{
    GeneralSingleBodyStateEffector::effectorID = 1;
}

/*! This method is used to reset the module.

 @param currentClock [ns] Time the method is called
*/
void GeneralSingleBodyStateEffector::Reset(uint64_t currentClock)
{
}

/*! This method takes the computed states and outputs them to the messaging system.

 @param currentClock [ns] Time the method is called
*/
void GeneralSingleBodyStateEffector::writeOutputStateMessages(uint64_t currentClock)
{
    // Write the config log message if it is linked
    if (this->generalSingleBodyConfigLogOutMsg.isLinked())
    {
        SCStatesMsgPayload configLogMsg = this->generalSingleBodyConfigLogOutMsg.zeroMsgPayload;

        // Note that the configLogMsg B frame represents the effector body frame (frame P)
        eigenVector3d2CArray(this->r_GcN_N, configLogMsg.r_BN_N);
        eigenVector3d2CArray(this->v_GcN_N, configLogMsg.v_BN_N);
        eigenMatrixXd2CArray(*this->sigma_GN, configLogMsg.sigma_BN);
        eigenMatrixXd2CArray(*this->omega_GN_G, configLogMsg.omega_BN_B);
        this->generalSingleBodyConfigLogOutMsg.write(&configLogMsg, this->moduleID, currentClock);
    }

    for (int dofIndex = 0; dofIndex < this->numDOF; dofIndex++) {
        int msgIndex = this->jointDOFList.at(dofIndex).index;

        if (this->jointDOFList.at(dofIndex).type == DOF::Type::ROTATION) {
            if (this->spinningBodyOutMsgs[msgIndex]->isLinked()) {
                HingedRigidBodyMsgPayload spinningBodyBuffer = this->spinningBodyOutMsgs[msgIndex]->zeroMsgPayload;

                spinningBodyBuffer.theta = this->jointDOFList.at(dofIndex).beta;
                spinningBodyBuffer.thetaDot = this->jointDOFList.at(dofIndex).betaDot;
                this->spinningBodyOutMsgs[msgIndex]->write(&spinningBodyBuffer, this->moduleID, currentClock);
            }
        } else {
            if (this->translatingBodyOutMsgs[msgIndex]->isLinked()) {
                LinearTranslationRigidBodyMsgPayload translatingBodyBuffer = this->translatingBodyOutMsgs[msgIndex]->zeroMsgPayload;

                translatingBodyBuffer.rho = this->jointDOFList.at(dofIndex).beta;
                translatingBodyBuffer.rhoDot = this->jointDOFList.at(dofIndex).betaDot;
                this->translatingBodyOutMsgs[msgIndex]->write(&translatingBodyBuffer, this->moduleID, currentClock);
            }
        }
    }
}

/*! This method allows the effector to have access to the hub states.

 @param statesIn Pointer to give the state effector access the hub states
*/
void GeneralSingleBodyStateEffector::linkInStates(DynParamManager& statesIn)
{
    // Get access to the hub states needed for dynamic coupling
    this->inertialPositionProperty = statesIn.getPropertyReference(this->propName_inertialPosition);
    this->inertialVelocityProperty = statesIn.getPropertyReference(this->propName_inertialVelocity);
}

/*! This method allows the state effector to register its states with the dynamic parameter manager.

 @param states Pointer to give the state effector access the hub states
*/
void GeneralSingleBodyStateEffector::registerStates(DynParamManager& states)
{
    Eigen::VectorXd betaVec = Eigen::VectorXd::Map(this->betaInitList.data(), this->betaInitList.size());
    this->betaState = states.registerState(this->numDOF, 1, this->nameOfBetaState);
    this->betaState->setState(betaVec);

    Eigen::VectorXd betaDotVec = Eigen::VectorXd::Map(this->betaDotInitList.data(), this->betaDotInitList.size());
    this->betaDotState = states.registerState(this->numDOF, 1, this->nameOfBetaDotState);
    this->betaDotState->setState(betaDotVec);

    this->registerProperties(states);

}

/*! This method allows the state effector to register its properties with the dynamic parameter manager.

 @param states Pointer to give the state effector access the hub states
*/
void GeneralSingleBodyStateEffector::registerProperties(DynParamManager& states)
{
    Eigen::Vector3d stateInit = Eigen::Vector3d::Zero();
    this->r_GN_N = states.createProperty(this->nameOfInertialPositionProperty, stateInit);
    this->v_GN_N = states.createProperty(this->nameOfInertialVelocityProperty, stateInit);
    this->sigma_GN = states.createProperty(this->nameOfInertialAttitudeProperty, stateInit);
    this->omega_GN_G = states.createProperty(this->nameOfInertialAngVelocityProperty, stateInit);
}

/*! This method allows the state effector to provide its contributions to the mass props and mass prop rates of the
 spacecraft.

 @param integTime [s] Time the method is called
*/
void GeneralSingleBodyStateEffector::updateEffectorMassProps(double integTime)
{
    // Set effProps.mEff
    this->effProps.mEff = this->mass;

    // Get current state
    this->beta = this->betaState->getState();
    this->betaDot = this->betaDotState->getState();

    // Set beta and betaDot for each DOF
    for (int idx = 0; idx < this->numDOF; idx++) {
        this->jointDOFList.at(idx).beta = this->beta[idx];
        this->jointDOFList.at(idx).betaDot = this->betaDot[idx];
    }

    // Compute general body attitudes
    std::vector<DOF>::iterator jointDOF;
    for(jointDOF = this->jointDOFList.begin(); jointDOF != this->jointDOFList.end(); jointDOF++) {
        if (jointDOF->type == DOF::Type::ROTATION) {

            Eigen::Vector3d prv_GG0 = jointDOF->beta * jointDOF->axis_G;
            double prv_GG0_array[3];
            eigenVector3d2CArray(prv_GG0, prv_GG0_array);

            double dcm_GG0_array[3][3];
            PRV2C(prv_GG0_array, dcm_GG0_array);
            Eigen::Matrix3d dcm_GG0 = c2DArray2EigenMatrix3d(dcm_GG0_array);

            if (jointDOF->index == 0) {
                jointDOF->dcm_GB = dcm_GG0 * jointDOF->dcm_G0P;
            } else {
                jointDOF->dcm_GB = dcm_GG0 * jointDOF->dcm_G0P * this->jointDOFList.at(jointDOF->index - 1).dcm_GB;
            }
        }
    }

    // Compute general body transformation matrix
    for(jointDOF = this->jointDOFList.begin(); jointDOF != this->jointDOFList.end(); jointDOF++) {

        int dofIndex = jointDOF->index;
        Eigen::Vector3d jointDOFAxis_B = jointDOF->dcm_GB.transpose() * jointDOF->axis_G;

        if (jointDOF->type == DOF::Type::ROTATION) {
            this->TMat.col(dofIndex).head<3>().setZero();
            this->TMat.col(dofIndex).tail<3>() = jointDOF->screwConstant * jointDOFAxis_B;
        } else {
            this->TMat.col(dofIndex).head<3>() = jointDOF->screwConstant * jointDOFAxis_B;
            this->TMat.col(dofIndex).tail<3>().setZero();
        }
    }

    // Compute and set effProps.rEff_CB_B
    Eigen::Matrix3d dcm_GB = this->jointDOFList.at(this->numDOF - 1).dcm_GB;
    Eigen::Vector3d r_GcG_B = dcm_GB.transpose() * this->r_GcG_G;
    Eigen::Vector3d r_GB_B = transMap * this->TMat * this->beta;
    Eigen::Vector3d r_GcB_B = r_GcG_B + r_GB_B;
    this->effProps.rEff_CB_B = r_GcB_B;

    // Compute and set effProps.IEffPntB_B
    Eigen::Matrix3d IPntGc_B = dcm_GB.transpose() * this->IPntGc_G * dcm_GB;
    Eigen::Matrix3d rTilde_GcB_B = eigenTilde(r_GcB_B);
    this->effProps.IEffPntB_B = IPntGc_B - this->mass * rTilde_GcB_B * rTilde_GcB_B;

    // Compute and set effProps.rEffPrime_CB_B
    Eigen::Matrix3d rTilde_GcG_B = eigenTilde(r_GcG_B);
    Eigen::Vector3d rPrime_GcB_B = (transMap - rTilde_GcG_B * rotMap) * this->TMat * this->betaDot;
    this->effProps.rEffPrime_CB_B = rPrime_GcB_B;

    // Compute and set effProps.IEffPrimePntB_B
    Eigen::Vector3d omega_GB_B = rotMap * this->TMat * this->betaDot;
    Eigen::Matrix3d omegaTilde_GB_B = eigenTilde(omega_GB_B);
    Eigen::Matrix3d rPrimeTilde_GcB_B = eigenTilde(rPrime_GcB_B);
    this->effProps.IEffPrimePntB_B = omegaTilde_GB_B * IPntGc_B - IPntGc_B * omegaTilde_GB_B
            - this->mass * (rPrimeTilde_GcB_B * rTilde_GcB_B + rTilde_GcB_B * rPrimeTilde_GcB_B);
}

/*! This method allows the state effector to give its contributions to the matrices needed for the back-sub.
 method

 @param integTime [s] Time the method is called
 @param backSubContr State effector contribution matrices for back-substitution
 @param sigma_BN Current B frame attitude with respect to the inertial frame
 @param omega_BN_B [rad/s] Angular velocity of the B frame with respect to the inertial frame, expressed in B frame
 components
 @param g_N [m/s^2] Gravitational acceleration in N frame components
*/
void GeneralSingleBodyStateEffector::updateContributions(double integTime,
                                                        BackSubMatrices & backSubContr,
                                                        Eigen::Vector3d sigma_BN,
                                                        Eigen::Vector3d omega_BN_B,
                                                        Eigen::Vector3d g_N)
{
    // Update dcm_BN
    Eigen::MRPd sigma_BNLoc;
    sigma_BNLoc = sigma_BN;
    this->dcm_BN = (sigma_BNLoc.toRotationMatrix()).transpose();

    // Update omega_BN_B
    this->omega_BN_B = omega_BN_B;
    Eigen::Matrix3d omegaTilde_BN_B = eigenTilde(this->omega_BN_B);

    // Define MBeta matrix
    Eigen::Matrix<double, 6, Eigen::Dynamic> MBeta1;
    MBeta1.resize(6, this->numDOF);
    Eigen::Matrix3d dcm_GB = this->jointDOFList.at(this->numDOF - 1).dcm_GB;
    Eigen::Vector3d r_GcG_B = dcm_GB.transpose() * this->r_GcG_G;
    Eigen::Matrix3d rTilde_GcG_B = eigenTilde(r_GcG_B);
    MBeta1.topRows(3) = this->mass * (transMap - rTilde_GcG_B * rotMap) * this->TMat;
    Eigen::Matrix3d IPntGc_B = dcm_GB.transpose() * this->IPntGc_G * dcm_GB;
    Eigen::Matrix3d IPntG_B = IPntGc_B - this->mass * rTilde_GcG_B * rTilde_GcG_B;
    MBeta1.bottomRows(3) = (IPntG_B * rotMap + this->mass * rTilde_GcG_B * transMap) * this->TMat;

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> MBeta;
    MBeta.resize(this->numDOF, this->numDOF);
    MBeta = this->TMat.transpose() * MBeta1;

    // Define ABetaStar matrix
    Eigen::Matrix<double, 6, 3> ABetaStar1;
    ABetaStar1.topRows(3) = - this->mass * Eigen::Matrix3d::Identity();
    ABetaStar1.bottomRows(3) = - this->mass * rTilde_GcG_B;

    Eigen::Matrix<double, Eigen::Dynamic, 3> ABetaStar;
    ABetaStar.resize(this->numDOF, 3);
    ABetaStar = this->TMat.transpose() * ABetaStar1;

    // Define BBetaStar matrix
    Eigen::Matrix<double, 6, 3> BBetaStar1;
    BBetaStar1.topRows(3) = this->mass * eigenTilde(r_GcG_B + transMap * this->TMat * this->beta);
    BBetaStar1.bottomRows(3) = - (IPntG_B - this->mass * rTilde_GcG_B * eigenTilde(transMap * this->TMat * this->beta));

    Eigen::Matrix<double, Eigen::Dynamic, 3> BBetaStar;
    BBetaStar.resize(this->numDOF, 3);
    BBetaStar = this->TMat.transpose() * BBetaStar1;

    // Define CBetaStar vector
    Eigen::VectorXd CBetaStar1;
    CBetaStar1.resize(6);
    CBetaStar1.head(3) = -2 * this->mass * omegaTilde_BN_B * (transMap - rTilde_GcG_B * rotMap) * this->TMat * this->betaDot
            - this->mass * omegaTilde_BN_B * omegaTilde_BN_B * (r_GcG_B + transMap * this->TMat * this->beta)
            - this->mass * eigenTilde(rotMap * this->TMat * this->betaDot) * eigenTilde(rotMap * this->TMat * this->betaDot) * r_GcG_B;
    CBetaStar1.tail(3) = - eigenTilde(rotMap * this->TMat * this->betaDot + this->omega_BN_B) * IPntG_B * (rotMap * this->TMat * this->betaDot + this->omega_BN_B)
            - IPntG_B * omegaTilde_BN_B * rotMap * this->TMat * this->betaDot
            - this->mass * rTilde_GcG_B * (2 * omegaTilde_BN_B * transMap * this->TMat * this->betaDot +
            omegaTilde_BN_B * omegaTilde_BN_B * transMap * this->TMat * this->beta);

    Eigen::VectorXd CBetaStar;
    CBetaStar = this->TMat.transpose() * CBetaStar1;

    for (int idx = 0; idx < this->numDOF; idx++) {

        double motorForceTorque{};
        if (this->jointDOFList.at(idx).type == DOF::Type::ROTATION) {
            motorForceTorque = this->jointDOFList.at(idx).u;
        } else {
            motorForceTorque = this->jointDOFList.at(idx).f;
        }

        CBetaStar[idx] += motorForceTorque
                          - this->jointDOFList.at(idx).k * (this->jointDOFList.at(idx).beta - this->jointDOFList.at(idx).betaRef)
                          - this->jointDOFList.at(idx).c * (this->jointDOFList.at(idx).betaDot - this->jointDOFList.at(idx).betaDotRef);
    }

    // Define ABeta, BBeta, and CBeta matrices
    this->ABeta = MBeta.inverse() * ABetaStar;
    this->BBeta = MBeta.inverse() * BBetaStar;
    this->CBeta = MBeta.inverse() * CBetaStar;

    // Define BSM [A] [B] [C] [D] contributions
    backSubContr.matrixA = this->mass * (transMap - rTilde_GcG_B * rotMap) * this->TMat * this->ABeta;
    backSubContr.matrixB = this->mass * (transMap - rTilde_GcG_B * rotMap) * this->TMat * this->BBeta;
    backSubContr.matrixC = (IPntGc_B * rotMap + this->mass * eigenTilde(r_GcG_B + transMap * this->TMat * this->beta) * (transMap - rTilde_GcG_B * rotMap)) * this->TMat * this->ABeta;
    backSubContr.matrixD = (IPntGc_B * rotMap + this->mass * eigenTilde(r_GcG_B + transMap * this->TMat * this->beta) * (transMap - rTilde_GcG_B * rotMap)) * this->TMat * this->BBeta;

    // Define BSM vecTrans and vecRot contributions
    backSubContr.vecTrans = - this->mass * eigenTilde(rotMap * this->TMat * this->betaDot) * eigenTilde(rotMap * this->TMat * this->betaDot) * r_GcG_B;
                            - this->mass * (transMap - rTilde_GcG_B * rotMap) * this->TMat * this->CBeta;
    backSubContr.vecRot = - this->mass * eigenTilde(r_GcG_B + transMap * this->TMat * this->beta) * eigenTilde(rotMap * this->TMat * this->betaDot) * eigenTilde(rotMap * this->TMat * this->betaDot) * r_GcG_B
            - eigenTilde(rotMap * this->TMat * this->betaDot + this->omega_BN_B) * IPntGc_B * rotMap * this->TMat * this->betaDot
            - this->mass * omegaTilde_BN_B * eigenTilde(r_GcG_B + transMap * this->TMat * this->beta) * (transMap - rTilde_GcG_B * rotMap) * this->TMat * this->betaDot
            - (IPntGc_B * rotMap + this->mass * eigenTilde(r_GcG_B + transMap * this->TMat * this->beta) * (transMap - rTilde_GcG_B * rotMap)) * this->TMat * this->CBeta;
}

/*! This method is for defining the state effector's MRP state derivative

 @param integTime [s] Time the method is called
 @param rDDot_BN_N [m/s^2] Acceleration of the vector pointing from the inertial frame origin to the B frame origin,
 expressed in inertial frame components
 @param omegaDot_BN_B [rad/s^2] Inertial time derivative of the angular velocity of the B frame with respect to the
 inertial frame, expressed in B frame components
 @param sigma_BN Current B frame attitude with respect to the inertial frame
*/
void GeneralSingleBodyStateEffector::computeDerivatives(double integTime,
                                                       Eigen::Vector3d rDDot_BN_N,
                                                       Eigen::Vector3d omegaDot_BN_B,
                                                       Eigen::Vector3d sigma_BN)
{
    // Update dcm_BN
    Eigen::MRPd sigma_BNLoc;
    sigma_BNLoc = sigma_BN;
    this->dcm_BN = (sigma_BNLoc.toRotationMatrix()).transpose();

    // Set betaState derivative
    this->betaState->setDerivative(this->betaDotState->getState());

    // Set betaDotState derivative
    Eigen::VectorXd betaDDot;
    betaDDot.resize(this->numDOF);
    Eigen::Vector3d rDDot_BN_B = this->dcm_BN * rDDot_BN_N;
    betaDDot = this->ABeta * rDDot_BN_B + this->BBeta * omegaDot_BN_B + this->CBeta;
    this->betaDotState->setDerivative(betaDDot);
}

/*! This method is for calculating the contributions of the effector to the energy and momentum of the spacecraft.

 @param integTime [s] Time the method is called
 @param rotAngMomPntCContr_B [kg m^2/s] Contribution of stateEffector to total rotational angular mom
 @param rotEnergyContr [J] Contribution of stateEffector to total rotational energy
 @param omega_BN_B [rad/s] Angular velocity of the B frame with respect to the inertial frame, expressed in B frame
 components
*/
void GeneralSingleBodyStateEffector::updateEnergyMomContributions(double integTime,
                                                                 Eigen::Vector3d & rotAngMomPntCContr_B,
                                                                 double & rotEnergyContr,
                                                                 Eigen::Vector3d omega_BN_B)
{
    // Update hub angular velocity
    this->omega_BN_B = omega_BN_B;

    // Rotational angular momentum contribution
    Eigen::Matrix3d omegaTilde_BN_B = eigenTilde(this->omega_BN_B);
    Eigen::Matrix3d dcm_GB = this->jointDOFList.at(this->numDOF - 1).dcm_GB;
    Eigen::Vector3d r_GcG_B = dcm_GB.transpose() * this->r_GcG_G;
    Eigen::Vector3d r_GB_B = transMap * this->TMat * this->beta;
    Eigen::Vector3d r_GcB_B = r_GcG_B + r_GB_B;
    Eigen::Matrix3d rTilde_GcG_B = eigenTilde(r_GcG_B);
    Eigen::Vector3d rPrime_GcB_B = (transMap - rTilde_GcG_B * rotMap) * this->TMat * this->betaDot;
    Eigen::Vector3d rDot_GcB_B = rPrime_GcB_B + omegaTilde_BN_B * r_GcB_B;
    Eigen::Matrix3d rTilde_GcB_B = eigenTilde(r_GcB_B);
    Eigen::Vector3d omega_GB_B = rotMap * this->TMat * this->betaDot;
    Eigen::Vector3d omega_GN_B = omega_GB_B + this->omega_BN_B;
    Eigen::Matrix3d IPntGc_B = dcm_GB.transpose() * this->IPntGc_G * dcm_GB;
    rotAngMomPntCContr_B = IPntGc_B * omega_GN_B + this->mass * rTilde_GcB_B * rDot_GcB_B;

    // Rotational energy contribution
    rotEnergyContr = 0.5 * omega_GN_B.dot(IPntGc_B * omega_GN_B)
                     + 0.5 * this->mass * rDot_GcB_B.dot(rDot_GcB_B);

    for (int idx = 0; idx < this->numDOF; idx++) {
        double rotEnergy = 0.5 * this->jointDOFList.at(idx).k * (this->jointDOFList.at(idx).beta
                - this->jointDOFList.at(idx).betaRef) * (this->jointDOFList.at(idx).beta
                        - this->jointDOFList.at(idx).betaRef);
        rotEnergyContr += rotEnergy;
    }
}

/*! This method computes the effector states relative to the inertial frame.

*/
void GeneralSingleBodyStateEffector::computeGeneralBodyInertialStates()
{
    // Inertial attitude
    Eigen::Matrix3d dcm_GB = this->jointDOFList.at(this->numDOF - 1).dcm_GB;
    Eigen::Matrix3d dcm_GN = dcm_GB * this->dcm_BN;
    *this->sigma_GN = eigenMRPd2Vector3d(eigenC2MRP(dcm_GN));

    // Inertial angular velocity
    Eigen::Vector3d omega_GB_B = rotMap * this->TMat * this->betaDot;
    Eigen::Vector3d omega_GN_B = omega_GB_B + this->omega_BN_B;
    *this->omega_GN_G = dcm_GB * omega_GN_B;

    // Inertial position
    Eigen::Vector3d r_GcG_B = dcm_GB.transpose() * this->r_GcG_G;
    Eigen::Vector3d r_GB_B = transMap * this->TMat * this->beta;
    Eigen::Vector3d r_GcB_B = r_GcG_B + r_GB_B;
    this->r_GcN_N = this->dcm_BN.transpose() * r_GcB_B + (Eigen::Vector3d)*this->inertialPositionProperty;
    *this->r_GN_N = this->dcm_BN.transpose() * r_GB_B + (Eigen::Vector3d)*this->inertialPositionProperty;

    // Inertial velocity
    Eigen::Matrix3d omegaTilde_BN_B = eigenTilde(this->omega_BN_B);
    Eigen::Matrix3d rTilde_GcG_B = eigenTilde(r_GcG_B);
    Eigen::Vector3d rPrime_GcB_B = (transMap - rTilde_GcG_B * rotMap) * this->TMat * this->betaDot;
    Eigen::Vector3d rDot_GcB_B = rPrime_GcB_B + omegaTilde_BN_B * r_GcB_B;
    this->v_GcN_N = this->dcm_BN.transpose() * rDot_GcB_B + (Eigen::Vector3d)*this->inertialVelocityProperty;
    Eigen::Vector3d rPrime_GB_B = transMap * this->TMat * this->betaDot;
    Eigen::Vector3d rDot_GB_B =  rPrime_GB_B + omegaTilde_BN_B * r_GB_B;
    *this->v_GN_N = this->dcm_BN.transpose() * rDot_GB_B + (Eigen::Vector3d)*this->inertialVelocityProperty;
}

/*! This method updates the effector state at the dynamics frequency.

 @param currentSimNanos [ns] Time the method is called
*/
void GeneralSingleBodyStateEffector::UpdateState(uint64_t currentSimNanos)
{
    // Call the method to compute the effector's inertial states
    this->computeGeneralBodyInertialStates();

    // Call the method to write the output messages
    this->writeOutputStateMessages(currentSimNanos);
}

/*! Setter method for the effector mass.
 @param mass [kg] Effector mass
*/
void GeneralSingleBodyStateEffector::setMass(const double mass) { this->mass = mass; }

/*! Setter method for IPntGc_G.
 @param IPntGc_G [kg-m^2] Effector's inertia matrix about its center of mass point Gc expressed in G frame components
*/
void GeneralSingleBodyStateEffector::setIPntGc_G(const Eigen::Matrix3d IPntGc_G) { this->IPntGc_G = IPntGc_G; }

/*! Setter method for r_GcG_G.
 @param r_GcG_G [m] Position vector of the effector's center of mass point Gc relative to the effector's body frame
 origin point G expressed in G frame components
*/
void GeneralSingleBodyStateEffector::setR_GcG_G(const Eigen::Vector3d r_GcG_G) { this->r_GcG_G = r_GcG_G; }

/*! Getter method for the effector mass.
 @return double
*/
double GeneralSingleBodyStateEffector::getMass() const { return this->mass; }

/*! Getter method for IPntGc_G.
 @return const Eigen::Matrix3d
*/
const Eigen::Matrix3d GeneralSingleBodyStateEffector::getIPntGc_G() const { return this->IPntGc_G; }

/*! Getter method for r_GcG_G.
 @return const Eigen::Vector3d
*/
const Eigen::Vector3d GeneralSingleBodyStateEffector::getR_GcG_G() const { return this->r_GcG_G; }

void GeneralSingleBodyStateEffector::addRotationalDOF(Eigen::Vector3d rotHat_G,
                                                      Eigen::Matrix3d dcm_G0P,
                                                      double thetaInit,
                                                      double thetaDotInit,
                                                      double springConstantK,
                                                      double damperConstantC) {
    this->numDOF++;
    this->numRotDOF++;
    this->TMat.conservativeResize(6, this->TMat.cols() + 1);
    this->ABeta.conservativeResize(this->ABeta.rows() + 1, 3);
    this->BBeta.conservativeResize(this->BBeta.rows() + 1, 3);
    this->CBeta.conservativeResize(this->CBeta.rows() + 1);
    this->spinningBodyRefInMsg.push_back(ReadFunctor<HingedRigidBodyMsgPayload>());
    this->motorTorqueInMsg.push_back(ReadFunctor<ArrayMotorTorqueMsgPayload>());
    this->spinningBodyOutMsgs.push_back(new Message<HingedRigidBodyMsgPayload>);

    // Create the new DOF
    DOF dof;
    dof.type = DOF::Type::ROTATION;
    dof.index = this->numRotDOF - 1;
    dof.axis_G = rotHat_G.normalized();
    dof.dcm_G0P = dcm_G0P;
    dof.betaInit = thetaInit;
    dof.betaDotInit = thetaDotInit;
    dof.k = springConstantK;
    dof.c = damperConstantC;

    this->jointDOFList.push_back(dof);
    this->betaInitList.push_back(dof.betaInit);
    this->betaDotInitList.push_back(dof.betaDotInit);
}

void GeneralSingleBodyStateEffector::addTranslationalDOF(Eigen::Vector3d transHat_G,
                                                         Eigen::Matrix3d dcm_G0P,
                                                         double rhoInit,
                                                         double rhoDotInit,
                                                         double springConstantK,
                                                         double damperConstantC) {
    this->numDOF++;
    this->numTransDOF++;
    this->TMat.conservativeResize(Eigen::NoChange, this->TMat.cols() + 1);
    this->ABeta.conservativeResize(this->ABeta.rows() + 1, 3);
    this->BBeta.conservativeResize(this->BBeta.rows() + 1, 3);
    this->CBeta.conservativeResize(this->CBeta.rows() + 1);
    this->translatingBodyRefInMsgs.push_back(ReadFunctor<LinearTranslationRigidBodyMsgPayload>());
    this->motorForceInMsg.push_back(ReadFunctor<ArrayMotorForceMsgPayload>());
    this->translatingBodyOutMsgs.push_back(new Message<LinearTranslationRigidBodyMsgPayload>);

    // Create the new DOF
    DOF dof;
    dof.type = DOF::Type::TRANSLATION;
    dof.index = this->numTransDOF -1;
    dof.axis_G = transHat_G.normalized();
    dof.dcm_G0P = dcm_G0P;
    dof.betaInit = rhoInit;
    dof.betaDotInit = rhoDotInit;
    dof.k = springConstantK;
    dof.c = damperConstantC;

    this->jointDOFList.push_back(dof);
    this->betaInitList.push_back(dof.betaInit);
    this->betaDotInitList.push_back(dof.betaDotInit);
}

void GeneralSingleBodyStateEffector::addRotScrewDOF(Eigen::Vector3d rotHat_G,
                                                    Eigen::Matrix3d dcm_G0P,
                                                    double thetaInit,
                                                    double thetaDotInit,
                                                    double screwConstant,
                                                    double springConstantK,
                                                    double damperConstantC) {
    this->numDOF++;
    this->numRotDOF++;
    this->TMat.conservativeResize(Eigen::NoChange, this->TMat.cols() + 1);
    this->ABeta.conservativeResize(this->ABeta.rows() + 1, 3);
    this->BBeta.conservativeResize(this->BBeta.rows() + 1, 3);
    this->CBeta.conservativeResize(this->CBeta.rows() + 1);
    this->spinningBodyRefInMsg.push_back(ReadFunctor<HingedRigidBodyMsgPayload>());
    this->motorTorqueInMsg.push_back(ReadFunctor<ArrayMotorTorqueMsgPayload>());
    this->spinningBodyOutMsgs.push_back(new Message<HingedRigidBodyMsgPayload>);

    // Create the new DOF
    DOF dof;
    dof.type = DOF::Type::ROTATION;
    dof.index = this->numRotDOF - 1;
    dof.axis_G = rotHat_G.normalized();
    dof.dcm_G0P = dcm_G0P;
    dof.betaInit = thetaInit;
    dof.betaDotInit = thetaDotInit;
    dof.screwConstant = screwConstant;
    dof.k = springConstantK;
    dof.c = damperConstantC;

    this->jointDOFList.push_back(dof);
    this->betaInitList.push_back(dof.betaInit);
    this->betaDotInitList.push_back(dof.betaDotInit);
}

void GeneralSingleBodyStateEffector::addTransScrewDOF(Eigen::Vector3d transHat_G,
                                                     Eigen::Matrix3d dcm_G0P,
                                                     double rhoInit,
                                                     double rhoDotInit,
                                                     double screwConstant,
                                                     double springConstantK,
                                                     double damperConstantC) {
    this->numDOF++;
    this->numTransDOF++;
    this->TMat.conservativeResize(Eigen::NoChange, this->TMat.cols() + 1);
    this->ABeta.conservativeResize(this->ABeta.rows() + 1, 3);
    this->BBeta.conservativeResize(this->BBeta.rows() + 1, 3);
    this->CBeta.conservativeResize(this->CBeta.rows() + 1);
    this->translatingBodyRefInMsgs.push_back(ReadFunctor<LinearTranslationRigidBodyMsgPayload>());
    this->motorForceInMsg.push_back(ReadFunctor<ArrayMotorForceMsgPayload>());
    this->translatingBodyOutMsgs.push_back(new Message<LinearTranslationRigidBodyMsgPayload>);

    // Create the new DOF
    DOF dof;
    dof.type = DOF::Type::TRANSLATION;
    dof.index = this->numTransDOF - 1;
    dof.axis_G = transHat_G.normalized();
    dof.dcm_G0P = dcm_G0P;
    dof.betaInit = rhoInit;
    dof.betaDotInit = rhoDotInit;
    dof.screwConstant = screwConstant;
    dof.k = springConstantK;
    dof.c = damperConstantC;

    this->jointDOFList.push_back(dof);
    this->betaInitList.push_back(dof.betaInit);
    this->betaDotInitList.push_back(dof.betaDotInit);
}
