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
                0, 0, 0,  1, 0, 0,
                0, 0, 0,  0, 1, 0,
                0, 0, 0,  0, 0, 1
        ).finished();
const Eigen::Matrix<double, 3, 6> rotMap =
        (Eigen::Matrix<double, 3, 6>() <<
                1, 0, 0,  0, 0, 0,
                0, 1, 0,  0, 0, 0,
                0, 0, 1,  0, 0, 0
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
    std::vector<DOF>::iterator jointDOF;
    for(jointDOF = this->jointDOFList.begin(); jointDOF != this->jointDOFList.end(); jointDOF++) {
        jointDOF->beta = this->beta[jointDOF->index];
        jointDOF->betaDot = this->betaDot[jointDOF->index];
    }

    // Compute general body attitudes
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
            this->TMat.col(dofIndex).tail<3>() = jointDOF->screwConstant * jointDOFAxis_B;
        } else {
            this->TMat.col(dofIndex).head<3>() = jointDOF->screwConstant * jointDOFAxis_B;
        }
    }

    // Compute and set effProps.rEff_CB_B
    Eigen::Matrix3d dcm_GB = this->jointDOFList.at(this->numDOF - 1).dcm_GB;
    Eigen::Vector3d r_GcG_B = dcm_GB.transpose() * this->r_GcG_G;
    Eigen::Vector3d r_GB_B = transMap * this->TMat * this->beta;
    this->r_GcB_B = r_GcG_B + r_GB_B;
    this->effProps.rEff_CB_B = this->r_GcB_B;

    // Compute and set effProps.IEffPntB_B
    this->IPntGc_B = dcm_GB.transpose() * this->IPntGc_G * dcm_GB;
    Eigen::Matrix3d rTilde_GcB_B = eigenTilde(this->r_GcB_B);
    this->effProps.IEffPntB_B = this->IPntGc_G - this->mass * rTilde_GcB_B * rTilde_GcB_B;

    // Compute and set effProps.rEffPrime_CB_B
    Eigen::Matrix3d rTilde_GcG_B = eigenTilde(r_GcG_B);
    this->rPrime_GcB_B = (transMap - rTilde_GcG_B * rotMap) * this->TMat * this->betaDot;
    this->effProps.rEffPrime_CB_B = this->rPrime_GcB_B;

    // Compute and set effProps.IEffPrimePntB_B
    this->omega_GB_B = rotMap * this->TMat * this->betaDot;
    Eigen::Matrix3d omegaTilde_GB_B = eigenTilde(this->omega_GB_B);
    Eigen::Matrix3d rPrimeTilde_GcB_B = eigenTilde(this->rPrime_GcB_B);
    this->effProps.IEffPrimePntB_B = omegaTilde_GB_B * this->IPntGc_B - this->IPntGc_B * omegaTilde_GB_B
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
//    // Update sigma_BN and dcm_BN
//    this->sigma_BN = sigma_BN;
//    this->dcm_BN = (this->sigma_BN.toRotationMatrix()).transpose();
//
//    // Update omega_BN_B
//    this->omega_BN_B = omega_BN_B;
//    Eigen::Matrix3d omegaTilde_BN_B = eigenTilde(this->omega_BN_B);
//
//    // Update sigma_PN
//    Eigen::Matrix3d dcm_PN = this->dcm_BP.transpose() * this->dcm_BN;
//    *this->sigma_PN = eigenMRPd2Vector3d(eigenC2MRP(dcm_PN));
//
//    // Update omega_PN_P
//    *this->omega_PN_P = *this->omega_PB_P + this->dcm_BP.transpose() * this->omega_BN_B;
//
//    Eigen::Matrix3d totMatrixA;
//    Eigen::Matrix3d totMatrixB;
//    Eigen::Matrix3d totMatrixC;
//    Eigen::Matrix3d totMatrixD;
//    Eigen::Vector3d totVecTrans;
//    Eigen::Vector3d totVecRot;
//    totMatrixA.setZero();
//    totMatrixB.setZero();
//    totMatrixC.setZero();
//    totMatrixD.setZero();
//    totVecTrans.setZero();
//    totVecRot.setZero();
//
//    // Loop through attached state effectors and compute their contributions
//    std::vector<StateEffector*>::iterator it;
//    for(it = this->stateEffectors.begin(); it != this->stateEffectors.end(); it++)
//    {
//        backSubContr.matrixA.setZero();
//        backSubContr.matrixB.setZero();
//        backSubContr.matrixC.setZero();
//        backSubContr.matrixD.setZero();
//        backSubContr.vecTrans.setZero();
//        backSubContr.vecRot.setZero();
//
//        (*it)->updateContributions(integTime, backSubContr, *this->sigma_PN, *this->omega_PN_P, g_N);
//        (*it)->addPrescribedMotionCouplingContributions(backSubContr);
//
//        totMatrixA += this->dcm_BP * backSubContr.matrixA * this->dcm_BP.transpose();
//        totMatrixB += this->dcm_BP * backSubContr.matrixB * this->dcm_BP.transpose();
//        totMatrixC += this->dcm_BP * backSubContr.matrixC * this->dcm_BP.transpose();
//        totMatrixD += this->dcm_BP * backSubContr.matrixD * this->dcm_BP.transpose();
//        totVecTrans += this->dcm_BP * backSubContr.vecTrans;
//        totVecRot += this->dcm_BP * backSubContr.vecRot;
//    }
//
//    backSubContr.matrixA = totMatrixA;
//    backSubContr.matrixB = totMatrixB;
//    backSubContr.matrixC = totMatrixC;
//    backSubContr.matrixD = totMatrixD;
//    backSubContr.vecTrans = totVecTrans;
//    backSubContr.vecRot = totVecRot;
//
//    // Prescribed motion translation contributions
//    Eigen::Matrix3d omegaPrimeTilde_PB_B = eigenTilde(this->omegaPrime_PB_B);
//    this->rPrimePrime_PcB_B = (omegaPrimeTilde_PB_B + this->omegaTilde_PB_B * this->omegaTilde_PB_B) * this->r_PcP_B +
//                              this->rPrimePrime_PM_B;
//    backSubContr.vecTrans += -this->mass * this->rPrimePrime_PcB_B;
//
//    // Prescribed motion rotation contributions
//    Eigen::Matrix3d IPrimePntPc_B;
//    IPrimePntPc_B = this->omegaTilde_PB_B * this->IPntGc_B - this->IPntGc_B * this->omegaTilde_PB_B;
//    backSubContr.vecRot += -(this->mass * this->rTilde_PcB_B * this->rPrimePrime_PcB_B)
//                          - (IPrimePntPc_B + this->omegaTilde_BN_B * this->IPntGc_B) * this->omega_PB_B
//                          - this->IPntGc_B * this->omegaPrime_PB_B
//                          - this->mass * this->omegaTilde_BN_B * rTilde_PcB_B * this->rPrime_PcB_B;
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
//    Eigen::MRPd sigma_PM_loc;
//    sigma_PM_loc = (Eigen::Vector3d)this->sigma_PMState->getState();
//    this->sigma_PMState->setDerivative(0.25*sigma_PM_loc.Bmat()*this->omega_PM_P);
//
//    // Loop through attached state effectors for compute derivatives
//    if (!this->stateEffectors.empty()) {
//        // Update sigma_BN and dcm_BN
//        this->sigma_BN = sigma_BN;
//        this->dcm_BN = (this->sigma_BN.toRotationMatrix()).transpose();
//
//        // Compute dcm_FN and sigma_FN
//        Eigen::Matrix3d dcm_PN = this->dcm_BP.transpose() * this->dcm_BN;
//        *this->sigma_PN = eigenMRPd2Vector3d(eigenC2MRP(dcm_PN));
//
//        // Compute omegaDot_PN_P
//        Eigen::Matrix3d omegaTilde_BN_B = eigenTilde(this->omega_BN_B);
//        Eigen::Vector3d omegaDot_PN_B = this->omegaPrime_PM_B + this->omegaTilde_BN_B * this->omega_PM_B + omegaDot_BN_B;
//        Eigen::Vector3d omegaDot_PN_P = this->dcm_BP.transpose() * omegaDot_PN_B;
//
//        // Compute rDDot_PN_N
//        Eigen::Matrix3d omegaDotTilde_BN_B = eigenTilde(omegaDot_BN_B);
//        Eigen::Vector3d rDDot_PB_B = this->rPrimePrime_PM_B + 2 * this->omegaTilde_BN_B * this->rPrime_PM_B
//                                     + omegaDotTilde_BN_B * (*this->r_PB_B)
//                                     + this->omegaTilde_BN_B * this->omegaTilde_BN_B * (*this->r_PB_B);
//        Eigen::Vector3d rDDot_PN_N = this->dcm_BN.transpose() * rDDot_PB_B + rDDot_BN_N;
//
//        std::vector<StateEffector*>::iterator it;
//        for(it = this->stateEffectors.begin(); it != this->stateEffectors.end(); it++) {
//            (*it)->computeDerivatives(integTime, rDDot_PN_N, omegaDot_PN_P, *this->sigma_PN);
//        }
//    }
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
    // Update angular velocities
    this->omega_BN_B = omega_BN_B;
    Eigen::Matrix3d omegaTilde_BN_B = eigenTilde(this->omega_BN_B);
    this->omega_GN_B = this->omega_GB_B + this->omega_BN_B;

    // Rotational angular momentum contribution
    this->rDot_GcB_B = this->rPrime_GcB_B + omegaTilde_BN_B * this->r_GcB_B;
    Eigen::Matrix3d rTilde_GcB_B = eigenTilde(this->r_GcB_B);
    rotAngMomPntCContr_B += this->IPntGc_B * this->omega_GN_B + this->mass * rTilde_GcB_B * this->rDot_GcB_B;

    // Rotational energy contribution
    rotEnergyContr += 0.5 * this->omega_GN_B.dot(this->IPntGc_B * this->omega_GN_B)
                     + 0.5 * this->mass * this->rDot_GcB_B.dot(this->rDot_GcB_B);
}

/*! This method computes the effector states relative to the inertial frame.

*/
void GeneralSingleBodyStateEffector::computeGeneralBodyInertialStates()
{
//    // inertial attitude
//    Eigen::Matrix3d dcm_SN;
//    dcm_SN = (this->dcm_BS).transpose() * this->dcm_BN;
//    *this->sigma_SN = eigenMRPd2Vector3d(eigenC2MRP(dcm_SN));
//    *this->omega_SN_S = (this->dcm_BS).transpose() * this->omega_SN_B;
//
//    // inertial position vector
//    this->r_ScN_N = (Eigen::Vector3d)*this->inertialPositionProperty + this->dcm_BN.transpose() * this->r_ScB_B;
//    *this->r_SN_N = (Eigen::Vector3d)(*this->inertialPositionProperty) + this->dcm_BN.transpose() * this->r_SB_B;
//
//    // inertial velocity vector
//    this->v_ScN_N = (Eigen::Vector3d)*this->inertialVelocityProperty + this->dcm_BN.transpose() * this->rDot_ScB_B;
//    *this->v_SN_N = (Eigen::Vector3d)*this->inertialVelocityProperty + this->dcm_BN.transpose() * this->rDot_SB_B;
//
//
//
//
//
//
//    // Compute the effector's attitude with respect to the inertial frame
//    Eigen::Matrix3d dcm_GN = this->dcm_GB * this->dcm_BN;
//    *this->sigma_GN = eigenMRPd2Vector3d(eigenC2MRP(dcm_GN));
//
//    // Compute the effector's inertial angular velocity
//    *this->omega_GN_G = this->Phi_theta * this->TMat *this->beta + this->dcm_GB * this->omega_BN_B;
//
//    // Compute the effector's inertial position vectors
//    this->r_GcN_N = (Eigen::Vector3d)(*this->inertialPositionProperty)
//            + this->dcm_BN.transpose() * (this->dcm_GB.transpose() * (this->r_GcG_G + this->Phi_rho * this->TMat * this->beta));
//    *this->r_GN_N = (Eigen::Vector3d)(*this->inertialPositionProperty)
//            + this->dcm_BN.transpose() * (this->dcm_GB.transpose() * (this->Phi_rho * this->TMat * this->beta));
//
//    // Compute the effector's inertial velocity vectors
//    this->v_PcN_N = (Eigen::Vector3d)*this->inertialVelocityProperty
//            + this->dcm_BN.transpose() * this->rDot_PcB_B;
//    *this->v_PN_N = (Eigen::Vector3d)*this->inertialVelocityProperty
//            + this->dcm_BN.transpose() * (*this->rPrime_PB_B +
//            this->omegaTilde_BN_B * *this->r_PB_B);
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
                                                      double thetaDotInit) {
    this->numDOF++;
    this->TMat.conservativeResize(Eigen::NoChange, this->TMat.cols() + 1);

    // Create the new DOF
    DOF dof;
    dof.type = DOF::Type::ROTATION;
    dof.index = this->numDOF;
    dof.axis_G = rotHat_G.normalized();
    dof.dcm_G0P = dcm_G0P;
    dof.betaInit = thetaInit;
    dof.betaDotInit = thetaDotInit;

    this->jointDOFList.push_back(dof);
    this->betaInitList.push_back(dof.betaInit);
    this->betaDotInitList.push_back(dof.betaDotInit);
}

void GeneralSingleBodyStateEffector::addTranslationalDOF(Eigen::Vector3d transHat_G,
                                                         Eigen::Matrix3d dcm_G0P,
                                                         double rhoInit,
                                                         double rhoDotInit) {
    this->numDOF++;
    this->TMat.conservativeResize(Eigen::NoChange, this->TMat.cols() + 1);

    // Create the new DOF
    DOF dof;
    dof.type = DOF::Type::TRANSLATION;
    dof.index = this->numDOF;
    dof.axis_G = transHat_G.normalized();
    dof.dcm_G0P = dcm_G0P;
    dof.betaInit = rhoInit;
    dof.betaDotInit = rhoDotInit;

    this->jointDOFList.push_back(dof);
    this->betaInitList.push_back(dof.betaInit);
    this->betaDotInitList.push_back(dof.betaDotInit);
}

void GeneralSingleBodyStateEffector::addRotScrewDOF(Eigen::Vector3d rotHat_G,
                                                    Eigen::Matrix3d dcm_G0P,
                                                    double thetaInit,
                                                    double thetaDotInit,
                                                    double screwConstant) {
    this->numDOF++;
    this->TMat.conservativeResize(Eigen::NoChange, this->TMat.cols() + 1);

    // Create the new DOF
    DOF dof;
    dof.type = DOF::Type::ROTATION;
    dof.index = this->numDOF;
    dof.axis_G = rotHat_G.normalized();
    dof.dcm_G0P = dcm_G0P;
    dof.betaInit = thetaInit;
    dof.betaDotInit = thetaDotInit;
    dof.screwConstant = screwConstant;

    this->jointDOFList.push_back(dof);
    this->betaInitList.push_back(dof.betaInit);
    this->betaDotInitList.push_back(dof.betaDotInit);
}

void GeneralSingleBodyStateEffector::addTransScrewDOF(Eigen::Vector3d transHat_G,
                                                     Eigen::Matrix3d dcm_G0P,
                                                     double rhoInit,
                                                     double rhoDotInit,
                                                     double screwConstant) {
    this->numDOF++;
    this->TMat.conservativeResize(Eigen::NoChange, this->TMat.cols() + 1);

    // Create the new DOF
    DOF dof;
    dof.type = DOF::Type::TRANSLATION;
    dof.index = this->numDOF;
    dof.axis_G = transHat_G.normalized();
    dof.dcm_G0P = dcm_G0P;
    dof.betaInit = rhoInit;
    dof.betaDotInit = rhoDotInit;
    dof.screwConstant = screwConstant;

    this->jointDOFList.push_back(dof);
    this->betaInitList.push_back(dof.betaInit);
    this->betaDotInitList.push_back(dof.betaDotInit);
}
