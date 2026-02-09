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
    int degsOfFreedom = static_cast<int>(this->beta.size());
    this->betaState = states.registerState(degsOfFreedom, 1, this->nameOfBetaState);
    this->betaState->setState(this->beta);
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

    // Compute general body attitudes
    for (int idx = 0; idx < this->numDOF; idx++) {
        if (this->isRotDOFList[idx]) {

            Eigen::Vector3d prv_GG0 = this->beta[idx] * this->freeAxisList;
            double prv_GG0_array[3];
            eigenVector3d2CArray(prv_GG0, prv_GG0_array);

            double dcm_GG0_array[3][3];
            PRV2C(prv_GG0_array, dcm_GG0_array);
            Eigen::Matrix3d dcm_GG0 = c2DArray2EigenMatrix3d(dcm_GG0_array);

            if (idx == 0) {
                this->dcm_GBList[idx] = dcm_GG0 * dcm_G0PList[idx];
            } else {
                this->dcm_GBList[idx] = dcm_GG0 * dcm_G0PList[idx] * this->dcm_GBList[idx-1];
            }
        }
    }

    // Compute general body transformation matrix
    Eigen::MatrixXd TMat(6, this->numDOF);
    TMat.setZero();
    for (int idx = 0; idx < this->numDOF; idx++) {
        Eigen::Vector3d idxDOFAxis_B = this->dcm_GBList[idx].transpose() * this->freeAxisList[idx];

        if (this->isRotDOFList[idx]) {
            TMat.col(idx).tail<3>() = idxDOFAxis_B;
        } else {
            TMat.col(idx).head<3>() = idxDOFAxis_B;
        }
    }
    this->T = TMat;

    // Compute and set effProps.rEff_CB_B
    Eigen::Vector3d r_GcG_B = dcm_GB.transpose() * this->r_GcG_G;
    Eigen::Vector3d r_GB_B = transMap * this->T * this->beta;
    Eigen::Vector3d r_GcB_B = r_GcG_B + r_GB_B;
    this->effProps.rEff_CB_B = r_GcB_B;

    // Compute and set effProps.IEffPntB_B
    Eigen::Matrix3d dcm_GB = this->dcm_GBList[numDOF -1];
    Eigen::Matrix3d IPntGc_B = dcm_GB.transpose() * IPntGc_G * dcm_GB;
    Eigen::Matrix3d rTilde_GcB_B = eigenTilde(r_GcB_B);
    this->effProps.IEffPntB_B = this->IPntGc_G - this->mass * rTilde_GcB_B * rTilde_GcB_B;

    // Compute and set effProps.rEffPrime_CB_B
    Eigen::Matrix3d rTilde_GcG_B = eigenTilde(r_GcG_B);
    Eigen::Vector3d rPrime_GcB_B = (transMap - rTilde_GcG_B * rotMap) * this->T * this->betaDot;
    this->effProps.rEffPrime_CB_B = rPrime_GcB_B;

    // Compute and set effProps.IEffPrimePntB_B
    Eigen::Vector3d omega_GB_B = rotMap * T * this->betaDot;
    Eigen::Matrix3d omegaTilde_GB_B = eigenTilde(omega_GB_B);
    Eigen::Vector3d rPrimeTilde_GcB_B = eigenTilde(rPrime_GcB_B);
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
    // Update sigma_BN and dcm_BN
    this->sigma_BN = sigma_BN;
    this->dcm_BN = (this->sigma_BN.toRotationMatrix()).transpose();

    // Update omega_BN_B
    this->omega_BN_B = omega_BN_B;
    this->omegaTilde_BN_B = eigenTilde(this->omega_BN_B);

    // Update sigma_PN
    Eigen::Matrix3d dcm_PN = this->dcm_BP.transpose() * this->dcm_BN;
    *this->sigma_PN = eigenMRPd2Vector3d(eigenC2MRP(dcm_PN));

    // Update omega_PN_P
    *this->omega_PN_P = *this->omega_PB_P + this->dcm_BP.transpose() * this->omega_BN_B;

    Eigen::Matrix3d totMatrixA;
    Eigen::Matrix3d totMatrixB;
    Eigen::Matrix3d totMatrixC;
    Eigen::Matrix3d totMatrixD;
    Eigen::Vector3d totVecTrans;
    Eigen::Vector3d totVecRot;
    totMatrixA.setZero();
    totMatrixB.setZero();
    totMatrixC.setZero();
    totMatrixD.setZero();
    totVecTrans.setZero();
    totVecRot.setZero();

    // Loop through attached state effectors and compute their contributions
    std::vector<StateEffector*>::iterator it;
    for(it = this->stateEffectors.begin(); it != this->stateEffectors.end(); it++)
    {
        backSubContr.matrixA.setZero();
        backSubContr.matrixB.setZero();
        backSubContr.matrixC.setZero();
        backSubContr.matrixD.setZero();
        backSubContr.vecTrans.setZero();
        backSubContr.vecRot.setZero();

        (*it)->updateContributions(integTime, backSubContr, *this->sigma_PN, *this->omega_PN_P, g_N);
        (*it)->addPrescribedMotionCouplingContributions(backSubContr);

        totMatrixA += this->dcm_BP * backSubContr.matrixA * this->dcm_BP.transpose();
        totMatrixB += this->dcm_BP * backSubContr.matrixB * this->dcm_BP.transpose();
        totMatrixC += this->dcm_BP * backSubContr.matrixC * this->dcm_BP.transpose();
        totMatrixD += this->dcm_BP * backSubContr.matrixD * this->dcm_BP.transpose();
        totVecTrans += this->dcm_BP * backSubContr.vecTrans;
        totVecRot += this->dcm_BP * backSubContr.vecRot;
    }

    backSubContr.matrixA = totMatrixA;
    backSubContr.matrixB = totMatrixB;
    backSubContr.matrixC = totMatrixC;
    backSubContr.matrixD = totMatrixD;
    backSubContr.vecTrans = totVecTrans;
    backSubContr.vecRot = totVecRot;

    // Prescribed motion translation contributions
    Eigen::Matrix3d omegaPrimeTilde_PB_B = eigenTilde(this->omegaPrime_PB_B);
    this->rPrimePrime_PcB_B = (omegaPrimeTilde_PB_B + this->omegaTilde_PB_B * this->omegaTilde_PB_B) * this->r_PcP_B +
                              this->rPrimePrime_PM_B;
    backSubContr.vecTrans += -this->mass * this->rPrimePrime_PcB_B;

    // Prescribed motion rotation contributions
    Eigen::Matrix3d IPrimePntPc_B;
    IPrimePntPc_B = this->omegaTilde_PB_B * this->IPntPc_B - this->IPntPc_B * this->omegaTilde_PB_B;
    backSubContr.vecRot += -(this->mass * this->rTilde_PcB_B * this->rPrimePrime_PcB_B)
                          - (IPrimePntPc_B + this->omegaTilde_BN_B * this->IPntPc_B) * this->omega_PB_B
                          - this->IPntPc_B * this->omegaPrime_PB_B
                          - this->mass * this->omegaTilde_BN_B * rTilde_PcB_B * this->rPrime_PcB_B;
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
    Eigen::MRPd sigma_PM_loc;
    sigma_PM_loc = (Eigen::Vector3d)this->sigma_PMState->getState();
    this->sigma_PMState->setDerivative(0.25*sigma_PM_loc.Bmat()*this->omega_PM_P);

    // Loop through attached state effectors for compute derivatives
    if (!this->stateEffectors.empty()) {
        // Update sigma_BN and dcm_BN
        this->sigma_BN = sigma_BN;
        this->dcm_BN = (this->sigma_BN.toRotationMatrix()).transpose();

        // Compute dcm_FN and sigma_FN
        Eigen::Matrix3d dcm_PN = this->dcm_BP.transpose() * this->dcm_BN;
        *this->sigma_PN = eigenMRPd2Vector3d(eigenC2MRP(dcm_PN));

        // Compute omegaDot_PN_P
        Eigen::Vector3d omegaDot_PN_B = this->omegaPrime_PM_B + this->omegaTilde_BN_B * this->omega_PM_B + omegaDot_BN_B;
        Eigen::Vector3d omegaDot_PN_P = this->dcm_BP.transpose() * omegaDot_PN_B;

        // Compute rDDot_PN_N
        Eigen::Matrix3d omegaDotTilde_BN_B = eigenTilde(omegaDot_BN_B);
        Eigen::Vector3d rDDot_PB_B = this->rPrimePrime_PM_B + 2 * this->omegaTilde_BN_B * this->rPrime_PM_B
                                     + omegaDotTilde_BN_B * (*this->r_PB_B)
                                     + this->omegaTilde_BN_B * this->omegaTilde_BN_B * (*this->r_PB_B);
        Eigen::Vector3d rDDot_PN_N = this->dcm_BN.transpose() * rDDot_PB_B + rDDot_BN_N;

        std::vector<StateEffector*>::iterator it;
        for(it = this->stateEffectors.begin(); it != this->stateEffectors.end(); it++) {
            (*it)->computeDerivatives(integTime, rDDot_PN_N, omegaDot_PN_P, *this->sigma_PN);
        }
    }
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
    // Update omega_BN_B, omega_PN_B, and omega_PN_P
    this->omega_BN_B = omega_BN_B;
    this->omegaTilde_BN_B = eigenTilde(this->omega_BN_B);
    this->omega_PN_B = this->omega_PB_B + this->omega_BN_B;
    *this->omega_PN_P = this->dcm_BP.transpose() * this->omega_PN_B;

    Eigen::Vector3d totRotAngMomPntC_B;
    totRotAngMomPntC_B.setZero();
    double totRotEnergy = 0.0;

    // Loop through attached state effectors for contributions
    std::vector<StateEffector*>::iterator it;
    for(it = this->stateEffectors.begin(); it != this->stateEffectors.end(); it++) {
        rotAngMomPntCContr_B.setZero();
        rotEnergyContr = 0.0;

        (*it)->updateEnergyMomContributions(integTime, rotAngMomPntCContr_B, rotEnergyContr, *this->omega_PN_P);

        // Additional terms for rotational angular momentum
        Eigen::Vector3d r_EcP_B = this->dcm_BP * (*it)->effProps.rEff_CB_B;
        Eigen::Matrix3d rTilde_EcP_B = eigenTilde(r_EcP_B);
        Eigen::Vector3d rDot_PB_B = *this->rPrime_PB_B + this->omegaTilde_BN_B * *this->r_PB_B;
        Eigen::Matrix3d rTilde_PB_B = eigenTilde(*this->r_PB_B);
        Eigen::Matrix3d omegaTilde_PN_B = eigenTilde(this->omega_PN_B);
        Eigen::Vector3d rDot_EcP_B = this->dcm_BP * (*it)->effProps.rEffPrime_CB_B + omegaTilde_PN_B * r_EcP_B;
        totRotAngMomPntC_B += this->dcm_BP * rotAngMomPntCContr_B
                                + (*it)->effProps.mEff * rTilde_EcP_B * rDot_PB_B
                                + (*it)->effProps.mEff * rTilde_PB_B * rDot_EcP_B
                                + (*it)->effProps.mEff * rTilde_PB_B * rDot_PB_B;

        // Additional terms for rotational energy
        totRotEnergy += rotEnergyContr
                        + (*it)->effProps.mEff * rDot_EcP_B.dot(rDot_PB_B )
                        + 0.5 * (*it)->effProps.mEff * rDot_PB_B.dot(rDot_PB_B);
    }

    rotAngMomPntCContr_B = totRotAngMomPntC_B;
    rotEnergyContr = totRotEnergy;

    // Prescribed motion rotational angular momentum contribution
    this->rDot_PcB_B = this->rPrime_PcB_B + this->omegaTilde_BN_B * this->r_PcB_B;
    rotAngMomPntCContr_B += this->IPntPc_B * this->omega_PN_B + this->mass * this->rTilde_PcB_B * this->rDot_PcB_B;

    // Prescribed motion rotational energy contribution
    rotEnergyContr += 0.5 * this->omega_PN_B.dot(this->IPntPc_B * this->omega_PN_B)
                     + 0.5 * this->mass * this->rDot_PcB_B.dot(this->rDot_PcB_B);
}

/*! This method computes the effector states relative to the inertial frame.

*/
void GeneralSingleBodyStateEffector::computeGeneralBodyInertialStates()
{
    // inertial attitude
    Eigen::Matrix3d dcm_SN;
    dcm_SN = (this->dcm_BS).transpose() * this->dcm_BN;
    *this->sigma_SN = eigenMRPd2Vector3d(eigenC2MRP(dcm_SN));
    *this->omega_SN_S = (this->dcm_BS).transpose() * this->omega_SN_B;

    // inertial position vector
    this->r_ScN_N = (Eigen::Vector3d)*this->inertialPositionProperty + this->dcm_BN.transpose() * this->r_ScB_B;
    *this->r_SN_N = (Eigen::Vector3d)(*this->inertialPositionProperty) + this->dcm_BN.transpose() * this->r_SB_B;

    // inertial velocity vector
    this->v_ScN_N = (Eigen::Vector3d)*this->inertialVelocityProperty + this->dcm_BN.transpose() * this->rDot_ScB_B;
    *this->v_SN_N = (Eigen::Vector3d)*this->inertialVelocityProperty + this->dcm_BN.transpose() * this->rDot_SB_B;






    // Compute the effector's attitude with respect to the inertial frame
    Eigen::Matrix3d dcm_GN = this->dcm_GB * this->dcm_BN;
    *this->sigma_GN = eigenMRPd2Vector3d(eigenC2MRP(dcm_GN));

    // Compute the effector's inertial angular velocity
    *this->omega_GN_G = this->Phi_theta * this->T *this->beta + this->dcm_GB * this->omega_BN_B;

    // Compute the effector's inertial position vectors
    this->r_GcN_N = (Eigen::Vector3d)(*this->inertialPositionProperty)
            + this->dcm_BN.transpose() * (this->dcm_GB.transpose() * (this->r_GcG_G + this->Phi_rho * this->T * this->beta));
    *this->r_GN_N = (Eigen::Vector3d)(*this->inertialPositionProperty)
            + this->dcm_BN.transpose() * (this->dcm_GB.transpose() * (this->Phi_rho * this->T * this->beta));

    // Compute the effector's inertial velocity vectors
    this->v_PcN_N = (Eigen::Vector3d)*this->inertialVelocityProperty
            + this->dcm_BN.transpose() * this->rDot_PcB_B;
    *this->v_PN_N = (Eigen::Vector3d)*this->inertialVelocityProperty
            + this->dcm_BN.transpose() * (*this->rPrime_PB_B +
            this->omegaTilde_BN_B * *this->r_PB_B);
}

/*! This method updates the effector state at the dynamics frequency.

 @param currentSimNanos [ns] Time the method is called
*/
void GeneralSingleBodyStateEffector::UpdateState(uint64_t currentSimNanos)
{
    // Store the current simulation time
    this->currentSimTimeSec = currentSimNanos * NANO2SEC;

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

/*! Setter method for betaInit.
 @param betaInit General effector initial state vector.
*/
void GeneralSingleBodyStateEffector::setBetaInit(const Eigen::VectorXd betaInit) { this->betaInit = betaInit; }

/*! Setter method for betaDotInit.
 @param betaDotInit General effector initial state vector derivative.
*/
void GeneralSingleBodyStateEffector::setBetaDotInit(const Eigen::VectorXd betaDotInit) { this->betaDotInit = betaDotInit; }

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

/*! Getter method for betaInit.
 @return const Eigen::VectorXd
*/
const Eigen::VectorXd GeneralSingleBodyStateEffector::getBetaInit() const { return this->betaInit; }

/*! Getter method for betaDotInit.
 @return const Eigen::VectorXd
*/
const Eigen::VectorXd GeneralSingleBodyStateEffector::getBetaDotInit() const { return this->betaDotInit; }

void GeneralSingleBodyStateEffector::addFreeAxis(Eigen::Vector3d gHat_G, Eigen::Matrix3d dcm_G0P, bool isRotDOF) {
    this->freeAxisList(gHat_G);
    this->dcm_G0PList.push_back(dcm_G0P);
    this->isRotDOFList.push_back(isRotDOF);
    this->numDOF++;
}
