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

#include "linearSpringMassDamper.h"
#include "architecture/utilities/avsEigenSupport.h"
#include <cmath>

/*! This is the constructor, setting variables to default values */
LinearSpringMassDamper::LinearSpringMassDamper()
{
	// - zero the contributions for mass props and mass rates
	this->effProps.mEff = 0.0;
	this->effProps.IEffPntB_B.setZero();
	this->effProps.rEff_CB_B.setZero();
	this->effProps.rEffPrime_CB_B.setZero();
	this->effProps.IEffPrimePntB_B.setZero();

	// - Initialize the variables to working values
	this->massSMD = 1.0;
	this->r_PB_B.setZero();
	this->pHat_B.setIdentity();
	this->k = 1.0;
	this->c = 0.0;
    this->rhoInit = 0.0;
    this->rhoDotInit = 0.0;
    this->massInit = 0.0;
	this->nameOfRhoState = "linearSpringMassDamperRho" + std::to_string(this->effectorID);
	this->nameOfRhoDotState = "linearSpringMassDamperRhoDot" + std::to_string(this->effectorID);
	this->nameOfMassState = "linearSpringMassDamperMass" + std::to_string(this->effectorID);
    this->effectorID++;

	return;
}

uint64_t LinearSpringMassDamper::effectorID = 1;

/*! @brief Validate the initial particle mass, retaining the supported empty-particle case. */
void LinearSpringMassDamper::validateConfiguration()
{
    if (!std::isfinite(this->massInit) || this->massInit < 0.0) {
        this->bskLogger.bskError("LinearSpringMassDamper: massInit must be finite and non-negative.");
    }
}

/*! @brief Validate configuration without restoring depleted mass or changing integrated states.
 * @param CurrentSimNanos [ns] Current simulation time.
 */
void LinearSpringMassDamper::Reset(uint64_t CurrentSimNanos [[maybe_unused]])
{
    this->validateConfiguration();
}

/*! This is the destructor, nothing to report here */
LinearSpringMassDamper::~LinearSpringMassDamper()
{
    return;
}

/*! Method for spring mass damper particle to access the states that it needs. It needs gravity and the hub states
 *
 * @param[in] states Dynamic parameter manager containing the required states.
 */
void LinearSpringMassDamper::linkInStates(DynParamManager& states)
{
    // - Grab access to gravity
    this->g_N = states.getPropertyReference(this->propName_vehicleGravity);

    // - Grab access to c_B and cPrime_B
    this->c_B = states.getPropertyReference(this->propName_centerOfMassSC);
    this->cPrime_B = states.getPropertyReference(this->propName_centerOfMassPrimeSC);

    return;
}

void LinearSpringMassDamper::linkInPrescribedMotionProperties(DynParamManager& states)
{
    this->prescribedPositionProperty = states.getPropertyReference(this->propName_prescribedPosition);
    this->prescribedVelocityProperty = states.getPropertyReference(this->propName_prescribedVelocity);
    this->prescribedAccelerationProperty = states.getPropertyReference(this->propName_prescribedAcceleration);
    this->prescribedAttitudeProperty = states.getPropertyReference(this->propName_prescribedAttitude);
    this->prescribedAngVelocityProperty = states.getPropertyReference(this->propName_prescribedAngVelocity);
    this->prescribedAngAccelerationProperty = states.getPropertyReference(this->propName_prescribedAngAcceleration);
}

/*! This is the method for the spring mass damper particle to register its states: rho and rhoDot
 *
 * @param[in,out] states Dynamic parameter manager used to register states or properties.
 */
void LinearSpringMassDamper::registerStates(DynParamManager& states)
{
    this->validateConfiguration();
    // - Register rho and rhoDot
	this->rhoState = states.registerState(1, 1, nameOfRhoState);
    Eigen::MatrixXd rhoInitMatrix(1,1);
    rhoInitMatrix(0,0) = this->rhoInit;
    this->rhoState->setState(rhoInitMatrix);
	this->rhoDotState = states.registerState(1, 1, nameOfRhoDotState);
    Eigen::MatrixXd rhoDotInitMatrix(1,1);
    rhoDotInitMatrix(0,0) = this->rhoDotInit;
    this->rhoDotState->setState(rhoDotInitMatrix);

	// - Register mass
	this->massState = states.registerState(1, 1, nameOfMassState);
    Eigen::MatrixXd massInitMatrix(1,1);
    massInitMatrix(0,0) = this->massInit;
    this->massState->setState(massInitMatrix);
    this->hasRegisteredStates = true;

	return;
}

// Create method linkInPrescribedMotionProperties

/*! This is the method for the SMD to add its contributions to the mass props and mass prop rates of the vehicle
 *
 * @param[in] integTime [s] Current integration time.
 */
void LinearSpringMassDamper::updateEffectorMassProps(double integTime [[maybe_unused]])
{
	// - Grab rho from state manager and define r_PcB_B
    this->rho = this->rhoState->stateView()(0, 0);
    this->r_PcB_B = this->rho * this->pHat_B + this->r_PB_B;
    this->massSMD = this->massState->stateView()(0, 0);

    // - Update the effectors mass
	this->effProps.mEff = this->massSMD;
	this->effProps.mEffDot = this->fuelMassDot;
	this->effProps.mEffDotDynamics =
        this->omitMassRateDynamics ? 0.0 : this->fuelMassDot;
	// - Update the position of CoM
	this->effProps.rEff_CB_B = this->r_PcB_B;
	// - Update the inertia about B
	this->rTilde_PcB_B = eigenTilde(this->r_PcB_B);
	this->effProps.IEffPntB_B = this->massSMD * this->rTilde_PcB_B * this->rTilde_PcB_B.transpose();

	// - Grab rhoDot from the stateManager and define rPrime_PcB_B
    this->rhoDot = this->rhoDotState->stateView()(0, 0);
    this->rPrime_PcB_B = this->rhoDot * this->pHat_B;
	this->effProps.rEffPrime_CB_B = this->rPrime_PcB_B;
    this->effProps.rEffPrime_CB_BDynamics = this->rPrime_PcB_B;

	// - Update the body time derivative of inertia about B
	this->rPrimeTilde_PcB_B = eigenTilde(this->rPrime_PcB_B);
	const Eigen::Matrix3d inertiaRateFromMotion =
        -this->massSMD*(this->rPrimeTilde_PcB_B*this->rTilde_PcB_B
                       + this->rTilde_PcB_B*this->rPrimeTilde_PcB_B);
    this->effProps.IEffPrimePntB_B =
        -this->fuelMassDot*this->rTilde_PcB_B*this->rTilde_PcB_B
        + inertiaRateFromMotion;
    this->effProps.IEffPrimePntB_BDynamics = inertiaRateFromMotion;
    this->effProps.hasMassPropertyRateDynamics =
        this->fuelMassDot != 0.0;

    return;
}

/*! This method is used to pass mass properties information to the fuel tank
 *
 * @param[in] integTime [s] Current integration time.
 */
void LinearSpringMassDamper::retrieveMassValue(double integTime [[maybe_unused]])
{
    if (this->massState == nullptr) {
        this->bskLogger.bskLog(
            BSK_ERROR,
            "LinearSpringMassDamper was attached to a FuelTank but never added to Spacecraft. Every particle "
            "passed to pushFuelSloshParticle must also be passed to addStateEffector.");
    }
    // Read the current RK-stage state before the tank allocates propellant flow.
    this->fuelMass = this->massState->stateView()(0, 0);
    if (this->fuelMass < 0.0) {
        this->fuelMass = 0.0;  // [kg]
        Eigen::MatrixXd massMatrix(1, 1);
        massMatrix(0, 0) = this->fuelMass;
        this->massState->setState(massMatrix);
    }

    return;
}

/*! This method is for the SMD to add its contributions to the back-sub method
 *
 * @param[in] integTime [s] Current integration time.
 * @param[in,out] backSubContr Backsubstitution contributions.
 * @param[in] sigma_BN Hub attitude relative to the inertial frame.
 * @param[in] omega_BN_B [rad/s] Hub angular velocity expressed in body-frame components.
 * @param[in] g_N [m/s^2] Gravitational acceleration expressed in inertial-frame components.
 */
void LinearSpringMassDamper::updateContributions(double integTime [[maybe_unused]], BackSubMatrices & backSubContr, Eigen::MRPd sigma_BN, Eigen::Vector3d omega_BN_B, Eigen::Vector3d g_N [[maybe_unused]])
{
    if (this->massSMD <= 0.0) {
        this->aRho.setZero();
        this->bRho.setZero();
        this->cRho = 0.0;  // [m/s^2]
        backSubContr.matrixA.setZero();
        backSubContr.matrixB.setZero();
        backSubContr.matrixC.setZero();
        backSubContr.matrixD.setZero();
        backSubContr.vecTrans.setZero();
        backSubContr.vecRot.setZero();
        return;
    }

    // - Find dcm_BN
    Eigen::MRPd sigmaLocal_BN;
    Eigen::Matrix3d dcm_BN;
    Eigen::Matrix3d dcm_NB;
    sigmaLocal_BN = sigma_BN;
    dcm_NB = sigmaLocal_BN.toRotationMatrix();
    dcm_BN = dcm_NB.transpose();

    // - Map gravity to body frame
    Eigen::Vector3d gLocal_N;
    Eigen::Vector3d g_B;
    gLocal_N = *this->g_N;
    g_B = dcm_BN*gLocal_N;

	// - Define aRho
    this->aRho = -this->pHat_B;

    // - Define bRho
    this->bRho = this->pHat_B.cross(this->r_PcB_B);

    // - Define cRho
    Eigen::Vector3d omega_BN_B_local = omega_BN_B;
	cRho = 1.0/(this->massSMD)*(this->pHat_B.dot(this->massSMD * g_B) - this->k*this->rho - this->c*this->rhoDot
		         - 2 * this->massSMD*this->pHat_B.dot(omega_BN_B_local.cross(this->rPrime_PcB_B))
		                   - this->massSMD*this->pHat_B.dot(omega_BN_B_local.cross(omega_BN_B_local.cross(this->r_PcB_B))));

	// - Compute matrix/vector contributions
	backSubContr.matrixA = this->massSMD*this->pHat_B*this->aRho.transpose();
    backSubContr.matrixB = this->massSMD*this->pHat_B*this->bRho.transpose();
    backSubContr.matrixC = -this->massSMD*this->bRho*this->aRho.transpose();
	backSubContr.matrixD = -this->massSMD*this->bRho*this->bRho.transpose();
	backSubContr.vecTrans = -this->massSMD*this->cRho*this->pHat_B;
	backSubContr.vecRot = -this->massSMD*omega_BN_B_local.cross(this->r_PcB_B.cross(this->rPrime_PcB_B)) +
	                                                             this->massSMD*this->cRho*this->bRho;
    return;
}

/*! This method is used to define the derivatives of the SMD. One is the trivial kinematic derivative and the other is
 derived using the back-sub method
 *
 * @param[in] integTime [s] Current integration time.
 * @param[in] rDDot_BN_N [m/s^2] Hub translational acceleration expressed in inertial-frame components.
 * @param[in] omegaDot_BN_B [rad/s^2] Hub angular acceleration expressed in body-frame components.
 * @param[in] sigma_BN Hub attitude relative to the inertial frame.
 */
void LinearSpringMassDamper::computeDerivatives(double integTime [[maybe_unused]], Eigen::Vector3d rDDot_BN_N, Eigen::Vector3d omegaDot_BN_B, Eigen::MRPd sigma_BN)
{

	// - Find DCM
	Eigen::MRPd sigmaLocal_BN;
	Eigen::Matrix3d dcm_BN;
	sigmaLocal_BN = sigma_BN;
	dcm_BN = (sigmaLocal_BN.toRotationMatrix()).transpose();

	// - Set the derivative of rho to rhoDot
    this->rhoState->setDerivative(this->rhoDotState->stateView());

    // - Compute rhoDDot
	Eigen::MatrixXd conv(1,1);
    Eigen::Vector3d omegaDot_BN_B_local = omegaDot_BN_B;
    Eigen::Vector3d rDDot_BN_N_local = rDDot_BN_N;
	Eigen::Vector3d rDDot_BN_B_local = dcm_BN*rDDot_BN_N_local;
    conv(0, 0) = this->aRho.dot(rDDot_BN_B_local) + this->bRho.dot(omegaDot_BN_B_local) + this->cRho;
	this->rhoDotState->setDerivative(conv);

    // - Set the massDot already computed from fuelTank to the stateDerivative of mass
    conv(0,0) = this->fuelMassDot;
    this->massState->setDerivative(conv);

    return;
}

void LinearSpringMassDamper::addPrescribedMotionCouplingContributions(BackSubMatrices& backSubContr)
{
    // Access prescribed motion properties
    Eigen::Vector3d r_PB_B = (Eigen::Vector3d)*this->prescribedPositionProperty;
    Eigen::Vector3d rPrime_PB_B = (Eigen::Vector3d)*this->prescribedVelocityProperty;
    Eigen::Vector3d rPrimePrime_PB_B = (Eigen::Vector3d)*this->prescribedAccelerationProperty;
    Eigen::MRPd sigma_PB(this->prescribedAttitudeProperty->data());
    Eigen::Vector3d omega_PB_P = (Eigen::Vector3d)*this->prescribedAngVelocityProperty;
    Eigen::Vector3d omegaPrime_PB_P = (Eigen::Vector3d)*this->prescribedAngAccelerationProperty;
    Eigen::Matrix3d dcm_PB = sigma_PB.toRotationMatrix().transpose();

    // Collect hub states
    Eigen::Vector3d omega_BN_B = this->hubOmega->stateView();
    Eigen::Vector3d omega_BN_P = dcm_PB * omega_BN_B;

    // Prescribed motion coupling contributions
    Eigen::Vector3d tHat_P = this->pHat_B;
    Eigen::Vector3d r_PB_P = dcm_PB * r_PB_B;
    Eigen::Matrix3d rTilde_PB_P = eigenTilde(r_PB_P);
    backSubContr.matrixB += - this->massSMD * tHat_P * this->aRho.transpose() * rTilde_PB_P;

    Eigen::Matrix3d omegaTilde_PB_P = eigenTilde(omega_PB_P);
    Eigen::Vector3d rPPrime_TB_B = this->rPrime_PcB_B;
    Eigen::Matrix3d omegaPrimeTilde_PB_P = eigenTilde(omegaPrime_PB_P);
    Eigen::Vector3d r_TB_P = this->r_PcB_B;
    Eigen::Vector3d rPrimePrime_PB_P = dcm_PB * rPrimePrime_PB_B;
    Eigen::Matrix3d omegaTilde_BN_P = eigenTilde(omega_BN_P);
    Eigen::Vector3d rPrime_PB_P = dcm_PB * rPrime_PB_B;
    Eigen::Vector3d term1 = 2.0 * omegaTilde_PB_P * rPPrime_TB_B
                            + omegaPrimeTilde_PB_P * r_TB_P
                            + omegaTilde_PB_P * omegaTilde_PB_P * r_TB_P
                            + rPrimePrime_PB_P;
    Eigen::Vector3d term2 = rPrimePrime_PB_P + 2.0 * omegaTilde_BN_P * rPrime_PB_P
                            + omegaTilde_BN_P * omegaTilde_BN_P * r_PB_P;
    Eigen::Vector3d term3 = omegaPrime_PB_P + omegaTilde_BN_P * omega_PB_P;
    backSubContr.vecTrans += - this->massSMD * term1
                             - this->massSMD * this->aRho.transpose() * term2 * tHat_P
                             - this->massSMD * this->bRho.transpose() * term3 * tHat_P;

    // Prescribed motion rotation coupling contributions
    backSubContr.matrixC += this->massSMD * rTilde_PB_P * tHat_P * this->aRho.transpose();

    Eigen::Vector3d r_FcB_P = r_TB_P + r_PB_P;
    Eigen::Matrix3d rTilde_FcB_P = eigenTilde(r_FcB_P);
    backSubContr.matrixD += + this->massSMD * rTilde_PB_P * tHat_P * this->bRho.transpose()
                            - this->massSMD * rTilde_FcB_P * tHat_P * this->aRho.transpose() * rTilde_PB_P;

    Eigen::Matrix3d rTilde_FcP_P = eigenTilde(r_TB_P);

    Eigen::Vector3d vecRotTerm2 = - this->massSMD * rTilde_FcB_P * term1;
    Eigen::Vector3d vecRotTerm3 = - this->massSMD * (omegaTilde_BN_P * rTilde_PB_P - omegaTilde_PB_P * rTilde_FcP_P) * rPPrime_TB_B
    - this->massSMD * omegaTilde_BN_P * rTilde_FcB_P * (omegaTilde_PB_P * r_TB_P + rPrime_PB_P);
    Eigen::Vector3d vecRotTerm4 = - this->massSMD * this->cRho * rTilde_PB_P * tHat_P;
    Eigen::Vector3d vecRotTerm5 = - this->massSMD * rTilde_FcB_P * tHat_P * (this->aRho.transpose() * term2)
            - this->massSMD * rTilde_FcB_P * tHat_P * (this->bRho.transpose() * term3);
    backSubContr.vecRot +=
            + vecRotTerm2
            + vecRotTerm3
            + vecRotTerm4
            + vecRotTerm5;

}

/*! This method is for the SMD to add its contributions to energy and momentum
 *
 * @param[in] integTime [s] Current integration time.
 * @param[in,out] rotAngMomPntCContr_B [kg*m^2/s] Rotational angular momentum contribution.
 * @param[in,out] rotEnergyContr [J] Rotational energy contribution.
 * @param[in] omega_BN_B [rad/s] Hub angular velocity expressed in body-frame components.
 */
void LinearSpringMassDamper::updateEnergyMomContributions(double integTime [[maybe_unused]], Eigen::Vector3d & rotAngMomPntCContr_B,
                                                          double & rotEnergyContr, Eigen::Vector3d omega_BN_B)
{
    //  - Get variables needed for energy momentum calcs
    Eigen::Vector3d omegaLocal_BN_B;
    omegaLocal_BN_B = omega_BN_B;
    Eigen::Vector3d rDotPcB_B;

    // - Find rotational angular momentum contribution from hub
    rDotPcB_B = this->rPrime_PcB_B + omegaLocal_BN_B.cross(this->r_PcB_B);
    rotAngMomPntCContr_B = this->massSMD*this->r_PcB_B.cross(rDotPcB_B);

    // - Find rotational energy contribution from the hub
    rotEnergyContr = 1.0/2.0*this->massSMD*rDotPcB_B.dot(rDotPcB_B) + 1.0/2.0*this->k*this->rho*this->rho;

    return;
}

/*! @brief Calculate the force and torque exerted on the attached body.
 *
 * @param[in] integTime [s] Current integration time.
 * @param[in] omega_BN_B [rad/s] Hub angular velocity expressed in body-frame components.
 */
void LinearSpringMassDamper::calcForceTorqueOnBody(double integTime [[maybe_unused]], Eigen::Vector3d omega_BN_B)
{
    // - Get the current omega state
    Eigen::Vector3d omegaLocal_BN_B;
    omegaLocal_BN_B = omega_BN_B;

    // - Get rhoDDot from last integrator call
    double rhoDDotLocal;
    rhoDDotLocal = rhoDotState->derivativeView()(0, 0);

    // - Calculate force that the FSP is applying to the spacecraft
    this->forceOnBody_B = -(this->massSMD*this->pHat_B*rhoDDotLocal + 2*this->massSMD
                            *this->rhoDot*omegaLocal_BN_B.cross(this->pHat_B));

    // - Calculate torque that the FSP is applying about point B
    this->torqueOnBodyPntB_B = -(this->massSMD*this->r_PcB_B.cross(this->pHat_B)*rhoDDotLocal
                                 + this->massSMD*omegaLocal_BN_B.cross(this->r_PcB_B.cross(this->rPrime_PcB_B))
                                 - this->massSMD*(this->rPrime_PcB_B.cross(this->r_PcB_B.cross(omegaLocal_BN_B))
                                                   + this->r_PcB_B.cross(this->rPrime_PcB_B.cross(omegaLocal_BN_B))));

    // - Define values needed to get the torque about point C
    Eigen::Vector3d cLocal_B = *this->c_B;
    Eigen::Vector3d cPrimeLocal_B = *this->cPrime_B;
    Eigen::Vector3d r_PcC_B = this->r_PcB_B - cLocal_B;
    Eigen::Vector3d rPrime_PcC_B = this->rPrime_PcB_B - cPrimeLocal_B;

    // - Calculate the torque about point C
    this->torqueOnBodyPntC_B = -(this->massSMD*r_PcC_B.cross(this->pHat_B)*rhoDDotLocal
                                 + this->massSMD*omegaLocal_BN_B.cross(r_PcC_B.cross(rPrime_PcC_B))
                                 - this->massSMD*(rPrime_PcC_B.cross(r_PcC_B.cross(omegaLocal_BN_B))
                                                   + r_PcC_B.cross(rPrime_PcC_B.cross(omegaLocal_BN_B))));

    return;
}
