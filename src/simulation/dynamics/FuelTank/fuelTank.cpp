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

#include "fuelTank.h"

#include <iostream>


FuelTank::FuelTank() {
    this->effProps.mEff = 0.0;
    this->effProps.IEffPntB_B.setZero();
    this->effProps.rEff_CB_B.setZero();
    this->effProps.rEffPrime_CB_B.setZero();
    this->effProps.IEffPrimePntB_B.setZero();
    this->setDcm_TB(Eigen::Matrix3d::Identity());
    this->setR_TB_B(Eigen::Vector3d::Zero());
    this->ITankPntT_B = Eigen::Matrix3d::Identity();
    this->r_TcB_B.setZero();

    this->effectorID++;
    this->setNameOfMassState("fuelTankMass" + std::to_string(this->effectorID));
}

uint64_t FuelTank::effectorID = 1;

FuelTank::~FuelTank() {
}

/*! optionally set the name of the mass state to be used by the state manager
 *
 * @param[in] nameOfMassState Name assigned to the tank mass state.
 */
void FuelTank::setNameOfMassState(const std::string &nameOfMassState) {
    this->nameOfMassState = nameOfMassState;
}

/*! get the name of the mass state used by the state manager */
std::string FuelTank::getNameOfMassState() const {
    return this->nameOfMassState;
}

/*! set fuel tank orientation relative to the hub frame
 *
 * @param[in] dcm_TB Direction cosine matrix defining the tank orientation relative to the hub.
 */
void FuelTank::setDcm_TB(const Eigen::Matrix3d &dcm_TB) {
    this->dcm_TB = dcm_TB;
}

/*! get fuel tank orientation relative to the hub frame */
Eigen::Matrix3d FuelTank::getDcm_TB() const {
    return this->dcm_TB;
}

/*! set fuel tank location relative to the hub frame
 *
 * @param[in] r_TB_B [m] Tank position relative to the hub, expressed in body-frame components.
 */
void FuelTank::setR_TB_B(const Eigen::Vector3d &r_TB_B) {
    this->r_TB_B = r_TB_B;
}

/*! get fuel tank location relative to the hub frame */
Eigen::Vector3d FuelTank::getR_TB_B() const {
    return this->r_TB_B;
}

/*! set update only mass depletion flag
 *
 * @param[in] updateOnly Flag selecting update-only mass depletion.
 */
void FuelTank::setUpdateOnly(bool updateOnly) {
    this->updateOnly = updateOnly;
}

/*! get update only mass depletion flag */
bool FuelTank::getUpdateOnly() const {
    return this->updateOnly;
}

/*! set fuel leak rate
 *
 * @param[in] fuelLeakRate [kg/s] Fuel leak mass flow rate.
 */
void FuelTank::setFuelLeakRate(double fuelLeakRate) {
    this->fuelLeakRate = fuelLeakRate;
}

/*! get fuel leak rate */
double FuelTank::getFuelLeakRate() const {
    return this->fuelLeakRate;
}

/*! set fuel tank model

 @param model fuel tank model type
 */
void FuelTank::setTankModel(std::shared_ptr<FuelTankModel> model) {
    this->fuelTankModel = model;
}

/*! Attach a fuel slosh particle to the tank
 *
 * @param[in] particle Fuel-slosh particle to attach.
 */
void FuelTank::pushFuelSloshParticle(FuelSlosh *particle) {
    // Add a fuel slosh particle to the vector of fuel slosh particles
    this->fuelSloshParticles.push_back(particle);
}

/*! Attach a thruster dynamic effector to the tank
 *
 * @param[in] dynEff Thruster dynamic effector to attach.
 */
void FuelTank::addThrusterSet(ThrusterDynamicEffector *dynEff) {
    thrDynEffectors.push_back(dynEff);
    dynEff->fuelMass = this->fuelTankModel->propMassInit;
}

/*! Attach a thruster state effector to the tank
 *
 * @param[in] stateEff Thruster state effector to attach.
 */
void FuelTank::addThrusterSet(ThrusterStateEffector *stateEff) {
    thrStateEffectors.push_back(stateEff);
}

/*! Link states that the module accesses
 *
 * @param[in] states Dynamic parameter manager containing the required states.
 */
void FuelTank::linkInStates(DynParamManager &states) {
    // Grab access to the hubs omega_BN_N
    this->omegaState = states.getStateObject(this->stateNameOfOmega);
    for (auto* fuelSloshParticle: this->fuelSloshParticles) {
        // runs after the whole registerStates pass, so the flag is scoped to one pass
        fuelSloshParticle->hasRegisteredStates = false;
    }
}

/*! Register the tank mass state before any attached fuel-slosh states.
 *
 * @param[in,out] states Dynamic parameter manager used to register the tank mass state.
 */
void FuelTank::registerStates(DynParamManager &states) {
    for (auto* fuelSloshParticle: this->fuelSloshParticles) {
        if (fuelSloshParticle->hasRegisteredStates) {
            this->bskLogger.bskLog(
                BSK_ERROR,
                "FuelTank must be added to Spacecraft before its attached fuel-slosh effectors.");
        }
    }
    // Register the mass state associated with the tank
    Eigen::MatrixXd massMatrix(1, 1);
    this->massState = states.registerState(1, 1, this->getNameOfMassState());
    massMatrix(0, 0) = this->fuelTankModel->propMassInit;
    this->massState->setState(massMatrix);
    this->emptyTankWarningPrinted = false;
}

/*! Update derivatives of the retained tank mass properties.
 *
 * @param[in] mass [kg] Mass value.
 * @param[in] massRate [kg/s] Mass rate.
 */
void FuelTank::updateRetainedMassPropertyDerivatives(double mass,
                                                     double massRate)
{
    this->fuelTankModel->computeTankPropDerivs(mass, massRate);
    const Eigen::Matrix3d dcm_TBLocal = this->getDcm_TB();
    this->effProps.rEffPrime_CB_B =
        dcm_TBLocal.transpose()*this->fuelTankModel->rPrime_TcT_T;
    const Eigen::Vector3d rPrime_TcB_B = this->effProps.rEffPrime_CB_B;
    this->effProps.IEffPrimePntB_B =
        dcm_TBLocal.transpose()
            * this->fuelTankModel->IPrimeTankPntT_T
            * dcm_TBLocal
        + massRate
            *(this->r_TcB_B.dot(this->r_TcB_B)
                  * Eigen::Matrix3d::Identity()
              - this->r_TcB_B*this->r_TcB_B.transpose())
        + mass
            *(2.0*this->r_TcB_B.dot(rPrime_TcB_B)
                  * Eigen::Matrix3d::Identity()
              - this->r_TcB_B*rPrime_TcB_B.transpose()
              - rPrime_TcB_B*this->r_TcB_B.transpose());
    this->effProps.rEffPrime_CB_BDynamics.setZero();
    this->effProps.IEffPrimePntB_BDynamics.setZero();
}

/*! Fuel tank adds its contribution to the mass of the vehicle.
 *
 * @param[in] integTime [s] Current integration time.
 */
void FuelTank::updateEffectorMassProps(double integTime) {
    // Add contributions of the mass of the tank
    double massLocal = this->massState->getState()(0, 0);
    if (massLocal < 0.0) {
        // Clamp integration overshoot at empty before tank models compute mass properties.
        massLocal = 0.0;  // [kg]
        Eigen::MatrixXd massMatrix(1, 1);
        massMatrix(0, 0) = massLocal;
        this->massState->setState(massMatrix);
    }
    this->fuelTankModel->computeTankProps(massLocal);
    const Eigen::Matrix3d dcm_TBLocal = this->getDcm_TB();
    this->r_TcB_B = this->getR_TB_B() + dcm_TBLocal.transpose() * this->fuelTankModel->r_TcT_T;
    this->effProps.mEff = massLocal;
    this->ITankPntT_B = dcm_TBLocal.transpose() * fuelTankModel->ITankPntT_T * dcm_TBLocal;
    this->effProps.IEffPntB_B = ITankPntT_B + massLocal * (r_TcB_B.dot(r_TcB_B) * Eigen::Matrix3d::Identity()
                                                           - r_TcB_B * r_TcB_B.transpose());
    this->effProps.rEff_CB_B = this->r_TcB_B;

    // Mass depletion (call thrusters attached to this tank to get their mDot, and contributions)
    this->fuelConsumption = 0.0;
    double fuelLeakRateLocal = this->getFuelLeakRate();
    if (this->fuelLeakRateInMsg.isLinked()) {
        MassFlowRateMsgPayload fuelLeakRateInMsgBuffer = this->fuelLeakRateInMsg();
        fuelLeakRateLocal = fuelLeakRateInMsgBuffer.massFlowRate;
    }
    this->fuelConsumption += fuelLeakRateLocal;
    for (auto &dynEffector: this->thrDynEffectors) {
        dynEffector->computeStateContribution(integTime);
        this->fuelConsumption += dynEffector->stateDerivContribution(0);
    }

    for (auto &stateEffector: this->thrStateEffectors) {
        stateEffector->updateEffectorMassProps(integTime);
        this->fuelConsumption += stateEffector->stateDerivContribution(0);
    }

    // Mass depletion (finding total mass in tank)
    double totalMass = massLocal;
    for (auto fuelSloshInt = this->fuelSloshParticles.begin();
         fuelSloshInt < this->fuelSloshParticles.end();
         fuelSloshInt++) {
        // Retrieve current mass value of fuelSlosh particle
        (*fuelSloshInt)->retrieveMassValue(integTime);
        (*fuelSloshInt)->omitMassRateDynamics = this->getUpdateOnly();
        // Add fuelSlosh mass to total mass of tank
        totalMass += (*fuelSloshInt)->fuelMass;
    }
    if (totalMass <= 0.0) {
        if (this->fuelConsumption > 0.0 && !this->emptyTankWarningPrinted) {
            this->bskLogger.bskLog(BSK_WARNING,
                                   "FuelTank: available propellant has reached zero. Fuel mass depletion is stopped.");
            this->emptyTankWarningPrinted = true;
        }
        this->fuelConsumption = 0.0;  // [kg/s]
        for (auto fuelSloshInt = this->fuelSloshParticles.begin();
             fuelSloshInt < this->fuelSloshParticles.end();
             fuelSloshInt++) {
            (*fuelSloshInt)->massToTotalTankMassRatio = 0.0;
            (*fuelSloshInt)->fuelMassDot = 0.0;  // [kg/s]
        }
        for (auto &dynEffector: this->thrDynEffectors) {
            dynEffector->fuelMass = 0.0;  // [kg]
        }
        this->tankFuelConsumption = 0.0;  // [kg/s]
        this->effProps.mEffDot = 0.0;  // [kg/s]
        this->effProps.mEffDotDynamics = 0.0;  // [kg/s]
        this->updateRetainedMassPropertyDerivatives(massLocal, 0.0);
        this->effProps.hasMassPropertyRateDynamics = false;
        return;
    }
    // Set mass depletion rate of fuelSloshParticles
    for (auto fuelSloshInt = this->fuelSloshParticles.begin();
         fuelSloshInt < this->fuelSloshParticles.end();
         fuelSloshInt++) {
        // Find fuelSlosh particle mass to fuel tank mass ratio
        (*fuelSloshInt)->massToTotalTankMassRatio = (*fuelSloshInt)->fuelMass / totalMass;
        // Scale total fuelConsumption by mass ratio to find fuelSloshParticle mass depletion rate
        (*fuelSloshInt)->fuelMassDot = (*fuelSloshInt)->massToTotalTankMassRatio * (-this->fuelConsumption);
    }

    // Set total fuel mass parameter for thruster dynamic effectors experiencing blow down effects
    for (auto &dynEffector: this->thrDynEffectors) {
        dynEffector->fuelMass = totalMass;
    }
    // Set fuel consumption rate of fuelTank (not negative because the negative sign is in the computeDerivatives call
    this->tankFuelConsumption = massLocal / totalMass * (this->fuelConsumption);
    this->effProps.mEffDot = -this->tankFuelConsumption;
    this->effProps.mEffDotDynamics =
        this->getUpdateOnly() ? 0.0 : this->effProps.mEffDot;
    this->updateRetainedMassPropertyDerivatives(
        massLocal,
        this->effProps.mEffDot
    );
    this->effProps.hasMassPropertyRateDynamics =
        this->effProps.mEffDot != 0.0;
}

/*! Fuel tank adds its contributions to the matrices for the back-sub method.
 *
 * @param[in] integTime [s] Current integration time.
 * @param[in,out] backSubContr Backsubstitution contributions.
 * @param[in] sigma_BN Hub attitude relative to the inertial frame.
 * @param[in] omega_BN_B [rad/s] Hub angular velocity expressed in body-frame components.
 * @param[in] g_N [m/s^2] Gravitational acceleration expressed in inertial-frame components.
 */
void FuelTank::updateContributions(double integTime [[maybe_unused]],
                                   BackSubMatrices &backSubContr,
                                   Eigen::MRPd sigma_BN [[maybe_unused]],
                                   Eigen::Vector3d omega_BN_B [[maybe_unused]],
                                   Eigen::Vector3d g_N [[maybe_unused]]) {
    Eigen::Vector3d omega_BN_BLocal;

    // Zero some matrices
    backSubContr.matrixA = backSubContr.matrixB = backSubContr.matrixC = backSubContr.matrixD = Eigen::Matrix3d::Zero();
    backSubContr.vecTrans = backSubContr.vecRot = Eigen::Vector3d::Zero();

    double massLocal = this->massState->getState()(0, 0);
    if (massLocal < 0.0) {
        massLocal = 0.0;  // [kg]
    }
    const double mDotTank = -this->tankFuelConsumption; // [kg/s] tank mass rate this substep
    omega_BN_BLocal = this->omegaState->getState();
    if (!this->getUpdateOnly()) {
        const Eigen::Matrix3d dcm_TBLocal = this->getDcm_TB();
        Eigen::Vector3d rPrime_TcB_B = dcm_TBLocal.transpose() * this->fuelTankModel->rPrime_TcT_T;
        Eigen::Vector3d rPPrime_TcB_B = dcm_TBLocal.transpose() * this->fuelTankModel->rPPrime_TcT_T;
        // Inertia derivative about point B: central term plus d/dt[m (r.r I - r r^T)], r = r_TcB_B
        Eigen::Matrix3d IPrimeTankPntB_BLocal =
          dcm_TBLocal.transpose() * this->fuelTankModel->IPrimeTankPntT_T * dcm_TBLocal +
          mDotTank * (this->r_TcB_B.dot(this->r_TcB_B) * Eigen::Matrix3d::Identity() -
                      this->r_TcB_B * this->r_TcB_B.transpose()) +
          massLocal * (2.0 * this->r_TcB_B.dot(rPrime_TcB_B) * Eigen::Matrix3d::Identity() -
                       this->r_TcB_B * rPrime_TcB_B.transpose() - rPrime_TcB_B * this->r_TcB_B.transpose());
        backSubContr.vecRot = -massLocal * this->r_TcB_B.cross(rPPrime_TcB_B) -
                              massLocal * omega_BN_BLocal.cross(this->r_TcB_B.cross(rPrime_TcB_B)) -
                              mDotTank * this->r_TcB_B.cross(rPrime_TcB_B);
        backSubContr.vecRot -= IPrimeTankPntB_BLocal * omega_BN_BLocal;
    }
}

/*! Fuel tank computes its derivative
 *
 * @param[in] integTime [s] Current integration time.
 * @param[in] rDDot_BN_N [m/s^2] Hub translational acceleration expressed in inertial-frame components.
 * @param[in] omegaDot_BN_B [rad/s^2] Hub angular acceleration expressed in body-frame components.
 * @param[in] sigma_BN Hub attitude relative to the inertial frame.
 */
void FuelTank::computeDerivatives(double integTime [[maybe_unused]],
                                  Eigen::Vector3d rDDot_BN_N [[maybe_unused]],
                                  Eigen::Vector3d omegaDot_BN_B [[maybe_unused]],
                                  Eigen::MRPd sigma_BN [[maybe_unused]]) {
    Eigen::MatrixXd conv(1, 1);
    double massLocal = this->massState->getState()(0, 0);
    double tankFuelConsumptionLocal = this->tankFuelConsumption;
    if (massLocal <= 0.0 && tankFuelConsumptionLocal > 0.0) {
        tankFuelConsumptionLocal = 0.0;  // [kg/s]
    }
    conv(0, 0) = -tankFuelConsumptionLocal;
    this->massState->setDerivative(conv);
}

/*! Fuel tank contributes to the energy and momentum calculations
 *
 * @param[in] integTime [s] Current integration time.
 * @param[in,out] rotAngMomPntCContr_B [kg*m^2/s] Rotational angular momentum contribution.
 * @param[in,out] rotEnergyContr [J] Rotational energy contribution.
 * @param[in] omega_BN_B [rad/s] Hub angular velocity expressed in body-frame components.
 */
void FuelTank::updateEnergyMomContributions(double integTime [[maybe_unused]],
                                            Eigen::Vector3d &rotAngMomPntCContr_B,
                                            double &rotEnergyContr,
                                            Eigen::Vector3d omega_BN_B [[maybe_unused]]) {
    // Get variables needed for energy momentum calcs
    Eigen::Vector3d omegaLocal_BN_B;
    omegaLocal_BN_B = this->omegaState->getState();
    Eigen::Vector3d rDot_TcB_B;

    // Find rotational angular momentum contribution from hub
    double massLocal = this->massState->getState()(0, 0);
    rDot_TcB_B = omegaLocal_BN_B.cross(this->r_TcB_B);
    rotAngMomPntCContr_B += this->ITankPntT_B * omegaLocal_BN_B + massLocal * this->r_TcB_B.cross(rDot_TcB_B);

    // Find rotational energy contribution from the hub
    rotEnergyContr += 1.0 / 2.0 * omegaLocal_BN_B.dot(this->ITankPntT_B * omegaLocal_BN_B) + 1.0 / 2.0 * massLocal *
                                                                                       rDot_TcB_B.dot(rDot_TcB_B);
}

/*! Compute fuel tank mass properties and outputs them as a message.

 @param currentClock The current simulation time (used for time stamping)
 */
void FuelTank::writeOutputMessages(uint64_t currentClock) {
    this->fuelTankMassPropMsg = this->fuelTankOutMsg.zeroMsgPayload;
    this->fuelTankMassPropMsg.fuelMass = this->effProps.mEff;
    this->fuelTankMassPropMsg.fuelMassDot = this->effProps.mEffDot;
    this->fuelTankMassPropMsg.maxFuelMass = this->fuelTankModel->maxFuelMass;
    this->fuelTankOutMsg.write(&this->fuelTankMassPropMsg, this->moduleID, currentClock);
}

/*! Fuel tank writes out its messages

 @param currentSimNanos The current simulation time in nanoseconds
 */
void FuelTank::UpdateState(uint64_t currentSimNanos) {
    this->writeOutputMessages(currentSimNanos);
}
