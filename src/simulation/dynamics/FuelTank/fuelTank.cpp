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
    this->dcm_TB = Eigen::Matrix3d::Identity();
    this->r_TB_B.setZero();
    this->ITankPntT_B = Eigen::Matrix3d::Identity();
    this->r_TcB_B.setZero();

    this->effectorID++;
    this->nameOfMassState = "fuelTankMass" + std::to_string(this->effectorID);
}

uint64_t FuelTank::effectorID = 1;

FuelTank::~FuelTank() {
    FuelTank::effectorID = 1;
}

/*! optionally set the name of the mass state to be used by the state manager */
void FuelTank::setNameOfMassState(const std::string nameOfMassState) {
    this->nameOfMassState = nameOfMassState;
}

/*! set fuel tank model

 @param model fuel tank model type
 */
void FuelTank::setTankModel(FuelTankModel *model) {
    this->fuelTankModel = model;
}

/*! Attach a fuel slosh particle to the tank */
void FuelTank::pushFuelSloshParticle(FuelSlosh *particle) {
    // Add a fuel slosh particle to the vector of fuel slosh particles
    this->fuelSloshParticles.push_back(particle);
}

/*! Attach a thruster dynamic effector to the tank */
void FuelTank::addThrusterSet(ThrusterDynamicEffector *dynEff) {
    thrDynEffectors.push_back(dynEff);
    dynEff->fuelMass = this->fuelTankModel->propMassInit;
}

/*! Attach a thruster state effector to the tank */
void FuelTank::addThrusterSet(ThrusterStateEffector *stateEff) {
    thrStateEffectors.push_back(stateEff);
}

/*! Link states that the module accesses */
void FuelTank::linkInStates(DynParamManager &statesIn) {
    // Grab access to the hubs omega_BN_N
    this->omegaState = statesIn.getStateObject(this->stateNameOfOmega);
}

/*! Register states. The fuel tank has one state associated with it: mass, and it also has the
 responsibility to call register states for the fuel slosh particles */
void FuelTank::registerStates(DynParamManager &statesIn) {
    // Register the mass state associated with the tank
    Eigen::MatrixXd massMatrix(1, 1);
    this->massState = statesIn.registerState(1, 1, this->nameOfMassState);
    massMatrix(0, 0) = this->fuelTankModel->propMassInit;
    this->massState->setState(massMatrix);
}

/*! Fuel tank add its contributions the mass of the vehicle. */
void FuelTank::updateEffectorMassProps(double integTime) {
    // Add contributions of the mass of the tank
    double massLocal = this->massState->getState()(0, 0);
    this->fuelTankModel->computeTankProps(massLocal);
    this->r_TcB_B = r_TB_B + this->dcm_TB.transpose() * this->fuelTankModel->r_TcT_T;
    this->effProps.mEff = massLocal;
    this->ITankPntT_B = this->dcm_TB.transpose() * fuelTankModel->ITankPntT_T * this->dcm_TB;
    this->effProps.IEffPntB_B = ITankPntT_B + massLocal * (r_TcB_B.dot(r_TcB_B) * Eigen::Matrix3d::Identity()
                                                           - r_TcB_B * r_TcB_B.transpose());
    this->effProps.rEff_CB_B = this->r_TcB_B;

    // This does not incorporate mEffDot into cPrime for high fidelity mass depletion
    this->effProps.rEffPrime_CB_B = Eigen::Vector3d::Zero();

    // Mass depletion (call thrusters attached to this tank to get their mDot, and contributions)
    this->fuelConsumption = 0.0;
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
        // Add fuelSlosh mass to total mass of tank
        totalMass += (*fuelSloshInt)->fuelMass;
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
    this->effProps.mEffDot = -this->fuelConsumption;
}

/*! Fuel tank adds its contributions to the matrices for the back-sub method. */
void FuelTank::updateContributions(double integTime,
                                   BackSubMatrices &backSubContr,
                                   Eigen::Vector3d sigma_BN,
                                   Eigen::Vector3d omega_BN_B,
                                   Eigen::Vector3d g_N) {
    Eigen::Vector3d r_TB_BLocal;
    Eigen::Vector3d rPrime_TB_BLocal;
    Eigen::Vector3d rPPrime_TB_BLocal;
    Eigen::Vector3d omega_BN_BLocal;

    // Zero some matrices
    backSubContr.matrixA = backSubContr.matrixB = backSubContr.matrixC = backSubContr.matrixD = Eigen::Matrix3d::Zero();
    backSubContr.vecTrans = backSubContr.vecRot = Eigen::Vector3d::Zero();

    // Calculate the fuel consumption properties for the tank
    this->tankFuelConsumption = this->fuelConsumption * this->massState->getState()(0, 0) / this->effProps.mEff;
    this->fuelTankModel->computeTankPropDerivs(this->massState->getState()(0, 0), -this->tankFuelConsumption);
    r_TB_BLocal = this->fuelTankModel->r_TcT_T;
    rPrime_TB_BLocal = this->fuelTankModel->rPrime_TcT_T;
    rPPrime_TB_BLocal = this->fuelTankModel->rPPrime_TcT_T;
    omega_BN_BLocal = this->omegaState->getState();
    if (!this->updateOnly) {
        backSubContr.vecRot = -this->massState->getState()(0, 0) * r_TB_BLocal.cross(rPPrime_TB_BLocal)
                              - this->massState->getState()(0, 0) * omega_BN_BLocal.cross(r_TB_BLocal.cross(rPrime_TB_BLocal))
                              - this->massState->getStateDeriv()(0, 0) * r_TB_BLocal.cross(rPrime_TB_BLocal);
        backSubContr.vecRot -= this->fuelTankModel->IPrimeTankPntT_T * omega_BN_BLocal;
    }

}

/*! Fuel tank computes its derivative */
void FuelTank::computeDerivatives(double integTime,
                                  Eigen::Vector3d rDDot_BN_N,
                                  Eigen::Vector3d omegaDot_BN_B,
                                  Eigen::Vector3d sigma_BN) {
    Eigen::MatrixXd conv(1, 1);
    conv(0, 0) = -this->tankFuelConsumption;
    this->massState->setDerivative(conv);
}

/*! Fuel tank contributes to the energy and momentum calculations */
void FuelTank::updateEnergyMomContributions(double integTime,
                                            Eigen::Vector3d &rotAngMomPntCContr_B,
                                            double &rotEnergyContr,
                                            Eigen::Vector3d omega_BN_B) {
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
