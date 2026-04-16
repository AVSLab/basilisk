/*
 ISC License

 Copyright (c) 2025, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#include "spacecraftCharging.h"
#include "../_GeneralModuleFiles/svIntegratorRK4.h"
#include "architecture/utilities/avsEigenSupport.h"
#include "architecture/utilities/macroDefinitions.h"
#include <iostream>
#include <cmath>

/*! Class constructor. */
SpacecraftCharging::SpacecraftCharging() {
    this->nameOfServicerPotentialState = "servicerPotential";
    this->nameOfTargetPotentialState = "targetPotential";

    // Set integrator as RK4
    this->integrator = new svIntegratorRK4(this);
}

/*! Reset method. */
void SpacecraftCharging::Reset(uint64_t CurrentSimNanos) {
    this->initializeDynamics();
    this->writeOutputStateMessages(CurrentSimNanos);
    this->timeBefore = CurrentSimNanos * NANO2SEC;
    this->timeBeforeNanos = CurrentSimNanos;
}

void SpacecraftCharging::initializeDynamics() {
    this->registerStates(this->dynManager);

    // Call equations of motion at time zero
    this->equationsOfMotion(0.0, 1.0);
}

void SpacecraftCharging::registerStates(DynParamManager& states) {
    this->servicerPotentialState = states.registerState(1, 1, this->nameOfServicerPotentialState);
    Eigen::MatrixXd servicerPotentialInitMatrix(1, 1);
    servicerPotentialInitMatrix(0, 0) = this->servicerPotentialInit;
    this->servicerPotentialState->setState(servicerPotentialInitMatrix);

    this->targetPotentialState = states.registerState(1, 1, this->nameOfTargetPotentialState);
    Eigen::MatrixXd targetPotentialInitMatrix(1, 1);
    targetPotentialInitMatrix(0, 0) = this->targetPotentialInit;
    this->targetPotentialState->setState(targetPotentialInitMatrix);
}

/*! Module update method. */
void SpacecraftCharging::UpdateState(uint64_t CurrentSimNanos) {
    // Read the servicer and target spacecraft sunlit facet area input messages if they are linked and written
    if (this->servicerSunlitAreaInMsg.isLinked() && this->servicerSunlitAreaInMsg.isWritten()) {
        ProjectedAreaMsgPayload servicerSunlitFacetAreaInMsgBuffer = this->servicerSunlitAreaInMsg();
        this->servicerSunlitArea = servicerSunlitFacetAreaInMsgBuffer.area;
    } else {
        this->bskLogger.bskLog(BSK_ERROR, "SpacecraftCharging.servicerSunlitAreaInMsg was not linked or written.");
        return;
    }
    if (this->targetSunlitAreaInMsg.isLinked() && this->targetSunlitAreaInMsg.isWritten()) {
        ProjectedAreaMsgPayload targetSunlitFacetAreaInMsgBuffer = this->targetSunlitAreaInMsg();
        this->targetSunlitArea = targetSunlitFacetAreaInMsgBuffer.area;
    } else {
        this->bskLogger.bskLog(BSK_ERROR, "SpacecraftCharging.targetSunlitAreaInMsg was not linked or written.");
        return;
    }

    // Read the electron beam input message if it is linked and written
    this->electronBeamEnergy = 0.0;
    this->electronBeamCurrent = 0.0;
    if (this->electronBeamInMsg.isLinked() && this->electronBeamInMsg.isWritten()) {
        ElectronBeamMsgPayload electronBeamInMsgBuffer = this->electronBeamInMsg();
        this->electronBeamEnergy = electronBeamInMsgBuffer.energyEB;
        this->electronBeamCurrent = electronBeamInMsgBuffer.currentEB;
        this->alphaEB = electronBeamInMsgBuffer.alphaEB;
    }

    // Integrate the state forward in time
    this->integrateState(CurrentSimNanos);

    // Write the module output messages
    this->writeOutputStateMessages(CurrentSimNanos);
}

/*! Method to write module output messages. */
void SpacecraftCharging::writeOutputStateMessages(uint64_t clockTime) {
    this->servicerPotential = this->servicerPotentialState->getState()(0, 0);
    this->targetPotential = this->targetPotentialState->getState()(0, 0);

    // Write out the servicer output messages
    VoltMsgPayload servicerVoltageMsgBuffer;
    servicerVoltageMsgBuffer = this->servicerPotentialOutMsg.zeroMsgPayload;
    servicerVoltageMsgBuffer.voltage = this->servicerPotential;
    this->servicerPotentialOutMsg.write(&servicerVoltageMsgBuffer, this->moduleID, clockTime);

    CurrentMsgPayload servicerEBCurrentMsgBuffer;
    servicerEBCurrentMsgBuffer = this->servicerEBCurrentOutMsg.zeroMsgPayload;
    servicerEBCurrentMsgBuffer.current = this->servicerEBCurrent;
    this->servicerEBCurrentOutMsg.write(&servicerEBCurrentMsgBuffer, this->moduleID, clockTime);

    CurrentMsgPayload servicerPhotoelectricCurrentMsgBuffer;
    servicerPhotoelectricCurrentMsgBuffer = this->servicerPhotoelectricCurrentOutMsg.zeroMsgPayload;
    servicerPhotoelectricCurrentMsgBuffer.current = this->servicerPhotoelectricCurrent;
    this->servicerPhotoelectricCurrentOutMsg.write(&servicerPhotoelectricCurrentMsgBuffer, this->moduleID, clockTime);

    // Write out the target output messages
    VoltMsgPayload targetVoltageMsgBuffer;
    targetVoltageMsgBuffer = this->targetPotentialOutMsg.zeroMsgPayload;
    targetVoltageMsgBuffer.voltage = this->targetPotential;
    this->targetPotentialOutMsg.write(&targetVoltageMsgBuffer, this->moduleID, clockTime);

    CurrentMsgPayload targetEBCurrentMsgBuffer;
    targetEBCurrentMsgBuffer = this->targetEBCurrentOutMsg.zeroMsgPayload;
    targetEBCurrentMsgBuffer.current = this->targetEBCurrent;
    this->targetEBCurrentOutMsg.write(&targetEBCurrentMsgBuffer, this->moduleID, clockTime);

    CurrentMsgPayload targetPhotoelectricCurrentMsgBuffer;
    targetPhotoelectricCurrentMsgBuffer = this->targetPhotoelectricCurrentOutMsg.zeroMsgPayload;
    targetPhotoelectricCurrentMsgBuffer.current = this->targetPhotoelectricCurrent;
    this->targetPhotoelectricCurrentOutMsg.write(&targetPhotoelectricCurrentMsgBuffer, this->moduleID, clockTime);
}

/*! Method for the charging equations of motion */
void SpacecraftCharging::equationsOfMotion(double integTimeSeconds, double timeStep) {
    this->servicerPotential = this->servicerPotentialState->getState()(0, 0);
    this->targetPotential = this->targetPotentialState->getState()(0, 0);

    // Compute the electron beam currents
    this->computeElectronBeamCurrent();

    // Compute the photoelectric currents
    this->computePhotoelectricCurrent();

    // Set the servicer potential derivative
    Eigen::MatrixXd servicerPotentialRate(1, 1);
    servicerPotentialRate(0, 0) = (this->servicerPhotoelectricCurrent + this->servicerEBCurrent) / this->servicerCapacitance;
    this->servicerPotentialState->setDerivative(servicerPotentialRate);

    // Set the target potential derivative
    Eigen::MatrixXd targetPotentialRate(1, 1);
    targetPotentialRate(0, 0) = (this->targetPhotoelectricCurrent + this->targetEBCurrent) / this->targetCapacitance;
    this->targetPotentialState->setDerivative(targetPotentialRate);
}

/*! Method to compute electron beam currents */
void SpacecraftCharging::computeElectronBeamCurrent() {
    if (this->electronBeamEnergy > (this->servicerPotential - this->targetPotential)) {
        double intermediateTerm = -1 * (this->electronBeamEnergy - this->servicerPotential + this->targetPotential) / 20.0;
        this->servicerEBCurrent = this->electronBeamCurrent * (1 - exp(intermediateTerm));
        this->targetEBCurrent = - this->electronBeamCurrent * (1 - exp(intermediateTerm));
    }
}

/*! Method to compute photoelectric currents */
void SpacecraftCharging::computePhotoelectricCurrent() {
    // Compute the servicer photoelectric current
    if (this->servicerPotential <= 0) {
        this->servicerPhotoelectricCurrent = this->fluxPhotoelectrons * this->servicerSunlitArea;
    } else {
        this->servicerPhotoelectricCurrent = this->fluxPhotoelectrons * this->servicerSunlitArea * exp(-1 * this->servicerPotential / this->tempPhotoelectrons);
    }

    // Compute the target photoelectric current
    if (this->targetPotential <= 0) {
        this->targetPhotoelectricCurrent = this->fluxPhotoelectrons * this->targetSunlitArea;
    } else {
        this->targetPhotoelectricCurrent = this->fluxPhotoelectrons * this->targetSunlitArea * exp(-1 * this->targetPotential / this->tempPhotoelectrons);
    }
}

/*! Method for pre-integration steps.
 @param integrateToThisTimeNanos Time to integrate to
 */
void SpacecraftCharging::preIntegration(uint64_t integrateToThisTimeNanos) {
    this->timeStep = diffNanoToSec(integrateToThisTimeNanos, this->timeBeforeNanos);
}

/*! Method for post-integration steps.
 @param integrateToThisTimeNanos Time to integrate to
 */
void SpacecraftCharging::postIntegration(uint64_t integrateToThisTimeNanos) {
    this->timeBeforeNanos = integrateToThisTimeNanos;
    this->timeBefore = integrateToThisTimeNanos*NANO2SEC;
}

/*! Setter for the servicer spacecraft capacitance.
 @param capacitance [farad] Servicer spacecraft capacitance
 */
void SpacecraftCharging::setServicerCapacitance(const double capacitance) {
    assert(capacitance > 0.0);
    this->servicerCapacitance = std::abs(capacitance);
}

/*! Setter for the target spacecraft capacitance.
 @param capacitance [farad] Target spacecraft capacitance
 */
void SpacecraftCharging::setTargetCapacitance(const double capacitance) {
    assert(capacitance > 0.0);
    this->targetCapacitance = std::abs(capacitance);
}

/*! Getter for the servicer spacecraft capacitance.
 @return double
*/
double SpacecraftCharging::getServicerCapacitance() const {
    return this->servicerCapacitance;
}

/*! Getter for the target spacecraft capacitance.
 @return double
*/
double SpacecraftCharging::getTargetCapacitance() const {
    return this->targetCapacitance;
}

/*! Setter for the photoelectron temperature.
 @param temp [eV] Photoelectron temperature
 */
void SpacecraftCharging::setTempPhotoelectrons(const double temp) {
    assert(temp > 0.0);
    this->tempPhotoelectrons = std::abs(temp);
}

/*! Setter for the photoelectron flux.
 @param flux [A/m^2] Photoelectron flux
 */
void SpacecraftCharging::setFluxPhotoelectrons(const double flux) {
    assert(flux > 0.0);
    this->fluxPhotoelectrons = std::abs(flux);
}

/*! Getter for the photoelectron temperature.
 @return double
*/
double SpacecraftCharging::getTempPhotoelectrons() const {
    return this->tempPhotoelectrons;
}

/*! Getter for the photoelectron flux.
 @return double
*/
double SpacecraftCharging::getFluxPhotoelectrons() const {
    return this->fluxPhotoelectrons;
}
