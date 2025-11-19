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

    // Read the servicer sunlit facet area input message if it is linked and written
    if (this->servicerSunlitAreaInMsg.isLinked() && this->servicerSunlitAreaInMsg.isWritten())
    {
        SCSunlitFacetAreaMsgPayload servicerSunlitFacetAreaInMsgBuffer = this->servicerSunlitAreaInMsg();
        this->servicerSunlitArea = servicerSunlitFacetAreaInMsgBuffer.area;
    }

    // Read the target sunlit facet area input message if it is linked and written
    if (this->targetSunlitAreaInMsg.isLinked() && this->targetSunlitAreaInMsg.isWritten())
    {
        SCSunlitFacetAreaMsgPayload targetSunlitFacetAreaInMsgBuffer = this->targetSunlitAreaInMsg();
        this->targetSunlitArea = targetSunlitFacetAreaInMsgBuffer.area;
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

    CurrentMsgPayload servicerPECurrentMsgBuffer;
    servicerPECurrentMsgBuffer = this->servicerPhotoElectricCurrentOutMsg.zeroMsgPayload;
    servicerPECurrentMsgBuffer.current = this->servicerPhotoElectricCurrent;
    this->servicerPhotoElectricCurrentOutMsg.write(&servicerPECurrentMsgBuffer, this->moduleID, clockTime);

    // Write out the target output messages
    VoltMsgPayload targetVoltageMsgBuffer;
    targetVoltageMsgBuffer = this->targetPotentialOutMsg.zeroMsgPayload;
    targetVoltageMsgBuffer.voltage = this->targetPotential;
    this->targetPotentialOutMsg.write(&targetVoltageMsgBuffer, this->moduleID, clockTime);

    CurrentMsgPayload targetPECurrentMsgBuffer;
    targetPECurrentMsgBuffer = this->targetPhotoElectricCurrentOutMsg.zeroMsgPayload;
    targetPECurrentMsgBuffer.current = this->targetPhotoElectricCurrent;
    this->targetPhotoElectricCurrentOutMsg.write(&targetPECurrentMsgBuffer, this->moduleID, clockTime);
}

/*! Method for charging equations of motion */
void SpacecraftCharging::equationsOfMotion(double integTimeSeconds, double timeStep) {
    this->servicerPotential = this->servicerPotentialState->getState()(0, 0);
    this->targetPotential = this->targetPotentialState->getState()(0, 0);

    // Solve for the beam current on the target and servicer
    double beam_current_servicer{};
    double beam_current_target{};
    if (this->E_eBeam > (this->servicerPotential - this->targetPotential)) {
        double intermediateTerm = -1 * (this->E_eBeam - this->servicerPotential + this->targetPotential) / 20.0;
        beam_current_servicer = this->I_eBeam * (1 - exp(intermediateTerm));
        beam_current_target = - this->I_eBeam * (1 - exp(intermediateTerm));
    }

    // Compute the servicer photoelectric current
    double temp_photons = 2.0; // [eV]
    double flux_photons = 1e-6; // [A/m^2]
    if (this->servicerPotential <= 0) {
        this->servicerPhotoElectricCurrent = flux_photons * this->servicerSunlitArea;
    } else {
        this->servicerPhotoElectricCurrent = flux_photons * this->servicerSunlitArea * exp(-1 * this->servicerPotential / temp_photons);
    }

    // Compute the target photoelectric current
    double photoelectric_current_target{};
    if (this->targetPotential <= 0) {
        this->targetPhotoElectricCurrent = flux_photons * this->targetSunlitArea;
    } else {
        this->targetPhotoElectricCurrent = flux_photons * this->targetSunlitArea * exp(-1 * this->targetPotential / temp_photons);
    }

    // Set the servicer potential derivative
    Eigen::MatrixXd servicerPotentialRate(1, 1);
    servicerPotentialRate(0, 0) = (beam_current_servicer + this->servicerPhotoElectricCurrent) / this->servicerCapacitance;
    this->servicerPotentialState->setDerivative(servicerPotentialRate);

    // Set the target potential derivative
    Eigen::MatrixXd targetPotentialRate(1, 1);
    targetPotentialRate(0, 0) = (beam_current_target + this->targetPhotoElectricCurrent) / this->targetCapacitance;
    this->targetPotentialState->setDerivative(targetPotentialRate);
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

//    this->servicerPotential = this->servicerPotentialState->getState()(0, 0);
//    this->targetPotential = this->targetPotentialState->getState()(0, 0);
//    if (this->servicerPotential >= this->E_eBeam) {
//        this->servicerPotential = this->E_eBeam;
//    }
//
//    Eigen::MatrixXd servicerPotentialMatrix(1, 1);
//    servicerPotentialMatrix(0, 0) = this->servicerPotential;
//    this->servicerPotentialState->setState(servicerPotentialMatrix);
}

/*! Setter for the electron beam current.
 @param I_eBeam [Amps] Electron beam current
 */
void SpacecraftCharging::setEBeamCurrent(const double I_eBeam) {
    this->I_eBeam = I_eBeam;
}

/*! Setter for the electron beam energy.
 @param I_eBeam [keV] Electron beam energy
 */
void SpacecraftCharging::setEBeamEnergy(const double E_eBeam) {
    assert(E_eBeam > 0.0);
    this->E_eBeam = std::abs(E_eBeam);
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

/*! Getter for the electron beam current.
 @return double
*/
double SpacecraftCharging::getEBeamCurrent() const {
    return this->I_eBeam;
}

/*! Getter for the electron beam energy.
 @return double
*/
double SpacecraftCharging::getEBeamEnergy() const {
    return this->E_eBeam;
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
