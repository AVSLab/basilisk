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

/*! Class constructor. */
SpacecraftCharging::SpacecraftCharging() {
    this->nameOfScPotentialState = "scPotential";

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
    this->scPotentialState = states.registerState(1, 1, this->nameOfScPotentialState);
    Eigen::MatrixXd scPotentialInitMatrix(1, 1);
    scPotentialInitMatrix(0, 0) = this->scPotentialInit;
    this->scPotentialState->setState(scPotentialInitMatrix);
}

/*! Module update method. */
void SpacecraftCharging::UpdateState(uint64_t CurrentSimNanos) {
    // Integrate the state forward in time
    this->integrateState(CurrentSimNanos);

    // Write the module output messages
    this->writeOutputStateMessages(CurrentSimNanos);
}

/*! Method to write module output messages. */
void SpacecraftCharging::writeOutputStateMessages(uint64_t clockTime) {
    this->scPotential = this->scPotentialState->getState()(0, 0);

    // Write out the spinning body output messages
    VoltMsgPayload voltageMsgBuffer;
    voltageMsgBuffer = this->scPotentialOutMsg.zeroMsgPayload;
    voltageMsgBuffer.voltage = this->scPotential;
    this->scPotentialOutMsg.write(&voltageMsgBuffer, this->moduleID, clockTime);
}

/*! Method for charging equations of motion */
void SpacecraftCharging::equationsOfMotion(double integTimeSeconds, double timeStep) {
    this->scPotential = this->scPotentialState->getState()(0, 0);

    double beam_current{};
    if (this->E_eBeam > this->scPotential) {
        beam_current = this->I_eBeam;
    }

    Eigen::MatrixXd scPotentialRate(1, 1);
    scPotentialRate(0, 0) = beam_current / this->scCapacitance;
    this->scPotentialState->setDerivative(scPotentialRate);
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

/*! Setter for the spacecraft capacitance.
 @param capacitance [farad] Spacecraft capacitance
 */
void SpacecraftCharging::setScCapacitance(const double capacitance) {
    assert(capacitance > 0.0);
    this->scCapacitance = std::abs(capacitance);
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

/*! Getter for the spacecraft capacitance.
 @return double
*/
double SpacecraftCharging::getScCapacitance() const {
    return this->scCapacitance;
}
