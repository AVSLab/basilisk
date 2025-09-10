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

/*! Class constructor. */
SpacecraftCharging::SpacecraftCharging() {
    // Set integrator as RK4
    this->integrator = new svIntegratorRK4(this);
}

/*! Reset method. */
void SpacecraftCharging::Reset(uint64_t CurrentSimNanos) {
}

/*! Module update method. */
void SpacecraftCharging::UpdateState(uint64_t CurrentSimNanos) {
    // Write the module output messages
    this->writeOutputStateMessages(CurrentSimNanos);
}

/*! Method to write module output messages. */
void SpacecraftCharging::writeOutputStateMessages(uint64_t clockTime) {
}

/*! Method for charging equations of motion */
void SpacecraftCharging::equationsOfMotion(double integTimeSeconds, double timeStep) {
}

/*! Method for pre-integration steps.
 @param integrateToThisTimeNanos Time to integrate to
 */
void SpacecraftCharging::preIntegration(uint64_t integrateToThisTimeNanos) {
}

/*! Method for post-integration steps.
 @param integrateToThisTimeNanos Time to integrate to
 */
void SpacecraftCharging::postIntegration(uint64_t integrateToThisTimeNanos) {
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
