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

#include "spacecraftChargingDynamics.h"
#include "../../dynamics/_GeneralModuleFiles/svIntegratorRK4.h"
#include "architecture/utilities/astroConstants.h"
#include "architecture/utilities/avsEigenSupport.h"
#include "architecture/utilities/macroDefinitions.h"
#include <cmath>

/*! Class constructor. */
SpacecraftChargingDynamics::SpacecraftChargingDynamics() {
    this->nameOfServicerPotentialState = "servicerPotential";
    this->nameOfTargetPotentialState = "targetPotential";

    // Set integrator as RK4
    this->integrator = new svIntegratorRK4(this);
}

/*! Reset method. */
void SpacecraftChargingDynamics::Reset(uint64_t CurrentSimNanos) {
    this->initializeDynamics();
    this->writeOutputStateMessages(CurrentSimNanos);
    this->timeBefore = CurrentSimNanos * NANO2SEC;
    this->timeBeforeNanos = CurrentSimNanos;
}

/*! Method to initialize dynamics. */
void SpacecraftChargingDynamics::initializeDynamics() {
    this->registerStates(this->dynManager);

    // Call equations of motion at time zero
    this->equationsOfMotion(0.0, 1.0);
}

/*! Method to register states. */
void SpacecraftChargingDynamics::registerStates(DynParamManager& states) {
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
void SpacecraftChargingDynamics::UpdateState(uint64_t CurrentSimNanos) {
    // Read the target and servicer spacecraft state input message if it is linked and written
    if (this->servicerStateInMsg.isLinked() && this->servicerStateInMsg.isWritten()) {
        SCStatesMsgPayload servicerStateInMsgBuffer = this->servicerStateInMsg();
        this->v_SN_N_norm = cArray2EigenVector3d(servicerStateInMsgBuffer.v_BN_N).norm();
    } else {
        this->bskLogger.bskError("SpacecraftChargingDynamics.servicerStateInMsg was not linked or written.");
    }
    if (this->targetStateInMsg.isLinked() && this->targetStateInMsg.isWritten()) {
        SCStatesMsgPayload targetStateInMsgBuffer = this->targetStateInMsg();
        this->v_TN_N_norm = cArray2EigenVector3d(targetStateInMsgBuffer.v_BN_N).norm();
    } else {
        this->bskLogger.bskError("SpacecraftChargingDynamics.targetStateInMsg was not linked or written.");
    }

    // Read the servicer and target spacecraft area input messages if they are linked and written
    if (this->servicerSurfaceAreaInMsg.isLinked() && this->servicerSurfaceAreaInMsg.isWritten()) {
        ProjectedAreaMsgPayload servicerSurfaceAreaInMsgBuffer = this->servicerSurfaceAreaInMsg();
        this->servicerSurfaceArea = servicerSurfaceAreaInMsgBuffer.area;
    } else {
        this->bskLogger.bskError("SpacecraftChargingDynamics.servicerSurfaceAreaInMsg was not linked or written.");
    }
    if (this->targetSurfaceAreaInMsg.isLinked() && this->targetSurfaceAreaInMsg.isWritten()) {
        ProjectedAreaMsgPayload targetSurfaceAreaInMsgBuffer = this->targetSurfaceAreaInMsg();
        this->targetSurfaceArea = targetSurfaceAreaInMsgBuffer.area;
    } else {
        this->bskLogger.bskError("SpacecraftChargingDynamics.targetSurfaceAreaInMsg was not linked or written.");
    }
    if (this->servicerSunlitAreaInMsg.isLinked() && this->servicerSunlitAreaInMsg.isWritten()) {
        ProjectedAreaMsgPayload servicerSunlitFacetAreaInMsgBuffer = this->servicerSunlitAreaInMsg();
        this->servicerSunlitArea = servicerSunlitFacetAreaInMsgBuffer.area;
    } else {
        this->bskLogger.bskError("SpacecraftChargingDynamics.servicerSunlitAreaInMsg was not linked or written.");
    }
    if (this->targetSunlitAreaInMsg.isLinked() && this->targetSunlitAreaInMsg.isWritten()) {
        ProjectedAreaMsgPayload targetSunlitFacetAreaInMsgBuffer = this->targetSunlitAreaInMsg();
        this->targetSunlitArea = targetSunlitFacetAreaInMsgBuffer.area;
    } else {
        this->bskLogger.bskError("SpacecraftChargingDynamics.targetSunlitAreaInMsg was not linked or written.");
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

    // Re-compute the currents before writing the module output messages
    this->computeCurrents();

    // Write the module output messages
    this->writeOutputStateMessages(CurrentSimNanos);
}

/*! Method to write module output messages. */
void SpacecraftChargingDynamics::writeOutputStateMessages(uint64_t clockTime) {
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

    CurrentMsgPayload servicerPlasmaElectronCurrentMsgBuffer;
    servicerPlasmaElectronCurrentMsgBuffer = this->servicerPlasmaElectronCurrentOutMsg.zeroMsgPayload;
    servicerPlasmaElectronCurrentMsgBuffer.current = this->servicerPlasmaElectronCurrent;
    this->servicerPlasmaElectronCurrentOutMsg.write(&servicerPlasmaElectronCurrentMsgBuffer, this->moduleID, clockTime);

    CurrentMsgPayload servicerPlasmaIonCurrentMsgBuffer;
    servicerPlasmaIonCurrentMsgBuffer = this->servicerPlasmaIonCurrentOutMsg.zeroMsgPayload;
    servicerPlasmaIonCurrentMsgBuffer.current = this->servicerPlasmaIonCurrent;
    this->servicerPlasmaIonCurrentOutMsg.write(&servicerPlasmaIonCurrentMsgBuffer, this->moduleID, clockTime);

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

    CurrentMsgPayload targetPlasmaElectronCurrentMsgBuffer;
    targetPlasmaElectronCurrentMsgBuffer = this->targetPlasmaElectronCurrentOutMsg.zeroMsgPayload;
    targetPlasmaElectronCurrentMsgBuffer.current = this->targetPlasmaElectronCurrent;
    this->targetPlasmaElectronCurrentOutMsg.write(&targetPlasmaElectronCurrentMsgBuffer, this->moduleID, clockTime);

    CurrentMsgPayload targetPlasmaIonCurrentMsgBuffer;
    targetPlasmaIonCurrentMsgBuffer = this->targetPlasmaIonCurrentOutMsg.zeroMsgPayload;
    targetPlasmaIonCurrentMsgBuffer.current = this->targetPlasmaIonCurrent;
    this->targetPlasmaIonCurrentOutMsg.write(&targetPlasmaIonCurrentMsgBuffer, this->moduleID, clockTime);
}

/*! Method for the charging equations of motion */
void SpacecraftChargingDynamics::equationsOfMotion(double integTimeSeconds, double timeStep) {
    this->servicerPotential = this->servicerPotentialState->getState()(0, 0);
    this->targetPotential = this->targetPotentialState->getState()(0, 0);

    // Compute all currents acting on the spacecraft
    this->computeCurrents();

    // Set the servicer potential derivative
    Eigen::MatrixXd servicerPotentialRate(1, 1);
    servicerPotentialRate(0, 0) = (this->servicerPlasmaElectronCurrent + this->servicerPlasmaIonCurrent
            + this->servicerPhotoelectricCurrent + this->servicerEBCurrent) / this->servicerCapacitance;
    this->servicerPotentialState->setDerivative(servicerPotentialRate);

    // Set the target potential derivative
    Eigen::MatrixXd targetPotentialRate(1, 1);
    targetPotentialRate(0, 0) = (this->targetPlasmaElectronCurrent + this->targetPlasmaIonCurrent
            + this->targetPhotoelectricCurrent + this->targetEBCurrent) / this->targetCapacitance;
    this->targetPotentialState->setDerivative(targetPotentialRate);
}

/*! Method for computing all currents acting on the spacecraft */
void SpacecraftChargingDynamics::computeCurrents() {
    // Compute the plasma electron currents
    this->servicerPlasmaElectronCurrent = this->computePlasmaElectronCurrent(this->servicerSurfaceArea,
                                                                             this->servicerPotential);
    this->targetPlasmaElectronCurrent = this->computePlasmaElectronCurrent(this->targetSurfaceArea,
                                                                           this->targetPotential);

    // Compute the plasma ion currents
    this->servicerPlasmaIonCurrent = this->computePlasmaIonCurrent(this->servicerSurfaceArea,
                                                                   this->servicerSunlitArea,
                                                                   this->servicerPotential,
                                                                   this->v_SN_N_norm);
    this->targetPlasmaIonCurrent = this->computePlasmaIonCurrent(this->targetSurfaceArea,
                                                                 this->targetSunlitArea,
                                                                 this->targetPotential,
                                                                 this->v_TN_N_norm);

    // Compute the electron beam currents
    this->computeElectronBeamCurrent();

    // Compute the photoelectric currents
    this->computePhotoelectricCurrent();
}

/*! Method to compute plasma electron current
 @param surfaceArea [m^2] Spacecraft surface area
 @param spacecraftPotential [Volts] Spacecraft potential
 @return double
*/
double SpacecraftChargingDynamics::computePlasmaElectronCurrent(double surfaceArea, double spacecraftPotential) {
    double thermalVelocityElectrons = std::sqrt((8 * Q_CHARGE * this->tempElectrons) / (MASS_ELECTRON * MPI));  // [m/s] thermal electron velocity

    double plasmaElectronCurrent{};
    if (spacecraftPotential <= 0.0) {
        plasmaElectronCurrent = (-0.25 * surfaceArea * Q_CHARGE * this->densityElectrons * thermalVelocityElectrons) * exp(spacecraftPotential / this->tempElectrons);
    } else {
        plasmaElectronCurrent = (-0.25 * surfaceArea * Q_CHARGE * this->densityElectrons * thermalVelocityElectrons) * (1 + (spacecraftPotential / this->tempElectrons));
    }

    return plasmaElectronCurrent;
}

/*! Method to compute plasma ion current
 @param surfaceArea [m^2] Spacecraft surface area
 @param sunlitArea [m^2] Spacecraft sunlit area
 @param spacecraftPotential [Volts] Spacecraft potential
 @param v_BN_N_norm [m/s] Spacecraft inertial velocity norm
 @return double
*/
double SpacecraftChargingDynamics::computePlasmaIonCurrent(double surfaceArea, double sunlitArea, double spacecraftPotential, double v_BN_N_norm) {
    double thermalVelocityIons = std::sqrt((8 * Q_CHARGE * this->tempIons) / (MASS_PROTON * MPI));  // [m/s] thermal ion velocity
    double relativeVelocityIons = std::abs(v_BN_N_norm - bulkVelocityIons);

    double plasmaIonCurrent{};

    if (thermalVelocityIons >= relativeVelocityIons) {
        if (spacecraftPotential <= 0.0) {
            plasmaIonCurrent = (0.25 * surfaceArea * Q_CHARGE * this->densityIons * thermalVelocityIons) * (1 - (spacecraftPotential / this->tempIons));
        } else {
            plasmaIonCurrent = (0.25 * surfaceArea * Q_CHARGE * this->densityIons * thermalVelocityIons) * exp(-spacecraftPotential / this->tempIons);
        }
    } else {
        plasmaIonCurrent = sunlitArea * Q_CHARGE * this->densityIons * relativeVelocityIons;
    }

    return plasmaIonCurrent;
}

/*! Method to compute electron beam currents */
void SpacecraftChargingDynamics::computeElectronBeamCurrent() {
    if (this->electronBeamEnergy > (this->servicerPotential - this->targetPotential)) {
        double intermediateTerm = -1 * (this->electronBeamEnergy - this->servicerPotential + this->targetPotential) / 20.0;
        this->servicerEBCurrent = this->electronBeamCurrent * (1 - exp(intermediateTerm));
        this->targetEBCurrent = - this->alphaEB * this->electronBeamCurrent * (1 - exp(intermediateTerm));
    } else {
        this->servicerEBCurrent = 0.0;
        this->targetEBCurrent = 0.0;
    }
}

/*! Method to compute photoelectric currents */
void SpacecraftChargingDynamics::computePhotoelectricCurrent() {
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
void SpacecraftChargingDynamics::preIntegration(uint64_t integrateToThisTimeNanos) {
    this->timeStep = diffNanoToSec(integrateToThisTimeNanos, this->timeBeforeNanos);
}

/*! Method for post-integration steps.
 @param integrateToThisTimeNanos Time to integrate to
*/
void SpacecraftChargingDynamics::postIntegration(uint64_t integrateToThisTimeNanos) {
    this->timeBeforeNanos = integrateToThisTimeNanos;
    this->timeBefore = integrateToThisTimeNanos*NANO2SEC;
}

/*! Setter for the initial servicer potential.
 @param potentialInit [Volts] Servicer initial potential
*/
void SpacecraftChargingDynamics::setServicerPotentialInit(const double potentialInit) {
    this->servicerPotentialInit = potentialInit;
}

/*! Setter for the initial target potential.
 @param potentialInit [Volts] Target initial potential
*/
void SpacecraftChargingDynamics::setTargetPotentialInit(const double potentialInit) {
    this->targetPotentialInit = potentialInit;
}

/*! Getter for the servicer initial potential.
 @return double
*/
double SpacecraftChargingDynamics::getServicerPotentialInit() const {
    return this->servicerPotentialInit;
}

/*! Getter for the target initial potential.
 @return double
*/
double SpacecraftChargingDynamics::getTargetPotentialInit() const {
    return this->targetPotentialInit;
}

/*! Setter for the servicer spacecraft capacitance.
 @param capacitance [farad] Servicer spacecraft capacitance
*/
void SpacecraftChargingDynamics::setServicerCapacitance(const double capacitance) {
    assert(capacitance > 0.0);
    this->servicerCapacitance = std::abs(capacitance);
}

/*! Setter for the target spacecraft capacitance.
 @param capacitance [farad] Target spacecraft capacitance
*/
void SpacecraftChargingDynamics::setTargetCapacitance(const double capacitance) {
    assert(capacitance > 0.0);
    this->targetCapacitance = std::abs(capacitance);
}

/*! Getter for the servicer spacecraft capacitance.
 @return double
*/
double SpacecraftChargingDynamics::getServicerCapacitance() const {
    return this->servicerCapacitance;
}

/*! Getter for the target spacecraft capacitance.
 @return double
*/
double SpacecraftChargingDynamics::getTargetCapacitance() const {
    return this->targetCapacitance;
}

/*! Setter for the photoelectron temperature.
 @param temp [eV] Photoelectron temperature
*/
void SpacecraftChargingDynamics::setTempPhotoelectrons(const double temp) {
    assert(temp > 0.0);
    this->tempPhotoelectrons = std::abs(temp);
}

/*! Setter for the photoelectron flux.
 @param flux [A/m^2] Photoelectron flux
*/
void SpacecraftChargingDynamics::setFluxPhotoelectrons(const double flux) {
    assert(flux > 0.0);
    this->fluxPhotoelectrons = std::abs(flux);
}

/*! Getter for the photoelectron temperature.
 @return double
*/
double SpacecraftChargingDynamics::getTempPhotoelectrons() const {
    return this->tempPhotoelectrons;
}

/*! Getter for the photoelectron flux.
 @return double
*/
double SpacecraftChargingDynamics::getFluxPhotoelectrons() const {
    return this->fluxPhotoelectrons;
}

/*! Setter for the electron temperature.
 @param temp [eV] Electron temperature
*/
void SpacecraftChargingDynamics::setTempElectrons(const double temp) {
    assert(temp > 0.0);
    this->tempElectrons = std::abs(temp);
}

/*! Getter for the electron temperature.
 @return double
*/
double SpacecraftChargingDynamics::getTempElectrons() const {
    return this->tempElectrons;
}

/*! Setter for the electron density.
 @param density [m^-3] Electron density
*/
void SpacecraftChargingDynamics::setDensityElectrons(const double density) {
    assert(density > 0.0);
    this->densityElectrons = std::abs(density);
}

/*! Getter for the electron density.
 @return double
*/
double SpacecraftChargingDynamics::getDensityElectrons() const {
    return this->densityElectrons;
}

/*! Setter for the ion temperature.
 @param temp [eV] Ion temperature
*/
void SpacecraftChargingDynamics::setTempIons(const double temp) {
    assert(temp > 0.0);
    this->tempIons = std::abs(temp);
}

/*! Getter for the ion temperature.
 @return double
*/
double SpacecraftChargingDynamics::getTempIons() const {
    return this->tempIons;
}

/*! Setter for the ion density.
 @param density [m^-3] Ion density
*/
void SpacecraftChargingDynamics::setDensityIons(const double density) {
    assert(density > 0.0);
    this->densityIons = std::abs(density);
}

/*! Getter for the ion density.
 @return double
*/
double SpacecraftChargingDynamics::getDensityIons() const {
    return this->densityIons;
}

/*! Setter for the bulk ion velocity.
 @param velocity [m/s] Bulk ion velocity
*/
void SpacecraftChargingDynamics::setBulkVelocityIons(const double velocity) {
    assert(velocity >= 0.0);
    this->bulkVelocityIons = std::abs(velocity);
}

/*! Getter for the bulk ion velocity.
 @return double
*/
double SpacecraftChargingDynamics::getBulkVelocityIons() const {
    return this->bulkVelocityIons;
}
