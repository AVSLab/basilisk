/*
 ISC License

 Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado Boulder

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


#include "simulation/environment/spacecraftChargingEquilibrium/spacecraftChargingEquilibrium.h"

#include "architecture/utilities/linearAlgebra.h"
#include "architecture/utilities/avsEigenSupport.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <vector>

using ::bisectionSolve;

namespace {
bool isValidYieldVector(const Eigen::VectorXd& yieldVector)
{
    if (yieldVector.size() == 0) {
        return false;
    }
    for (Eigen::Index i = 0; i < yieldVector.size(); i++) {
        if (!std::isfinite(yieldVector[i]) || yieldVector[i] < 0.0) {
            return false;
        }
    }
    return true;
}

double validateAndClampAlphaEB(double alphaEB, unsigned int spacecraftIndex, BSKLogger& bskLogger)
{
    if (!std::isfinite(alphaEB)) {
        bskLogger.bskLog(
            BSK_WARNING,
            "SpacecraftChargingEquilibrium: spacecraft %d received non-finite alphaEB (%g). Using default alphaEB=1.0.",
            spacecraftIndex,
            alphaEB);
        return 1.0;
    }
    const double clampedAlpha = std::clamp(alphaEB, 0.0, 1.0);
    if (clampedAlpha != alphaEB) {
        bskLogger.bskLog(
            BSK_WARNING,
            "SpacecraftChargingEquilibrium: spacecraft %d alphaEB=%g is outside [0,1]. Clamping to %g.",
            spacecraftIndex,
            alphaEB,
            clampedAlpha);
    }
    return clampedAlpha;
}
}



/*! This is the constructor for the module class. */
SpacecraftChargingEquilibrium::SpacecraftChargingEquilibrium() = default;

/*! Module Destructor */
SpacecraftChargingEquilibrium::~SpacecraftChargingEquilibrium()
{
    /* free up output message objects */
    for (long unsigned int c=0; c<this->voltOutMsgs.size(); c++) {
        delete this->voltOutMsgs.at(c);
        delete this->currentsOutMsgs.at(c);
    }
}

void SpacecraftChargingEquilibrium::setEnableDebugPrints(bool enabled)
{
    this->enableDebugPrints = enabled;
}

bool SpacecraftChargingEquilibrium::getEnableDebugPrints() const
{
    return this->enableDebugPrints;
}

void SpacecraftChargingEquilibrium::setYieldSEEelectron(const Eigen::VectorXd& yieldVector)
{
    if (!isValidYieldVector(yieldVector)) {
        this->bskLogger.bskLog(BSK_ERROR,
                               "SpacecraftChargingEquilibrium: setYieldSEEelectron requires a non-empty vector with finite non-negative values.");
        return;
    }
    this->yieldSEEelectron = yieldVector;
}

void SpacecraftChargingEquilibrium::setYieldSEEion(const Eigen::VectorXd& yieldVector)
{
    if (!isValidYieldVector(yieldVector)) {
        this->bskLogger.bskLog(BSK_ERROR,
                               "SpacecraftChargingEquilibrium: setYieldSEEion requires a non-empty vector with finite non-negative values.");
        return;
    }
    this->yieldSEEion = yieldVector;
}

void SpacecraftChargingEquilibrium::setYieldBackscattered(const Eigen::VectorXd& yieldVector)
{
    if (!isValidYieldVector(yieldVector)) {
        this->bskLogger.bskLog(BSK_ERROR,
                               "SpacecraftChargingEquilibrium: setYieldBackscattered requires a non-empty vector with finite non-negative values.");
        return;
    }
    this->yieldBackscattered = yieldVector;
}

Eigen::VectorXd SpacecraftChargingEquilibrium::getYieldSEEelectron() const
{
    return this->yieldSEEelectron;
}

Eigen::VectorXd SpacecraftChargingEquilibrium::getYieldSEEion() const
{
    return this->yieldSEEion;
}

Eigen::VectorXd SpacecraftChargingEquilibrium::getYieldBackscattered() const
{
    return this->yieldBackscattered;
}

void SpacecraftChargingEquilibrium::setSunlitAreaDefault(unsigned int spacecraftIndex, double sunlitArea)
{
    if (!std::isfinite(sunlitArea) || sunlitArea < 0.0) {
        this->bskLogger.bskLog(BSK_ERROR, "SpacecraftChargingEquilibrium: setSunlitAreaDefault requires a finite non-negative area.");
        return;
    }
    if (spacecraftIndex >= this->scSunlitAreaDefaults.size()) {
        this->bskLogger.bskLog(BSK_ERROR,
                               "SpacecraftChargingEquilibrium: setSunlitAreaDefault index %u is out of range (configured spacecraft: %lu).",
                               spacecraftIndex,
                               this->scSunlitAreaDefaults.size());
        return;
    }
    this->scSunlitAreaDefaults[spacecraftIndex] = sunlitArea;
    if (spacecraftIndex < this->scSunlitAreaWarned.size()) {
        this->scSunlitAreaWarned[spacecraftIndex] = false;
    }
}

double SpacecraftChargingEquilibrium::getSunlitAreaDefault(unsigned int spacecraftIndex) const
{
    if (spacecraftIndex >= this->scSunlitAreaDefaults.size()) {
        return NAN;
    }
    return this->scSunlitAreaDefaults[spacecraftIndex];
}

void SpacecraftChargingEquilibrium::clearSunlitAreaDefault(unsigned int spacecraftIndex)
{
    if (spacecraftIndex >= this->scSunlitAreaDefaults.size()) {
        this->bskLogger.bskLog(BSK_ERROR,
                               "SpacecraftChargingEquilibrium: clearSunlitAreaDefault index %u is out of range (configured spacecraft: %lu).",
                               spacecraftIndex,
                               this->scSunlitAreaDefaults.size());
        return;
    }
    this->scSunlitAreaDefaults[spacecraftIndex] = std::numeric_limits<double>::quiet_NaN();
}

void SpacecraftChargingEquilibrium::setSunlitAreaFallback(double sunlitArea)
{
    if (!std::isfinite(sunlitArea) || sunlitArea < 0.0) {
        this->bskLogger.bskLog(BSK_ERROR, "SpacecraftChargingEquilibrium: setSunlitAreaFallback requires a finite non-negative area.");
        return;
    }
    this->sunlitAreaFallback = sunlitArea;
}

double SpacecraftChargingEquilibrium::getSunlitAreaFallback() const
{
    return this->sunlitAreaFallback;
}

void SpacecraftChargingEquilibrium::setRootSolveBounds(double lowerBound, double upperBound)
{
    if (!std::isfinite(lowerBound) || !std::isfinite(upperBound) || lowerBound >= upperBound) {
        this->bskLogger.bskLog(BSK_ERROR,
                               "SpacecraftChargingEquilibrium: setRootSolveBounds requires finite lowerBound < upperBound.");
        return;
    }
    this->rootSolveLowerBound = lowerBound;
    this->rootSolveUpperBound = upperBound;
}

double SpacecraftChargingEquilibrium::getRootSolveLowerBound() const
{
    return this->rootSolveLowerBound;
}

double SpacecraftChargingEquilibrium::getRootSolveUpperBound() const
{
    return this->rootSolveUpperBound;
}

void SpacecraftChargingEquilibrium::setSurfaceAreaDefault(double area)
{
    if (!std::isfinite(area) || area <= 0.0) {
        this->bskLogger.bskLog(BSK_ERROR, "SpacecraftChargingEquilibrium: setSurfaceAreaDefault requires a finite positive area.");
        return;
    }
    this->defaultSurfaceArea = area;
}

double SpacecraftChargingEquilibrium::getSurfaceAreaDefault() const
{
    return this->defaultSurfaceArea;
}

void SpacecraftChargingEquilibrium::setPhotoelectronFlux(double photoelectronFluxIn)
{
    if (!std::isfinite(photoelectronFluxIn) || photoelectronFluxIn < 0.0) {
        this->bskLogger.bskLog(BSK_ERROR, "SpacecraftChargingEquilibrium: setPhotoelectronFlux requires a finite non-negative value.");
        return;
    }
    this->photoelectronFlux = photoelectronFluxIn;
}

double SpacecraftChargingEquilibrium::getPhotoelectronFlux() const
{
    return this->photoelectronFlux;
}

void SpacecraftChargingEquilibrium::setPhotoelectronTemperature(double photoelectronTemperatureIn)
{
    if (!std::isfinite(photoelectronTemperatureIn) || photoelectronTemperatureIn <= 0.0) {
        this->bskLogger.bskLog(BSK_ERROR, "SpacecraftChargingEquilibrium: setPhotoelectronTemperature requires a finite positive value.");
        return;
    }
    this->photoelectronTemperature = photoelectronTemperatureIn;
}

double SpacecraftChargingEquilibrium::getPhotoelectronTemperature() const
{
    return this->photoelectronTemperature;
}

void SpacecraftChargingEquilibrium::setSecondaryElectronTemperature(double secondaryElectronTemperatureIn)
{
    if (!std::isfinite(secondaryElectronTemperatureIn) || secondaryElectronTemperatureIn <= 0.0) {
        this->bskLogger.bskLog(BSK_ERROR, "SpacecraftChargingEquilibrium: setSecondaryElectronTemperature requires a finite positive value.");
        return;
    }
    this->secondaryElectronTemperature = secondaryElectronTemperatureIn;
}

double SpacecraftChargingEquilibrium::getSecondaryElectronTemperature() const
{
    return this->secondaryElectronTemperature;
}

void SpacecraftChargingEquilibrium::setBackscatterElectronTemperature(double backscatterElectronTemperatureIn)
{
    if (!std::isfinite(backscatterElectronTemperatureIn) || backscatterElectronTemperatureIn <= 0.0) {
        this->bskLogger.bskLog(BSK_ERROR, "SpacecraftChargingEquilibrium: setBackscatterElectronTemperature requires a finite positive value.");
        return;
    }
    this->backscatterElectronTemperature = backscatterElectronTemperatureIn;
}

double SpacecraftChargingEquilibrium::getBackscatterElectronTemperature() const
{
    return this->backscatterElectronTemperature;
}

void SpacecraftChargingEquilibrium::setBeamElectronTemperature(double beamElectronTemperatureIn)
{
    if (!std::isfinite(beamElectronTemperatureIn) || beamElectronTemperatureIn <= 0.0) {
        this->bskLogger.bskLog(BSK_ERROR, "SpacecraftChargingEquilibrium: setBeamElectronTemperature requires a finite positive value.");
        return;
    }
    this->beamElectronTemperature = beamElectronTemperatureIn;
}

double SpacecraftChargingEquilibrium::getBeamElectronTemperature() const
{
    return this->beamElectronTemperature;
}

void SpacecraftChargingEquilibrium::setTrapzBins(int trapzBinsIn)
{
    if (trapzBinsIn <= 0) {
        this->bskLogger.bskLog(BSK_ERROR, "SpacecraftChargingEquilibrium: setTrapzBins requires a strictly positive integer.");
        return;
    }
    this->trapzBins = trapzBinsIn;
}

int SpacecraftChargingEquilibrium::getTrapzBins() const
{
    return this->trapzBins;
}

bool SpacecraftChargingEquilibrium::validateSolveConfiguration()
{
    if (!std::isfinite(this->rootSolveLowerBound) || !std::isfinite(this->rootSolveUpperBound) ||
        this->rootSolveLowerBound >= this->rootSolveUpperBound) {
        this->bskLogger.bskLog(BSK_ERROR,
                               "SpacecraftChargingEquilibrium: root solve bounds are invalid (lower=%g, upper=%g).",
                               this->rootSolveLowerBound,
                               this->rootSolveUpperBound);
        return false;
    }
    if (this->trapzBins <= 0) {
        this->bskLogger.bskLog(BSK_ERROR, "SpacecraftChargingEquilibrium: trapzBins must be strictly positive.");
        return false;
    }
    return true;
}

/*! This method is used to reset the module and checks that required input messages are connected.
 @param CurrentSimNanos current simulation time in nano-seconds
*/
void SpacecraftChargingEquilibrium::Reset(uint64_t CurrentSimNanos)
{
    (void) CurrentSimNanos;

    if (!this->plasmaFluxInMsg.isLinked()) {
        this->bskLogger.bskLog(BSK_ERROR, "SpacecraftChargingEquilibrium.plasmaFluxInMsg was not linked.");
    }

    for (unsigned int c = 0; c < this->scStateInMsgs.size(); c++) {
        if (!this->scStateInMsgs[c].isLinked()) {
            this->bskLogger.bskLog(BSK_ERROR, "SpacecraftChargingEquilibrium.scStateInMsgs[%d] was not linked.", c);
        }
    }

    this->numSat = static_cast<unsigned int>(this->scStateInMsgs.size());
    if (this->numSat != 2) {
        this->bskLogger.bskLog(
            BSK_ERROR,
            "SpacecraftChargingEquilibrium requires exactly 2 spacecraft (index 0 servicer, index 1 target). You added %lu.",
            this->numSat);
    }

    if (!this->validateSolveConfiguration()) {
        return;
    }

    for (unsigned int i = 0; i < this->numSat; i++) {
        const bool msgLinked = (i < this->scSunlitAreaInMsgs.size()) && this->scSunlitAreaInMsgs[i].isLinked();
        const bool defaultSet = (i < this->scSunlitAreaDefaults.size()) && std::isfinite(this->scSunlitAreaDefaults[i]);
        if (!msgLinked && !defaultSet && i < this->scSunlitAreaWarned.size() && !this->scSunlitAreaWarned[i]) {
            this->bskLogger.bskLog(
                BSK_WARNING,
                "SpacecraftChargingEquilibrium: sunlit area not provided for spacecraft %d (no msg linked and no default set). Using fallback value %g m^2.",
                i,
                this->sunlitAreaFallback);
            this->scSunlitAreaWarned[i] = true;
        }
    }
}

/*! This method updates the state of the module
 @param CurrentSimNanos current simulation time in nano-seconds
*/
void SpacecraftChargingEquilibrium::UpdateState(uint64_t CurrentSimNanos)
{
    if (this->numSat != 2) {
        this->bskLogger.bskLog(
            BSK_ERROR,
            "SpacecraftChargingEquilibrium.UpdateState called with numSat=%lu. Exactly two spacecraft are required.",
            this->numSat);
        return;
    }
    if (!this->validateSolveConfiguration()) {
        return;
    }

    // Read input messages and construct local spectra and spacecraft state vectors.
    this->readMessages();
    if (this->energies.size() < 2) {
        this->bskLogger.bskLog(BSK_ERROR, "SpacecraftChargingEquilibrium.UpdateState: energy grid has fewer than 2 valid bins.");
        return;
    }
    if (this->yieldSEEelectron.size() < this->energies.size() ||
        this->yieldSEEion.size() < this->energies.size() ||
        this->yieldBackscattered.size() < this->energies.size()) {
        this->bskLogger.bskLog(
            BSK_ERROR,
            "SpacecraftChargingEquilibrium.UpdateState: yield vectors must have at least %d entries to match the plasma energy grid.",
            this->energies.size());
        return;
    }

    if (this->enableDebugPrints && this->energies.size() > 0) {
        std::cout << "\n--- DEBUG START (Time: " << CurrentSimNanos << ") ---" << std::endl;
        std::cout << "Plasma Energies size: " << this->energies.size() << std::endl;
        std::cout << "First Energy Level: " << this->energies[0] << " eV" << std::endl;
        std::cout << "First Electron Flux: " << this->electronFlux[0] << std::endl;
    }

    std::vector<ChargingSpacecraftState> spaceCrafts(this->numSat);
    for (unsigned int i = 0; i < this->numSat; i++) {
        spaceCrafts[i].A = this->defaultSurfaceArea;

        // Sunlit area message overrides user default which overrides fallback.
        double sunlitArea_m2 = this->sunlitAreaFallback;
        const bool defaultSet = (i < this->scSunlitAreaDefaults.size()) && std::isfinite(this->scSunlitAreaDefaults[i]);
        if (defaultSet) {
            sunlitArea_m2 = this->scSunlitAreaDefaults[i];
        }
        if (i < this->scSunlitAreaInMsgs.size() &&
            this->scSunlitAreaInMsgs[i].isLinked() &&
            this->scSunlitAreaInMsgs[i].isWritten()) {
            const ScSunlitFacetAreaMsgPayload sunBuf = this->scSunlitAreaInMsgs[i]();
            if (std::isfinite(sunBuf.area) && sunBuf.area >= 0.0) {
                sunlitArea_m2 = sunBuf.area;
            } else {
                this->bskLogger.bskLog(
                    BSK_WARNING,
                    "SpacecraftChargingEquilibrium: spacecraft %d sunlit area message was invalid (%g). Using fallback/default value %g m^2.",
                    i,
                    sunBuf.area,
                    sunlitArea_m2);
            }
        }
        spaceCrafts[i].A_sunlit = sunlitArea_m2;

        if (this->eBeamInMsgs[i].isLinked()) {
            const ElectronBeamMsgPayload beam = this->eBeamInMsgs[i]();
            spaceCrafts[i].electronGun.currentEB = beam.currentEB;
            spaceCrafts[i].electronGun.energyEB = beam.energyEB;
            spaceCrafts[i].electronGun.alphaEB = validateAndClampAlphaEB(beam.alphaEB, i, this->bskLogger);
            spaceCrafts[i].emitsEB = true;
            if (this->enableDebugPrints) {
                std::cout << "[DEBUG] SC " << i << " Beam linked. Current: "
                          << beam.currentEB << " Energy: " << beam.energyEB << std::endl;
            }
        } else {
            spaceCrafts[i].emitsEB = false;
            if (this->enableDebugPrints) {
                std::cout << "[DEBUG] SC " << i << " Beam NOT linked." << std::endl;
            }
        }
    }

    std::vector<double> equilibriums(this->numSat, 0.0);
    double bracket[2] = {this->rootSolveLowerBound, this->rootSolveUpperBound};
    const double servicerTargetReferencePotential = 0.0; // [V] reference target potential used in servicer solve.

    const int sID = 0;
    std::function<double(double)> sumCurrentsServicer = [&](double phiS) -> double {
        const double nonBeamCurrents =
            electronCurrent(phiS, spaceCrafts[sID].A) +
            ionCurrent(phiS, spaceCrafts[sID].A) +
            SEEelectronCurrent(phiS, spaceCrafts[sID].A) +
            SEEionCurrent(phiS, spaceCrafts[sID].A) +
            backscatteringCurrent(phiS, spaceCrafts[sID].A) +
            photoelectricCurrent(phiS, spaceCrafts[sID].A_sunlit);

        const double iBeamS = electronBeamCurrent(
            phiS,
            servicerTargetReferencePotential,
            "servicer",
            spaceCrafts[sID].electronGun.energyEB,
            spaceCrafts[sID].electronGun.currentEB,
            spaceCrafts[sID].electronGun.alphaEB);

        const double totalCurrentS = nonBeamCurrents + iBeamS;
        if (this->enableDebugPrints) {
            std::cout << "  [Servicer Loop] phiS: " << std::setw(10) << phiS
                      << " | EnvI: " << nonBeamCurrents
                      << " | BeamI: " << iBeamS
                      << " | TOTAL: " << totalCurrentS << std::endl;
        }
        return totalCurrentS;
    };
    equilibriums[0] = bisectionSolve(bracket, this->servicerSolveAccuracy, sumCurrentsServicer);
    if (!std::isfinite(equilibriums[0])) {
        this->bskLogger.bskLog(
            BSK_ERROR,
            "SpacecraftChargingEquilibrium: servicer root solve returned a non-finite value. Check solve bounds and current model configuration.");
        return;
    }

    const int tID = 1;
    const double fixedPhiS = equilibriums[0];
    std::function<double(double)> sumCurrentsTarget = [&](double phiT) -> double {
        const double nonBeamCurrents =
            electronCurrent(phiT, spaceCrafts[tID].A) +
            ionCurrent(phiT, spaceCrafts[tID].A) +
            SEEelectronCurrent(phiT, spaceCrafts[tID].A) +
            SEEionCurrent(phiT, spaceCrafts[tID].A) +
            backscatteringCurrent(phiT, spaceCrafts[tID].A) +
            photoelectricCurrent(phiT, spaceCrafts[tID].A_sunlit);

        const double iBeamT = electronBeamCurrent(
            fixedPhiS,
            phiT,
            "target",
            spaceCrafts[sID].electronGun.energyEB,
            spaceCrafts[sID].electronGun.currentEB,
            spaceCrafts[sID].electronGun.alphaEB);
        const double iSEE_EB = SEEelectronBeamCurrent(
            fixedPhiS,
            phiT,
            spaceCrafts[sID].electronGun.energyEB,
            spaceCrafts[sID].electronGun.currentEB,
            spaceCrafts[sID].electronGun.alphaEB);
        const double iBS_EB = electronBeamBackscattering(
            fixedPhiS,
            phiT,
            spaceCrafts[sID].electronGun.energyEB,
            spaceCrafts[sID].electronGun.currentEB,
            spaceCrafts[sID].electronGun.alphaEB);

        const double totalCurrentT = nonBeamCurrents + iBeamT + iSEE_EB + iBS_EB;
        if (this->enableDebugPrints) {
            std::cout << "  [Target Loop]   phiT: " << std::setw(10) << phiT
                      << " | EnvI: " << nonBeamCurrents
                      << " | BeamArrivalI: " << iBeamT
                      << " | BeamSEE: " << iSEE_EB
                      << " | TOTAL: " << totalCurrentT << std::endl;
        }
        return totalCurrentT;
    };
    equilibriums[1] = bisectionSolve(bracket, this->targetSolveAccuracy, sumCurrentsTarget);
    if (!std::isfinite(equilibriums[1])) {
        this->bskLogger.bskLog(
            BSK_ERROR,
            "SpacecraftChargingEquilibrium: target root solve returned a non-finite value. Check solve bounds and current model configuration.");
        return;
    }

    if (this->enableDebugPrints) {
        std::cout << "FINAL Result -> Servicer: " << equilibriums[0]
                  << " V | Target: " << equilibriums[1] << " V" << std::endl;
        std::cout << "--- DEBUG END ---\n" << std::endl;
    }

    for (unsigned int i = 0; i < this->numSat; i++) {
        VoltMsgPayload voltMsgBuffer = this->voltOutMsgs.at(i)->zeroMsgPayload;
        voltMsgBuffer.voltage = equilibriums[i];
        this->voltOutMsgs.at(i)->write(&voltMsgBuffer, this->moduleID, CurrentSimNanos);

        ScChargingCurrentsMsgPayload currentMsgBuffer = this->currentsOutMsgs.at(i)->zeroMsgPayload;
        const double phi = equilibriums[i];

        currentMsgBuffer.electronCurrent = electronCurrent(phi, spaceCrafts[i].A);
        currentMsgBuffer.ionCurrent = ionCurrent(phi, spaceCrafts[i].A);
        currentMsgBuffer.seeElectronCurrent = SEEelectronCurrent(phi, spaceCrafts[i].A);
        currentMsgBuffer.seeIonCurrent = SEEionCurrent(phi, spaceCrafts[i].A);
        currentMsgBuffer.backscatteringCurrent = backscatteringCurrent(phi, spaceCrafts[i].A);
        currentMsgBuffer.photoelectricCurrent = photoelectricCurrent(phi, spaceCrafts[i].A_sunlit);
        currentMsgBuffer.nonBeamCurrent =
            currentMsgBuffer.electronCurrent +
            currentMsgBuffer.ionCurrent +
            currentMsgBuffer.seeElectronCurrent +
            currentMsgBuffer.seeIonCurrent +
            currentMsgBuffer.backscatteringCurrent +
            currentMsgBuffer.photoelectricCurrent;

        currentMsgBuffer.beamCurrent = 0.0;
        currentMsgBuffer.beamSeeCurrent = 0.0;
        currentMsgBuffer.beamBackscatteringCurrent = 0.0;
        if (i == 0) {
            currentMsgBuffer.beamCurrent = electronBeamCurrent(
                equilibriums[0],
                servicerTargetReferencePotential,
                "servicer",
                spaceCrafts[0].electronGun.energyEB,
                spaceCrafts[0].electronGun.currentEB,
                spaceCrafts[0].electronGun.alphaEB);
        } else {
            currentMsgBuffer.beamCurrent = electronBeamCurrent(
                equilibriums[0],
                equilibriums[1],
                "target",
                spaceCrafts[0].electronGun.energyEB,
                spaceCrafts[0].electronGun.currentEB,
                spaceCrafts[0].electronGun.alphaEB);
            currentMsgBuffer.beamSeeCurrent = SEEelectronBeamCurrent(
                equilibriums[0],
                equilibriums[1],
                spaceCrafts[0].electronGun.energyEB,
                spaceCrafts[0].electronGun.currentEB,
                spaceCrafts[0].electronGun.alphaEB);
            currentMsgBuffer.beamBackscatteringCurrent = electronBeamBackscattering(
                equilibriums[0],
                equilibriums[1],
                spaceCrafts[0].electronGun.energyEB,
                spaceCrafts[0].electronGun.currentEB,
                spaceCrafts[0].electronGun.alphaEB);
        }

        currentMsgBuffer.totalCurrent =
            currentMsgBuffer.nonBeamCurrent +
            currentMsgBuffer.beamCurrent +
            currentMsgBuffer.beamSeeCurrent +
            currentMsgBuffer.beamBackscatteringCurrent;
        this->currentsOutMsgs.at(i)->write(&currentMsgBuffer, this->moduleID, CurrentSimNanos);
    }
}

/*! This method adds a spacecraft to the charging model.
    Exactly two spacecraft can be added:
    index 0 = servicer, index 1 = target.
*/
void SpacecraftChargingEquilibrium::addSpacecraft(Message<SCStatesMsgPayload> *tmpScMsg)
{
    if (this->scStateInMsgs.size() >= 2) {
        this->bskLogger.bskLog(
            BSK_ERROR,
            "SpacecraftChargingEquilibrium.addSpacecraft only supports two spacecraft (index 0 servicer, index 1 target).");
        return;
    }

    // Add the input spacecraft-state subscriber.
    this->scStateInMsgs.push_back(tmpScMsg->addSubscriber());

    ReadFunctor<ElectronBeamMsgPayload> beamMsg;
    this->eBeamInMsgs.push_back(beamMsg);

    // Create slot for optional sunlit area input/default.
    this->scSunlitAreaInMsgs.emplace_back();
    this->scSunlitAreaDefaults.push_back(std::numeric_limits<double>::quiet_NaN());
    this->scSunlitAreaWarned.push_back(false);

    Eigen::Vector3d zero;
    zero << 0.0, 0.0, 0.0;
    this->r_BN_NList.push_back(zero);
    Eigen::MRPd zeroMRP;
    zeroMRP = zero;
    this->sigma_BNList.push_back(zeroMRP);

    // Create output message objects.
    Message<VoltMsgPayload> *msgVolt;
    msgVolt = new Message<VoltMsgPayload>;
    this->voltOutMsgs.push_back(msgVolt);

    Message<ScChargingCurrentsMsgPayload> *msgCurrent;
    msgCurrent = new Message<ScChargingCurrentsMsgPayload>;
    this->currentsOutMsgs.push_back(msgCurrent);
}

/*!  Read in the input messages
 @return void
 */
void SpacecraftChargingEquilibrium::readMessages()
{
    PlasmaFluxMsgPayload plasmaFluxInMsgBuffer; //!< local copy of plasma flux input message buffer
    SCStatesMsgPayload scStateInMsgsBuffer;     //!< local copy of spacecraft state input message buffer

    plasmaFluxInMsgBuffer = this->plasmaFluxInMsg();
    int validEnergyCount = 0;
    double previousEnergy = 0.0;
    for (int i = 0; i < MAX_PLASMA_FLUX_SIZE; i++) {
        const double energy = plasmaFluxInMsgBuffer.energies[i];
        if (!std::isfinite(energy) || energy <= 0.0) {
            break;
        }
        if (validEnergyCount > 0 && energy <= previousEnergy) {
            break;
        }
        previousEnergy = energy;
        validEnergyCount++;
    }

    if (validEnergyCount < 2) {
        this->bskLogger.bskLog(
            BSK_ERROR,
            "SpacecraftChargingEquilibrium: plasmaFluxInMsg.energies must contain at least 2 strictly increasing positive bins; found %d.",
            validEnergyCount);
        this->energies = Eigen::VectorXd(2);
        this->energies << this->minIntegrationEnergy, this->minIntegrationEnergy + 0.9;
        this->electronFlux = Eigen::VectorXd::Zero(2);
        this->ionFlux = Eigen::VectorXd::Zero(2);
    } else {
        this->energies.resize(validEnergyCount);
        this->electronFlux.resize(validEnergyCount);
        this->ionFlux.resize(validEnergyCount);
        for (int i = 0; i < validEnergyCount; i++) {
            this->energies[i] = plasmaFluxInMsgBuffer.energies[i];
            this->electronFlux[i] = plasmaFluxInMsgBuffer.meanElectronFlux[i];
            this->ionFlux[i] = plasmaFluxInMsgBuffer.meanIonFlux[i];
        }
    }

    for (unsigned int c = 0; c < this->numSat; c++) {
        scStateInMsgsBuffer = this->scStateInMsgs.at(c)();
        this->r_BN_NList.at(c) = cArray2EigenVector3d(scStateInMsgsBuffer.r_BN_N);
        this->sigma_BNList.at(c) = cArray2EigenVector3d(scStateInMsgsBuffer.sigma_BN);
    }
}

/*!  This function takes in a given potential and area value and calculates the electron current
 @return double
 @param phi double defining value for spacecraft potential
 @param A double defining value for area exposed to plasma [m^2]
 */
double SpacecraftChargingEquilibrium::electronCurrent(double phi, double A)
{
    const double constant = -this->elementaryCharge * A; // [A] constant multiplier for integral
    const double maxEnergy = this->energies[this->energies.size() - 1];

    // Integrand for electron collection from energy-shifted ambient electron flux.
    std::function<double(double)> integrand = [&](double E) { return (E / (E - phi)) * getFlux(E - phi, "electron"); };

    double lowerBound = 0.0; // [eV] integration lower bound
    double upperBound = 0.0; // [eV] integration upper bound
    if (phi < 0.) {
        lowerBound = this->minIntegrationEnergy;
        upperBound = maxEnergy;
    } else {
        lowerBound = this->minIntegrationEnergy + std::abs(phi);
        upperBound = maxEnergy + std::abs(phi);
    }

    const double integral = trapz(integrand, lowerBound, upperBound, this->trapzBins);
    const double electronCurrentValue = constant * integral; // [A] collected electron current
    return electronCurrentValue;
}

/*!  This function takes in a given potential and area value and calculates the ion current
 @return double
 @param phi double defining value for spacecraft potential
 @param A double defining value for area exposed to plasma [m^2]
 */
double SpacecraftChargingEquilibrium::ionCurrent(double phi, double A)
{
    const double constant = this->elementaryCharge * A; // [A] constant multiplier for integral
    const double maxEnergy = this->energies[this->energies.size() - 1];

    // Integrand for ion collection from energy-shifted ambient ion flux.
    std::function<double(double)> integrand = [&](double E) { return (E / (E + phi)) * getFlux(E + phi, "ion"); };

    double lowerBound = 0.0; // [eV] integration lower bound
    double upperBound = 0.0; // [eV] integration upper bound
    if (phi > 0.) {
        lowerBound = this->minIntegrationEnergy;
        upperBound = maxEnergy;
    } else {
        lowerBound = this->minIntegrationEnergy + std::abs(phi);
        upperBound = maxEnergy + std::abs(phi);
    }

    const double integral = trapz(integrand, lowerBound, upperBound, this->trapzBins);
    const double ionCurrentValue = constant * integral; // [A] collected ion current
    return ionCurrentValue;
}

/*!  This function takes in a given potential and area value and calculates the SEE current due to electrons
 @return double
 @param phi double defining value for spacecraft potential
 @param A double defining value for area exposed to plasma [m^2]
 */
double SpacecraftChargingEquilibrium::SEEelectronCurrent(double phi, double A)
{
    const double constant = this->elementaryCharge * A; // [A] constant multiplier for integral
    const double maxEnergy = this->energies[this->energies.size() - 1];

    std::function<double(double)> integrand = [&](double E) {
        return getYield(E, "electron") * (E / (E - phi)) * getFlux(E - phi, "electron");
    };

    double lowerBound = 0.0; // [eV] integration lower bound
    double upperBound = 0.0; // [eV] integration upper bound
    if (phi < 0.) {
        lowerBound = this->minIntegrationEnergy;
        upperBound = maxEnergy;
    } else {
        lowerBound = this->minIntegrationEnergy + std::abs(phi);
        upperBound = maxEnergy + std::abs(phi);
    }

    const double integral = trapz(integrand, lowerBound, upperBound, this->trapzBins);
    const double iSEEElectron = constant * integral; // [A] emitted SEE current before recollection

    if (phi <= 0.) {
        return iSEEElectron;
    } else if (phi > 0.) {
        return iSEEElectron * exp(-phi / this->secondaryElectronTemperature);
    } else {
        this->bskLogger.bskLog(BSK_ERROR, "SpacecraftChargingEquilibrium.SEEelectronCurrent: phi not a real number");
        return NAN;
    }
}

/*!  This function takes in a given potential and area value and calculates the SEE current due to ions
 @return double
 @param phi double defining value for spacecraft potential
 @param A double defining value for area exposed to plasma [m^2]
 */
double SpacecraftChargingEquilibrium::SEEionCurrent(double phi, double A)
{
    const double constant = this->elementaryCharge * A; // [A] constant multiplier for integral
    const double maxEnergy = this->energies[this->energies.size() - 1];

    std::function<double(double)> integrand = [&](double E) {
        return getYield(E, "ion") * (E / (E + phi)) * getFlux(E + phi, "ion");
    };

    double lowerBound = 0.0; // [eV] integration lower bound
    double upperBound = 0.0; // [eV] integration upper bound
    if (phi > 0.) {
        lowerBound = this->minIntegrationEnergy;
        upperBound = maxEnergy;
    } else {
        lowerBound = this->minIntegrationEnergy + std::abs(phi);
        upperBound = maxEnergy + std::abs(phi);
    }

    const double integral = trapz(integrand, lowerBound, upperBound, this->trapzBins);
    const double iSEEIon = constant * integral; // [A] emitted SEE current before recollection

    if (phi <= 0.) {
        return iSEEIon;
    } else if (phi > 0.) {
        return iSEEIon * exp(-phi / this->secondaryElectronTemperature);
    } else {
        this->bskLogger.bskLog(BSK_ERROR, "SpacecraftChargingEquilibrium.SEEionCurrent: phi not a real number");
        return NAN;
    }
}

/*!  This function takes in a given potential and area value and calculates the SEE current due to backscattering
 @return double
 @param phi double defining value for spacecraft potential
 @param A double defining value for area exposed to plasma [m^2]
 */
double SpacecraftChargingEquilibrium::backscatteringCurrent(double phi, double A)
{
    const double constant = this->elementaryCharge * A; // [A] constant multiplier for integral
    const double maxEnergy = this->energies[this->energies.size() - 1];

    std::function<double(double)> integrand = [&](double E) {
        return getYield(E, "backscattered") * (E / (E - phi)) * getFlux(E - phi, "electron");
    };

    double lowerBound = 0.0; // [eV] integration lower bound
    double upperBound = 0.0; // [eV] integration upper bound
    if (phi < 0.) {
        lowerBound = this->minIntegrationEnergy;
        upperBound = maxEnergy;
    } else {
        lowerBound = this->minIntegrationEnergy + std::abs(phi);
        upperBound = maxEnergy + std::abs(phi);
    }

    const double integral = trapz(integrand, lowerBound, upperBound, this->trapzBins);
    const double iBackscatter = constant * integral; // [A] emitted backscatter current before recollection

    if (phi <= 0.) {
        return iBackscatter;
    } else if (phi > 0.) {
        return iBackscatter * exp(-phi / this->backscatterElectronTemperature);
    } else {
        this->bskLogger.bskLog(BSK_ERROR, "SpacecraftChargingEquilibrium.backscatteringCurrent: phi not a real number");
        return NAN;
    }
}

/*!  This function takes in a given potential and area value and calculates the current due to the photoelectric effect
 @return double
 @param phi double defining value for spacecraft potential
 @param A double defining value for area exposed to plasma
 */
double SpacecraftChargingEquilibrium::photoelectricCurrent(double phi, double A)
{
    double photoelectricCurrentValue = 0.0; // [A] photoelectric current contribution
    if (phi > 0) {
        photoelectricCurrentValue = this->photoelectronFlux * A * exp(-phi / this->photoelectronTemperature);
    } else if (phi <= 0) {
        photoelectricCurrentValue = this->photoelectronFlux * A;
    } else {
        this->bskLogger.bskLog(BSK_ERROR, "SpacecraftChargingEquilibrium.photoelectricCurrent: phi not a real number");
        photoelectricCurrentValue = NAN;
    }
    return photoelectricCurrentValue;
}


double SpacecraftChargingEquilibrium::electronBeamCurrent(double phiS, double phiT, const std::string& craftType, double EEB, double IEB, double alphaEB)
{
    double iBeamServicer = 0.0; // [A] beam emission current contribution on servicer
    double iBeamTarget = 0.0;   // [A] beam arrival current contribution on target

    if (craftType == "servicer") {
        if (EEB > (phiS - phiT)) {
            iBeamServicer = IEB * (1 - exp(-(EEB - phiS + phiT) / this->beamElectronTemperature));
        } else if (EEB <= (phiS - phiT)) {
            iBeamServicer = 0.;
        } else {
            this->bskLogger.bskLog(BSK_ERROR, "SpacecraftChargingEquilibrium.electronBeamCurrent: EEB not a real number");
            iBeamServicer = NAN;
        }
        return iBeamServicer;
    } else if (craftType == "target") {
        if (EEB > (phiS - phiT)) {
            iBeamTarget = -alphaEB * IEB * (1 - exp(-(EEB - phiS + phiT) / this->beamElectronTemperature));
        } else if (EEB <= (phiS - phiT)) {
            iBeamTarget = 0.;
        } else {
            this->bskLogger.bskLog(BSK_ERROR, "SpacecraftChargingEquilibrium.electronBeamCurrent: EEB not a real number");
            iBeamTarget = NAN;
        }
        return iBeamTarget;
    } else {
        this->bskLogger.bskLog(BSK_ERROR, "SpacecraftChargingEquilibrium.electronBeamCurrent: incorrect craftType. Must specify 'servicer' or 'target'.");
        return NAN;
    }
}

double SpacecraftChargingEquilibrium::SEEelectronBeamCurrent(double phiS, double phiT, double EEB, double IEB, double alphaEB)
{
    const double Eeff = EEB - phiS + phiT; // [eV] effective target landing energy for beam electrons
    const double iBeamTarget = electronBeamCurrent(phiS, phiT, "target", EEB, IEB, alphaEB);
    const double incomingBeamMagnitude = std::max(0.0, -iBeamTarget); // [A] incoming electron current magnitude
    const double iSEEbeam = getYield(Eeff, "electron") * incomingBeamMagnitude;
    return iSEEbeam;
}

double SpacecraftChargingEquilibrium::electronBeamBackscattering(double phiS, double phiT, double EEB, double IEB, double alphaEB)
{
    const double Eeff = EEB - phiS + phiT; // [eV] effective target landing energy for beam electrons
    const double iBeamTarget = electronBeamCurrent(phiS, phiT, "target", EEB, IEB, alphaEB);
    const double incomingBeamMagnitude = std::max(0.0, -iBeamTarget); // [A] incoming electron current magnitude
    const double iBackscatterBeam = getYield(Eeff, "backscattered") * incomingBeamMagnitude;
    return iBackscatterBeam;
}

/*! This function takes in a given set of discrete data points and an x-value
    and performs linear interpolation to compute the corresponding y-value.
    @param xVector vector containing the independent variable data points
    @param yVector vector containing the dependent variable data points
    @param x x-value at which the data is linearly interpolated
    @return interpolated y-value
*/
double SpacecraftChargingEquilibrium::interp(Eigen::VectorXd& xVector, Eigen::VectorXd& yVector, double x)
{
    if (xVector.size() < 2 || yVector.size() < 2 || yVector.size() < xVector.size()) {
        this->bskLogger.bskLog(BSK_ERROR, "SpacecraftChargingEquilibrium.interp: input vectors must have compatible size >= 2.");
        return NAN;
    }

    // find the index corresponding to the first element in xVector that is greater than x
    // (assumes xVector is sorted)
    int idx1 = -1; // initialize as -1 (if no index can be found)
    for (int c = 0; c < xVector.size(); c++){
        if (xVector[c] > x){
            idx1 = c;
            break;
        }
    }
    if (idx1 == -1){
        // if no index can be found, x is greater than last element in xVector. Return last index
        idx1 = xVector.size() - 1;
    }
    else if (idx1 == 0){
        // increase index by one as idx0 = idx1 - 1.
        idx1 = 1;
    }

    // y vector indices and their corresponding y values
    int indX0 = idx1 - 1, indX1 = idx1;
    double y0 = yVector[indX0], y1 = yVector[indX1];

    // linear interpolation formula
    double y = y0 + ((y1-y0)/(xVector[indX1] - xVector[indX0])) * (x - xVector[indX0]);

    return y;
}

/*!  This function computes the integral of the passed function using trapezoidal integration
 @return double
 @param f function to compute the integral of
 @param a lower limit of integration
 @param b upper limit of integration
 @param N number of trapezoids to use
 */
double SpacecraftChargingEquilibrium::trapz(std::function<double(double)>& f, double a, double b, int N)
{
    if (!std::isfinite(a) || !std::isfinite(b) || N <= 0 || b == a) {
        return 0.0;
    }
    const double h = (b - a) / static_cast<double>(N);
    double sum = 0.0;

    for (int i = 1; i < N; ++i) {
        sum += f(a + i * h);          // <-- shift by 'a'
    }
    // trapezoid rule: h * [ 0.5*f(a) + sum + 0.5*f(b) ]
    return h * (0.5 * f(a) + sum + 0.5 * f(b));
}


/*!  This function returns the flux type for a given energy and impacting particle type
 @return double
 @param E energy of interest [eV]
 @param particleType particle of interest ("electron" or "ion")
 */
double SpacecraftChargingEquilibrium::getFlux(double E, const std::string& particleType)
{
    if (particleType == "electron"){
        // find flux for given energy
        double flux = interp(energies, electronFlux, E);
        if (flux < 0.){
            // if flux is negative (due to extrapolation), set equal to zero
            flux = 0.;
        }
        return flux;
    } else if (particleType == "ion"){
        // find flux for given energy
        double flux = interp(energies, ionFlux, E);
        if (flux < 0.){
            // if flux is negative (due to extrapolation), set equal to zero
            flux = 0.;
        }
        return flux;
    } else{
        this->bskLogger.bskLog(BSK_ERROR, "SpacecraftChargingEquilibrium.getFlux: particle must be an electron or ion");
        return NAN;
    }
}

/*!  This function returns the yield for a given energy and yield type
 @return double
 @param E energy of interest [eV]
 @param yieldType yield of interest ("electron", "ion", "backscattered")
 */
double SpacecraftChargingEquilibrium::getYield(double E, const std::string& yieldType)
{
    if (yieldType == "electron"){
        // find yield for given energy
        double yield = interp(energies, yieldSEEelectron, E);
        if (yield < 0.){
            // if yield is negative (due to extrapolation), set equal to zero
            yield = 0.;
        }
        return yield;
    } else if (yieldType == "ion"){
        // find yield for given energy
        double yield = interp(energies, yieldSEEion, E);
        if (yield < 0.){
            // if yield is negative (due to extrapolation), set equal to zero
            yield = 0.;
        }
        return yield;
    } else if (yieldType == "backscattered"){
        // find yield for given energy
        double yield = interp(energies, yieldBackscattered, E);
        if (yield < 0.){
            // if yield is negative (due to extrapolation), set equal to zero
            yield = 0.;
        }
        return yield;
    } else{
        this->bskLogger.bskLog(BSK_ERROR, "SpacecraftChargingEquilibrium.getYield: yield type must be electron, ion, or backscattered");
        return NAN;
    }
}
