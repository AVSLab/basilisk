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

#ifndef SPACECRAFT_CHARGING_DYNAMICS_H
#define SPACECRAFT_CHARGING_DYNAMICS_H

#include "simulation/dynamics/_GeneralModuleFiles/stateData.h"
#include "simulation/dynamics/_GeneralModuleFiles/dynParamManager.h"
#include "simulation/dynamics/_GeneralModuleFiles/dynamicObject.h"
#include "simulation/dynamics/_GeneralModuleFiles/stateVecIntegrator.h"
#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/messaging/messaging.h"
#include "architecture/msgPayloadDefC/CurrentMsgPayload.h"
#include "architecture/msgPayloadDefC/ElectronBeamMsgPayload.h"
#include "architecture/msgPayloadDefC/ProjectedAreaMsgPayload.h"
#include "architecture/msgPayloadDefC/SCStatesMsgPayload.h"
#include "architecture/msgPayloadDefC/VoltMsgPayload.h"
#include "architecture/utilities/bskLogging.h"

/*! @brief spacecraft charging dynamics module */
class SpacecraftChargingDynamics : public DynamicObject{
public:
    SpacecraftChargingDynamics();  //!< Constructor
    ~SpacecraftChargingDynamics() = default;  //!< Destructor
    void initializeDynamics();  //!< Method to initialize dynamics
    void Reset(uint64_t CurrentSimNanos);  //!< Reset method
    void registerStates(DynParamManager& states);  //!< Method to register states
    void writeOutputStateMessages(uint64_t clockTime);  //!< Method to write output messages
    void UpdateState(uint64_t CurrentSimNanos);  //!< Update method
    void equationsOfMotion(double integTimeSeconds, double timeStep);  //!< Method defining equations of motion
    void preIntegration(uint64_t callTimeNanos) final;  //!< Pre-integration method
    void postIntegration(uint64_t callTimeNanos) final;  //!< Post-integration method

    void setServicerPotentialInit(const double potentialInit);  //!< Setter for the initial servicer potential
    void setTargetPotentialInit(const double potentialInit);  //!< Setter for the initial target potential
    double getServicerPotentialInit() const;  //!< Getter for the initial servicer potential
    double getTargetPotentialInit() const;  //!< Getter for the initial target potential
    void setServicerCapacitance(const double capacitance);  //!< Setter for the servicer capacitance
    void setTargetCapacitance(const double capacitance);  //!< Setter for the target capacitance
    double getServicerCapacitance() const;  //!< Getter for the servicer capacitance
    double getTargetCapacitance() const;  //!< Getter for the target capacitance

    void setFluxPhotoelectrons(const double flux);  //!< Setter for the photoelectron flux
    void setTempPhotoelectrons(const double temp);  //!< Setter for the photoelectron temperature
    double getFluxPhotoelectrons() const;  //!< Getter for the photoelectron flux
    double getTempPhotoelectrons() const;  //!< Getter for the photoelectron temperature

    void setTempElectrons(const double temp);  //!< Setter for the electron temperature
    double getTempElectrons() const;  //!< Getter for the electron temperature
    void setDensityElectrons(const double density);  //!< Setter for the electron density
    double getDensityElectrons() const;  //!< Getter for the electron density

    void setTempIons(const double temp);  //!< Setter for the ion temperature
    double getTempIons() const;  //!< Getter for the ion temperature
    void setDensityIons(const double density);  //!< Setter for the ion density
    double getDensityIons() const;  //!< Getter for the ion density
    void setBulkVelocityIons(const double density);  //!< Setter for the bulk ion velocity
    double getBulkVelocityIons() const;  //!< Getter for the bulk ion velocity

    ReadFunctor<SCStatesMsgPayload> servicerStateInMsg;  //!< Servicer state input message
    ReadFunctor<SCStatesMsgPayload> targetStateInMsg;  //!< Target state input message
    ReadFunctor<ElectronBeamMsgPayload> electronBeamInMsg; //!< Electron beam input message (optional)
    ReadFunctor<ProjectedAreaMsgPayload> servicerSurfaceAreaInMsg;  //!< Servicer surface area input message
    ReadFunctor<ProjectedAreaMsgPayload> targetSurfaceAreaInMsg;  //!< Target surface area input message
    ReadFunctor<ProjectedAreaMsgPayload> servicerSunlitAreaInMsg;  //!< Total servicer sunlit facet area input message
    ReadFunctor<ProjectedAreaMsgPayload> targetSunlitAreaInMsg;  //!< Total target sunlit facet area input message

    Message<VoltMsgPayload> servicerPotentialOutMsg;  //!< Servicer potential (voltage) output message
    Message<VoltMsgPayload> targetPotentialOutMsg;  //!< Target potential (voltage) output message
    Message<CurrentMsgPayload> servicerEBCurrentOutMsg;  //!< Servicer electron beam current output message
    Message<CurrentMsgPayload> targetEBCurrentOutMsg;  //!< Target electron beam current output message
    Message<CurrentMsgPayload> servicerPhotoelectricCurrentOutMsg;  //!< Servicer photoelectric current output message
    Message<CurrentMsgPayload> targetPhotoelectricCurrentOutMsg;  //!< Target photoelectric current output message
    Message<CurrentMsgPayload> servicerPlasmaElectronCurrentOutMsg;  //!< Servicer plasma electron current output message
    Message<CurrentMsgPayload> targetPlasmaElectronCurrentOutMsg;  //!< Target plasma electron current output message
    Message<CurrentMsgPayload> servicerPlasmaIonCurrentOutMsg;  //!< Servicer plasma ion current output message
    Message<CurrentMsgPayload> targetPlasmaIonCurrentOutMsg;  //!< Target plasma ion current output message

    BSKLogger bskLogger;  //!< BSK Logging

private:
    double servicerPotentialInit{};  //!< [Volts] Initial servicer potential
    double targetPotentialInit{};  //!< [Volts] Initial target potential
    double servicerCapacitance{1e-9};  //!< [farads] Servicer capacitance
    double targetCapacitance{1e-9};  //!< [farads] Target capacitance
    double servicerSurfaceArea{};  //!< [m^2] Servicer surface area
    double targetSurfaceArea{};  //!< [m^2] Target surface area
    double servicerSunlitArea{};  //!< [m^2] Servicer sunlit area
    double targetSunlitArea{};  //!< [m^2] Target sunlit area

    double tempPhotoelectrons = 2.0;  //!< [eV] Photoelectron temperature
    double fluxPhotoelectrons = 1e-6;  //!< [A/m^2] Photoelectron flux

    double tempElectrons = 2.0;  //!< [eV] Electron temperature
    double densityElectrons = 950000; //!< [m^-3] Electron density

    double tempIons = 2.0;  //!< [eV] Ion temperature
    double densityIons = 950000; //!< [m^-3] Ion density
    double bulkVelocityIons{400000};  //!< [m/s] Bulk ion velocity

    double electronBeamEnergy{};  //!< [eV] Electron beam energy
    double electronBeamCurrent{};  //!< [Amps] Electron beam current
    double alphaEB{};  //!< [-] Scaling term for the fraction of current reaching the target

    double v_SN_N_norm{};  //!< [m/s] Servicer inertial velocity norm
    double v_TN_N_norm{};  //!< [m/s] Target inertial velocity norm

    double servicerPotential{};  //!< [Volts] Servicer potential
    double targetPotential{};  //!< [Volts] Target potential
    double servicerEBCurrent{};  //!< [Amps] Servicer electron beam current
    double targetEBCurrent{};  //!< [Amps] Target electron beam current
    double servicerPhotoelectricCurrent{};  //!< [Amps] Servicer photoelectric current
    double targetPhotoelectricCurrent{};  //!< [Amps] Target photoelectric current
    double servicerPlasmaElectronCurrent{};  //!< [Amps] Servicer plasma electron current
    double targetPlasmaElectronCurrent{};  //!< [Amps] Target plasma electron current
    double servicerPlasmaIonCurrent{};  //!< [Amps] Servicer plasma ion current
    double targetPlasmaIonCurrent{};  //!< [Amps] Target plasma ion current

    void computeCurrents();  //!< Method to compute all electric currents
    void computeElectronBeamCurrent();  //!< Method to compute electron beam current
    void computePhotoelectricCurrent();  //!< Method to compute photoelectric current
    double computePlasmaElectronCurrent(double sunlitArea, double spacecraftPotential);  //!< Method to compute plasma electron current
    double computePlasmaIonCurrent(double surfaceArea, double sunlitArea, double spacecraftPotential, double v_BN_N_norm);  //!< Method to compute plasma ion current

    StateData *servicerPotentialState = nullptr;  //!< State data container for servicer potential
    StateData *targetPotentialState = nullptr;  //!< State data container for target potential
    std::string nameOfServicerPotentialState;  //!< Name of servicer potential state
    std::string nameOfTargetPotentialState;  //!< Name of target potential state
};


#endif /* SPACECRAFT_CHARGING_DYNAMICS_H */
