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

#ifndef SPACECRAFT_CHARGING_H
#define SPACECRAFT_CHARGING_H

#include "simulation/dynamics/_GeneralModuleFiles/stateData.h"
#include "simulation/dynamics/_GeneralModuleFiles/dynParamManager.h"
#include "simulation/dynamics/_GeneralModuleFiles/dynamicObject.h"
#include "simulation/dynamics/_GeneralModuleFiles/stateVecIntegrator.h"
#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/messaging/messaging.h"
#include "architecture/msgPayloadDefC/CurrentMsgPayload.h"
#include "architecture/msgPayloadDefC/ProjectedAreaMsgPayload.h"
#include "architecture/msgPayloadDefC/VoltMsgPayload.h"
#include "architecture/utilities/bskLogging.h"

/*! @brief spacecraft charging module */
class SpacecraftCharging : public DynamicObject{
public:
    SpacecraftCharging();
    ~SpacecraftCharging() = default;
    void initializeDynamics();
    void Reset(uint64_t CurrentSimNanos);
    void registerStates(DynParamManager& states);
    void writeOutputStateMessages(uint64_t clockTime);
    void UpdateState(uint64_t CurrentSimNanos);
    void equationsOfMotion(double integTimeSeconds, double timeStep);
    void computePhotoelectricCurrent();
    void preIntegration(uint64_t callTimeNanos) final;
    void postIntegration(uint64_t callTimeNanos) final;

    void setServicerCapacitance(const double capacitance);  //!< Setter for the servicer capacitance
    void setTargetCapacitance(const double capacitance);  //!< Setter for the target capacitance
    double getServicerCapacitance() const;  //!< Getter for the servicer capacitance
    double getTargetCapacitance() const;  //!< Getter for the target capacitance

    void setFluxPhotoelectrons(const double flux);  //!< Setter for the photoelectron flux
    void setTempPhotoelectrons(const double temp);  //!< Setter for the photoelectron temperature
    double getFluxPhotoelectrons() const;  //!< Getter for the photoelectron flux
    double getTempPhotoelectrons() const;  //!< Getter for the photoelectron temperature

    ReadFunctor<ProjectedAreaMsgPayload> servicerSunlitAreaInMsg;  //!< Total servicer sunlit facet area input message
    ReadFunctor<ProjectedAreaMsgPayload> targetSunlitAreaInMsg;  //!< Total target sunlit facet area input message
    Message<VoltMsgPayload> servicerPotentialOutMsg;  //!< Servicer potential (voltage) output message
    Message<VoltMsgPayload> targetPotentialOutMsg;  //!< Target potential (voltage) output message
    Message<CurrentMsgPayload> servicerPhotoelectricCurrentOutMsg;  //!< Servicer photoelectric current output message
    Message<CurrentMsgPayload> targetPhotoelectricCurrentOutMsg;  //!< Target photoelectric current output message

    BSKLogger bskLogger;  //!< BSK Logging

private:
    double servicerCapacitance{};  //!< [farads] Servicer capacitance
    double targetCapacitance{};  //!< [farads] Target capacitance
    double servicerPotentialInit{};  //!< [Volts] Initial servicer potential
    double targetPotentialInit{};  //!< [Volts] Initial target potential
    double servicerSunlitArea{};  //!< [m^2] Servicer sunlit area
    double targetSunlitArea{};  //!< [m^2] Target sunlit area

    double tempPhotoelectrons = 2.0;  //!< [eV] Photoelectron temperature
    double fluxPhotoelectrons = 1e-6;  //!< [A/m^2] Photoelectron flux

    double servicerPotential{};  //!< [Volts] Servicer potential
    double targetPotential{};  //!< [Volts] Target potential
    double servicerPhotoelectricCurrent{};  //!< [Amps] Servicer photoelectric current
    double targetPhotoelectricCurrent{};  //!< [Amps] Target photoelectric current

    StateData *servicerPotentialState = nullptr;  //!< State data container for servicer potential
    StateData *targetPotentialState = nullptr;  //!< State data container for target potential
    std::string nameOfServicerPotentialState;  //!< Name of servicer potential state
    std::string nameOfTargetPotentialState;  //!< Name of target potential state
};


#endif /* SPACECRAFT_CHARGING_H */
