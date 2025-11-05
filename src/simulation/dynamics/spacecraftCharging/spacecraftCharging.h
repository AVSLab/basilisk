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
#include "architecture/msgPayloadDefC/VoltMsgPayload.h"
#include "architecture/msgPayloadDefC/SCSunlitFacetAreaMsgPayload.h"

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
    void preIntegration(uint64_t callTimeNanos) final;
    void postIntegration(uint64_t callTimeNanos) final;

    void setEBeamCurrent(const double I_eBeam);  //!< Setter for the electron beam current
    void setEBeamEnergy(const double E_eBeam);  //!< Setter for the electron beam energy
    void setServicerCapacitance(const double capacitance);  //!< Setter for the servicer spacecraft capacitance
    void setTargetCapacitance(const double capacitance);  //!< Setter for the target spacecraft capacitance
    double getEBeamCurrent() const;  //!< Getter for the electron beam current
    double getEBeamEnergy() const;  //!< Getter for the electron beam energy
    double getServicerCapacitance() const;  //!< Getter for the servicer spacecraft capacitance
    double getTargetCapacitance() const;  //!< Getter for the target spacecraft capacitance

    ReadFunctor<SCSunlitFacetAreaMsgPayload> servicerSunlitAreaInMsg;  //!< Total servicer sunlit facet area input message
    ReadFunctor<SCSunlitFacetAreaMsgPayload> targetSunlitAreaInMsg;  //!< Total target sunlit facet area input message
    Message<VoltMsgPayload> servicerPotentialOutMsg;     //!< Servicer spacecraft potential (voltage) output message
    Message<VoltMsgPayload> targetPotentialOutMsg;     //!< Target spacecraft potential (voltage) output message

private:
    double E_eBeam;  //!< [keV] Electron beam energy
    double I_eBeam;  //!< [Amps] Electron beam current

    double servicerCapacitance;  //!< [farads] Servicer spacecraft capacitance
    double servicerSunlitArea;  //!< [m^2] Servicer sunlit area
    std::string nameOfServicerPotentialState;
    double servicerPotentialInit{};  //!< [Volts] Initial servicer spacecraft potential
    double servicerPotential{};  //!< [Volts] Servicer spacecraft potential
    StateData *servicerPotentialState = nullptr;  //!< State data container for servicer spacecraft potential

    double targetCapacitance{};  //!< [farads] Target spacecraft capacitance
    double targetSunlitArea;  //!< [m^2] Target sunlit area
    std::string nameOfTargetPotentialState;
    double targetPotentialInit{};  //!< [Volts] Initial target spacecraft potential
    double targetPotential{};  //!< [Volts] Target spacecraft potential
    StateData *targetPotentialState = nullptr;  //!< State data container for target spacecraft potential
};


#endif /* SPACECRAFT_CHARGING_H */
