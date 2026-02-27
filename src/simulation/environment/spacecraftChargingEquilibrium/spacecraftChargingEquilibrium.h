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

#ifndef SPACECRAFT_CHARGING_EQUILIBRIUM_H
#define SPACECRAFT_CHARGING_EQUILIBRIUM_H

#include <functional>
#include <limits>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/messaging/messaging.h"
#include "architecture/msgPayloadDefC/ElectronBeamMsgPayload.h"
#include "architecture/msgPayloadDefC/PlasmaFluxMsgPayload.h"
#include "architecture/msgPayloadDefC/SCStatesMsgPayload.h"
#include "architecture/msgPayloadDefC/ScSunlitFacetAreaMsgPayload.h"
#include "architecture/msgPayloadDefC/ScChargingCurrentsMsgPayload.h"
#include "architecture/msgPayloadDefC/VoltMsgPayload.h"
#include "architecture/utilities/avsEigenSupport.h"
#include "architecture/utilities/bskLogging.h"

/*! @brief This module computes coupled equilibrium electric potentials for exactly two spacecraft
           (servicer index 0 and target index 1).
 */
class SpacecraftChargingEquilibrium : public SysModel {
public:
    /*! Constructor. */
    SpacecraftChargingEquilibrium();
    /*! Destructor. */
    ~SpacecraftChargingEquilibrium();

    /*! Reset module state and validate required inputs. */
    void Reset(uint64_t CurrentSimNanos) override;
    /*! Update module outputs by solving coupled equilibrium potentials. */
    void UpdateState(uint64_t CurrentSimNanos) override;

    /*! Add one spacecraft state input. Exactly two spacecraft are required.
        @param tmpScMsg spacecraft state message (index 0: servicer, index 1: target)
    */
    void addSpacecraft(Message<SCStatesMsgPayload> *tmpScMsg);

    /*! Enable or disable verbose debug prints in UpdateState(). */
    void setEnableDebugPrints(bool enabled);
    /*! Return whether verbose debug prints are enabled. */
    bool getEnableDebugPrints() const;

    /*! Set electron-impact SEE yield table aligned with the plasma energy grid. */
    void setYieldSEEelectron(const Eigen::VectorXd& yieldVector);
    /*! Set ion-impact SEE yield table aligned with the plasma energy grid. */
    void setYieldSEEion(const Eigen::VectorXd& yieldVector);
    /*! Set backscatter yield table aligned with the plasma energy grid. */
    void setYieldBackscattered(const Eigen::VectorXd& yieldVector);
    /*! Get the configured electron-impact SEE yield table. */
    Eigen::VectorXd getYieldSEEelectron() const;
    /*! Get the configured ion-impact SEE yield table. */
    Eigen::VectorXd getYieldSEEion() const;
    /*! Get the configured backscatter yield table. */
    Eigen::VectorXd getYieldBackscattered() const;

    /*! Set optional default sunlit area [m^2] for one spacecraft index. */
    void setSunlitAreaDefault(unsigned int spacecraftIndex, double sunlitArea);
    /*! Get optional default sunlit area [m^2] for one spacecraft index. */
    double getSunlitAreaDefault(unsigned int spacecraftIndex) const;
    /*! Clear optional default sunlit area for one spacecraft index. */
    void clearSunlitAreaDefault(unsigned int spacecraftIndex);

    /*! Set fallback sunlit area [m^2] used when message/default are not available. */
    void setSunlitAreaFallback(double sunlitArea);
    /*! Get fallback sunlit area [m^2]. */
    double getSunlitAreaFallback() const;

    /*! Set lower/upper [V] root-bracket bounds used by bisection solves. */
    void setRootSolveBounds(double lowerBound, double upperBound);
    /*! Get lower [V] root-bracket bound used by bisection solves. */
    double getRootSolveLowerBound() const;
    /*! Get upper [V] root-bracket bound used by bisection solves. */
    double getRootSolveUpperBound() const;

    /*! Set default plasma-exposed spacecraft area [m^2]. */
    void setSurfaceAreaDefault(double area);
    /*! Get default plasma-exposed spacecraft area [m^2]. */
    double getSurfaceAreaDefault() const;

    /*! Set nominal photoelectron flux [A/m^2]. */
    void setPhotoelectronFlux(double photoelectronFluxIn);
    /*! Get nominal photoelectron flux [A/m^2]. */
    double getPhotoelectronFlux() const;

    /*! Set photoelectron characteristic energy [eV]. */
    void setPhotoelectronTemperature(double photoelectronTemperatureIn);
    /*! Get photoelectron characteristic energy [eV]. */
    double getPhotoelectronTemperature() const;

    /*! Set secondary-electron characteristic energy [eV]. */
    void setSecondaryElectronTemperature(double secondaryElectronTemperatureIn);
    /*! Get secondary-electron characteristic energy [eV]. */
    double getSecondaryElectronTemperature() const;

    /*! Set backscatter-electron characteristic energy [eV]. */
    void setBackscatterElectronTemperature(double backscatterElectronTemperatureIn);
    /*! Get backscatter-electron characteristic energy [eV]. */
    double getBackscatterElectronTemperature() const;

    /*! Set beam electron characteristic energy [eV] for exponential turn-on. */
    void setBeamElectronTemperature(double beamElectronTemperatureIn);
    /*! Get beam electron characteristic energy [eV] for exponential turn-on. */
    double getBeamElectronTemperature() const;

    /*! Set number of trapezoids used by numerical integration. */
    void setTrapzBins(int trapzBinsIn);
    /*! Get number of trapezoids used by numerical integration. */
    int getTrapzBins() const;

    // Messaging
    std::vector<ReadFunctor<SCStatesMsgPayload>> scStateInMsgs;                 //!< spacecraft state input messages
    ReadFunctor<PlasmaFluxMsgPayload> plasmaFluxInMsg;                           //!< plasma flux input message
    std::vector<ReadFunctor<ElectronBeamMsgPayload>> eBeamInMsgs;                //!< optional electron beam parameters
    std::vector<ReadFunctor<ScSunlitFacetAreaMsgPayload>> scSunlitAreaInMsgs;    //!< optional sunlit area input messages

    std::vector<Message<VoltMsgPayload>*> voltOutMsgs;                           //!< spacecraft equilibrium potential output messages
    std::vector<Message<ScChargingCurrentsMsgPayload>*> currentsOutMsgs;          //!< spacecraft current-term output messages

    BSKLogger bskLogger;                                                         //!< Basilisk logging interface

private:
    struct ElectronGunConfig {
        double alphaEB = 1.0;                                                    //!< [-] fraction of beam current reaching target
        double currentEB = 0.0;                                                  //!< [A] beam current magnitude
        double energyEB = 0.0;                                                   //!< [eV] beam energy
    };

    struct ChargingSpacecraftState {
        double A = 0.0;                                                          //!< [m^2] plasma-exposed area
        double A_sunlit = 0.0;                                                   //!< [m^2] sunlit area used for photoelectric current
        bool emitsEB = false;                                                    //!< true if e-beam message is linked for this spacecraft
        ElectronGunConfig electronGun;                                           //!< e-beam configuration for this spacecraft
    };

    void readMessages();
    bool validateSolveConfiguration();

    double electronCurrent(double phi, double A);
    double ionCurrent(double phi, double A);
    double SEEelectronCurrent(double phi, double A);
    double SEEionCurrent(double phi, double A);
    double backscatteringCurrent(double phi, double A);
    double photoelectricCurrent(double phi, double A);
    double electronBeamCurrent(double phiS, double phiT, const std::string& craftType,
                               double EEB, double IEB, double alphaEB);
    double SEEelectronBeamCurrent(double phiS, double phiT,
                                  double EEB, double IEB, double alphaEB);
    double electronBeamBackscattering(double phiS, double phiT,
                                      double EEB, double IEB, double alphaEB);
    double interp(Eigen::VectorXd& xVector, Eigen::VectorXd& yVector, double x);
    double trapz(std::function<double(double)>& f, double a, double b, int N);
    double getFlux(double E, const std::string& particleType);
    double getYield(double E, const std::string& yieldType);

    std::vector<Eigen::Vector3d> r_BN_NList;                                     //!< [m] inertial position vectors for each spacecraft
    std::vector<Eigen::MRPd> sigma_BNList;                                       //!< [-] MRP attitude vectors for each spacecraft
    Eigen::VectorXd energies;                                                    //!< [eV] plasma energy bins
    Eigen::VectorXd electronFlux;                                                //!< [cm^-2 s^-1 sr^-2 eV^-1] ambient electron flux spectrum
    Eigen::VectorXd ionFlux;                                                     //!< [cm^-2 s^-1 sr^-2 eV^-1] ambient ion flux spectrum
    unsigned int numSat = 0;                                                     //!< number of spacecraft configured through addSpacecraft()

    Eigen::VectorXd yieldSEEelectron;                                            //!< [-] electron-impact SEE yield table
    Eigen::VectorXd yieldSEEion;                                                 //!< [-] ion-impact SEE yield table
    Eigen::VectorXd yieldBackscattered;                                          //!< [-] backscatter yield table

    std::vector<double> scSunlitAreaDefaults;                                    //!< [m^2] optional per-spacecraft sunlit-area defaults
    std::vector<bool> scSunlitAreaWarned;                                        //!< warn-once latch for missing sunlit area
    bool enableDebugPrints = false;                                              //!< toggles verbose UpdateState diagnostics

    // Configurable model parameters
    double elementaryCharge = 1.60217663e-19;                                    //!< [C] elementary charge
    int trapzBins = 1000;                                                        //!< [-] trapezoids used by trapz integration
    double minIntegrationEnergy = 0.1;                                           //!< [eV] lower baseline bound used in current integrals
    double photoelectronFlux = 40e-6;                                            //!< [A/m^2] photoelectron current density at non-positive potential
    double photoelectronTemperature = 2.0;                                       //!< [eV] photoelectron characteristic energy
    double secondaryElectronTemperature = 5.0;                                   //!< [eV] secondary-electron emission characteristic energy
    double backscatterElectronTemperature = 5.0;                                 //!< [eV] backscatter emission characteristic energy
    double beamElectronTemperature = 20.0;                                       //!< [eV] beam turn-on characteristic energy

    double defaultSurfaceArea = 50.264;                                          //!< [m^2] default spacecraft plasma-exposed area
    double sunlitAreaFallback = 0.0;                                             //!< [m^2] fallback sunlit area when neither message nor default exists
    double rootSolveLowerBound = -400000.0;                                      //!< [V] lower bracket for bisection equilibrium solve
    double rootSolveUpperBound = 400000.0;                                       //!< [V] upper bracket for bisection equilibrium solve
    double servicerSolveAccuracy = 1e-6;                                         //!< [V] bisection tolerance for servicer equilibrium
    double targetSolveAccuracy = 1e-8;                                           //!< [V] bisection tolerance for target equilibrium
};

#endif
