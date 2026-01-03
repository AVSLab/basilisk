/*
 ISC License

 Copyright (c) 2023, Autonomous Vehicle Systems Lab, University of Colorado Boulder

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


#include "simulation/environment/scCharging/scCharging.h"
#include "architecture/utilities/linearAlgebra.h"
#include <iostream>
#include <iomanip>
#include <cstring>
#include <fstream>
#include <math.h>
#include <algorithm>
#include <string>
#include "architecture/utilities/avsEigenSupport.h"
#include <vector>
#include <cmath>
using ::bisectionSolve;



/*! This is the constructor for the module class.  It sets default variable
    values and initializes the various parts of the model */
ScCharging::ScCharging()
{
}

/*! Module Destructor */
ScCharging::~ScCharging()
{
    /* free up output message objects */
    for (long unsigned int c=0; c<this->voltOutMsgs.size(); c++) {
        delete this->voltOutMsgs.at(c);
    }
}

/*! This method is used to reset the module and checks that required input messages are connected.
 @return void
 @param CurrentSimNanos current simulation time in nano-seconds
*/
void ScCharging::Reset(uint64_t CurrentSimNanos)
{
    // check that required input messages are connected
    if (!this->plasmaFluxInMsg.isLinked())
    {
        bskLogger.bskLog(BSK_ERROR, "ScCharging.plasmaFluxInMsg was not linked.");
    }

    for (long unsigned int c=0; c < this->scStateInMsgs.size(); c++ ){
        if (!this->scStateInMsgs.at(c).isLinked()) {
            bskLogger.bskLog(BSK_ERROR, "ScCharging.scStateInMsgs[%d] was not linked.", c);
        }
    }

    // check for other requirements for this module
    this->numSat = (uint32_t) this->scStateInMsgs.size();
    if (this->numSat < 1) {
        bskLogger.bskLog(BSK_ERROR, "ScCharging must have 1 or more spacecraft added. You added %lu.", this->numSat);
    }
}

/*! This is the main method that gets called every time the module is updated.  Calculates total current and finds equilibrium potential.
 @return void
 @param CurrentSimNanos current simulation time in nano-seconds
*/
void ScCharging::UpdateState(uint64_t CurrentSimNanos)
{
    // read the input messages
    this->readMessages();

    // Diagnostic check for plasma environment
    if (this->energies.size() > 0) {
        std::cout << "\n--- DEBUG START (Time: " << CurrentSimNanos << ") ---" << std::endl;
        std::cout << "Plasma Energies size: " << this->energies.size() << std::endl;
        std::cout << "First Energy Level: " << this->energies[0] << " eV" << std::endl;
        std::cout << "First Electron Flux: " << this->electronFlux[0] << std::endl;
    }

    /* Create and populate all instances of chargedSpaceCraft objects */
    std::vector<chargedSpaceCraft> spaceCrafts(this->numSat);

    // Servicer must be index 0, Target index 1 for coupling logic
    std::string names[] = {"servicer", "target"};

    //Populate SpaceCraft-Beam data from Messages
    for (int i = 0; i < (int)this->numSat; i++) {
        spaceCrafts[i].A = 50.264; // Area will come from message
        spaceCrafts[i].A_sunlit = 25.132; // Sunlit-Area will come from computeSCSunlitFacetArea module
        if (this->eBeamInMsgs[i].isLinked()) {
            ElectronBeamMsgPayload beam = this->eBeamInMsgs[i]();
            spaceCrafts[i].electronGun.currentEB = beam.currentEB;
            spaceCrafts[i].electronGun.energyEB = beam.energyEB;
            spaceCrafts[i].electronGun.alphaEB = beam.alphaEB;
            spaceCrafts[i].emitsEB = true;
            std::cout << "[DEBUG] SC " << i << " Beam linked. Current: "
                      << beam.currentEB << " Energy: " << beam.energyEB << std::endl;
        }
        if (!this->eBeamInMsgs[i].isLinked()){
            spaceCrafts[i].emitsEB = false;
            std::cout << "[DEBUG] SC " << i << " Beam NOT linked." << std::endl;
        }
    }

    std::vector<double> equilibriums(this->numSat, 0.0);
    double bracket[2] = {-400000.0, 400000.0};

    // SOLVE SERVICER POTENTIAL (ID 0)
    int sID = 0; // use index 0 for servicer
    std::function<double(double)> sumCurrentsServicer = [&](double phiS) -> double {
        // Environmental currents (including SEE and Backscatter from plasma)
        double nonBeamCurrents = electronCurrent(phiS, spaceCrafts[sID].A) +
                        ionCurrent(phiS, spaceCrafts[sID].A) +
                        SEEelectronCurrent(phiS, spaceCrafts[sID].A) +
                        SEEionCurrent(phiS, spaceCrafts[sID].A) +
                        backscatteringCurrent(phiS, spaceCrafts[sID].A) +
                        photoelectricCurrent(phiS, spaceCrafts[sID].A_sunlit);

        // Beam emission current from Servicer
        double iBeamS = electronBeamCurrent(phiS, 0.0, "servicer",
                                            spaceCrafts[sID].electronGun.energyEB,
                                            spaceCrafts[sID].electronGun.currentEB,
                                            spaceCrafts[sID].electronGun.alphaEB);

        double totalCurrentS = nonBeamCurrents + iBeamS;

        // Print components and TOTAL for Servicer potential calculation
        std::cout << "  [Servicer Loop] phiS: " << std::setw(10) << phiS
                  << " | EnvI: " << nonBeamCurrents
                  << " | BeamI: " << iBeamS
                  << " | TOTAL: " << totalCurrentS << std::endl;

        return totalCurrentS;
    };
    equilibriums[0] = bisectionSolve(bracket, 1e-6, sumCurrentsServicer);

    // SOLVE TARGET POTENTIAL (ID 1)
    int tID = 1; // use index 1 for target
    double fixedPhiS = equilibriums[0]; // use the solved servicer potential

    std::function<double(double)> sumCurrentsTarget = [&](double phiT) -> double {
        // Environmental currents
        double nonBeamCurrents = electronCurrent(phiT, spaceCrafts[tID].A) +
                        ionCurrent(phiT, spaceCrafts[tID].A) +
                        SEEelectronCurrent(phiT, spaceCrafts[tID].A) +
                        SEEionCurrent(phiT, spaceCrafts[tID].A) +
                        backscatteringCurrent(phiT, spaceCrafts[tID].A) +
                        photoelectricCurrent(phiT, spaceCrafts[tID].A_sunlit);

        // Beam current arriving at target (Negative current)
        // Note: sID is still 0 here because the beam parameters belong to the Servicer
        double iBeamT = electronBeamCurrent(fixedPhiS, phiT, "target",
                                            spaceCrafts[sID].electronGun.energyEB,
                                            spaceCrafts[sID].electronGun.currentEB,
                                            spaceCrafts[sID].electronGun.alphaEB);

        // SEE and Backscatter caused by the beam (Positive currents)
        double iSEE_EB = SEEelectronBeamCurrent(fixedPhiS, phiT,
                                                spaceCrafts[sID].electronGun.energyEB,
                                                spaceCrafts[sID].electronGun.currentEB,
                                                spaceCrafts[sID].electronGun.alphaEB);

        double iBS_EB = electronBeamBackscattering(fixedPhiS, phiT,
                                                   spaceCrafts[sID].electronGun.energyEB,
                                                   spaceCrafts[sID].electronGun.currentEB,
                                                   spaceCrafts[sID].electronGun.alphaEB);

        double totalCurrentT = nonBeamCurrents + iBeamT + iSEE_EB + iBS_EB;

        // Print components and TOTAL for Target potential calculation
        std::cout << "  [Target Loop]   phiT: " << std::setw(10) << phiT
                  << " | EnvI: " << nonBeamCurrents
                  << " | BeamArrivalI: " << iBeamT
                  << " | BeamSEE: " << iSEE_EB
                  << " | TOTAL: " << totalCurrentT << std::endl;

        return totalCurrentT;
    };
    equilibriums[1] = bisectionSolve(bracket, 1e-8, sumCurrentsTarget);

    std::cout << "FINAL Result -> Servicer: " << equilibriums[0] << " V | Target: " << equilibriums[1] << " V" << std::endl;
    std::cout << "--- DEBUG END ---\n" << std::endl;

    // WRITE MESSAGES
    for (int i = 0; i < (int)this->numSat; i++) {
        VoltMsgPayload voltMsgBuffer;
        voltMsgBuffer.voltage = equilibriums[i];
        this->voltOutMsgs.at(i)->write(&voltMsgBuffer, this->moduleID, CurrentSimNanos);

        /* KEEP THESE FOR MESSAGING SYSTEM LATER */
        // CurrentMsgPayload peMsg = {};
        // peMsg.current = photoelectricCurrent(equilibriums[i], spaceCrafts[i].A_sunlit);
        // this->photoelectricOutMsgs.at(i)->write(&peMsg, this->moduleID, CurrentSimNanos);
    }
}

/*!   Add spacecraft to charging module
 @return void
 @param tmpScMsg spacecraft state input message
 */
void ScCharging::addSpacecraft(Message<SCStatesMsgPayload> *tmpScMsg)
{
    /* add the message reader to the vector of input spacecraft state messages */
    this->scStateInMsgs.push_back(tmpScMsg->addSubscriber());

    ReadFunctor<ElectronBeamMsgPayload> beamMsg;
    this->eBeamInMsgs.push_back(beamMsg);
    Eigen::Vector3d zero;
    zero << 0.0, 0.0, 0.0;
    this->r_BN_NList.push_back(zero);
    Eigen::MRPd zeroMRP;
    zeroMRP = zero;
    this->sigma_BNList.push_back(zeroMRP);

    /* create output message objects */
    Message<VoltMsgPayload> *msgVolt;
    msgVolt = new Message<VoltMsgPayload>;
    this->voltOutMsgs.push_back(msgVolt);
}

/*!  Read in the input messages
 @return void
 */
void ScCharging::readMessages()
{
    PlasmaFluxMsgPayload PlasmaFluxInMsgBuffer; //!< local copy of plasma flux input message buffer
    SCStatesMsgPayload scStateInMsgsBuffer;     //!< local copy of spacecraft state input message buffer
    long unsigned int c;                        //!< spacecraft loop counter

    PlasmaFluxInMsgBuffer = this->plasmaFluxInMsg();
    this->energies = cArray2EigenMatrixXd(PlasmaFluxInMsgBuffer.energies,MAX_PLASMA_FLUX_SIZE,1);
    this->electronFlux = cArray2EigenMatrixXd(PlasmaFluxInMsgBuffer.meanElectronFlux,MAX_PLASMA_FLUX_SIZE,1);
    this->ionFlux = cArray2EigenMatrixXd(PlasmaFluxInMsgBuffer.meanIonFlux,MAX_PLASMA_FLUX_SIZE,1);

    for (c = 0; c < this->numSat; c++) {
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
double ScCharging::electronCurrent(double phi, double A)
{
    double constant = -Q0 * A; // constant multiplier for integral

    // term to be integrated by trapz
    std::function<double(double)> integrand = [&](double E){return (E/(E - phi)) * getFlux(E - phi, "electron");};

    // integral bounds
    double lowerBound;
    double upperBound;
    if (phi < 0.){
        lowerBound = 0.1;
        upperBound = energies[MAX_PLASMA_FLUX_SIZE - 1];
    } else{
        lowerBound = 0.1 + abs(phi);
        upperBound = energies[MAX_PLASMA_FLUX_SIZE - 1] + abs(phi);
    }

    // integral calculated with trapz
    double integral = trapz(integrand, lowerBound, upperBound, TRAPZN);

    double Ie = constant * integral;
    return Ie;
}

/*!  This function takes in a given potential and area value and calculates the ion current
 @return double
 @param phi double defining value for spacecraft potential
 @param A double defining value for area exposed to plasma [m^2]
 */
double ScCharging::ionCurrent(double phi, double A)
{
    double constant = Q0 * A; // constant multiplier for integral

    // term to be integrated by trapz
    std::function<double(double)> integrand = [&](double E){return (E/(E + phi)) * getFlux(E + phi, "ion");};

    // integral bounds
    double lowerBound;
    double upperBound;
    if (phi > 0.){
        lowerBound = 0.1;
        upperBound = energies[MAX_PLASMA_FLUX_SIZE - 1];
    } else{
        lowerBound = 0.1 + abs(phi);
        upperBound = energies[MAX_PLASMA_FLUX_SIZE - 1] + abs(phi);
    }

    // integral calculated with trapz
    double integral = trapz(integrand, lowerBound, upperBound, TRAPZN);

    double Ii = constant * integral;
    return Ii;
}

/*!  This function takes in a given potential and area value and calculates the SEE current due to electrons
 @return double
 @param phi double defining value for spacecraft potential
 @param A double defining value for area exposed to plasma [m^2]
 */
double ScCharging::SEEelectronCurrent(double phi, double A)
{
    double constant = Q0 * A; // constant multiplier for integral

    // term to be integrated by trapz
    std::function<double(double)> integrand = [&](double E){return getYield(E, "electron") * (E/(E - phi)) * getFlux(E - phi, "electron");};

    // integral bounds
    double lowerBound;
    double upperBound;
    if (phi < 0.){
        lowerBound = 0.1;
        upperBound = energies[MAX_PLASMA_FLUX_SIZE - 1];
    } else{
        lowerBound = 0.1 + abs(phi);
        upperBound = energies[MAX_PLASMA_FLUX_SIZE - 1] + abs(phi);
    }

    // integral calculated with trapz
    double integral = trapz(integrand, lowerBound, upperBound, TRAPZN);

    // current before debris charge taken into account
    double ISEEe = constant * integral;

    // check how debris potential affects ISEEe
    if (phi <= 0.){
        return ISEEe;
    } else if (phi > 0.){
        return ISEEe * exp(-phi / Tsee);
    } else{
        bskLogger.bskLog(BSK_ERROR, "ScCharging.SEEelectronCurrent: phi not a real number");
        return NAN;
    }
}

/*!  This function takes in a given potential and area value and calculates the SEE current due to ions
 @return double
 @param phi double defining value for spacecraft potential
 @param A double defining value for area exposed to plasma [m^2]
 */
double ScCharging::SEEionCurrent(double phi, double A)
{
    double constant = Q0 * A; // constant multiplier for integral

    // term to be integrated by trapz
    std::function<double(double)> integrand = [&](double E){return getYield(E, "ion") * (E/(E + phi)) * getFlux(E + phi, "ion");};

    // integral bounds
    double lowerBound;
    double upperBound;
    if (phi > 0.){
        lowerBound = 0.1;
        upperBound = energies[MAX_PLASMA_FLUX_SIZE - 1];
    } else{
        lowerBound = 0.1 + abs(phi);
        upperBound = energies[MAX_PLASMA_FLUX_SIZE - 1] + abs(phi);
    }

    // integral calculated with trapz
    double integral = trapz(integrand, lowerBound, upperBound, TRAPZN);

    // current before debris charge taken into account
    double ISEEi = constant * integral;

    // check how debris potential affects ISEEi
    if (phi <= 0.){
        return ISEEi;
    } else if (phi > 0.){
        return ISEEi * exp(-phi / Tsee);
    } else{
        bskLogger.bskLog(BSK_ERROR, "ScCharging.SEEionCurrent: phi not a real number");
        return NAN;
    }
}

/*!  This function takes in a given potential and area value and calculates the SEE current due to backscattering
 @return double
 @param phi double defining value for spacecraft potential
 @param A double defining value for area exposed to plasma [m^2]
 */
double ScCharging::backscatteringCurrent(double phi, double A)
{
    double constant = Q0 * A; // constant multiplier for integral

    // term to be integrated by trapz
    std::function<double(double)> integrand = [&](double E){return getYield(E, "backscattered") * (E/(E - phi)) * getFlux(E - phi, "electron");};

    // integral bounds
    double lowerBound;
    double upperBound;
    if (phi < 0.){
        lowerBound = 0.1;
        upperBound = energies[MAX_PLASMA_FLUX_SIZE - 1];
    } else{
        lowerBound = 0.1 + abs(phi);
        upperBound = energies[MAX_PLASMA_FLUX_SIZE - 1] + abs(phi);
    }

    // integral calculated with trapz
    double integral = trapz(integrand, lowerBound, upperBound, TRAPZN);

    // current before debris charge taken into account
    double Ibs = constant * integral;

    // check how debris potential affects Ibs
    if (phi <= 0.){
        return Ibs;
    } else if (phi > 0.){
        return Ibs * exp(-phi / Tsee);
    } else{
        bskLogger.bskLog(BSK_ERROR, "ScCharging.backscatteringCurrent: phi not a real number");
        return NAN;
    }
}

/*!  This function takes in a given potential and area value and calculates the current due to the photoelectric effect
 @return double
 @param phi double defining value for spacecraft potential
 @param A double defining value for area exposed to plasma
 */
double ScCharging::photoelectricCurrent(double phi, double A)
{
    double Ip;
    if (phi > 0){
        Ip = Jph * A * exp(-phi / kTph);
    } else if (phi <= 0){
        Ip = Jph * A;
    } else {
        bskLogger.bskLog(BSK_ERROR, "ScCharging.photoelectricCurrent: phi not a real number");
        Ip = NAN;
    }
    return Ip;
}


double ScCharging::electronBeamCurrent(double phiS, double phiT, std::string craftType, double EEB, double IEB, double alphaEB)
{
    double IEBs, IEBt;
    // find respective craft type's current due to electron beam
    if (craftType == "servicer"){   // servicer current
        if (EEB > (phiS - phiT)){
            IEBs = IEB;
        } else if (EEB <= (phiS - phiT)){
            IEBs = 0.;
        } else {
            bskLogger.bskLog(BSK_ERROR, "ScCharging.electronBeamCurrent: EEB not a real number");
            IEBs = NAN;
        }
        return IEBs;
    } else if (craftType == "target"){  // target current
        if (EEB > (phiS - phiT)){
            IEBt = -alphaEB * IEB;
        } else if (EEB <= (phiS - phiT)){
            IEBt = 0.;
        } else {
            bskLogger.bskLog(BSK_ERROR, "ScCharging.electronBeamCurrent: EEB not a real number");
            IEBt = NAN;
        }
        return IEBt;
    } else {
        bskLogger.bskLog(BSK_ERROR, "ScCharging.electronBeamCurrent: incorrect craftType."
                         "Must specify 'servicer' or 'target'");
        return NAN;
    }
}

double ScCharging::SEEelectronBeamCurrent(double phiS, double phiT, double EEB, double IEB, double alphaEB)
{
    double Eeff = EEB - phiS + phiT;
    double ISEEEB = getYield(Eeff, "electron") * electronBeamCurrent(phiS, phiT, "target", EEB, IEB, alphaEB);
    return ISEEEB;
}

double ScCharging::electronBeamBackscattering(double phiS, double phiT, double EEB, double IEB, double alphaEB)
{
    double Eeff = EEB - phiS + phiT;
    double IbsEB = getYield(Eeff, "backscattered") * electronBeamCurrent(phiS, phiT, "target", EEB, IEB, alphaEB);
    return IbsEB;
}

/*!  This function takes in a given vector of data and an x-value and performs linear interpolation to find the closest corresponding y-value
 @return double
 @param data vector containing datapoints to use in linear interpolation (y-values)
 @param x x-value being linear interpolated to
 */
double ScCharging::interp(Eigen::VectorXd& xVector, Eigen::VectorXd& yVector, double x)
{
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
double ScCharging::trapz(std::function<double(double)>& f, double a, double b, int N)
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
double ScCharging::getFlux(double E, std::string particleType)
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
        bskLogger.bskLog(BSK_ERROR, "ScCharging.getFlux: particle must be an electron or ion");
        return NAN;
    }
}

/*!  This function returns the yield for a given energy and yield type
 @return double
 @param E energy of interest [eV]
 @param yieldType yield of interest ("electron", "ion", "backscattered")
 */
double ScCharging::getYield(double E, std::string yieldType)
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
        bskLogger.bskLog(BSK_ERROR, "ScCharging.getYield: yield type must be electron, ion, or backscattered");
        return NAN;
    }
}
