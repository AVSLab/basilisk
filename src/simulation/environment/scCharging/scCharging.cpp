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
    
    // define area and interval
    double a = 12.566370614359172;
    double interval [2] = {-1e8, 1e8};
    
    /* Create and populate all instances of chargedSpaceCraft objects */
    chargedSpaceCraft spaceCrafts[this->numSat];
    
    // define spacecraft member values (user inputs)
    std::string names[] = {"target", "servicer"}; //!< all craft names
    std::string electronGunCrafts[] = {"servicer"};   //!< names off all e- gun equipped craft
    double alphaEB = 1.1;   //!< electron gun variable
    double currentEB = 4.5; //!< electron gun variable
    double energyEB = 3.3;  //!< electron gun variable
    double electronGunParameters[][3] = {{alphaEB, currentEB, energyEB}};
    int priorities[] = {1, 2};
    double Avec[] = {a, a*10};
    double A_sunlitVec[] = {a/2, a};
    
    // populate object members
    for (int w = 0; w < this->numSat; w++) {
        // assign each object an ID
        spaceCrafts[w].setID("ID", w);
        
        // give object members values as defined by user
        spaceCrafts[w].name = names[w];
        spaceCrafts[w].priority = priorities[w];
        spaceCrafts[w].A = Avec[w];
        spaceCrafts[w].A_sunlit = A_sunlitVec[w];
        
        // fill out electron gun values for objects that emit EB
        for (int p = 0; p < (sizeof(electronGunCrafts) / sizeof(electronGunCrafts[0])); p++) {
            // populates EB values if has electron gun, otherwise sets all values to NAN
            if (names[w] == electronGunCrafts[p]) {
                spaceCrafts[w].setID("electronGunID", spaceCrafts[w].getID("ID"));
                spaceCrafts[w].emitsEB = true;
                spaceCrafts[w].electronGun.alphaEB = electronGunParameters[w][0];
                spaceCrafts[w].electronGun.currentEB = electronGunParameters[w][1];
                spaceCrafts[w].electronGun.energyEB = electronGunParameters[w][2];
            } else {
                spaceCrafts[w].emitsEB = false;
                spaceCrafts[w].electronGun.alphaEB = NAN;
                spaceCrafts[w].electronGun.currentEB = NAN;
                spaceCrafts[w].electronGun.energyEB = NAN;
            }
        }
    };

    int orderByPriority[this->numSat];      //!< unsorted array of spacecraft ID's
    for (int n = 0; n < this->numSat; n++) {
        orderByPriority[n] = spaceCrafts[n].getID("ID");
    }
    // selection sort priority array from highest to lowest priority
    int minInd;
    for (int k = 0; k < this->numSat - 1; k++) {
        minInd = k;     //!< index of lowest valued element
        for (int m = k + 1; m < this->numSat; m++) {
            if (spaceCrafts[m].priority > spaceCrafts[minInd].priority) {
                minInd = m;
            }
        }
        if (minInd != k) {
            int temp = orderByPriority[minInd];
            orderByPriority[minInd] = orderByPriority[k];
            orderByPriority[k] = temp;
        }
    }
    /* Error messages for user inputs (need to move to Reset) */
    // notifies user there are more craft specified to emit an EB than there are craft
    if ((sizeof(electronGunCrafts) / sizeof(electronGunCrafts[0])) > this->numSat) {
        bskLogger.bskLog(BSK_ERROR, "ScCharging.Reset: More craft equipped with electron gun than total craft");
    }
    // notifies user that input priority settings are incorrect
    for (int z = 0; z < this->numSat; z++) {
        if ((spaceCrafts[z].priority > spaceCrafts[z + 1].priority) && (!spaceCrafts[z].emitsEB) && (spaceCrafts[z + 1].priority)) {
            bskLogger.bskLog(BSK_ERROR, "ScCharging.Reset: One or more target crafts designated higher priority than servicer craft");
        }
    }
    
    double equilibriums[this->numSat];  //!< equilibriums for each craft [eV]
    VoltMsgPayload voltMsgBuffer;  //!< [] voltage out message buffer

    // loop over all satellites
    for (long unsigned int c = 0; c < this->numSat; c++) {
        // store voltage of each spacecraft
        voltMsgBuffer.voltage = -1000;
        this->voltOutMsgs.at(c)->write(&voltMsgBuffer, this->moduleID, CurrentSimNanos);
        
        // index of craft based on priority
        int ID = orderByPriority[c];
        
        // values required for sumCurrents
        double A = spaceCrafts[ID].A;
        double A_sunlit = spaceCrafts[ID].A_sunlit;
        
        // function that sums all calculated currents to be fed to bisectionSolve
        std::function<double(double)> sumCurrents = [&](double phi)-> double
        {
            if (spaceCrafts[ID].emitsEB) {
            return electronCurrent(phi, A) + ionCurrent(phi, A) + SEEelectronCurrent(phi, A) + SEEionCurrent(phi, A) + backscatteringCurrent(phi, A) + photoelectricCurrent(phi, A_sunlit);
            } else if (!spaceCrafts[ID].emitsEB) {
                return electronCurrent(phi, A) + ionCurrent(phi, A) + SEEelectronCurrent(phi, A) + SEEionCurrent(phi, A) + backscatteringCurrent(phi, A) + photoelectricCurrent(phi, A_sunlit) + SEEelectronBeamCurrent(equilibriums[0], 0, spaceCrafts[orderByPriority[0]].electronGun.energyEB, spaceCrafts[orderByPriority[0]].electronGun.currentEB, spaceCrafts[orderByPriority[0]].electronGun.alphaEB) + electronBeamBackscattering(equilibriums[0], 0, spaceCrafts[orderByPriority[0]].electronGun.energyEB, spaceCrafts[orderByPriority[0]].electronGun.currentEB, spaceCrafts[orderByPriority[0]].electronGun.alphaEB);
            } else {
                bskLogger.bskLog(BSK_ERROR, "ScCharging.UpdateState: EB emission boolean not specified");
                return NAN;
            }
        };
        // find equilibrium
        equilibriums[c] = bisectionSolve(interval, 1e-8, sumCurrents);
        std::cout << spaceCrafts[ID].name << " equilibrium: " << equilibriums[c] << std::endl;
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
double ScCharging::trapz(std::function< double(double) >& f, double a, double b, int N)
{
    double h = (b-a)/N;    // trapezoid width
    double sum = 0;
    
    for (int i=1; i < N; i++){
        sum += f(i*h);
    }

    double integral = h * (sum + (f(a)+f(b))/2.0);
    return integral;
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
