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
#include <vector>
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

/*! This method is used to reset the module and checks that required input messages are connect.
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
    
    // sum currents and find root
    //double phi = -24810;
    double A = 12.566370614359172;
    
    std::function<double(double)> sumCurrents = [this, A](double phi)-> double {
        
//        std::cout << "Electron Current: " << electronCurrent(phi, A) << std::endl;
//        std::cout << "Ion Current: " << ionCurrent(phi, A) << std::endl;
//        std::cout << "Total Current: " << electronCurrent(phi, A) + ionCurrent(phi, A) << std::endl;
        
        return electronCurrent(phi, A) + ionCurrent(phi, A);
    };
    
    double interval [2] = {-1e8, 1e8};
    
    double equilibrium = bisectionSolve(interval, 1e-8, sumCurrents);
    
    std::cout << "Equilibrium: " << std::setprecision(10) << equilibrium << std::endl;
    
    // create output messages
    VoltMsgPayload voltMsgBuffer;  //!< [] voltage out message buffer
    // loop over all satellites
    for (long unsigned int c=0; c < this->numSat; c++) {
        // store voltage of each spacecraft
        voltMsgBuffer.voltage = -1000;
        this->voltOutMsgs.at(c)->write(&voltMsgBuffer, this->moduleID, CurrentSimNanos);
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
    PlasmaFluxMsgPayload PlasmaFluxInMsgBuffer;          //!< local copy of plasma flux input message buffer
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
 @param A double defining value for area exposed to plasma
 */
double ScCharging::electronCurrent(double phi, double A)
{
    double constant = -Q0 * A; // constant multiplier for integral

    double electronArr[MAX_PLASMA_FLUX_SIZE];
    eigenMatrixXd2CArray(electronFlux, electronArr); // convert electronFlux to array
    int n = sizeof(electronArr) / sizeof(electronArr[0]);
    std::vector<double> electronVec(electronArr, electronArr + n); // convert electronArr to vector
    
    double energyArr[MAX_PLASMA_FLUX_SIZE];
    eigenMatrixXd2CArray(energies, energyArr); // convert energies to array
    int j = sizeof(energyArr) / sizeof(energyArr[0]);
    std::vector<double> energyVec(energyArr, energyArr + j); // convert energyArr to vector

    std::function<double(double)> getFlux = [&](double E){
        // find flux for given energy
        double F = interp(energyVec, electronVec, E);
        if (F < 0.){
            // if flux is negative (due to extrapolation), set equal to zero
            F = 0.;
        }
        return F;
    };

    // term to be integrated by trapz
    std::function<double(double)> integrand = [&](double E){return (E/(E - phi)) * getFlux(E - phi);};
    
    // integral bounds
    double lowerBound;
    double upperBound;
    if (phi < 0.){
        lowerBound = 0.1;
        upperBound = energyVec.back();
    }
    else{
        lowerBound = 0.1 + abs(phi);
        upperBound = energyVec.back() + abs(phi);
    }
    
    // integral calculated with trapz
    double integral = trapz(integrand, lowerBound, upperBound, 1000);
    
    double Ie = constant * integral;
    
    std::cout << "Function Ie: " << Ie << std::endl;
    
    return Ie;
}

/*!  This function takes in a given potential and area value and calculates the ion current
 @return double
 @param phi double defining value for spacecraft potential
 @param A double defining value for area exposed to plasma
 */
double ScCharging::ionCurrent(double phi, double A)
{
    double constant = Q0 * A; // constant multiplier for integral

    double ionArr[MAX_PLASMA_FLUX_SIZE];
    eigenMatrixXd2CArray(ionFlux, ionArr); // convert ionFlux to array
    int n = sizeof(ionArr) / sizeof(ionArr[0]);
    std::vector<double> ionVec(ionArr, ionArr + n); // convert ionArr to vector
    
    double energyArr[MAX_PLASMA_FLUX_SIZE];
    eigenMatrixXd2CArray(energies, energyArr); // convert energies to array
    int j = sizeof(energyArr) / sizeof(energyArr[0]);
    std::vector<double> energyVec(energyArr, energyArr + j); // convert energyArr to vector

    std::function<double(double)> getFlux = [&](double E){
        // find flux for given energy
        double F = interp(energyVec, ionVec, E);
        if (F < 0.){
            // if flux is negative (due to extrapolation), set equal to zero
            F = 0.;
        }
        return F;
    };

    // term to be integrated by trapz
    std::function<double(double)> integrand = [&](double E){return (E/(E + phi)) * getFlux(E + phi);};
    
    // integral bounds
    double lowerBound;
    double upperBound;
    if (phi > 0.){
        lowerBound = 0.1;
        upperBound = energyVec.back();
    }
    else{
        lowerBound = 0.1 + abs(phi);
        upperBound = energyVec.back() + abs(phi);
    }
    
    // integral calculated with trapz
    double integral = trapz(integrand, lowerBound, upperBound, 1000);
    
    std::cout << "Ion Lower: " << lowerBound << std::endl;
    std::cout << "Ion Upper: " << upperBound << std::endl;
    std::cout << "Ion Integral: " << integral << std::endl;
    
    double Ii = constant * integral;
    
    std::cout << "Function Ii: " << Ii << std::endl;
    
    return Ii;
}

/*!  This function takes in a given vector of data and an x-value and performs linear interpolation to find the closest corresponding y-value
 @return double
 @param data vector containing datapoints to use in linear interpolation (y-values)
 @param x x-value being linear interpolated to
 */
double ScCharging::interp(std::vector<double>& xVector, std::vector<double>& yVector, double x)
{
    // find the index corresponding to the first element in xVector that is greater than x
    // (assumes xVector is sorted)
    int idx1 = -1; // initialize as -1 (if no index can be found)
    for (int c=0; c < xVector.size(); c++) {
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
    
    for (int i=1; i < N; i++) {
        sum += f(i*h);
    }

    double integral = h * (sum + (f(a)+f(b))/2.0);
    
    return integral;
}

/*!  This function returns the flux for a specific energy
 @return double
 @param E energy of interest [eV]
 @param particle particle of interest ("electron" or "ion")
 */
double ScCharging::getFlux(double E, std::string particle)
{
    double flux;
    
    if (particle == "electron")
    {
        flux = 2*E;
    }
    else
    {
        flux = 1*E;
    }
    
    return flux;
}
