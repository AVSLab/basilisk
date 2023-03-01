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
    
    double d = 2.0;
    double k = -1.0;
    std::function<double(double)> f = [this, d, k](double x)-> double {
        double xn = this->getFlux(x, "electron");
        return k*xn + d;
    };
    
    double integral = trapz(f, 0, 1, 100);
    
    std::cout << "Integral is " << std::setprecision(10) << integral << std::endl;
    
    
    //std::cout << "electron message is: " << electronFlux << std::endl;
    double testArr[MAX_PLASMA_FLUX_SIZE];
    eigenMatrixXd2CArray(electronFlux, testArr); // convert electronFlux to array
    int j = sizeof(testArr) / sizeof(testArr[0]);
    std::vector<double> testYVec(testArr, testArr + j); // convert electronArr to vector
    
    double testXArr[MAX_PLASMA_FLUX_SIZE];
    eigenMatrixXd2CArray(energies, testXArr); // convert electronFlux to array
    int pl = sizeof(testXArr) / sizeof(testXArr[0]);
    std::vector<double> testXVec(testXArr, testXArr + pl); // convert electronArr to vector
    
    double interpTest = interp(testXVec, testYVec, 1.3257); // should return close 188959.1466
    std::cout << "interpTest: " << interpTest << std::endl;
    
    
    double interval [2] = {0, 4};
    double x0 = bisectionSolve(interval, 1e-8, f);
    
    std::cout << "Bisection Method found root at " << std::setprecision(10) << x0 << std::endl;
    
    double ie = electronCurrent(-24810, 12.566370614359172);
    std::cout<< "ie = " << ie << std::endl;
    
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

double ScCharging::electronCurrent(double phi, double A)
{
    
    std::cout << "electronFlux[-1]: " << electronFlux[-1] << std::endl;
    std::cout << "electronFlux[0]: " << electronFlux[0] << std::endl;
    
    double constant = -Q0 * 2 * M_PI * (A/10000); // constant multiplier for integral

    double electronArr[MAX_PLASMA_FLUX_SIZE];
    eigenMatrixXd2CArray(electronFlux, electronArr); // convert electronFlux to array
    int n = sizeof(electronArr) / sizeof(electronArr[0]);
    std::vector<double> electronVec(electronArr, electronArr + n); // convert electronArr to vector
    
    double testXArr[MAX_PLASMA_FLUX_SIZE];
    eigenMatrixXd2CArray(energies, testXArr); // convert electronFlux to array
    int pl = sizeof(testXArr) / sizeof(testXArr[0]);
    std::vector<double> testXVec(testXArr, testXArr + pl); // convert electronArr to vector
    
    // term to be integrated by trapz
    std::function<double(double)> integrand = [&](double E){return (E/(E - phi)) * interp(testXVec, electronVec, E - phi);};
//
//    std::cout << "interp(lowerBound): " << interp(electronVec, 0) << std::endl;
//    std::cout << "interp(upperBound + phi): " << interp(electronVec, 40000 + phi) << std::endl;
    //std::cout<< "interp gives " << interp(electronVec, E - phi) << std::endl;
    
    // integral bounds
    double lowerBound = 0, upperBound = 1000000;
    // integral calculated with trapz
    double integral = trapz(integrand, lowerBound, upperBound, 1000);
    std::cout<< "integral = " << integral << std::endl;
    
    double Ie = constant * integral;
    return Ie;
}

/*!  This function takes in a given vector of data and an x-value and performs linear interpolation to find the closest corresponding y-value
 @return double
 @param data vector containing datapoints to use in linear interpolation (y-values)
 @param x x-value being linear interpolated to
 */
double ScCharging::interp(std::vector<double>& xVector, std::vector<double>& yVector, double x)
{
    // get iterator >= given x's corresponding iterator
    auto iterator = std::lower_bound(yVector.begin(), yVector.end(), x);
    // find closest iterator to x
    double a = *(iterator - 1);
    double b = *(iterator);
    long closestIterator;
    if (fabs(x - a) < fabs(x - b)){
        closestIterator =  iterator - yVector.begin() - 1;
    }
    closestIterator = iterator - yVector.begin();
    // check if closest iterator is above or below x and create bounds for linear interpolation
    double indX0, indX1, y0, y1;
    indX0 = closestIterator, indX1 = closestIterator + 1;
    y0 = yVector[indX0], y1 = yVector[indX1];
    
//    std::cout << "interp x: " << x << std::endl;
//    std::cout << "interp x0: " << xVector[indX0] << std::endl;
//    std::cout << "interp x1: " << xVector[indX1] << std::endl;
//    std::cout << "interp y0: " << y0 << std::endl;
//    std::cout << "interp y1: " << y1 << std::endl;
    
    double y = y0 + ((y1-y0)/(xVector[indX1] - xVector[indX0])) * (x - xVector[indX0]);
    
//    std::cout << "interp y: " << y << std::endl;
    
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
    
    std::cout << "b: " << b << std::endl;
    std::cout << "f(a): " << f(a) << std::endl;
    std::cout << "f(b): " << f(b) << std::endl;
    
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
