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
#include <iostream>
#include <iomanip>
#include <cstring>
#include <fstream>
#include <math.h>

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

/*!  This function performs linear interpolation between the provided values
 @return double
 @param x0 data point 0
 @param x1 data point 1
 @param y0 function value for data point 0
 @param y1 function value for data point 1
 */
double ScCharging::interp(double x0, double x1, double y0, double y1, double x)
{
    double y = y0 + ((y1-y0)/(x1-x0)) * (x - x0);
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