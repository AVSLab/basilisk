/*
 ISC License

 Copyright (c) 2021, Autonomous Vehicle Systems Lab, University of Colorado Boulder

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
#include <cstring>
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

}

/*! This method is used to reset the module and checks that required input messages are connect.
    @return void
*/
void ScCharging::Reset(uint64_t CurrentSimNanos)
{
    // check that required input messages are connected
    if (!this->plasmaFluxInMsg.isLinked())
    {
        bskLogger.bskLog(BSK_ERROR, "ScCharging.plasmaFluxInMsg was not linked.");
    }
}

/*!  Read in the input messages
 */
void ScCharging::readMessages()
{
    /*! - read in plasma flux  message (required) */
    PlasmaFluxMsgPayload plasmaFluxMsgData;
    plasmaFluxMsgData = this->plasmaFluxInMsg();
    this->energies = cArray2EigenMatrixXd(plasmaFluxMsgData.energies,MAX_PLASMA_FLUX_SIZE,1);
    this->electronFlux = cArray2EigenMatrixXd(plasmaFluxMsgData.meanElectronFlux,MAX_PLASMA_FLUX_SIZE,1);
    this->ionFlux = cArray2EigenMatrixXd(plasmaFluxMsgData.meanIonFlux,MAX_PLASMA_FLUX_SIZE,1);
}

/*!  Subfunction to calculate electron current
 */
double electronCurrent(double q0, double A, double phi, double energy, double (*flux)(double)){
    /*
    //  INPUTS:
    //           q0  = particle charge (positive or negative)
    //           A   = area exposed to plasma
    //           phi = spacecraft potential
    //           E   = energy
    //           flux = pointer for flux distribution function
    //  OUTPUTS:
    //           Ie  = electron thermal current
    */
    double L = 0.1;   // since electron, L = 0, but need 0.1 so no /0
    
    // building integrand
    double E;   // TODO get energies for E
    double integrand = flux(E + phi);
    double constMult = E / (E + phi);
    
    // integrate
    double integral = constMult * trapzegator(0,10,integrand);
    
    // solve
    Ie = (q0 * 2.0 * M_PI * A) *integral;
    
    return Ie;
}

/*!  Subfunction to calculate ion plasma current
 */
double ionPlasmaCurrent(){
    double L = fabs(phi);
}

/*!  Subfunction to calculate secondary electron current
 */
double secondaryElectronCurrent(){
    
}

/*!  Subfunction to calculate photoelectron current
 */
double photoelectronCurrent(){
    
}

/*! Subfunction to represent f(x)
 */
double f(double x)
{
    double a = x;
    return a;
}

/*!  Subfunction to perform trapezoidal integration
 */
double trapzegator(double a, double b, int N, (*f)(double) )
{
    int n, i;   // n is # of subintervals, i for looping
    double leftSide, rightSide, h, sum=0, integral;
    leftSide = a;
    rightSide = b;
    n = N;
    double x[n+1],y[n+1];
    h = (b-a)/n;    // subinterval width
    
    for (i=0; i <= n, i++){
        x[i] = a + i*h;
        y[i] = f(x[i]);
    }
    for (i=1; i< n, i++){
        sum = sum + h*y[i];
    }
    
    integral = h / 2.0 * (y[0]+y[n])+sum;
    return integral;
}

/*!  Subfunction to perform linear interpolation
 */
double lerp(double x0, double x1, double y0, double y1, double xPoint )
{
    double yLerped = y0 + ((y1-y0)/(x1-x0)) * (xPoint - x0);
    return yLerped;
}

/*! This is the main method that gets called every time the module is updated.  Calculates total current and finds equilibrium potential.
    @return void
*/
void ScCharging::UpdateState(uint64_t CurrentSimNanos)
{
    // read the input messages
    this->readMessages();
    // call function for each current
    Ie = ellectronCurrent(
    // get equilibrium potential using bisection method
    
    
    
}
