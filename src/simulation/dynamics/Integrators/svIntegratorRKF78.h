/*
 ISC License

 Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#ifndef svIntegratorRKF78_h
#define svIntegratorRKF78_h

#include "../_GeneralModuleFiles/stateVecIntegrator.h"
#include "../_GeneralModuleFiles/dynParamManager.h"
#include <stdint.h>

/*! @brief 7/8 order Runge-Kutta integrator */
class svIntegratorRKF78 : public StateVecIntegrator
{
public:
    svIntegratorRKF78(DynamicObject* dyn);            //!< class method
    virtual ~svIntegratorRKF78();
    virtual void integrate(double currentTime, double timeStep); //!< class method
    double alphaMatrix[13];         //!< matrix of coefficients for time steps
    double betaMatrix[13][12];      //!< matrix of coefficients for state steps
    double chMatrix[13];            //!< matrix of coefficients for the result
    double ctMatrix[13];            //!< matrix of coefficients for the error
    double kMatrix[13];             //!< matrix of the k coefficients

    double absTol;      //!< absolute tolerance
    double relTol;      //!< relative tolerance
};



#endif /* svIntegratorRKF78_h */
