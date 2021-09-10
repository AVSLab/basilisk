/*
 ISC License

 Copyright (c) 2021, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#ifndef svIntegratorRKF45_h
#define svIntegratorRKF45_h

#include "../_GeneralModuleFiles/stateVecIntegrator.h"
#include "../_GeneralModuleFiles/dynParamManager.h"
#include <stdint.h>

/*! @brief 4th order Runge-Kutta-Fehlberg variable time step integrator */
class svIntegratorRKF45 : public StateVecIntegrator
{
public:
    svIntegratorRKF45(DynamicObject* dyn); //!< class method
    virtual ~svIntegratorRKF45();
    virtual void integrate(double currentTime, double timeStep); //!< class method
    double aMatrix[6];
    double bMatrix[6][5];
    double cMatrix[6];
    double dMatrix[6];
};


#endif /* svIntegratorRKF45_h */
