/*
 Copyright (c) 2016, Autonomous Vehicle Systems Lab, Univeristy of Colorado at Boulder
 
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


#include "rk4Integrator.h"
#include <stdio.h>

rk4Integrator::rk4Integrator(dynObject* dyn) : integrator(dyn)
{
    return;
}

rk4Integrator::~rk4Integrator()
{
    return;
}

void rk4Integrator::integrate(double currentTime, double timeStep, double* currentState, double* nextState, unsigned int NStates)
{
    double X2[NStates];       /* integration state space */
    double k1[NStates];       /* intermediate RK results */
    double k2[NStates];
    double k3[NStates];
    double k4[NStates];
    
    unsigned int i;
    
    this->_dyn->equationsOfMotion(currentTime, currentState, k1);
    for(i = 0; i < NStates; i++) {
        X2[i] = currentState[i] + 0.5 * timeStep * k1[i];
    }
    
    this->_dyn->equationsOfMotion(currentTime + timeStep * 0.5, X2, k2);
    for(i = 0; i < NStates; i++) {
        X2[i] = currentState[i] + 0.5 * timeStep * k2[i];
    }
    
    this->_dyn->equationsOfMotion(currentTime + timeStep * 0.5, X2, k3);
    for(i = 0; i < NStates; i++) {
        X2[i] = currentState[i] + timeStep * k3[i];
    }
    
    this->_dyn->equationsOfMotion(currentTime + timeStep, X2, k4);
    for(i = 0; i < NStates; i++) {
        nextState[i] = currentState[i] + timeStep / 6.0 * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]);
    }

    return;
}


