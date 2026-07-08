/*
 ISC License

 Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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
#include "svStochasticIntegratorSIESME.h"

#include <cmath>

// Coefficients for the SIEA/SMEA/SIEB/SMEB tableaux (Tocino & Vigo-Aguiar).

svStochasticIntegratorSIEA::svStochasticIntegratorSIEA(DynamicObject* dyn)
    : svIntegratorWeakSIESME(dyn, svStochasticIntegratorSIEA::getCoefficients())
{}

SIESMECoefficients svStochasticIntegratorSIEA::getCoefficients()
{
    SIESMECoefficients c;
    c.alpha1 = 0.5;  c.alpha2 = 0.5;
    c.gamma1 = 0.5;
    c.lambda1 = 0.25; c.lambda2 = -0.25; c.lambda3 = 0.25;
    c.mu1 = 0.25; c.mu2 = 0.25; c.mu3 = -0.25;
    c.mu0 = 1.0; c.mubar0 = 1.0;
    c.lambda0 = 1.0; c.lambdabar0 = 1.0;
    c.nu1 = 1.0; c.nu2 = 0.0;
    c.beta2 = 1.0; c.beta3 = 0.0;
    c.delta2 = -1.0; c.delta3 = 0.0;
    return c;
}

svStochasticIntegratorSMEA::svStochasticIntegratorSMEA(DynamicObject* dyn)
    : svIntegratorWeakSIESME(dyn, svStochasticIntegratorSMEA::getCoefficients())
{}

SIESMECoefficients svStochasticIntegratorSMEA::getCoefficients()
{
    SIESMECoefficients c;
    c.alpha1 = 0.0;  c.alpha2 = 1.0;
    c.gamma1 = 0.5;
    c.lambda1 = 0.25; c.lambda2 = -0.25; c.lambda3 = 0.25;
    c.mu1 = 0.25; c.mu2 = 0.25; c.mu3 = -0.25;
    c.mu0 = 0.5; c.mubar0 = 1.0;
    c.lambda0 = 0.5; c.lambdabar0 = 1.0;
    c.nu1 = (2.0 - std::sqrt(6.0)) / 4.0; c.nu2 = std::sqrt(6.0) / 12.0;
    c.beta2 = 1.0; c.beta3 = 0.0;
    c.delta2 = -1.0; c.delta3 = 0.0;
    return c;
}

svStochasticIntegratorSIEB::svStochasticIntegratorSIEB(DynamicObject* dyn)
    : svIntegratorWeakSIESME(dyn, svStochasticIntegratorSIEB::getCoefficients())
{}

SIESMECoefficients svStochasticIntegratorSIEB::getCoefficients()
{
    SIESMECoefficients c;
    c.alpha1 = 0.5;  c.alpha2 = 0.5;
    c.gamma1 = -0.2;
    c.lambda1 = 0.6; c.lambda2 = 1.5; c.lambda3 = -0.5;
    c.mu1 = 0.6; c.mu2 = -1.5; c.mu3 = 0.5;
    c.mu0 = 1.0; c.mubar0 = 5.0 / 12.0;
    c.lambda0 = 1.0; c.lambdabar0 = 5.0 / 12.0;
    c.nu1 = 1.0; c.nu2 = 0.0;
    c.beta2 = 0.0; c.beta3 = -1.0 / 6.0;
    c.delta2 = 0.0; c.delta3 = 1.0 / 6.0;
    return c;
}

svStochasticIntegratorSMEB::svStochasticIntegratorSMEB(DynamicObject* dyn)
    : svIntegratorWeakSIESME(dyn, svStochasticIntegratorSMEB::getCoefficients())
{}

SIESMECoefficients svStochasticIntegratorSMEB::getCoefficients()
{
    SIESMECoefficients c;
    c.alpha1 = 0.0;  c.alpha2 = 1.0;
    c.gamma1 = -0.2;
    c.lambda1 = 0.6; c.lambda2 = 1.5; c.lambda3 = -0.5;
    c.mu1 = 0.6; c.mu2 = -1.5; c.mu3 = 0.5;
    c.mu0 = 0.5; c.mubar0 = 5.0 / 12.0;
    c.lambda0 = 0.5; c.lambdabar0 = 5.0 / 12.0;
    c.nu1 = (2.0 - std::sqrt(6.0)) / 4.0; c.nu2 = std::sqrt(6.0) / 12.0;
    c.beta2 = 0.0; c.beta3 = -1.0 / 6.0;
    c.delta2 = 0.0; c.delta3 = 1.0 / 6.0;
    return c;
}
