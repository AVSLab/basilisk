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
#include "svStochasticIntegratorSRA1.h"

svStochasticIntegratorSRA1::svStochasticIntegratorSRA1(DynamicObject* dyn)
    : svIntegratorStrongStochasticRungeKuttaSRA(dyn, svStochasticIntegratorSRA1::getCoefficients())
{}

// Coefficients for the SRA1 tableau. Matrix
// entries use the plain [stage i][sub-term j] convention of SRACoefficients (note that
// some reference implementations store the transpose A0' internally; here we write A0 directly).
SRACoefficients<2> svStochasticIntegratorSRA1::getCoefficients()
{
    SRACoefficients<2> c;

    c.A0[1][0] = 3. / 4.;
    c.B0[1][0] = 3. / 2.;

    c.alpha = {1. / 3., 2. / 3.};
    c.beta1 = {1., 0.};
    c.beta2 = {-1., 1.};

    c.c0 = {0., 3. / 4.};
    c.c1 = {1., 0.};

    return c;
}
