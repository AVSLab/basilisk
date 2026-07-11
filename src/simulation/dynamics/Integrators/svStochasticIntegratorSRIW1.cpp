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
#include "svStochasticIntegratorSRIW1.h"

svStochasticIntegratorSRIW1::svStochasticIntegratorSRIW1(DynamicObject* dyn)
    : svIntegratorStrongStochasticRungeKuttaSRI(dyn, svStochasticIntegratorSRIW1::getCoefficients())
{}

// Coefficients for the SRIW1 tableau, in unrolled per-stage form. Matrix entries
// use the plain [stage i][sub-term j] convention of SRICoefficients (see
// svIntegratorStrongStochasticRungeKuttaSRI.h).
SRICoefficients<4> svStochasticIntegratorSRIW1::getCoefficients()
{
    SRICoefficients<4> c;

    // Drift stage matrix A0 (only the H0[1] stage has a nonzero row)
    c.A0[1][0] = 3. / 4.;

    // Diffusion-stage drift matrix A1 (constructSRIW1 A^(1): row 3 couples stage 2)
    c.A1[1][0] = 1. / 4.;
    c.A1[2][0] = 1.;
    c.A1[3][2] = 1. / 4.;

    // Drift stage diffusion matrix B0
    c.B0[1][0] = 3. / 2.;

    // Diffusion-stage diffusion matrix B1
    c.B1[1][0] = 1. / 2.;
    c.B1[2][0] = -1.;
    c.B1[3][0] = -5.;
    c.B1[3][1] = 3.;
    c.B1[3][2] = 1. / 2.;

    // Weights
    c.alpha = {1. / 3., 2. / 3., 0., 0.};
    c.beta1 = {-1., 4. / 3., 2. / 3., 0.};
    c.beta2 = {-1., 4. / 3., -1. / 3., 0.};
    c.beta3 = {2., -4. / 3., -2. / 3., 0.};
    c.beta4 = {-2., 5. / 3., -2. / 3., 1.};

    // Nodes (row sums of A0 and A1 respectively)
    c.c0 = {0., 3. / 4., 0., 0.};
    c.c1 = {0., 1. / 4., 1., 1. / 4.};

    return c;
}
