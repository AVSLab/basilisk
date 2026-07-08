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
#include "svStochasticIntegratorSOSRA.h"

svStochasticIntegratorSOSRA::svStochasticIntegratorSOSRA(DynamicObject* dyn)
    : svIntegratorStrongStochasticRungeKuttaSRA(dyn, svStochasticIntegratorSOSRA::getCoefficients())
{}

// Coefficients for the SOSRA (three-stage SRA) tableau. Matrix entries use the
// plain [stage i][sub-term j] convention of SRACoefficients (a21 -> A0[1][0],
// a31 -> A0[2][0], a32 -> A0[2][1], etc.).
SRACoefficients<3> svStochasticIntegratorSOSRA::getCoefficients()
{
    SRACoefficients<3> c;

    // Drift stage matrix A0
    c.A0[1][0] = 0.6923962376159507;
    c.A0[2][0] = -3.1609142252828395;
    c.A0[2][1] = 4.1609142252828395;

    // Diffusion stage matrix B0
    c.B0[1][0] = 1.3371632704399763;
    c.B0[2][0] = 1.442371048468624;
    c.B0[2][1] = 1.8632741501139225;

    // Weights
    c.alpha = {0.2889874966892885, 0.6859880440839937, 0.025024459226717772};
    c.beta1 = {-16.792534242221663, 17.514995785380226, 0.27753845684143835};
    c.beta2 = {0.4237535769069274, 0.6010381474428539, -1.0247917243497813};

    // Nodes
    c.c0 = {0.0, 0.6923962376159507, 1.0};
    c.c1 = {0.0, 0.041248171110700504, 1.0};

    return c;
}
