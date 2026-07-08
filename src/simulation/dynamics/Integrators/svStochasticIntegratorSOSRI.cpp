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
#include "svStochasticIntegratorSOSRI.h"

svStochasticIntegratorSOSRI::svStochasticIntegratorSOSRI(DynamicObject* dyn)
    : svIntegratorStrongStochasticRungeKuttaSRI(dyn, svStochasticIntegratorSOSRI::getCoefficients())
{}

// Coefficients for the SOSRI tableau (stability-optimized SRI, four stages). Matrix
// entries use the plain [stage i][sub-term j] convention of SRICoefficients (see
// svIntegratorStrongStochasticRungeKuttaSRI.h).
SRICoefficients<4> svStochasticIntegratorSOSRI::getCoefficients()
{
    SRICoefficients<4> c;

    // Drift stage matrix A0
    c.A0[1][0] = -0.04199224421316468;
    c.A0[2][0] = 2.842612915017106;
    c.A0[2][1] = -2.0527723684000727;
    c.A0[3][0] = 4.338237071435815;
    c.A0[3][1] = -2.8895936137439793;
    c.A0[3][2] = 2.3017575594644466;

    // Diffusion-stage drift matrix A1
    c.A1[1][0] = 0.26204282091330466;
    c.A1[2][0] = 0.20903646383505375;
    c.A1[2][1] = -0.1502377115150361;
    c.A1[3][0] = 0.05836595312746999;
    c.A1[3][1] = 0.6149440396332373;
    c.A1[3][2] = 0.08535117634046772;

    // Drift stage diffusion matrix B0
    c.B0[1][0] = -0.21641093549612528;
    c.B0[2][0] = 1.5336352863679572;
    c.B0[2][1] = 0.26066223492647056;
    c.B0[3][0] = -1.0536037558179159;
    c.B0[3][1] = 1.7015284721089472;
    c.B0[3][2] = -0.20725685784180017;

    // Diffusion-stage diffusion matrix B1
    c.B1[1][0] = -0.5119011827621657;
    c.B1[2][0] = 2.67767339866713;
    c.B1[2][1] = -4.9395031322250995;
    c.B1[3][0] = 0.15580956238299215;
    c.B1[3][1] = 3.2361551006624674;
    c.B1[3][2] = -1.4223118283355949;

    // Weights
    c.alpha = {1.140099274172029, -0.6401334255743456, 0.4736296532772559,
               0.026404498125060714};
    c.beta1 = {-1.8453464565104432, 2.688764531100726, -0.2523866501071323,
               0.40896857551684956};
    c.beta2 = {0.4969658141589478, -0.5771202869753592, -0.12919702470322217,
               0.2093514975196336};
    c.beta3 = {2.8453464565104425, -2.688764531100725, 0.2523866501071322,
               -0.40896857551684945};
    c.beta4 = {0.11522663875443433, -0.57877086147738, 0.2857851028163886,
               0.17775911990655704};

    // Nodes (row sums of A0 and A1 respectively)
    c.c0 = {0.0, -0.04199224421316468, 0.7898405466170333, 3.7504010171562823};
    c.c1 = {0.0, 0.26204282091330466, 0.05879875232001766, 0.758661169101175};

    return c;
}
