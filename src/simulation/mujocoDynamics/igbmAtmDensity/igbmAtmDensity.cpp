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
#include "igbmAtmDensity.h"

#include <algorithm>

void IgbmAtmDensity::writeOutput(uint64_t CurrentSimNanos, double x)
{
    AtmoPropsMsgPayload out = atmoDensInMsg();
    // x is the IGBM factor (mean-reverting to mu); a multiplicative density correction.
    // The exact IGBM process stays positive, but an explicit integrator can occasionally
    // produce a non-positive factor for a large step/increment, so clamp the corrected
    // density to be non-negative (a negative density is unphysical).
    out.neutralDensity = std::max(0.0, out.neutralDensity * x);
    atmoDensOutMsg.write(&out, this->moduleID, CurrentSimNanos);
}
