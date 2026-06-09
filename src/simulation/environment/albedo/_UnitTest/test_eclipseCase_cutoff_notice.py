#
#  ISC License
#
#  Copyright (c) 2026, PIC4SeR & AVS Lab, Politecnico di Torino & Argotec S.R.L., University of Colorado Boulder
#
#  Permission to use, copy, modify, and/or distribute this software for any
#  purpose with or without fee is hereby granted, provided that the above
#  copyright notice and this permission notice appear in all copies.
#
#  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
#  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
#  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
#  ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
#  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
#  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
#  OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
#

from datetime import date
import warnings


def test_eclipseCase_cutoff_notice():
    """Emit a visible reminder after the cutoff date without failing tests."""
    if date.today() <= date(2027, 5, 1):
        return

    warnings.warn(
        "The cutoff date for 'eclipseCase' has passed. Please remove the deprecated public 'eclipseCase' from 'planetRadiationBase.h' usage along with this test script.\n"
        "Rename the protected variable 'm_eclipseCase' to 'eclipseCase' both in 'planetRadiationBase.h' and 'planetRadiationBase.cpp', remove the deprecation code from getter/setter.",
        UserWarning,
        stacklevel=1,
    )
