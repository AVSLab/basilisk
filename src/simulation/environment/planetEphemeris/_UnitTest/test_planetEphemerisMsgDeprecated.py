
# ISC License
#
# Copyright (c) 2024, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
#
# Permission to use, copy, modify, and/or distribute this software for any
# purpose with or without fee is hereby granted, provided that the above
# copyright notice and this permission notice appear in all copies.
#
# THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
# WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
# ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
# WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
# ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
# OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

#
#   Unit Test Script
#   Module Name:        planetEphemeris
#   Author:             Hanspeter Schaub
#   Creation Date:      August 5, 2024
#


# Import all of the modules that we are going to be called in this simulation
from Basilisk.simulation import planetEphemeris

import warnings
from Basilisk.utilities import deprecated



# update "module" in this function name to reflect the module name
def test_module(show_plots):
    """Module Unit Test"""

    testResults = 0
    testMessage = ""

    # when the depreciation warning period is over, then the following tasks must be done:
    # - remove this file
    # - delete the swig deprecation warning in planetEphemeris.i
    # - delete the typedef of classicalElements in PlanetEphemerisMsgPayload.h
    warnings.filterwarnings("ignore", category=deprecated.BSKDeprecationWarning)

    try:
        # This setting of this message structure is depreciate. But, this test won't raise
        # a depreciation warning until the 1-year grace period is passed.
        msg = planetEphemeris.ClassicElementsMsgPayload()
    except:
        testResults += 1
        testMessage = "planetEphemeris not able to set ClassicElementsMsgPayload"

    assert testResults < 1, testMessage
