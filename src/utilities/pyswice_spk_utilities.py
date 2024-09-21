#
#  ISC License
#
#  Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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


import numpy
# -------------------------------------------------------------
#
# ... SPKReader.py
#
# ... Purpose - Read a SPICE SPK file to provide Ephemeris
# ...           object state
#
# ... Input:
# ...            target - name of the target body, can be ID or object name
# ...            time - UTC Gregorian time string at the desired instance
# ...            ref - Reference frame string
# ...            observer - name of the observing body
#
# ... Output:
# ...            state - position and velocity vector
# ...                    first 3 values are position cartesian coordinates
# ...                    last 3 values are velocity components
# ...            lt - light time in seconds
#
# ... G. D. Chapel     LASP             27 Jun 2016
# ...                  ADCS Team
# ...
# --------------------------------------------------------------
from Basilisk.topLevelModules import pyswice


def spkRead(target, time, ref, observer):
    """Spice spk read method"""
    # Convert time to elapsed seconds from ephemeris
    et = pyswice.new_doubleArray(1)
    pyswice.str2et_c(time, et)

    # Get position and velocity
    state = pyswice.new_doubleArray(6)
    lt = pyswice.new_doubleArray(1)
    pyswice.spkezr_c(target, pyswice.doubleArray_getitem(et, 0), ref, "NONE", observer, state, lt)

    # Format state into output array
    stateArray = numpy.zeros(6)
    for i in range(6):
        stateArray[i] = pyswice.doubleArray_getitem(state, i)

    # Delete variables
    pyswice.delete_doubleArray(state)
    pyswice.delete_doubleArray(lt)
    pyswice.delete_doubleArray(et)

    return stateArray
