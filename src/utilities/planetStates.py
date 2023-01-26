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


from Basilisk import __path__
from Basilisk.topLevelModules import pyswice
from Basilisk.utilities.pyswice_spk_utilities import spkRead

bskPath = __path__[0]

def planetPositionVelocity(planetName, time, ephemerisPath = '/supportData/EphemerisData/pck00010.tpc', observer = 'SSB', frame = 'J2000'):
    """
        A convenience function to get planet position from spice

        Parameters
        ----------
        planetName : name of planet to get position of
            planet name must be a valid SPICE celestial body string.
        time : UTC time as string
        ephemerisPath : a string path to ephemeris file if something other than the default is desired
        observer : observer to get vectors relative to

        Returns
        -------
        position and velocity vector of planet in Solar System Barycenter inertial frame as lists [m], [m/s]
    """

    pyswice.furnsh_c(bskPath + '/supportData/EphemerisData/de430.bsp')
    pyswice.furnsh_c(bskPath + '/supportData/EphemerisData/naif0012.tls') #load leap seconds
    pyswice.furnsh_c(bskPath + ephemerisPath)
    positionVelocity = spkRead(planetName, time, frame, observer)
    position = positionVelocity[0:3] * 1000
    velocity = positionVelocity[3:6] * 1000
    pyswice.unload_c(bskPath + ephemerisPath)

    return position, velocity # [m], [m/s]
