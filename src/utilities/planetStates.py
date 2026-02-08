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


from pathlib import Path
from Basilisk import __path__
from Basilisk.topLevelModules import pyswice
from Basilisk.utilities.pyswice_spk_utilities import spkRead
from Basilisk.utilities.supportDataTools.dataFetcher import get_path, DataFile

bskPath = __path__[0]


def planetPositionVelocity(
    planetName,
    time,
    ephemerisPath="/supportData/EphemerisData/pck00010.tpc",
    observer="SSB",
    frame="J2000",
):
    """
    Convenience function to get planet position from SPICE.
    """

    from Basilisk.utilities.supportDataTools.dataFetcher import get_path, DataFile
    from pathlib import Path
    from Basilisk.topLevelModules import pyswice
    from Basilisk.utilities.pyswice_spk_utilities import spkRead

    # --- Resolve kernel paths ---
    de430_path = Path(get_path(DataFile.EphemerisData.de430))
    naif0012_path = Path(get_path(DataFile.EphemerisData.naif0012))

    print("\n==== planetPositionVelocity DEBUG ====")
    print("planetName:", planetName)
    print("time:", time)
    print("ephemerisPath:", ephemerisPath)
    print("de430_path:", de430_path)
    print("naif0012_path:", naif0012_path)

    # --- Load global kernels ---
    pyswice.furnsh_c(str(de430_path))
    pyswice.furnsh_c(str(naif0012_path))

    # --- Resolve ephemeris path ---
    eph = Path(ephemerisPath)

    if eph.is_dir():
        # historical Basilisk behavior:
        # when passed a directory, only load pck00010.tpc
        candidate = eph / "pck00010.tpc"
        if not candidate.exists():
            raise RuntimeError(
                f"Directory {eph} does not contain pck00010.tpc "
                "(required for old-style planetPositionVelocity)"
            )
        eph = candidate

    # Load ephemeris file
    pyswice.furnsh_c(str(eph))
    positionVelocity = spkRead(planetName, time, frame, observer)
    position = positionVelocity[0:3] * 1000
    velocity = positionVelocity[3:6] * 1000

    # Unload only the ephemeris file we loaded
    pyswice.unload_c(str(eph))

    return position, velocity
