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


from Basilisk.utilities import astroFunctions
import warnings
from Basilisk.utilities import deprecated
import numpy as np

def test_legacy_astrofunctions_constants(show_plots):
    __tracebackhide__ = True

    warnings.filterwarnings("ignore", category=deprecated.BSKDeprecationWarning)

    # check that deprecated astroFunction constants still work
    # when it is time to remove this deprectated functionality,
    # go to src/utilities/astroFunctions.py and remove
    # - from Basilisk.utilities import deprecated
    # - remove __getattr__() method near end of file
    # - remove this test file

    # define astroConstant names
    AU = 1.49597870691e8        # [km]
    astroConstants = {
        # Useful conversions
        'R2D': 180.0 / np.pi,          # [deg]
        'D2R': np.pi / 180.0,         # [rad]
        'AU': AU,
        'SEC2DAY': 1.0 / (3600*24),
        'DAY2SEC': 1.0 * 3600 * 24,
        'PI': np.pi,

        # Gravitational parameters  [km^3 / s^2]
        'mu_S': 1.32712440018e11,
        'mu_E': 3.986004415e5,
        'mu_M': 4.28283100e4,
        'mu_V': 3.24858599e5,
        'mu_J': 1.26686534e8,
        'mu_Saturn': 3.79395000e7,
        'mu_U': 5.79396566e6,
        'mu_N': 6.83509920e6,
        'mu_Moon': 4902.799,

        # Planets' Radius   [km]
        'S_radius': 695508,
        'J_radius': 71492,
        'V_radius': 6051.8,
        'E_radius': 6378.1363,
        'M_radius': 3396.19,
        'Saturn_radius': 60268,
        'U_radius': 25559,
        'N_radius': 24764,
        'Moon_radius': 1738.1,

        # Planetary Heliocentric orbits     [km]
        'a_E': 1.0 * AU,
        'a_M': 1.52367934 * AU,
        'a_Mercury': 0.387 * AU,
        'a_V': 0.723 * AU,
        'a_J': 5.203 * AU,
        'a_Saturn': 9.537 * AU,
        'a_U': 19.191 * AU,
        'a_N': 30.069 * AU,
        'a_P': 39.482 * AU,
    }

    for key in astroConstants:
        np.testing.assert_almost_equal(getattr(astroFunctions, key), astroConstants[key])

''
if __name__ == '__main__':
    test_legacy_astrofunctions_constants(False)
