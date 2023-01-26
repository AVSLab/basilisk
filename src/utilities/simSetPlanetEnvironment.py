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

import numpy as np
from Basilisk.utilities import macros


def exponentialAtmosphere(atmosModule, name):
    """
    Sets the exponential atmosphere model parameters for a particular planet.

    :param atmosModule: atmospheric environment module
    :param name: planet name string

    """
    if name == "earth":
        atmosModule.planetRadius = 6378136.6   # meters
        atmosModule.baseDensity = 1.217  # kg/m^3
        atmosModule.scaleHeight = 8500.0 # meters
        atmosModule.localTemp = 293.0

    else:
        print("ERROR: " + name + " not setup for exponential atmosphere model\n")

    return





def centeredDipoleMagField(magFieldModule, name):
    """
    Sets the centered dipole magnetic field model parameters for a particular planet

    :param magFieldModule: magnetic field environment module
    :param name: planet name string

    """
    if name == "earth":
        # The following parameters are from the 2020 IGRF model
        # (https://www.ngdc.noaa.gov/IAGA/vmod/igrf.html)
        magFieldModule.g10 = -30926.00/1e9     # Tesla
        magFieldModule.g11 =  -2318.00/1e9     # Tesla
        magFieldModule.h11 =   5817.00/1e9     # Tesla
        magFieldModule.planetRadius = 6371.2*1000   # meters

    elif name == "mercury":
        # The following parameters are from NASA planetary fact sheet
        # (https://nssdc.gsfc.nasa.gov/planetary/planetfact.html)
        magFieldModule.planetRadius = 2440.0*1000   # meters
        convertToIgrfDipoleCoefficients(0.002/10000,        # [T]   dipole strength
                                        0.0*macros.D2R,     # [rad] Dipole tilt to rotational axis
                                        0.0*macros.D2R,     # [rad] Longitude of tilt
                                        magFieldModule)

    elif name == "jupiter":
        # The following parameters are from NASA planetary fact sheet
        # (https://nssdc.gsfc.nasa.gov/planetary/planetfact.html)
        magFieldModule.planetRadius = 71398.0*1000   # meters
        convertToIgrfDipoleCoefficients(4.30/10000,         # [T]   dipole strength
                                        9.4*macros.D2R,     # [rad] Dipole tilt to rotational axis
                                        200.1*macros.D2R,   # [rad] Longitude of tilt
                                        magFieldModule)

    elif name == "saturn":
        # The following parameters are from NASA planetary fact sheet
        # (https://nssdc.gsfc.nasa.gov/planetary/planetfact.html)
        magFieldModule.planetRadius = 60330.0*1000   # meters
        convertToIgrfDipoleCoefficients(0.215/10000,        # [T]   dipole strength
                                        0.0*macros.D2R,     # [rad] Dipole tilt to rotational axis
                                        0.0*macros.D2R,     # [rad] Longitude of tilt
                                        magFieldModule)

    elif name == "uranus":
        # The following parameters are from NASA planetary fact sheet
        # (https://nssdc.gsfc.nasa.gov/planetary/planetfact.html)
        magFieldModule.planetRadius = 25600.0*1000   # meters
        convertToIgrfDipoleCoefficients(0.228/10000,        # [T]   dipole strength
                                        58.6*macros.D2R,    # [rad] Dipole tilt to rotational axis
                                        53.6*macros.D2R,    # [rad] Longitude of tilt
                                        magFieldModule)

    elif name == "neptune":
        # The following parameters are from NASA planetary fact sheet
        # (https://nssdc.gsfc.nasa.gov/planetary/planetfact.html)
        magFieldModule.planetRadius = 24765.0*1000   # meters
        convertToIgrfDipoleCoefficients(0.142/10000,        # [T]   dipole strength
                                        46.9*macros.D2R,    # [rad] Dipole tilt to rotational axis
                                        288.*macros.D2R,    # [rad] Longitude of tilt
                                        magFieldModule)

    else:
        print("ERROR: " + name + " not setup for centered dipole magnetic field model. Options include mercury, earth, jupiter, saturn, uranus and neptune. \n")

    return



def convertToIgrfDipoleCoefficients(nominalField, tilt, longitudeOfTilt, magFieldModule):
    """
    Converts the NASA Magnetosphere parameters from https://nssdc.gsfc.nasa.gov/planetary/planetfact.html
    to IGRF compatible dipole coefficients.

    :param nominalField: nominal magnetic field parameter given in Tesla
    :param tilt: Dipole tilt to rotational axis in radians
    :param longitudeOfTilt: Longitude of tilt in radians
    :param magFieldModule: magnetic field environment module
    """

    # the following conversion is taken from Appendix D of doi:10.1007/978-1-4939-0802-8
    theta_m = np.pi - tilt
    alpha_m = np.pi - longitudeOfTilt
    magFieldModule.g11 = nominalField*np.sin(theta_m)*np.cos(alpha_m)
    magFieldModule.h11 = nominalField*np.sin(theta_m)*np.sin(alpha_m)
    magFieldModule.g10 = nominalField*np.cos(theta_m)

    return


