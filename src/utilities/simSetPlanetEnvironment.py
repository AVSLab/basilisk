''' '''
'''
 ISC License

 Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

'''

from Basilisk.simulation import atmosphere


def exponentialAtmosphere(atmosModule,name):
    """
    Sets the exponential atmosphere model parameters for Earth
    :param atmosModule: atmospheric environment module
    :param name: planet name string
    :return:
    """
    if name is "earth":
        atmosModule.setEnvType(atmosphere.MODEL_EXPONENTIAL)
        atmosModule.planetRadius = 6378136.6   # meters
        atmosModule.exponentialParams.baseDensity = 1.217  # kg/m^3
        atmosModule.exponentialParams.scaleHeight = 8500.0 # meters

    else:
        print "ERROR: " + name + " not setup for exponential atmosphere model\n"

    return

