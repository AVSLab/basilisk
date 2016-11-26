''' '''
'''
 ISC License

 Copyright (c) 2016-2017, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#
#   support file to simplify the inclusion of gravity objects for
#   a range of celestial objects
#

import gravityEffector
import spice_interface


def addEarth():
    gravBody = gravityEffector.GravBodyData()
    gravBody.bodyMsgName = "earth_planet_data"
    gravBody.outputMsgName = "earth_display_frame_data"
    gravBody.mu = 0.3986004415E+15 # meters!
    gravBody.radEquator = 6378136.6 # meters
    gravBody.isCentralBody = False
    gravBody.useSphericalHarmParams = False

    ephemData = spice_interface.SpicePlanetState()
    ephemData.J2000Current = 0.0
    ephemData.PositionVector = [0.0, 0.0, 0.0]
    ephemData.VelocityVector = [0.0, 0.0, 0.0]
    ephemData.J20002Pfix = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    ephemData.J20002Pfix_dot = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
    ephemData.PlanetName = "earth"

    return gravBody, ephemData

def addMars():
    gravBody = gravityEffector.GravBodyData()
    gravBody.bodyMsgName = "mars_barycenter_planet_data"
    gravBody.outputMsgName = "mars_barycenter_display_frame_data"
    gravBody.mu = 4.28283100e13 # meters!
    gravBody.radEquator = 3396190 # meters
    gravBody.isCentralBody = False
    gravBody.useSphericalHarmParams = False

    ephemData = spice_interface.SpicePlanetState()
    ephemData.J2000Current = 0.0
    ephemData.PositionVector = [0.0, 0.0, 0.0]
    ephemData.VelocityVector = [0.0, 0.0, 0.0]
    ephemData.J20002Pfix = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    ephemData.J20002Pfix_dot = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
    ephemData.PlanetName = "mars"

    return gravBody, ephemData
