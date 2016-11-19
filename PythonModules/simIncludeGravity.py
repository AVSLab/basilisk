'''
Copyright (c) 2016, Autonomous Vehicle Systems Lab, Univeristy of Colorado at Boulder

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
    earthGravBody = gravityEffector.GravBodyData()
    earthGravBody.bodyMsgName = "earth_planet_data"
    earthGravBody.outputMsgName = "earth_display_frame_data"
    earthGravBody.mu = 0.3986004415E+15 # meters!
    earthGravBody.isCentralBody = False
    earthGravBody.useSphericalHarmParams = False
    return earthGravBody

def addEarthEphemData():
    earthEphemData = spice_interface.SpicePlanetState()
    earthEphemData.J2000Current = 0.0
    earthEphemData.PositionVector = [0.0, 0.0, 0.0]
    earthEphemData.VelocityVector = [0.0, 0.0, 0.0]
    earthEphemData.J20002Pfix = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    earthEphemData.J20002Pfix_dot = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
    earthEphemData.PlanetName = "earth"
    return earthEphemData