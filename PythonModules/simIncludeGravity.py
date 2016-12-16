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

def addSun():
    gravBody = gravityEffector.GravBodyData()
    gravBody.bodyMsgName = "sun_planet_data"
    gravBody.outputMsgName = "sun_display_frame_data"
    gravBody.mu = 1.32712440018E20 # meters^3/s^2
    gravBody.radEquator = 695508000.0 # meters
    gravBody.isCentralBody = False
    gravBody.useSphericalHarmParams = False

    return gravBody, defaultEphemData("sun")

def addMercury():
    gravBody = gravityEffector.GravBodyData()
    gravBody.bodyMsgName = "mercury_planet_data"
    gravBody.outputMsgName = "mercury_display_frame_data"
    gravBody.mu = 4.28283100e13 # meters^3/s^2
    gravBody.radEquator = 2439700.0 # meters
    gravBody.isCentralBody = False
    gravBody.useSphericalHarmParams = False

    return gravBody, defaultEphemData("mercury")

def addVenus():
    gravBody = gravityEffector.GravBodyData()
    gravBody.bodyMsgName = "venus_planet_data"
    gravBody.outputMsgName = "venus_display_frame_data"
    gravBody.mu = 3.24858599e14 # meters^3/s^2
    gravBody.radEquator = 6051800.0 # meters
    gravBody.isCentralBody = False
    gravBody.useSphericalHarmParams = False

    return gravBody, defaultEphemData("venus")

def addEarth():
    gravBody = gravityEffector.GravBodyData()
    gravBody.bodyInMsgName = "earth_planet_data"
    gravBody.outputMsgName = "earth_display_frame_data"
    gravBody.mu = 0.3986004415E+15 # meters^3/s^2
    gravBody.radEquator = 6378136.6 # meters
    gravBody.isCentralBody = False
    gravBody.useSphericalHarmParams = False

    return gravBody, defaultEphemData("earth")

def addMoon():
    gravBody = gravityEffector.GravBodyData()
    gravBody.bodyMsgName = "moon_planet_data"
    gravBody.outputMsgName = "moon_display_frame_data"
    gravBody.mu = 4.902799E12 # meters^3/s^2
    gravBody.radEquator = 1738100.0 # meters
    gravBody.isCentralBody = False
    gravBody.useSphericalHarmParams = False

    return gravBody, defaultEphemData("moon")

def addMars():
    gravBody = gravityEffector.GravBodyData()
    gravBody.bodyInMsgName = "mars_barycenter_planet_data"
    gravBody.outputMsgName = "mars_barycenter_display_frame_data"
    gravBody.mu = 4.28283100e13 # meters^3/s^2
    gravBody.radEquator = 3396190 # meters
    gravBody.isCentralBody = False
    gravBody.useSphericalHarmParams = False

    return gravBody, defaultEphemData("mars")


def addJupiter():
    gravBody = gravityEffector.GravBodyData()
    gravBody.bodyMsgName = "jupiter_barycenter_planet_data"
    gravBody.outputMsgName = "jupiter_display_frame_data"
    gravBody.mu = 1.266865349093058E17 # meters^3/s^2
    gravBody.radEquator = 71492000.0 # meters
    gravBody.isCentralBody = False
    gravBody.useSphericalHarmParams = False

    return gravBody, defaultEphemData("jupiter")

def addSaturn():
    gravBody = gravityEffector.GravBodyData()
    gravBody.bodyMsgName = "saturn_barycenter_planet_data"
    gravBody.outputMsgName = "saturn_display_frame_data"
    gravBody.mu = 3.79395000E16 # meters^3/s^2
    gravBody.radEquator = 60268000.0 # meters
    gravBody.isCentralBody = False
    gravBody.useSphericalHarmParams = False

    return gravBody, defaultEphemData("saturn")

def addUranus():
    gravBody = gravityEffector.GravBodyData()
    gravBody.bodyMsgName = "uranus_barycenter_planet_data"
    gravBody.outputMsgName = "uranus_display_frame_data"
    gravBody.mu = 5.79396566E15 # meters^3/s^2
    gravBody.radEquator = 25559000.0 # meters
    gravBody.isCentralBody = False
    gravBody.useSphericalHarmParams = False

    return gravBody, defaultEphemData("uranus")

def addNeptune():
    gravBody = gravityEffector.GravBodyData()
    gravBody.bodyMsgName = "neptune_barycenter_planet_data"
    gravBody.outputMsgName = "neptune_display_frame_data"
    gravBody.mu = 6.83509920E15 # meters^3/s^2
    gravBody.radEquator = 24764000.0 # meters
    gravBody.isCentralBody = False
    gravBody.useSphericalHarmParams = False

    return gravBody, defaultEphemData("neptune")



def defaultEphemData(name):
    ephemData = spice_interface.SpicePlanetState()
    ephemData.J2000Current = 0.0
    ephemData.PositionVector = [0.0, 0.0, 0.0]
    ephemData.VelocityVector = [0.0, 0.0, 0.0]
    ephemData.J20002Pfix = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    ephemData.J20002Pfix_dot = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
    ephemData.PlanetName = name

    return ephemData