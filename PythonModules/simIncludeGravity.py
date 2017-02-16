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
import pyswice

gravBodyList = []
spiceObject = 0
spicePlanetNames = []

def addSun():
    global gravBodyList
    global spicePlanetNames
    gravBody = gravityEffector.GravBodyData()
    gravBody.bodyInMsgName = "sun_planet_data"
    gravBody.outputMsgName = "sun_display_frame_data"
    gravBody.mu = 1.32712440018E20 # meters^3/s^2
    gravBody.radEquator = 695508000.0 # meters
    gravBody.isCentralBody = False
    gravBody.useSphericalHarmParams = False
    gravBodyList.append(gravBody)
    spicePlanetNames.append(gravBody.bodyInMsgName[:-12])

    return

def addMercury():
    global gravBodyList
    global spicePlanetNames
    gravBody = gravityEffector.GravBodyData()
    gravBody.bodyInMsgName = "mercury_planet_data"
    gravBody.outputMsgName = "mercury_display_frame_data"
    gravBody.mu = 4.28283100e13 # meters^3/s^2
    gravBody.radEquator = 2439700.0 # meters
    gravBody.isCentralBody = False
    gravBody.useSphericalHarmParams = False
    gravBodyList.append(gravBody)
    spicePlanetNames.append(gravBody.bodyInMsgName[:-12])

    return

def addVenus():
    global gravBodyList
    global spicePlanetNames
    gravBody = gravityEffector.GravBodyData()
    gravBody.bodyInMsgName = "venus_planet_data"
    gravBody.outputMsgName = "venus_display_frame_data"
    gravBody.mu = 3.24858599e14 # meters^3/s^2
    gravBody.radEquator = 6051800.0 # meters
    gravBody.isCentralBody = False
    gravBody.useSphericalHarmParams = False
    gravBodyList.append(gravBody)
    spicePlanetNames.append(gravBody.bodyInMsgName[:-12])

    return

def addEarth():
    global gravBodyList
    global spicePlanetNames
    gravBody = gravityEffector.GravBodyData()
    gravBody.bodyInMsgName = "earth_planet_data"
    gravBody.outputMsgName = "earth_display_frame_data"
    gravBody.mu = 0.3986004415E+15 # meters^3/s^2
    gravBody.radEquator = 6378136.6 # meters
    gravBody.isCentralBody = False
    gravBody.useSphericalHarmParams = False
    gravBodyList.append(gravBody)
    spicePlanetNames.append(gravBody.bodyInMsgName[:-12])

    return

def addMoon():
    global gravBodyList
    global spicePlanetNames
    gravBody = gravityEffector.GravBodyData()
    gravBody.bodyInMsgName = "moon_planet_data"
    gravBody.outputMsgName = "moon_display_frame_data"
    gravBody.mu = 4.902799E12 # meters^3/s^2
    gravBody.radEquator = 1738100.0 # meters
    gravBody.isCentralBody = False
    gravBody.useSphericalHarmParams = False
    gravBodyList.append(gravBody)
    spicePlanetNames.append(gravBody.bodyInMsgName[:-12])

    return

def addMars():
    global gravBodyList
    global spicePlanetNames
    gravBody = gravityEffector.GravBodyData()
    gravBody.bodyInMsgName = "mars barycenter_planet_data"
    gravBody.outputMsgName = "mars_barycenter_display_frame_data"
    gravBody.mu = 4.28283100e13 # meters^3/s^2
    gravBody.radEquator = 3396190 # meters
    gravBody.isCentralBody = False
    gravBody.useSphericalHarmParams = False
    gravBodyList.append(gravBody)
    spicePlanetNames.append(gravBody.bodyInMsgName[:-12])

    return


def addJupiter():
    global gravBodyList
    global spicePlanetNames
    gravBody = gravityEffector.GravBodyData()
    gravBody.bodyInMsgName = "jupiter barycenter_planet_data"
    gravBody.outputMsgName = "jupiter_display_frame_data"
    gravBody.mu = 1.266865349093058E17 # meters^3/s^2
    gravBody.radEquator = 71492000.0 # meters
    gravBody.isCentralBody = False
    gravBody.useSphericalHarmParams = False
    gravBodyList.append(gravBody)
    spicePlanetNames.append(gravBody.bodyInMsgName[:-12])

    return

def addSaturn():
    global spicePlanetNames
    global gravBodyList
    gravBody = gravityEffector.GravBodyData()
    gravBody.bodyInMsgName = "saturn barycenter_planet_data"
    gravBody.outputMsgName = "saturn_display_frame_data"
    gravBody.mu = 3.79395000E16 # meters^3/s^2
    gravBody.radEquator = 60268000.0 # meters
    gravBody.isCentralBody = False
    gravBody.useSphericalHarmParams = False
    gravBodyList.append(gravBody)
    spicePlanetNames.append(gravBody.bodyInMsgName[:-12])

    return

def addUranus():
    global spicePlanetNames
    global gravBodyList
    gravBody = gravityEffector.GravBodyData()
    gravBody.bodyInMsgName = "uranus barycenter_planet_data"
    gravBody.outputMsgName = "uranus_display_frame_data"
    gravBody.mu = 5.79396566E15 # meters^3/s^2
    gravBody.radEquator = 25559000.0 # meters
    gravBody.isCentralBody = False
    gravBody.useSphericalHarmParams = False
    gravBodyList.append(gravBody)
    spicePlanetNames.append(gravBody.bodyInMsgName[:-12])

    return

def addNeptune():
    global spicePlanetNames
    global gravBodyList
    gravBody = gravityEffector.GravBodyData()
    gravBody.bodyInMsgName = "neptune barycenter_planet_data"
    gravBody.outputMsgName = "neptune_display_frame_data"
    gravBody.mu = 6.83509920E15 # meters^3/s^2
    gravBody.radEquator = 24764000.0 # meters
    gravBody.isCentralBody = False
    gravBody.useSphericalHarmParams = False
    gravBodyList.append(gravBody)
    spicePlanetNames.append(gravBody.bodyInMsgName[:-12])

    return



def defaultEphemData(name):
    ephemData = spice_interface.SpicePlanetStateSimMsg()
    ephemData.J2000Current = 0.0
    ephemData.PositionVector = [0.0, 0.0, 0.0]
    ephemData.VelocityVector = [0.0, 0.0, 0.0]
    ephemData.J20002Pfix = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    ephemData.J20002Pfix_dot = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
    ephemData.PlanetName = name

    return ephemData

def clearSetup():
    global gravBodyList
    global spiceObject
    global spicePlanetNames

    gravBodyList = []
    spiceObject = 0
    spicePlanetNames = []

    return

def addSpiceInterface(path, timeString):
    global spicePlanetNames
    global spiceObject

    # setup SPICE ephemerise support
    spiceObject = spice_interface.SpiceInterface()
    spiceObject.ModelTag = "SpiceInterfaceData"
    spiceObject.SPICEDataPath = path + 'External/EphemerisData/'
    spiceObject.OutputBufferCount = 10000
    spiceObject.PlanetNames = spice_interface.StringVector(spicePlanetNames)

    #
    # pull in SPICE support libraries
    #
    pyswice.furnsh_c(spiceObject.SPICEDataPath + 'de430.bsp')           # solar system bodies
    pyswice.furnsh_c(spiceObject.SPICEDataPath + 'naif0011.tls')        # leap second file
    pyswice.furnsh_c(spiceObject.SPICEDataPath + 'de-403-masses.tpc')   # solar system masses
    pyswice.furnsh_c(spiceObject.SPICEDataPath + 'pck00010.tpc')        # generic Planetary Constants Kernel

    spiceObject.UTCCalInit = timeString


    return


def unloadDefaultSpiceLibraries():
    global spiceObject

    pyswice.unload_c(spiceObject.SPICEDataPath + 'de430.bsp')
    pyswice.unload_c(spiceObject.SPICEDataPath + 'naif0011.tls')
    pyswice.unload_c(spiceObject.SPICEDataPath + 'de-403-masses.tpc')
    pyswice.unload_c(spiceObject.SPICEDataPath + 'pck00010.tpc')

    return

def addDefaultEphemerisMsg(obj,processName):
    global gravBodyList

    if len(gravBodyList)>1:
        print "WARNING:  addDefaultEphemerisMsg() should not be used if more than one gravity body is setup."
        exit()

    msgName = gravBodyList[0].bodyInMsgName
    ephemData = defaultEphemData(msgName)
    messageSize = ephemData.getStructSize()
    obj.CreateNewMessage(processName,
                         msgName, messageSize, 2)
    obj.WriteMessageData(msgName, messageSize, 0,
                                    ephemData)
    return
