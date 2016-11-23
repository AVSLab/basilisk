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

import numpy as np
import sys, os, inspect
filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
splitPath = path.split('SimCode')
sys.path.append(splitPath[0] + '/modules')
sys.path.append(splitPath[0] + '/PythonModules')


import six_dof_eom
import spice_interface
import six_dof_eom
import SimulationBaseClass

#Define success criteria
propTime = int(3600*24*2*1E9) #Propagate for 2 days worth of nanoseconds
allowPosError = 50000 #Allow for a 50 km error over 10 days of propagation
allowVelError = 0.1 #Allow for the velocity to degrade by 10 cm/s


mu_earth = 0.3986004415E+15 # [m^3/s^2]
reference_radius_earth = 0.6378136300E+07 # [m]
max_degree_earth = 10

mu_mars= 4.2828371901284001E+13 # [m^3/s^2]
reference_radius_mars = 3.3970000000000000E+06 # [m]
max_degree_mars = 10

EarthGravFile = splitPath[0] + 'External/LocalGravData/GGM03S.txt'
MarsGravFile = splitPath[0] +'External/LocalGravData/GGM2BData.txt'

#Create a sim module as an empty container
testSim = SimulationBaseClass.SimBaseClass()
testProc = testSim.CreateNewProcess("TestProc")
testProc.addTask(testSim.CreateNewTask("sixDynTestTask", int(1E10)))

#Now initialize the modules that we are using.  I got a little better as I went along
VehDynObject = six_dof_eom.SixDofEOM()
spiceObject = spice_interface.SpiceInterface()

#Initialize the ephemeris module
spiceObject.ModelTag = "SpiceInterfaceData"
spiceObject.SPICEDataPath = splitPath[0] + 'External/EphemerisData/'
spiceObject.UTCCalInit = "2014 March 27, 14:00:00.0"
spiceObject.OutputBufferCount = 2
spiceObject.PlanetNames = spice_interface.StringVector(
    ["earth", "sun"])
spiceObject.loadSpiceKernel('de430.bsp', spiceObject.SPICEDataPath)
#spiceObject.loadSpiceKernel('jup260.bsp', path + '/')

SunGravBody = six_dof_eom.GravityBodyData()
SunGravBody.BodyMsgName = "sun_planet_data"
SunGravBody.outputMsgName = "sun_display_frame_data"
SunGravBody.mu = 1.32712440018E20 #meters!
SunGravBody.IsCentralBody = False
SunGravBody.IsDisplayBody = False
SunGravBody.UseJParams = False

EarthGravBody = six_dof_eom.GravityBodyData(EarthGravFile, max_degree_earth, mu_earth, reference_radius_earth)
#EarthGravBody = six_dof_eom.GravityBodyData()
EarthGravBody.BodyMsgName = "earth_planet_data"
EarthGravBody.outputMsgName = "earth_display_frame_data"
EarthGravBody.IsCentralBody = True
EarthGravBody.IsDisplayBody = False

# MarsGravBody = six_dof_eom.GravityBodyData(MarsGravFile, max_degree_mars, mu_mars, reference_radius_mars)
# #MarsGravBody = six_dof_eom.GravityBodyData()
# MarsGravBody.BodyMsgName = "mars_planet_data"
# MarsGravBody.outputMsgName = "mars_display_frame_data"
# MarsGravBody.IsCentralBody = False
# MarsGravBody.IsDisplayBody = False

VehDynObject.ModelTag = "VehicleDynamicsData"
VehDynObject.PositionInit = six_dof_eom.DoubleVector([8.69348239e+06, 3.46901324e+06, 1.62459568e+06])
VehDynObject.VelocityInit = six_dof_eom.DoubleVector([-1.47129063e+03, 6.72412434e+02, 6.43731339e+03])
#Note that the above position/velocity get overwritten by the ICs from the target ephemeris
VehDynObject.AttitudeInit = six_dof_eom.DoubleVector([0.4, 0.2, 0.1])
VehDynObject.AttRateInit = six_dof_eom.DoubleVector([0.0001, 0.0, 0.0])
VehDynObject.baseMass = 1500.0 - 812.3
VehDynObject.baseInertiaInit = six_dof_eom.DoubleVector([900, 0.0, 0.0,
                                                         0.0, 900.0, 0.0,
                                                         0.0, 0.0, 900.0])
VehDynObject.T_Str2BdyInit = six_dof_eom.DoubleVector([1.0, 0.0, 0.0,
                                                           0.0, 1.0, 0.0,
                                                           0.0, 0.0, 1.0])
VehDynObject.baseCoMInit = six_dof_eom.DoubleVector([0.0, 0.0, 1.0])
#Add the three gravity bodies in to the simulation
VehDynObject.AddGravityBody(SunGravBody)
VehDynObject.AddGravityBody(EarthGravBody)

testSim.AddModelToTask("sixDynTestTask", spiceObject)
testSim.AddModelToTask("sixDynTestTask", VehDynObject)
#TotalSim.AddModelToTask("sixDynTestTaskMars", spiceObject)
#TotalSim.AddModelToTask("sixDynTestTaskMars", VehDynObject)

testSim.TotalSim.logThisMessage("inertial_state_output", int(1E12))

testSim.InitializeSimulation()
testSim.ConfigureStopTime(60*1e9) #Just a simple run to get initial conditions from ephem
testSim.ExecuteSimulation()