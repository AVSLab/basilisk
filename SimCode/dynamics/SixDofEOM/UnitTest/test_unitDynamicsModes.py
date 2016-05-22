import sys, os, inspect
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import numpy
import ctypes
import math
import csv
import logging

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
splitPath = path.split('Basilisk')
sys.path.append(splitPath[0] + '/Basilisk/modules')
sys.path.append(splitPath[0] + '/Basilisk/PythonModules')

import spice_interface
import six_dof_eom
import MessagingAccess
import SimulationBaseClass
import sim_model
import unitTestSupport                  # general support file with common unit test functions

TotalSim = SimulationBaseClass.SimBaseClass()
DynUnitTestProc = TotalSim.CreateNewProcess("DynUnitTestProcess")
DynUnitTestProc.addTask(TotalSim.CreateNewTask("sixDynTestTask", int(1E10)))

VehDynObject = six_dof_eom.SixDofEOM()
spiceObject = spice_interface.SpiceInterface()

#Initialize the ephemeris module
spiceObject.ModelTag = "SpiceInterfaceData"
spiceObject.SPICEDataPath = splitPath[0] + '/Basilisk/External/EphemerisData/'
spiceObject.UTCCalInit = "2014 March 27, 14:00:00.0"
spiceObject.OutputBufferCount = 2
spiceObject.PlanetNames = spice_interface.StringVector(
    ["earth"])
spiceObject.zeroBase = "earth"

mu_earth = 0.3986004415E+15 # [m^3/s^2]
reference_radius_earth = 0.6378136300E+07 # [m]
EarthGravBody = six_dof_eom.GravityBodyData()
EarthGravBody.BodyMsgName = "earth_planet_data"
EarthGravBody.outputMsgName = "earth_display_frame_data"
EarthGravBody.IsCentralBody = True
EarthGravBody.mu = mu_earth

VehDynObject.ModelTag = "VehicleDynamicsData"
VehDynObject.PositionInit = six_dof_eom.DoubleVector([-1.784938418967935e+11, -1.609707049168820e+10, -1.627664958116536e+09])
VehDynObject.VelocityInit = six_dof_eom.DoubleVector([-3.120111634914843e+03, -2.471539811502987e+04, -1.119615706657671e+04])
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

VehDynObject.AddGravityBody(EarthGravBody)

VehDynObject.useTranslation = 1;        # turn on translational mode 
VehDynObject.useRotation = 1;            # turn on rotational mode 

TotalSim.AddModelToTask("sixDynTestTask", spiceObject)
TotalSim.AddModelToTask("sixDynTestTask", VehDynObject)

TotalSim.InitializeSimulation()
TotalSim.ConfigureStopTime(unitTestSupport.sec2nano(60.)) #Just a simple run to get initial conditions from ephem
TotalSim.ExecuteSimulation()

#moduleOutput = TotalSim.pullMessageLogData('OutputStateData' + '.' + 'r_N', range(3))
#print moduleOutput

                                                    
                                                    
                                                
