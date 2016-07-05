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
import sys, os, inspect  # Don't worry about this, standard stuff plus file discovery

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
sys.path.append(path + '/../PythonModules/')
import AVSSim  # The simulation
import matplotlib.pyplot as plt  # shorthand makes plotting cmds easier
import ctypes  # This is to allow you to set pointers in your c-code
import math  # Isn't it obvious?
import sim_model  # Sometimes need utilities out of here
import logging  # Handy logging feature for test issues
import SimulationBaseClass  # Need this to access some pointer manipulation
import spice_interface  # Need a local copy of the SPICE interface for file manipulation
import RigidBodyKinematics
import csv


def generateAttCSV(EMMSim, outputName):
    with open(outputName, 'w') as csvfile:
        missWriter = csv.writer(csvfile, delimiter=',')
        missWriter.writerow(['Time (s)', 'Q Scalar (-)', 'Q Vector 1 (-)',
                             'Q Vector 2 (-)', 'Q Vector 3 (-)', 'Omega x (r/s)', 'Omega y (r/s)',
                             'Omega z (r/s)'])
        attitudeMRP = EMMSim.pullMessageLogData("inertial_state_output.sigma", range(3))
        attitudeRate = EMMSim.pullMessageLogData("inertial_state_output.omega", range(3))
        for i in range(attitudeMRP.shape[0]):
            currentAttitude = attitudeMRP[i, 1:4].reshape(3, 1)
            currentQuat = RigidBodyKinematics.MRP2EP(currentAttitude)
            currentRow = [attitudeMRP[i, 0] / 1.0E9]
            currentRow.extend(currentQuat.reshape(4).tolist()[0])
            currentRow.extend(attitudeRate[i, 1:4].reshape(3).tolist())
            missWriter.writerow(currentRow)


# Instantiate a copy of the EMM vehicle/FSW
TheAVSSim = AVSSim.AVSSim()
TheAVSSim.isUsingVisulatization = True

# Log a handful of messages to examine vehicle performance
TheAVSSim.TotalSim.logThisMessage("inertial_state_output", int(6E10))  # inertial states
# TheAVSSim.TotalSim.logThisMessage("att_cmd_output", int(1E10))  # inertial states
TheAVSSim.TotalSim.logThisMessage("OrbitalElements", int(6E10))  # orbital elements
# TheAVSSim.TotalSim.logThisMessage("css_wls_est", int(1E10))  # FSW weighted least squares sun-vector
# TheAVSSim.TotalSim.logThisMessage("spacecraft_mass_props", int(1E10))  # spacecraft mass properties
TheAVSSim.TotalSim.logThisMessage("solar_array_sun_bore", int(6E10))  # solar array boresight angles
# TheAVSSim.TotalSim.logThisMessage("high_gain_earth_bore", int(1E10))  # solar array boresight angles
# TheAVSSim.TotalSim.logThisMessage("instrument_mars_bore", int(1E10))  # solar array boresight angles
TheAVSSim.TotalSim.logThisMessage("reactionwheel_output_states", int(6E10))
# TheAVSSim.TotalSim.logThisMessage("controlTorqueRaw", int(1E9))

# Time of maneuver start is used to inject into mars orbit
spiceLocal = spice_interface.SpiceInterface()
spiceLocal.SPICEDataPath = TheAVSSim.simBasePath + 'External/EphemerisData/'
spiceLocal.loadSpiceKernel('../External/EphemerisData/mro_cruise.bsp',
                           path + '/')
spiceLocal.UTCCalInit = 'Oct 12,2005  00:00:00.0 (UTC)'
spiceLocal.OutputBufferCount = 2
spiceLocal.PlanetNames = spice_interface.StringVector(["-74"])
spiceLocal.referenceBase = "J2000"
spiceLocal.zeroBase = "SSB"
spiceLocal.SelfInit()
spiceLocal.UpdateState(0)
scName = "-74"

spacecraftMessageName = scName + "_planet_data"

scPosition = TheAVSSim.pullMessageLogData(spacecraftMessageName + ".PositionVector", range(3))
scVelocity = TheAVSSim.pullMessageLogData(spacecraftMessageName + ".VelocityVector", range(3))

TheAVSSim.VehDynObject.PositionInit = sim_model.DoubleVector(scPosition[-1, 1:].tolist())
TheAVSSim.VehDynObject.VelocityInit = sim_model.DoubleVector(scVelocity[-1, 1:].tolist())
print scPosition[0, :]
print scVelocity[0, :]
TheAVSSim.SpiceObject.UTCCalInit = spiceLocal.UTCCalInit
TheAVSSim.VehDynObject.baseCoMInit[0] = 0.05

# runTime = int(20*86400 * 1E9)  # 20 Day
runTime = int(60 * 1 * 1E9)
TheAVSSim.InitializeSimulation()
TheAVSSim.ConfigureStopTime(int(30 * 1E9))
TheAVSSim.ExecuteSimulation()
TheAVSSim.modeRequest = 'safeMode'
TheAVSSim.ConfigureStopTime(int(60 * 1 * 1E9))
TheAVSSim.ExecuteSimulation()
TheAVSSim.modeRequest = 'sunPoint'
TheAVSSim.ConfigureStopTime(runTime)
TheAVSSim.ExecuteSimulation()

semiMajor = TheAVSSim.pullMessageLogData("OrbitalElements.a")
posMag = TheAVSSim.pullMessageLogData("OrbitalElements.rmag")
radApo = TheAVSSim.pullMessageLogData("OrbitalElements.rApoap")
radPeri = TheAVSSim.pullMessageLogData("OrbitalElements.rPeriap")
DataSigma = TheAVSSim.pullMessageLogData("inertial_state_output.sigma", range(3))
# FSWsHat = TheAVSSim.pullMessageLogData("css_wls_est.sHatBdy", range(3))
rwLog = TheAVSSim.pullMessageLogData("reactionwheel_output_states.wheelSpeeds", range(4))
# rwJs = TheAVSSim.pullMessageLogData("reactionwheel_output_states")
# FSWControlOut = TheAVSSim.pullMessageLogData("controlTorqueRaw.torqueRequestBody", range(3))
solarArrayMiss = TheAVSSim.pullMessageLogData("solar_array_sun_bore.missAngle")

# Plotting
nano2Days = 1.0E-9 * 1.0 / 86400.0

fig, axarr = plt.subplots(3, sharex=True)
axarr[0].plot(semiMajor[:, 0] * nano2Days, semiMajor[:, 1])
axarr[0].set_title('Orbital Elements')
axarr[0].set_ylabel('Semi Major Axis (m)')
axarr[1].plot(radPeri[:, 0] * nano2Days, radPeri[:, 1])
axarr[1].set_ylabel('Radius of Periapses (m)')
axarr[2].plot(radApo[:, 0] * nano2Days, radApo[:, 1])
axarr[2].set_xlabel('Time (days)')
axarr[2].set_ylabel('Radius of Apoapsis (m)')

plt.figure(2)
plt.plot(DataSigma[:, 0] * nano2Days, DataSigma[:, 1], 'b')
plt.plot(DataSigma[:, 0] * nano2Days, DataSigma[:, 2], 'r')
plt.plot(DataSigma[:, 0] * nano2Days, DataSigma[:, 3], 'g')
plt.xlabel('Time (days)')
plt.ylabel('Attitude MRP (-)')

plt.figure(3)
plt.plot(rwLog[:, 0] * nano2Days, rwLog[:, 1] / (2.0 * math.pi) * 60.0, 'b')
plt.plot(rwLog[:, 0] * nano2Days, rwLog[:, 2] / (2.0 * math.pi) * 60.0, 'g')
plt.plot(rwLog[:, 0] * nano2Days, rwLog[:, 3] / (2.0 * math.pi) * 60.0, 'r')
plt.plot(rwLog[:, 0] * nano2Days, rwLog[:, 4] / (2.0 * math.pi) * 60.0, 'c')
plt.xlabel('Time (days)')
plt.ylabel('RWA Speeds (RPM)')

plt.figure(4)
plt.plot(solarArrayMiss[:, 0] * nano2Days, solarArrayMiss[:, 1] * 180 / math.pi)
plt.xlabel('Time (days)')
plt.ylabel('Solar Array Miss (d)')

plt.figure(5)
plt.plot(rwLog[:, 0] * nano2Days, rwLog[:, 1] * TheAVSSim.rwDynObject.ReactionWheelData[0].Js, 'b')
plt.plot(rwLog[:, 0] * nano2Days, rwLog[:, 2] * TheAVSSim.rwDynObject.ReactionWheelData[1].Js, 'g')
plt.plot(rwLog[:, 0] * nano2Days, rwLog[:, 3] * TheAVSSim.rwDynObject.ReactionWheelData[2].Js, 'r')
plt.plot(rwLog[:, 0] * nano2Days, rwLog[:, 4] * TheAVSSim.rwDynObject.ReactionWheelData[3].Js, 'c')
plt.xlabel('Time (days)')
plt.ylabel('Vehicle Angular Momentum (N.m.s)')

# If requested, generate plots
if (len(sys.argv) > 1):
    if (sys.argv[1] == 'True'):
        plt.show()
