'''This document contains technology controlled by the U.S. Export Administration 
Regulations, 15 C.F.R. Parts 740-774 (EAR). Transfer, disclosure, or export to 
foreign persons without prior U.S. Government approval may be prohibited. 
Violations of these export laws and regulations are subject to severe civil 
and criminal penalties.

'''
import sys, os, inspect
filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
sys.path.append(path + '/../PythonModules/')
import AVSSim
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import numpy as np
import ctypes
import MessagingAccess
import sim_model
import spice_interface
import logging
import attMnvrPoint


def initializeAVSCruiseSunPoint():
    theSim = AVSSim.AVSSim()
    # theSim.TotalSim.logThisMessage("acs_thruster_cmds", int(1E9))
    # theSim.TotalSim.logThisMessage("nom_att_guid_out", int(1E9))
    # theSim.TotalSim.logThisMessage("dv_thruster_cmds", int(1E8))
    # theSim.TotalSim.logThisMessage("sun_safe_att_err", int(1E10))
    theSim.TotalSim.logThisMessage("inertial_state_output", int(1E9))
    theSim.TotalSim.logThisMessage("spacecraft_mass_props", int(1E9))
    # theSim.TotalSim.logThisMessage("OrbitalElements", int(1E10))
    # theSim.TotalSim.logThisMessage("css_wls_est", int(1E10))
    # theSim.TotalSim.logThisMessage("controlTorqueRaw", int(1E10))
    # theSim.TotalSim.logThisMessage("spacecraft_mass_props", int(1E10))
    # theSim.AddVectorForLogging('CSSPyramid1HeadA.sHatStr', 'double', 0, 2, int(1E10))
    # theSim.AddVariableForLogging('CSSWlsEst.numActiveCss', int(1E10))
    # theSim.AddVectorForLogging('attMnvrPoint.sigmaCmd_BR', 'double', 0, 2, int(1E10))
    # theSim.AddVectorForLogging('attMnvrPoint.omegaCmd_BR_B', 'double', 0, 2, int(1E10))
    # theSim.AddVectorForLogging('VehicleDynamicsData.omega', 'double', 0, 2,  int(1E10))
    # theSim.AddVariableForLogging('dvGuidance.burnExecuting', int(1E9))
    # theSim.AddVariableForLogging('dvGuidance.burnComplete', int(1E9))
    # theSim.TotalSim.logThisMessage("high_gain_earth_bore", int(1E9))
    theSim.TotalSim.logThisMessage("solar_array_sun_bore", int(1E9))
    theSim.AddVectorForLogging('VehicleDynamicsData.totScAngMomentum_N', 'double', 0, 2, int(1E9))
    theSim.AddVariableForLogging('VehicleDynamicsData.totScAngMomentumMag', int(1E9))
    theSim.AddVariableForLogging('VehicleDynamicsData.totScRotKinEnergy', int(1E9))
    theSim.AddVariableForLogging('VehicleDynamicsData.scRotPower', int(1E7))
    theSim.AddVariableForLogging('VehicleDynamicsData.scEnergyRate', int(1E7))
    # theSim.TotalSim.logThisMessage("earth_planet_data", int(1E9))

    # Approx T+1 month from MRO launch
    theSim.SpiceObject.UTCCalInit = "2005 October 15, 00:00:00"
    theSim.SpiceObject.referenceBase = "ECLIPJ2000"

    spiceLocal = spice_interface.SpiceInterface()
    spiceLocal.loadSpiceKernel('../External/EphemerisData/mro_cruise.bsp',
         path + '/')
    spiceLocal.SPICEDataPath = theSim.simBasePath + 'External/EphemerisData/'
    spiceLocal.UTCCalInit = theSim.SpiceObject.UTCCalInit
    spiceLocal.OutputBufferCount = 2
    spiceLocal.PlanetNames = spice_interface.StringVector(["-74"])
    spiceLocal.referenceBase = "ECLIPJ2000"
    spiceLocal.zeroBase = "SSB"
    spiceLocal.SelfInit()
    spiceLocal.UpdateState(0)
    mroName = "-74"
    spacecraftMessageName = mroName + "_planet_data"

    mroPosition = theSim.pullMessageLogData(spacecraftMessageName + ".PositionVector", range(3))
    mroVelocity = theSim.pullMessageLogData(spacecraftMessageName + ".VelocityVector", range(3))
    theSim.VehDynObject.PositionInit = sim_model.DoubleVector(mroPosition[0, 1:].tolist())
    theSim.VehDynObject.VelocityInit = sim_model.DoubleVector(mroVelocity[0, 1:].tolist())
    return theSim


def executeMROCruiseSunPoint(TheAVSSim):
    theSim.InitializeSimulation()
    theSim.modeRequest = 'safeMode'
    theSim.ConfigureStopTime(int((5*1E9)))
    theSim.ExecuteSimulation()
    theSim.modeRequest = 'sunPoint'
    theSim.ConfigureStopTime(int((60*20*1E9)))
    theSim.ExecuteSimulation()

if __name__ == "__main__":
    theSim = initializeAVSCruiseSunPoint()
    theSim.isUsingVisualization = True
    executeMROCruiseSunPoint(theSim)

    # FSWsHat = TheAVSSim.pullMessageLogData("css_wls_est.sHatBdy", range(3))
    # DataCSSTruth = TheAVSSim.GetLogVariableData('CSSPyramid1HeadA.sHatStr')
    # numCSSActive = TheAVSSim.GetLogVariableData('CSSWlsEst.numActiveCss')
    # attMnvrCmd = TheAVSSim.GetLogVariableData('attMnvrPoint.sigmaCmd_BR')
    # bodyRateCmd = TheAVSSim.GetLogVariableData('attMnvrPoint.omegaCmd_BR_B')
    # FSWControlOut = TheAVSSim.pullMessageLogData("controlTorqueRaw.torqueRequestBody", range(3))
    vehInertia = theSim.pullMessageLogData("spacecraft_mass_props.InertiaTensor", range(9))
    # bodyRateObs =  TheAVSSim.GetLogVariableData('VehicleDynamicsData.omega')
    dataSigma = theSim.pullMessageLogData("inertial_state_output.sigma", range(3))
    dataomega = theSim.pullMessageLogData("inertial_state_output.omega", range(3))
    # SCPos = TheAVSSim.pullMessageLogData("inertial_state_output.r_N", range(3))
    # DataDV = TheAVSSim.pullMessageLogData("inertial_state_output.TotalAccumDVBdy", range(3))
    # thrustLog = TheAVSSim.pullMessageLogData("dv_thruster_cmds.effectorRequest", range(6))
    # semiMajor = TheAVSSim.pullMessageLogData("OrbitalElements.a")
    # posMag = TheAVSSim.pullMessageLogData("OrbitalElements.rmag")
    # radApo = TheAVSSim.pullMessageLogData("OrbitalElements.rApoap")
    # radPeri = TheAVSSim.pullMessageLogData("OrbitalElements.rPeriap")
    # ecc = TheAVSSim.pullMessageLogData("OrbitalElements.e")
    # inc = TheAVSSim.pullMessageLogData("OrbitalElements.i")
    # RightAsc = TheAVSSim.pullMessageLogData("OrbitalElements.Omega")
    # argperi = TheAVSSim.pullMessageLogData("OrbitalElements.omega")
    # TrueAnom = TheAVSSim.pullMessageLogData('OrbitalElements.f')
    # AntBoreAngleMiss = TheAVSSim.pullMessageLogData('high_gain_earth_bore.missAngle')
    solarBoreAnglesMiss = theSim.pullMessageLogData('solar_array_sun_bore.missAngle')
    totAngMom_N = theSim.GetLogVariableData('VehicleDynamicsData.totScAngMomentum_N')
    totAngMom = theSim.GetLogVariableData('VehicleDynamicsData.totScAngMomentumMag')
    totScRotKinEnergy = theSim.GetLogVariableData('VehicleDynamicsData.totScRotKinEnergy')
    scRotPower = theSim.GetLogVariableData('VehicleDynamicsData.scRotPower')
    scEnergyRate = theSim.GetLogVariableData('VehicleDynamicsData.scEnergyRate')

    # AntBoreAngleAzi = TheAVSSim.pullMessageLogData('high_gain_earth_bore.azimuth')

    # EarthPos = TheAVSSim.pullMessageLogData('earth_planet_data.PositionVector', range(3))
    #
    # burnActive = TheAVSSim.GetLogVariableData('dvGuidance.burnExecuting')
    # burnComplete = TheAVSSim.GetLogVariableData('dvGuidance.burnComplete')
    # sigmaError = TheAVSSim.pullMessageLogData('nom_att_guid_out.sigma_BR', range(3))

    # print semiMajor[0,1]
    # print ecc[0,1]
    # print inc[0,1]
    # print RightAsc[0,1]
    # print argperi[0,1]
    # print radApo[0,1]
    # print radPeri[0,1]
    # print TrueAnom[0,1]
    # print '\n'
    #
    # print semiMajor[-1,1]
    # print ecc[-1,1]
    # print inc[-1,1]
    # print RightAsc[-1,1]
    # print argperi[-1,1]
    # print radApo[-1,1]
    # print radPeri[-1,1]
    # print TrueAnom[-1,1]
    # print '\n'
    #
    # print DataDV[-1,:]
    #
    # print AntBoreAngleMiss[-1,1]
    # print AntBoreAngleAzi[-1,1]
    #
    #
    # print np.cos(AntBoreAngleMiss[-1,1])




    # CSSEstAccuracyThresh = 17.5*np.pi/180.0
    # plt.figure(1)
    # plt.plot(FSWsHat[:,0]*1.0E-9, FSWsHat[:,1], 'b', DataCSSTruth[:,0]*1.0E-9, DataCSSTruth[:,1], 'b--')
    # plt.plot(FSWsHat[:,0]*1.0E-9, FSWsHat[:,2], 'g', DataCSSTruth[:,0]*1.0E-9, DataCSSTruth[:,2], 'g--')
    # plt.plot(FSWsHat[:,0]*1.0E-9, FSWsHat[:,3], 'r', DataCSSTruth[:,0]*1.0E-9, DataCSSTruth[:,3], 'r--')

    #plt.figure(2)
    #plt.plot(numCSSActive[:,0]*1.0E-9, numCSSActive[:,1])

    # plt.figure(3)
    # plt.plot(DataSigma[:,0]*1.0E-9, DataSigma[:,1], 'b--', attMnvrCmd[:,0]*1.0E-9, -attMnvrCmd[:,1], 'b')
    # plt.plot(DataSigma[:,0]*1.0E-9, DataSigma[:,2], 'r--', attMnvrCmd[:,0]*1.0E-9, -attMnvrCmd[:,2], 'r')
    # plt.plot(DataSigma[:,0]*1.0E-9, DataSigma[:,3], 'g--', attMnvrCmd[:,0]*1.0E-9, -attMnvrCmd[:,3], 'g')

    #plt.figure(4)
    #plt.plot(FSWControlOut[:,0]*1.0E-9, FSWControlOut[:,1])
    #plt.plot(FSWControlOut[:,0]*1.0E-9, FSWControlOut[:,2])
    #plt.plot(FSWControlOut[:,0]*1.0E-9, FSWControlOut[:,3])

    # plt.figure(5)
    # plt.plot(bodyRateObs[:,0]*1.0E-9, bodyRateObs[:,1], 'b--', bodyRateCmd[:,0]*1.0E-9, bodyRateCmd[:,1], 'b')
    # plt.plot(bodyRateObs[:,0]*1.0E-9, bodyRateObs[:,2], 'r--', bodyRateCmd[:,0]*1.0E-9, bodyRateCmd[:,2], 'r')
    # plt.plot(bodyRateObs[:,0]*1.0E-9, bodyRateObs[:,3], 'g--', bodyRateCmd[:,0]*1.0E-9, bodyRateCmd[:,3], 'g')

    #plt.figure(6)
    #plt.plot(DataDV[:,0]*1.0E-9, DataDV[:,1])
    #plt.plot(DataDV[:,0]*1.0E-9, DataDV[:,2])
    #plt.plot(DataDV[:,0]*1.0E-9, DataDV[:,3])

    #plt.figure(7)
    #plt.plot(vehInertia[:,0]*1.0E-9, vehInertia[:,1])
    #plt.plot(vehInertia[:,0]*1.0E-9, vehInertia[:,5])
    #plt.plot(vehInertia[:,0]*1.0E-9, vehInertia[:,9])

    #plt.figure(8)
    #plt.plot(thrustLog[:,0]*1.0E-9, thrustLog[:,1])
    #plt.plot(thrustLog[:,0]*1.0E-9, thrustLog[:,2])
    #plt.plot(thrustLog[:,0]*1.0E-9, thrustLog[:,3])
    #plt.plot(thrustLog[:,0]*1.0E-9, thrustLog[:,4])
    #plt.plot(thrustLog[:,0]*1.0E-9, thrustLog[:,5])
    #plt.plot(thrustLog[:,0]*1.0E-9, thrustLog[:,6])

    #plt.figure(9)
    #plt.plot(semiMajor[:,0]*1.0E-9, semiMajor[:,1])

    #plt.figure(10)
    #plt.plot(posMag[:,0]*1.0E-9, posMag[:,1])

    #plt.figure(11)
    #plt.plot(radApo[:,0]*1.0E-9, radApo[:,1])

    #plt.figure(12)
    #plt.plot(radPeri[:,0]*1.0E-9, radPeri[:,1])

    #plt.figure(13)
    #plt.plot(burnActive[:,0]*1.0E-9, burnActive[:,1])
    #plt.plot(burnComplete[:,0]*1.0E-9, burnComplete[:,1], 'g--')

    #plt.figure(14)
    #plt.plot(TrueAnom[:,0]*1E-9, TrueAnom[:,1])

    #plt.figure(14)
    #plt.plot(sigmaError[:,0]*1.0E-9, sigmaError[:,1])
    #plt.plot(sigmaError[:,0]*1.0E-9, sigmaError[:,2])
    #plt.plot(sigmaError[:,0]*1.0E-9, sigmaError[:,3])

    # plt.figure(15)
    # plt.plot(AntBoreAngleMiss[:,0]*1.0E-9, AntBoreAngleMiss[:,1]*180/np.pi)

    plt.figure(16)
    plt.plot(solarBoreAnglesMiss[:, 0]*1.0E-9, solarBoreAnglesMiss[:, 1]*180/np.pi)

    plt.figure(17)
    plt.plot(totAngMom_N[:, 0] * 1E-9/60.0, totAngMom_N[:, 1], totAngMom_N[:, 0] * 1E-9/60.0, totAngMom_N[:, 2], totAngMom_N[:, 0] * 1E-9/60.0, totAngMom_N[:, 3])
    plt.xlabel('Time (min)')
    plt.ylabel('Inertial Angular Momentum Vector')

    plt.figure(18)
    plt.plot(totAngMom[:, 0] * 1E-9/60.0, (totAngMom[:, 1]-totAngMom[0, 1]))
    plt.xlabel('Time (min)')
    plt.ylabel('Change in Inertial Angular Momentum Magnitude')

    plt.figure(19)
    plt.plot(totScRotKinEnergy[:, 0] * 1E-9/60.0, totScRotKinEnergy[:, 1])
    plt.xlabel('Time (min)')
    plt.ylabel('Rotational Kinetic Energy')

    plt.figure(20)
    plt.plot(scRotPower[:, 0] * 1E-9/60.0, scRotPower[:, 1], label="Power")
    plt.plot(scEnergyRate[:, 0] * 1E-9/60.0, scEnergyRate[:, 1], label="EnergyRate")
    plt.xlabel('Time (min)')
    plt.ylabel('Rotational Power vs Energy Rate')
    plt.legend()

    plt.figure(21)
    plt.plot(dataomega[:, 0]*1E-9/60.0, dataomega[:, 1], dataomega[:, 0]*1E-9/60.0, dataomega[:, 2], dataomega[:, 0]*1E-9/60.0, dataomega[:, 3])
    plt.xlabel('Time (min)')
    plt.ylabel('Omega_B')

    plt.figure(22)
    plt.plot(dataSigma[:, 0]*1E-9/60.0, dataSigma[:, 1], dataSigma[:, 0]*1E-9/60.0, dataSigma[:, 2], dataSigma[:, 0]*1E-9/60.0, dataSigma[:, 3])
    plt.xlabel('Time (min)')
    plt.ylabel('Sigma')

    # plt.figure(17)
    # plt.plot(AntBoreAngleAzi[:,0]*1E-9,AntBoreAngleAzi[:,1]*180/np.pi)

    plt.show()

    #sys.exit()
