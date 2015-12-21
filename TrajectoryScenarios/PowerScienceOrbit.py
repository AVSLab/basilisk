import sys, os, inspect #Don't worry about this, standard stuff plus file discovery
filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
sys.path.append(path + '/../PythonModules/')
import matplotlib #plotting package
matplotlib.use('TkAgg') #Plotter dies quietly on OSX sometimes with default
import EMMSim #The simulation
import matplotlib.pyplot as plt #shorthand makes plotting cmds easier
import ctypes #This is to allow you to set pointers in your c-code
import math #Isn't it obvious?
import sim_model #Sometimes need utilities out of here
import numpy #Lets us interact with nump objects which work like matlab arrays
import logging #Handy logging feature for test issues
import SimulationBaseClass #Need this to access some pointer manipulation
import spice_interface #Need a local copy of the SPICE interface for file manipulation
import RigidBodyKinematics
import csv

#O1	38.5
#O2	71.5
#O3	91.5
#O4	25.5
#O5	65.5
#O6	66
#C1	112.82
#C2	86.32
#C3	59.82
#C4	72.57
#C5	105.57
#C6	85.32
#C7	92.82
#C8	119.32
#C9	99.07
#C10	125.82
#C11	79.82
#C12	96.32
#C13	119.32
#C14	102.82
#C15	105.82
#C16	99.32

class SequenceElement:
 def __init__(self, times, modeSet = 'sunPoint', rasterSet=[[0.0, 0.0, 0.0]], rateSet=[[0.0, 0.0, 0.0]]):
     self.rasterAngles = rasterSet
     self.rasterRates = rateSet
     self.rasterMode = modeSet
     self.rasterTimes = times

class powerSimBase(SimulationBaseClass.SimBaseClass):
 def __init__(self):
    self.modeData = [[0, 'Unknown']]
    self.TheEMMSim = EMMSim.EMMSim()
    self.TheEMMSim.TotalSim.logThisMessage("inertial_state_output", int(1E9)) #inertial states
    self.TheEMMSim.TotalSim.logThisMessage("solar_array_sun_bore", int(60E9)) #solar array boresight angles
    self.TheEMMSim.TotalSim.logThisMessage("high_gain_earth_bore", int(1E9)) #solar array boresight angles
    self.TheEMMSim.TotalSim.logThisMessage("instrument_mars_bore", int(1E9)) #solar array boresight angles
    self.TheEMMSim.TotalSim.logThisMessage("att_cmd_output", int(1E9)) #inertial states
    self.TheEMMSim.TotalSim.logThisMessage("OrbitalElements", int(1E10)) #orbital elements
    self.TheEMMSim.AddVectorForLogging('instrumentBoresight.boreVecPoint', 'double', 0, 2, int(1E9))

    self.TheEMMSim.VehDynObject.GravData[0].IsCentralBody = False
    self.TheEMMSim.VehDynObject.GravData[0].IsDisplayBody = False
    self.TheEMMSim.VehDynObject.GravData[2].IsCentralBody = True
    self.TheEMMSim.VehDynObject.GravData[2].IsDisplayBody = True
    self.TheEMMSim.VehOrbElemObject.mu = self.TheEMMSim.MarsGravBody.mu #central body of OE object

    #These mass properties are built assuming that we have already emptied the tank
    #inserting into Mars orbit.  I'm assuming that we have 40% of the prop remaining 
    #because that feels right.  Fix that!
    DVpropCM = [0.0, 0.0, 1.0] #Rough center of mass for the prop tank
    DVpropMass = (812.3-40)*0.4 #A theoretical "heavy" prop mass
    DVpropRadius = 46.0/2.0/3.2808399/12.0 #Radius of propellant tank
    sphereInerita = 2.0/5.0*DVpropMass*DVpropRadius*DVpropRadius #assume it is a sphere because why not?
    DVInertia = [sphereInerita, 0, 0, 0, sphereInerita, 0, 0, 0, sphereInerita] #Inertia tensor
    #now go and set the actual simulation variables with the calculated values
    self.TheEMMSim.DVThrusterDynObject.objProps.Mass = DVpropMass
    SimulationBaseClass.SetCArray(DVpropCM, 'double', self.TheEMMSim.DVThrusterDynObject.objProps.CoM)
    SimulationBaseClass.SetCArray(DVInertia, 'double', self.TheEMMSim.DVThrusterDynObject.objProps.InertiaTensor)

    #General place in the science orbit.  These parameters were made up without access 
    #to any textbook or even the internet.  They are probably wrong.
    self.spiceLocal = spice_interface.SpiceInterface()
    self.spiceLocal.loadSpiceKernel('Amal_LD0711_v151203.bsp',
       path + '/')
    self.spiceLocal.SPICEDataPath = self.TheEMMSim.simBasePath + '/External/EphemerisData/'
    self.spiceLocal.UTCCalInit = self.TheEMMSim.SpiceObject.UTCCalInit
    self.spiceLocal.OutputBufferCount = 2
    self.spiceLocal.PlanetNames = spice_interface.StringVector(["-62"])
    self.spiceLocal.referenceBase = "MARSIAU"
    self.spiceLocal.zeroBase = "MARS"
    self.amalName = "-62"  #Really???? Really.  No Really??????? Realz.
    self.spacecraftMessageName = self.amalName + "_planet_data"

    self.orbitSequence = {}
    self.orbitSequence['C1'] = SequenceElement([int((112.82+5)*60.0)])
    self.orbitSequence['C2'] = SequenceElement([int((86.32+5)*60.0)])
    self.orbitSequence['C3'] = SequenceElement([int((59.82+5)*60.0)])
    self.orbitSequence['C4'] = SequenceElement([int((72.57+5)*60.0)])
    self.orbitSequence['C5'] = SequenceElement([int((105.57+5)*60.0)])
    self.orbitSequence['C6'] = SequenceElement([int((85.32+5)*60.0)])
    self.orbitSequence['C7'] = SequenceElement([int((92.82+5)*60.0)])
    self.orbitSequence['C8'] = SequenceElement([int((119.32+5)*60.0)])
    self.orbitSequence['C9'] = SequenceElement([int((99.07+5)*60.0)])
    self.orbitSequence['C10'] = SequenceElement([int((125.82+5)*60.0)])
    self.orbitSequence['C11'] = SequenceElement([int((79.82+5)*60.0)])
    self.orbitSequence['C12'] = SequenceElement([int((96.32+5)*60.0)])
    self.orbitSequence['C13'] = SequenceElement([int((119.32+5)*60.0)])
    self.orbitSequence['C14'] = SequenceElement([int((102.82+5)*60.0)])
    self.orbitSequence['C15'] = SequenceElement([int((105.82+5)*60.0)])
    self.orbitSequence['C16'] = SequenceElement([int((99.32+5)*60.0)])
    longTimes = [int(30*60), int(1.5*60), int(26.5*60), int(1.5*60), int(26.5*60)]
    rastAngRad = 10.0*math.pi/180.0
    discAngleRad = 15.0*math.pi/180.0
    rasterTime = 26.5*60.0
    discAngRate = 2.0*discAngleRad/rasterTime
    longAngles = [ \
                            [rastAngRad, 0.0, -discAngleRad],
                            [0.0, 0.0, 0.0],
                            [0.0, 0.0, -discAngleRad],
                            [0.0, 0.0, 0.0],
                            [-rastAngRad, 0.0, discAngleRad] \
                            ]
    longRates = [ \
                    [0.0, 0.0, -discAngRate],
                    [0.0, 0.0, 0.0],
                    [0.0, 0.0, -discAngRate],
                    [0.0, 0.0, 0.0],
                    [0.0, 0.0, discAngRate]\
                    ]
    self.orbitSequence['O3'] = SequenceElement(longTimes, 'marsPoint', longAngles,
                                               longRates)
    self.orbitSequence['O1'] = SequenceElement(longTimes[1:4], 'marsPoint', longAngles[1:4],
                                               longRates[1:4])
 
    mediumTimes = [int(15.0*60), int(11*60), int(11*60), int(1.5*60), int(26.5*60)]
    medUpAngRad = 10.0*math.pi/180.0
    medScanAngRad = 11.4*math.pi/180.0
    mediumTime = 10.0*60.0
    medAngRate = 2.0*medScanAngRad/mediumTime
    mediumAngles = [ \
                    [medUpAngRad, 0.0, -medScanAngRad],
                    [0.0, 0.0, medScanAngRad],
                    [-medUpAngRad, 0.0, -medScanAngRad],
                    [0.0, 0.0, 0.0],
                    [0.0, 0.0, -discAngleRad],
                    [0.0, 0.0, 0.0] \
                    ]
    mediumRates = [ \
                    [0.0, 0.0, -medAngRate],
                    [0.0, 0.0, medAngRate],
                    [0.0, 0.0, -medAngRate],
                    [0.0, 0.0, 0.0],
                    [0.0, 0.0, -discAngRate],
                    [0.0, 0.0, 0.0]
                ]
    self.orbitSequence['O2'] = SequenceElement(mediumTimes, 'marsPoint', mediumAngles,
                            mediumRates)
 
    BOSshortTimes = [int(5.0*60), int(26.5*60), int(1.5*60), int(26.5*60.0)]
    BOSshortAngles = [ \
                        [0.0, 0.0, 0.0],
                        [rastAngRad/2.0, 0.0, -discAngleRad],
                        [0.0, 0.0, 0.0],
                        [-rastAngRad/2.0, 0.0, -discAngleRad] \
                      ]
    BOSshortRates = [ \
                    [0.0, 0.0, 0.0],
                    [0.0, 0.0, -discAngRate],
                    [0.0, 0.0, 0.0],
                    [0.0, 0.0, -discAngRate]\
                    ]
    self.orbitSequence['O6'] = SequenceElement(BOSshortTimes, 'marsPoint', BOSshortAngles,
                                                   BOSshortRates)
    MOSshortTimes = [int(15*60), int(11*60), int(1.5*60), int(26.5*60), int(1.5*60)]
    MOSshortAngles = [ \
                      [medUpAngRad/2.0, 0.0, -medScanAngRad],
                      [-medUpAngRad/2.0, 0.0, medScanAngRad],
                      [0.0, 0.0, 0.0],
                      [0.0, 0.0, -discAngleRad],
                      [0.0, 0.0, 0.0] \
                      ]
    MOSshortRates = [ \
                    [0.0, 0.0, -medAngRate],
                    [0.0, 0.0, medAngRate],
                    [0.0, 0.0, 0.0],
                    [0.0, 0.0, -discAngRate],
                    [0.0, 0.0, 0.0]\
                    ]
    self.orbitSequence['O5'] = SequenceElement(MOSshortTimes, 'marsPoint', MOSshortAngles,
                                                MOSshortRates)
    SOSShortAngle = 7.0*math.pi/180.0
    SOSShortTime = 12.0*60.0
    SOSShortRate = 2.0*SOSShortAngle/SOSShortTime
    SOSshortTimes = [int(5.5*60), int(12*60), int(1.5*60)]
    SOSshortAngles = [ \
                      [0.0, 0.0, 0.0],
                      [0.0, 0.0, -SOSShortAngle],
                      [0.0, 0.0, 0.0] \
                      ]
    SOSshortRates = [ \
                     [0.0, 0.0, 0.0],
                     [0.0, 0.0, -SOSShortRate],
                     [0.0, 0.0, 0.0]\
                     ]
    self.orbitSequence['O4'] = SequenceElement(SOSshortTimes, 'marsPoint', SOSshortAngles,
                                                   SOSshortRates)
    self.orbitSequence['EP'] = SequenceElement([int(6*60*60)], 'earthPoint')
 


 def updateStartCalendar(self, newCalendar):
     self.spiceLocal.UTCCalInit = newCalendar
     self.TheEMMSim.SpiceObject.UTCCalInit = newCalendar
 def applyInitialState(self):
    self.spiceLocal.SelfInit()
    self.spiceLocal.UpdateState(0)
    AmalPosition = self.TheEMMSim.pullMessageLogData(self.spacecraftMessageName + ".PositionVector", range(3))
    AmalVelocity = self.TheEMMSim.pullMessageLogData(self.spacecraftMessageName + ".VelocityVector", range(3))
    self.TheEMMSim.VehDynObject.PositionInit = sim_model.DoubleVector(AmalPosition[0, 1:].tolist())
    self.TheEMMSim.VehDynObject.VelocityInit = sim_model.DoubleVector(AmalVelocity[0, 1:].tolist())
 def iterateThroughSequence(self, newSequence):
    for operation in newSequence:
        opElement = self.orbitSequence[operation]
        operationTime = int(sum(opElement.rasterTimes)*1E9)
        self.TheEMMSim.ConfigureStopTime(self.TheEMMSim.TotalSim.CurrentNanos + 
                                        operationTime)
        self.TheEMMSim.scanAnglesUse = opElement.rasterAngles
        self.TheEMMSim.scanRate = opElement.rasterRates
        self.TheEMMSim.rasterTimes = opElement.rasterTimes
        self.TheEMMSim.modeRequest = opElement.rasterMode
        self.modeData.append([self.TheEMMSim.TotalSim.CurrentNanos, opElement.rasterMode])
        self.TheEMMSim.ExecuteSimulation()
 def generateMissCSV(self, outputName):
    with open(outputName, 'w') as csvfile:
        missWriter = csv.writer(csvfile, delimiter=',')
        missWriter.writerow(['Time (s)', 'Solar Array Miss (d)', 'ADCS State'])
        modeIndex = 0
        currentMode = self.modeData[modeIndex]
        nextMode = self.modeData[modeIndex+1]
        solarArrayMiss = self.TheEMMSim.pullMessageLogData("solar_array_sun_bore.missAngle")
        for i in range(solarArrayMiss.shape[0]):
            currentRow = solarArrayMiss[i, :]
            while(currentRow[0] >= nextMode[0] and modeIndex+1 < len(self.modeData)):
                modeIndex += 1
                currentMode = nextMode
                if(modeIndex+1 < len(self.modeData)):
                    nextMode = self.modeData[modeIndex+1]
            missWriter.writerow([currentRow[0]*1.0E-9/60.0, currentRow[1]*180.0/math.pi,
               currentMode[1]])




#Instantiate a copy of the EMM vehicle/FSW
obsSequence = ['O4', 'C10', 'O4', 'C10', 'O4', 'C5', 'O6', 'C5', 'O4',
                 'C10', 'O4', 'C10', 'O4', 'C10', 'O4', 'C7', 'O3', 'C2', 'O1', 'C2', 'O3', 'C2',
                  'O1', 'C2', 'O3', 'EP', 'O1', 'C9', 'O6', 'C5', 'O4', 'C5', 'O6',
                  'C5', 'O4', 'C10', 'O4', 'C10', 'O4', 'C10']

powerSimOP = powerSimBase()
powerSimOP.updateStartCalendar('AUG 27,2021  01:30:15.0 (UTC)')
powerSimOP.applyInitialState()
powerSimOP.TheEMMSim.InitializeSimulation()
powerSimOP.ConfigureStopTime(int(10*1E9))
powerSimOP.TheEMMSim.modeRequest = 'safeMode'
powerSimOP.TheEMMSim.ConfigureStopTime(int(30*1E9))
powerSimOP.TheEMMSim.ExecuteSimulation()

powerSimOP.iterateThroughSequence(obsSequence)# 'C2', 'O4', 'C3', 'O2'])
powerSimOP.generateMissCSV('optimalZeroNuPrime.csv')
powerSimOP.TheEMMSim.TotalSim.terminateSimulation()

powerSimSub = powerSimBase()
powerSimSub.updateStartCalendar('AUG 27,2021  01:30:15.0 (UTC)')
baseMarsTrans = numpy.array([0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0]).reshape((3,3))
powerSimSub.TheEMMSim.marsPointData.inputSecMessName = "" #No constraint
betaPrime = -math.pi/2.0 +math.pi - 27.5*math.pi/180.0
transMat = RigidBodyKinematics.Euler1232C(numpy.array([0.0, betaPrime, 0.0]).reshape((3,1)))
totalTrans = transMat*baseMarsTrans
powerSimSub.TheEMMSim.baseMarsTrans = totalTrans.reshape((1,9)).tolist()[0]
SimulationBaseClass.SetCArray(totalTrans.reshape((1,9)).tolist()[0], 'double', powerSimSub.TheEMMSim.marsPointData.TPoint2Bdy)
powerSimSub.TheEMMSim.VehOrbElemObject.CurrentElem.f = 10.0

powerSimSub.TheEMMSim.createNewEvent("constFlip", int(1E10), True, ["self.VehOrbElemObject.CurrentElem.f < 3.14/2.0"],
                                     ["import math",
                                      "betaPrime = -math.pi/2.0 + 27.5*math.pi/180.0",
                                      "import RigidBodyKinematics",
                                      "baseMarsTrans = numpy.array([0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0]).reshape((3,3))",
                                      "transMat = RigidBodyKinematics.Euler1232C(numpy.array([0.0, betaPrime, 0.0]).reshape((3,1)))",
                                      "totalTrans = transMat*baseMarsTrans",
                                      "self.baseMarsTrans = totalTrans.reshape((1,9)).tolist()[0]",
                                      "print 'Oh I made it'"])

powerSimSub.applyInitialState()
powerSimSub.TheEMMSim.InitializeSimulation()
powerSimSub.ConfigureStopTime(int(10*1E9))
powerSimSub.TheEMMSim.modeRequest = 'safeMode'
powerSimSub.TheEMMSim.ConfigureStopTime(int(30*1E9))
powerSimSub.TheEMMSim.ExecuteSimulation()
powerSimSub.iterateThroughSequence(obsSequence)
powerSimSub.generateMissCSV('constantZeroNuPrime.csv')


trueAnomOP = powerSimSub.TheEMMSim.pullMessageLogData("OrbitalElements.f")
solarArrayMissOP = powerSimOP.TheEMMSim.pullMessageLogData("solar_array_sun_bore.missAngle")
dataInstBoreOP = powerSimOP.TheEMMSim.GetLogVariableData('instrumentBoresight.boreVecPoint')
sigmaCMDOP = powerSimOP.TheEMMSim.pullMessageLogData('att_cmd_output.sigma_BR', range(3))
sigmaTruthOP = powerSimOP.TheEMMSim.pullMessageLogData('inertial_state_output.sigma', range(3))

highGainMess = powerSimOP.TheEMMSim.pullMessageLogData("high_gain_earth_bore.missAngle")

solarArrayMissSub = powerSimSub.TheEMMSim.pullMessageLogData("solar_array_sun_bore.missAngle")
dataInstBoreSub = powerSimSub.TheEMMSim.GetLogVariableData('instrumentBoresight.boreVecPoint')
sigmaCMDSub = powerSimSub.TheEMMSim.pullMessageLogData('att_cmd_output.sigma_BR', range(3))
sigmaTruthSub = powerSimSub.TheEMMSim.pullMessageLogData('inertial_state_output.sigma', range(3))

plt.figure(1)
plt.plot(solarArrayMissOP[:,0]*1.0E-9, solarArrayMissOP[:,1]*180/math.pi)
plt.plot(solarArrayMissSub[:,0]*1.0E-9, solarArrayMissSub[:,1]*180/math.pi, 'g--')
plt.xlabel('Time (s)')
plt.ylabel('Solar Array Miss (d)')

plt.figure(2)
plt.plot(dataInstBoreOP[:,2], dataInstBoreOP[:,3], dataInstBoreSub[:,2], dataInstBoreSub[:,3], 'g--')
plt.xlabel('y-component (-)')
plt.ylabel('z-component (-)')

plt.figure(3)
plt.plot(sigmaTruthOP[:,0]*1.0E-9, sigmaTruthOP[:,1], 'b')
plt.plot(sigmaTruthOP[:,0]*1.0E-9, sigmaTruthOP[:,2], 'g')
plt.plot(sigmaTruthOP[:,0]*1.0E-9, sigmaTruthOP[:,3], 'r')
plt.xlabel('Time (s)')
plt.ylabel('Attitude MRP (-)')

plt.figure(4)
plt.plot(trueAnomOP[:,0]*1.0E-9, trueAnomOP[:,1]*180.0/math.pi)
plt.xlabel('Time (s)')
plt.ylabel('True Anomaly (d)')

plt.figure(5)
plt.plot(highGainMess[:,0]*1.0E-9, highGainMess[:,1]*180/math.pi)
plt.xlabel('Time (s)')
plt.ylabel('Solar Array Miss (d)')

plt.show()
##Log a handful of messages to examine vehicle performance
#TheEMMSim.TotalSim.logThisMessage("inertial_state_output", int(1E10)) #inertial states
#TheEMMSim.TotalSim.logThisMessage("att_cmd_output", int(1E10)) #inertial states
#TheEMMSim.TotalSim.logThisMessage("OrbitalElements", int(1E10)) #orbital elements
#TheEMMSim.TotalSim.logThisMessage("css_wls_est", int(1E10)) #FSW weighted least squares sun-vector
#TheEMMSim.TotalSim.logThisMessage("spacecraft_mass_props", int(1E10)) #spacecraft mass properties
#TheEMMSim.TotalSim.logThisMessage("solar_array_sun_bore", int(1E10)) #solar array boresight angles
#TheEMMSim.TotalSim.logThisMessage("high_gain_earth_bore", int(1E10)) #solar array boresight angles
#TheEMMSim.TotalSim.logThisMessage("instrument_mars_bore", int(1E10)) #solar array boresight angles
#TheEMMSim.AddVectorForLogging('CSSPyramid1HeadA.sHatStr', 'double', 0, 2, int(1E10))
#TheEMMSim.TotalSim.logThisMessage("controlTorqueRaw", int(1E9))
#TheEMMSim.AddVectorForLogging('instrumentBoresight.boreVecPoint', 'double', 0, 2, int(1E9)) 
##Setup a time in the science orbit well past our transition to science
#TheEMMSim.SpiceObject.UTCCalInit = "2021 December 25, 00:00:00.0"

##Reset the dynamics to change the central body from Sun (index 0) to Mars (index 2)
#TheEMMSim.VehDynObject.GravData[0].IsCentralBody = False
#TheEMMSim.VehDynObject.GravData[0].IsDisplayBody = False
#TheEMMSim.VehDynObject.GravData[2].IsCentralBody = True
#TheEMMSim.VehDynObject.GravData[2].IsDisplayBody = True
#TheEMMSim.VehOrbElemObject.mu = TheEMMSim.MarsGravBody.mu #central body of OE object

##Setting a dry mass CoM offset to be conservative
#TheEMMSim.VehDynObject.baseCoMInit[0] = 0.05 #meters

##These mass properties are built assuming that we have already emptied the tank
##inserting into Mars orbit.  I'm assuming that we have 40% of the prop remaining 
##because that feels right.  Fix that!
#DVpropCM = [0.0, 0.0, 1.0] #Rough center of mass for the prop tank
#DVpropMass = (812.3-40)*0.4 #A theoretical "heavy" prop mass
#DVpropRadius = 46.0/2.0/3.2808399/12.0 #Radius of propellant tank
#sphereInerita = 2.0/5.0*DVpropMass*DVpropRadius*DVpropRadius #assume it is a sphere because why not?
#DVInertia = [sphereInerita, 0, 0, 0, sphereInerita, 0, 0, 0, sphereInerita] #Inertia tensor
##now go and set the actual simulation variables with the calculated values
#TheEMMSim.DVThrusterDynObject.objProps.Mass = DVpropMass
#SimulationBaseClass.SetCArray(DVpropCM, 'double', TheEMMSim.DVThrusterDynObject.objProps.CoM)
#SimulationBaseClass.SetCArray(DVInertia, 'double', TheEMMSim.DVThrusterDynObject.objProps.InertiaTensor)

##General place in the science orbit.  These parameters were made up without access 
##to any textbook or even the internet.  They are probably wrong.
#spiceLocal = spice_interface.SpiceInterface()
#spiceLocal.loadSpiceKernel('Amal_LD0711_v151203.bsp',
#    '/Users/piggott/adcs_codebase/PDRVehicleTrajectories/')
#spiceLocal.SPICEDataPath = TheEMMSim.simBasePath + '/External/EphemerisData/'
#spiceLocal.UTCCalInit = TheEMMSim.SpiceObject.UTCCalInit
#spiceLocal.OutputBufferCount = 2
#spiceLocal.PlanetNames = spice_interface.StringVector(["-62"])
#spiceLocal.referenceBase = "MARSIAU"
#spiceLocal.zeroBase = "MARS"
#spiceLocal.SelfInit()
#spiceLocal.UpdateState(0)
#amalName = "-62"  #Really???? Really.  No Really??????? Realz.
#spacecraftMessageName = amalName + "_planet_data"
#TheEMMSim.MRP_SteeringRWAData.K1 = 1.5
##TheEMMSim.MRP_SteeringRWAData.p = 300.0

#AmalPosition = TheEMMSim.pullMessageLogData(spacecraftMessageName + ".PositionVector", range(3))
#AmalVelocity = TheEMMSim.pullMessageLogData(spacecraftMessageName + ".VelocityVector", range(3))
#TheEMMSim.VehDynObject.PositionInit = sim_model.DoubleVector(AmalPosition[0, 1:].tolist())
#TheEMMSim.VehDynObject.VelocityInit = sim_model.DoubleVector(AmalVelocity[0, 1:].tolist())
#print AmalPosition[0, :]
#print AmalVelocity[0, :]

##Initialize simulation and free-drift for 30 seconds to let everything populate
#TheEMMSim.InitializeSimulation()
#TheEMMSim.ConfigureStopTime(int(30*1E9))
#TheEMMSim.ExecuteSimulation()
##Command the FSW to go into safe mode and advance to ~ periapsis
#TheEMMSim.modeRequest = 'safeMode'
#TheEMMSim.ConfigureStopTime(int(60*11*1*1E9))
#TheEMMSim.ExecuteSimulation()
##Take the vehicle into sun pointing mode and begin the science sequencing
#TheEMMSim.modeRequest = 'sunPoint'
#TheEMMSim.ConfigureStopTime(int(60*60*1*1E9))
#TheEMMSim.ExecuteSimulation()
##Take the vehicle into earth pointing mode and begin the science sequencing
#TheEMMSim.modeRequest = 'earthPoint'
#TheEMMSim.ConfigureStopTime(int(60*60*2*1E9))
#TheEMMSim.ExecuteSimulation()
##Take the vehicle into mars pointing mode and begin the science sequencing
#TheEMMSim.modeRequest = 'marsPoint'
#TheEMMSim.ConfigureStopTime(int(60*60*5.0*1E9))
#TheEMMSim.scanAnglesUse = TheEMMSim.sideScanAngles
#TheEMMSim.ExecuteSimulation()
#TheEMMSim.setEventActivity('initiateSunPoint', True)
#TheEMMSim.setEventActivity('initiateMarsPoint', True)
#TheEMMSim.modeRequest = 'sunPoint'
#TheEMMSim.ConfigureStopTime(int(60*60*7.0*1E9))
#TheEMMSim.ExecuteSimulation()
##TheEMMSim.modeRequest = 'marsPoint'
##TheEMMSim.scanAnglesUse = TheEMMSim.sideScanAngles
##TheEMMSim.ConfigureStopTime(int(60*60*8.0*1E9))
##TheEMMSim.ExecuteSimulation()

###Simulation complete.  Pull off a selected set of values from the variable logs
#semiMajor = TheEMMSim.pullMessageLogData("OrbitalElements.a")
#posMag = TheEMMSim.pullMessageLogData("OrbitalElements.rmag")
#radApo = TheEMMSim.pullMessageLogData("OrbitalElements.rApoap")
#radPeri = TheEMMSim.pullMessageLogData("OrbitalElements.rPeriap")
#trueAnom = TheEMMSim.pullMessageLogData("OrbitalElements.f")
#FSWControlOut = TheEMMSim.pullMessageLogData("controlTorqueRaw.torqueRequestBody", range(3))
#solarArrayMiss = TheEMMSim.pullMessageLogData("solar_array_sun_bore.missAngle")
#highGainMiss = TheEMMSim.pullMessageLogData("high_gain_earth_bore.missAngle")
#instrumentMiss = TheEMMSim.pullMessageLogData("instrument_mars_bore.missAngle")
#instrumentAz = TheEMMSim.pullMessageLogData("instrument_mars_bore.azimuth")
#DataCSSTruth = TheEMMSim.GetLogVariableData('CSSPyramid1HeadA.sHatStr')
#sigmaTruth = TheEMMSim.pullMessageLogData('inertial_state_output.sigma', range(3))
#omegaTruth = TheEMMSim.pullMessageLogData('inertial_state_output.omega', range(3))
#sigmaCMD = TheEMMSim.pullMessageLogData('att_cmd_output.sigma_BR', range(3))
#dataInstBore = TheEMMSim.GetLogVariableData('instrumentBoresight.boreVecPoint')

##Plot true anomaly for the simulation
#plt.figure(1)
#plt.plot(trueAnom[:,0]*1.0E-9, trueAnom[:,1]*180.0/math.pi)
#plt.xlabel('Time (s)')
#plt.ylabel('True Anomaly (d)')

#plt.figure(2)
#plt.plot(DataCSSTruth[:,0]*1.0E-9, DataCSSTruth[:,1], 'b--')
#plt.plot(DataCSSTruth[:,0]*1.0E-9, DataCSSTruth[:,2], 'g--')
#plt.plot(DataCSSTruth[:,0]*1.0E-9, DataCSSTruth[:,3], 'r--')
#plt.xlabel('Time (s)')
#plt.ylabel('Structural Frame Sun Vector (-)')

#plt.figure(3)
#plt.plot(sigmaCMD[:,0]*1.0E-9, sigmaCMD[:,1], 'b', sigmaTruth[:,0]*1.0E-9, sigmaTruth[:,1], 'b--')
#plt.plot(sigmaCMD[:,0]*1.0E-9, sigmaCMD[:,2], 'g', sigmaTruth[:,0]*1.0E-9, sigmaTruth[:,2], 'g--')
#plt.plot(sigmaCMD[:,0]*1.0E-9, sigmaCMD[:,3], 'r', sigmaTruth[:,0]*1.0E-9, sigmaTruth[:,3], 'r--')
#plt.xlabel('Time (s)')
#plt.ylabel('Attitude MRP (-)')

#plt.figure(4)
#plt.plot(solarArrayMiss[:,0]*1.0E-9, solarArrayMiss[:,1]*180/math.pi)
#plt.xlabel('Time (s)')
#plt.ylabel('Solar Array Miss (d)')

#plt.figure(5)
#plt.plot(highGainMiss[:,0]*1.0E-9, highGainMiss[:,1]*180/math.pi)
#plt.xlabel('Time (s)')
#plt.ylabel('High Gain Miss (d)')

#plt.figure(6)
#plt.plot(instrumentMiss[:,0]*1.0E-9, instrumentMiss[:,1]*180.0/math.pi)
#plt.xlabel('Time (s)')
#plt.ylabel('Instrument Nadir Miss (d)')

#plt.figure(7)
#plt.plot(instrumentAz[:,0]*1.0E-9, instrumentAz[:,1]*180.0/math.pi)
#plt.xlabel('Time (s)')
#plt.ylabel('Instrument Nadir Azimuth (d)')

#plt.figure(8)
#plt.plot(dataInstBore[:,2], dataInstBore[:,3])
#plt.xlabel('y-component (-)')
#plt.ylabel('z-component (-)')

#plt.figure(9)
#plt.plot(omegaTruth[:,0]*1.0E-9, omegaTruth[:,1]*180.0/math.pi)
#plt.plot(omegaTruth[:,0]*1.0E-9, omegaTruth[:,2]*180.0/math.pi)
#plt.plot(omegaTruth[:,0]*1.0E-9, omegaTruth[:,3]*180.0/math.pi)
#plt.xlabel('Time (s)')
#plt.ylabel('Body Rate (d/s)')

#plt.figure(10)
#plt.plot(FSWControlOut[:,0]*1.0E-9, FSWControlOut[:,1])
#plt.plot(FSWControlOut[:,0]*1.0E-9, FSWControlOut[:,2])
#plt.plot(FSWControlOut[:,0]*1.0E-9, FSWControlOut[:,3])

##If requested, generate plots
#if(len(sys.argv) > 1):
#   if(sys.argv[1] == 'True'):
#      plt.show()

