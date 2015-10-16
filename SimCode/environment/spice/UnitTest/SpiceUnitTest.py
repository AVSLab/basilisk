#Very simple simulation.  Just sets up and calls the SPICE interface.  Could 
#be the basis for a unit test of SPICE
import sys, os
sys.path.append(os.environ['SIMULATION_BASE']+'/modules')
sys.path.append(os.environ['SIMULATION_BASE']+'/PythonModules/')

#Import all of the modules that we are going to call in this simulation
import spice_interface
import MessagingAccess
import SimulationBaseClass
import numpy
import ctypes

TestResults = {}

#Create a sim module as an empty container
TotalSim = SimulationBaseClass.SimBaseClass() 
TotalSim.CreateNewTask("TimeTestTask", int(1E9))

#Now initialize the modules that we are using.  I got a little better as I went along
SpiceObject = spice_interface.SpiceInterface()

SpiceObject.ModelTag = "SpiceInterfaceData"
SpiceObject.SPICEDataPath = os.environ['SIMULATION_BASE'] + '/External/EphemerisData/'
SpiceObject.OutputBufferCount = 10000
SpiceObject.PlanetNames = spice_interface.StringVector(["earth", "mars", "sun"])
SpiceObject.UTCCalInit = "2016 June 16, 00:00:00.0 TDB"
TotalSim.AddModelToTask("TimeTestTask", SpiceObject)

TotalSim.ConfigureStopTime(int(60.0*1E9))
TotalSim.AddVariableForLogging('SpiceInterfaceData.GPSSeconds')
TotalSim.AddVariableForLogging('SpiceInterfaceData.J2000Current')
TotalSim.AddVariableForLogging('SpiceInterfaceData.JulianDateCurrent')
TotalSim.AddVariableForLogging('SpiceInterfaceData.GPSWeek')

#Just running these tests to make sure that I cover all of the code

TotalSim.InitializeSimulation()
TotalSim.ExecuteSimulation()
DataGPSSec = TotalSim.GetLogVariableData('SpiceInterfaceData.GPSSeconds')
DataJD = TotalSim.GetLogVariableData('SpiceInterfaceData.JulianDateCurrent')

AllowTolerance = 1E-6
GPSRow = DataGPSSec[0,:]
InitDiff = GPSRow[1] - GPSRow[0]*1.0E-9
i=1
TestResults['TimeDeltaCheck'] = True
while(i<DataGPSSec.shape[0]):
   GPSRow = DataGPSSec[i,:]
   CurrDiff = GPSRow[1] - GPSRow[0]*1.0E-9
   if(abs(CurrDiff - InitDiff) > AllowTolerance):
      print "Time delta check failed with difference of: %(DiffVal)f" % \
         {"DiffVal": CurrDiff - InitDiff} 
      TestResults['TimeDeltaCheck'] = False
   i=i+1

GPSEndTime = 1150070417.0 + 60.0 -0.184 - 68.0
GPSWeek = int(GPSEndTime/(86400*7))
GPSSecondAssumed = GPSEndTime - GPSWeek*86400*7

GPSSecDiff = abs(GPSRow[1] - GPSSecondAssumed)
TestResults['GPSAbsTimeCheck'] = True
if(GPSSecDiff > AllowTolerance):
   TestResults['GPSAbsTimeCheck'] = False
   print "Absolute GPS time check failed with difference of: %(DiffVal)f" % \
      {"DiffVal": GPSSecDiff}


JDTimeErrorAllow = 0.1/(24.0*3600.0)
JDEndTime = 2457555.5006944440 - 68.184/(86400)
JDEndSim = DataJD[i-1, 1]
TestResults['JDEndCheck'] = True
if(abs(JDEndSim - JDEndTime) > JDTimeErrorAllow):
   TestResults['JDEndCheck'] = False
   print "Absolute Julian Date time check failed with difference of: %(DiffVal)f" % \
      {"DiffVal": abs(JDEndSim - JDEndTime)}
   
PosErrTolerance = 1000

MarsPosEnd = [-5.837178848407724E+07, -1.956015572423997E+08, -8.816253265780403E+07]
MarsPosEnd = numpy.array(MarsPosEnd)
MarsPosEnd = MarsPosEnd*1000.0

FinalMarsMessage = spice_interface.SpicePlanetState()
TotalSim.TotalSim.GetWriteData("mars_planet_data", 120, FinalMarsMessage, 0)
MarsPosVec = ctypes.cast(FinalMarsMessage.PositionVector.__long__(), 
   ctypes.POINTER(ctypes.c_double))

MarsPosArray = numpy.array([MarsPosVec[0], MarsPosVec[1], MarsPosVec[2]])
MarsPosDiff = MarsPosArray - MarsPosEnd
PosDiffNorm =  numpy.linalg.norm(MarsPosDiff)
TestResults['MarsPosCheck'] = True
if(PosDiffNorm > PosErrTolerance):
   print "Mars position check failed with difference of: %(DiffVal)f" % \
         {"DiffVal": PosDiffNorm}
   TestResults['MarsPosCheck'] = False

EarthPosEnd = numpy.array([-1.252088742656509E+07, -1.385541731163796E+08, -6.009129302923465E+07])
EarthPosEnd = EarthPosEnd*1000.0

FinalEarthMessage = spice_interface.SpicePlanetState()
TotalSim.TotalSim.GetWriteData("earth_planet_data", 120, FinalEarthMessage, 0)
EarthPosVec = ctypes.cast(FinalEarthMessage.PositionVector.__long__(),
   ctypes.POINTER(ctypes.c_double))

EarthPosArray = numpy.array([EarthPosVec[0], EarthPosVec[1], EarthPosVec[2]])
EarthPosDiff = EarthPosArray - EarthPosEnd
PosDiffNorm =  numpy.linalg.norm(EarthPosDiff)
TestResults['EarthPosCheck'] = True
if(PosDiffNorm > PosErrTolerance):
   print "Earth position check failed with difference of: %(DiffVal)f" % \
         {"DiffVal": PosDiffNorm}
   TestResults['EarthPosCheck'] = False

SunPosEnd = numpy.array([5.590530479848895E+05,  3.620962731831532E+05,  1.304501641512279E+05])
SunPosEnd = SunPosEnd*1000.0

FinalSunMessage = spice_interface.SpicePlanetState()
TotalSim.TotalSim.GetWriteData("sun_planet_data", 120, FinalSunMessage, 0)
SunPosVec = ctypes.cast(FinalSunMessage.PositionVector.__long__(),
   ctypes.POINTER(ctypes.c_double))

SunPosArray = numpy.array([SunPosVec[0], SunPosVec[1], SunPosVec[2]])
SunPosDiff = SunPosArray - SunPosEnd
PosDiffNorm =  numpy.linalg.norm(SunPosDiff)
TestResults['SunPosCheck'] = True
if(PosDiffNorm > PosErrTolerance):
   print "Sun position check failed with difference of: %(DiffVal)f" % \
         {"DiffVal": PosDiffNorm}
   TestResults['SunPosCheck'] = False

SpiceObject.SPICEDataPath = "ADirectoryThatDoesntreallyexist"
SpiceObject.SPICELoaded = False
TotalSim.ConfigureStopTime(int(1E9))
TotalSim.InitializeSimulation()
TotalSim.ExecuteSimulation()
SpiceObject.SPICEDataPath = ""
SpiceObject.SPICELoaded = False
SpiceObject.PlanetNames = spice_interface.StringVector(["earth", "mars", "sun", "thisisaplanetthatisntreallyanythingbutIneedthenametobesolongthatIhitaninvalidconditioninmycode"])
TotalSim.InitializeSimulation()
TotalSim.ExecuteSimulation()
TotalSim.ConfigureStopTime(int(1E9))
