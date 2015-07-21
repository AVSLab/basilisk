#Very simple simulation.  Just sets up and calls the SPICE interface.  Could 
#be the basis for a unit test of SPICE

#Import some architectural stuff that we will probably always use
import sys, os
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
#Get access to the area where the modules are built
sys.path.append(os.environ['SIMULATION_BASE']+'/Binary/modules')
sys.path.append(os.environ['SIMULATION_BASE']+'/PythonModules/')
import numpy

#Import all of the modules that we are going to call in this simulation
import spice_interface
import sim_model
import sys_model_thread
import MessagingAccess

#Create a sim module as an empty container
TotalSim = sim_model.SimModel()

#Create a thread that we are going to call at a fixed rate (nanoseconds)
SensorThread = sys_model_thread.SysModelThread(int(1E8))

#Add the empty thread to the simulation's schedule
TotalSim.AddNewThread(SensorThread)

#Initialize the SPICE object we are going to use to get the current time
SpiceObject = spice_interface.SpiceInterface()
SpiceObject.ModelTag = "SpiceInterfaceData"
SpiceObject.SPICEDataPath = os.environ['SIMULATION_BASE'] + '/../AVSRepo/Simulation/data/'
SpiceObject.UTCCalInit = "2015 June 15, 00:00:00.0"
SpiceObject.OutputBufferCount = 50000
#Add the SPICE module to the schedule
SensorThread.AddNewObject(SpiceObject)

#Initialize all of the threads to the right state
TotalSim.InitThreads()

CurrentTime = 0
TimeStepUse = int(60E9) #Log out every 60 seconds.  Model runs internally at thread rate
TimeStop = int(86400*7*1E9) #one week

#init display variables
SimTimeVariable = []
GPSTimeVariable = []
GPSWeekVariable = []

#Step the simulation according to the Time step and save off the time variables
while CurrentTime < TimeStop:
   CurrentTime += TimeStepUse
   TotalSim.StepUntilTime(CurrentTime)
   SimTimeVariable.append(TotalSim.CurrentNanos*1.0E-9)
   GPSTimeVariable.append(SpiceObject.GPSSeconds)
   GPSWeekVariable.append(SpiceObject.GPSWeek)

FinalTimeMessage = spice_interface.SpiceTimeOutput()
TotalSim.GetWriteData("spice_time_output_data", 40, FinalTimeMessage, 0)

#Pull the message that we wrote from the internal database (50000 entries)
ListMessage = MessagingAccess.ObtainMessageList("spice_time_output_data", 'spice_interface',
   'SpiceTimeOutput', 50000, TotalSim)

#Data comes back from messaging system as a dictionary of lists
GPSSecondsMessage = numpy.array(ListMessage['GPSSeconds'])
MessageTimeMessage = numpy.array(ListMessage['MessageTime'])

#Print out the information on what messages were created/written
TotalSim.PrintSimulatedMessageData()

#Print out a bunch of output variables
print SpiceObject.ModelTag
print "Current Julian Date is: %(JulDate)f" % \
   {"JulDate": SpiceObject.JulianDateCurrent}
print "Current J2000 Elapsed is: %(J2000El)f" % \
   {"J2000El": SpiceObject.J2000Current}
print "Current GPS Seconds  is: %(GPSSec)f" % \
   {"GPSSec": SpiceObject.GPSSeconds}
print "Current GPS week is: %(GPSWeek)d" % \
   {"GPSWeek": SpiceObject.GPSWeek}
print "Current week rollovers is: %(GPSRoll)d" % \
   {"GPSRoll": SpiceObject.GPSRollovers}


print "Elapsed nanoseconds: %(ElapsedTime)d" % \
   {"ElapsedTime": TotalSim.CurrentNanos}

#Now plot some of those output variables
plt.figure(1)
plt.plot(SimTimeVariable, GPSTimeVariable )

plt.figure(2)
plt.plot(SimTimeVariable, GPSWeekVariable )

plt.figure(3)
plt.plot(MessageTimeMessage*1.0E-9, GPSSecondsMessage)

plt.show()


