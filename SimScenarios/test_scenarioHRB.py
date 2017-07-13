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
# Hinged Rigid Body Basilisk Tutorial
#
# Purpose:  Demonstrate the usage of hinged rigid body
# Author:   Scott Carnahan
# Creation Date:  July 6, 2017
#

#Non-Basilisk imports
import sys, os, inspect         #sys and os needed to add everything to the path.
import numpy as np              #numpy is used for some basic mathematical operations in this example.
import matplotlib.pyplot as plt #import plotting capabilities

# @cond DOXYGEN_IGNORE
filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
bskName = 'Basilisk'
splitPath = path.split(bskName)
bskPath = splitPath[0] + '/' + bskName + '/' #default filepath to basilisk only if scenario is run from within the basilisk folder.
# if this script is run from a custom folder outside of the Basilisk folder, then uncomment the
# following line and specify the absolute path to the Basilisk folder
#bskPath = '/Users/scottcarnahan/' + bskName + '/' #path to basilisk folder. The manually-typed portion of the path needs to be changed based on user-specific basilisk installation
sys.path.append(bskPath + 'modules') #add basilisk modules to system path
sys.path.append(bskPath + 'PythonModules') #add PythonModules to system path
# @endcond

#Basilisk imports
import SimulationBaseClass  #Need this to get simulationBaseClass, which is the simulation in which everything happens.
import unitTestSupport      # general support file with common unit test functions #sounds unecessary if not testing, but unsure how to remove it thus faar.
import macros               #seems like it is just used for some basic unit conversions
import orbitalMotion        #a namespace within ClassicElements class. Does a lot of conversion between types of orbit descriptions
import spacecraftPlus       #use this to make a spacecraft in the simulation.
import simIncludeGravity    #a namespsace reference via which we can get/apply different gravity models (different planets, etc.)

# Create simulation variable names
dynamicsTaskName = "dynamicsTask"
dynamicsProcessName = "simProcess"

#  Create a sim module as an empty container. Spacecraft, planets, dynamics, etc. can be put into this container.
simulationBase = SimulationBaseClass.SimBaseClass()  #Create a world in which a spacecraft can be simulated
simulationBase.TotalSim.terminateSimulation()               #Make sure nothing is left over in the world from a previous simulation

# Create the simulation process. It is automatically placed in the simulationBase with this line.
dynamicsProcess = simulationBase.CreateNewProcess(dynamicsProcessName)

# create the dynamics task and specify the integration update time
simulationTimeStep = macros.sec2nano(10.)                                          #set the simulation timestep
dynamicsTask = simulationBase.CreateNewTask(dynamicsTaskName, simulationTimeStep)#create a task within the simulationBase to handle dynamics and evaluate them every simulationTimeStep
dynamicsProcess.addTask(dynamicsTask)                                               #add the dynamics task to the dynamics process within the simulationBase


#
#   setup the simulation tasks/objects
#

# initialize spacecraftPlus object and set properties
satellite = spacecraftPlus.SpacecraftPlus()  #initialize satellite
satellite.ModelTag = "satellite"             #name the satellite
satellite.hub.useTranslation = True          #allow the satellite to translate
satellite.hub.useRotation = False            #do not allow the satellite to rotate about its center of mass

# add spacecraftPlus object to the simulation process
simulationBase.AddModelToTask(dynamicsTaskName, satellite)

# clear prior gravity data left over from earlier sims
simIncludeGravity.clearSetup()

# setup Gravity Body (Sun)
simIncludeGravity.addSun()                            #add the sun to the simulation including the radius and gravitional parameter
simIncludeGravity.gravBodyList[-1].isCentralBody = True #make the most recently added gravBody be the central body.
mu = simIncludeGravity.gravBodyList[-1].mu              #get mu from the added Earth data.

simIncludeGravity.addEarth()
simIncludeGravity.addMars()



# attach gravity model to spaceCraftPlus
satellite.gravField.gravBodies = spacecraftPlus.GravBodyVector(simIncludeGravity.gravBodyList) #Tell the satellite that every body in the gravBodyList is acting on it.

#
#   setup orbit
#
# setup the orbit using classical orbit elements
oe = orbitalMotion.ClassicElements() #create an empty set of orbital elements
AU = 1.496e11      # meters from the center of the sun
#Now set up the orbital elements for a LEO circular orbit
oe.a     = AU
oe.e     = 0.5
oe.i     = 90.0*macros.D2R
oe.Omega = 48.2 * macros.D2R
oe.omega = 347.8 * macros.D2R
oe.f = 85.3 * macros.D2R
rN, vN = orbitalMotion.elem2rv(mu, oe) #convert the orbital elements to inertial position and velocity
oe = orbitalMotion.rv2elem(mu, rN, vN) # this stores consistent initial orbit elements

# with circular or equatorial orbit, some angles are arbitrary

#
#   initialize Spacecraft States with the initialization variables
#
satellite.hub.r_CN_NInit = unitTestSupport.np2EigenVectorXd(rN)  # meters from center of inertial frame
satellite.hub.v_CN_NInit = unitTestSupport.np2EigenVectorXd(vN)  # meters/second velocity relative to inertial frame


# set the simulation time
n = np.sqrt(mu/oe.a/oe.a/oe.a)  #The orbital frequency of the orbit set above
P = 2.*np.pi/n                  #The period, P of the orbit set above

simulationTime = macros.sec2nano(P/100)    #run the simulation for three-quarters of a period

#
#   Setup data logging before the simulation is initialized
#

numDataPoints = 100 #number of points during the simulation to record data.

samplingTime = simulationTime / (numDataPoints-1)                                   #time between samplings
simulationBase.TotalSim.logThisMessage(satellite.scStateOutMsgName, samplingTime)   #log the satellite state message at every sampling time step

#
# create simulation messages
# the addDefaultEphemerisMsg can be used if only one gravBody is present
#simIncludeGravity.addDefaultEphemerisMsg(simulationBase.TotalSim, dynamicsProcessName) #create a message from simulationBase.TotalSim which includes the default gravBody information
#above message not needed with multiple gravBodies for some reason - SJKC.

#
#   initialize Simulation:  This function clears the simulation log, and runs the self_init()
#   cross_init() and reset() routines on each module.
#   If the routine InitializeSimulationAndDiscover() is run instead of InitializeSimulation(),
#   then the all messages are auto-discovered that are shared across different BSK threads.
#
simulationBase.InitializeSimulationAndDiscover()

#
#   configure a simulation stop time time and execute the simulation run
#
simulationBase.ConfigureStopTime(simulationTime)    #stop when the entire simulation has been run
simulationBase.ExecuteSimulation()                  #run the simulation

#
#   retrieve the logged position and velocity (of the satellite) data from throughout the entire simulation execution
#
posData = simulationBase.pullMessageLogData(satellite.scStateOutMsgName+'.r_BN_N',range(3))
velData = simulationBase.pullMessageLogData(satellite.scStateOutMsgName+'.v_BN_N',range(3))


#
#   plot the results
#
fileNameString = filename[len(path)+6:-3] #grab the name of this test (scenarioHRB) from the filepath.
    
# draw the inertial position vector components
# This plots the X, Y, and Z positions of the spacecraft relative to the planet vs. time.
plt.close("all")    # clears out plots from earlier test runs
plt.figure(1)       # select figure 1 for plotting
fig = plt.gcf()     # create a handle for figure 1 called fig
ax = fig.gca()      # create a handle for the current axis called ax
ax.ticklabel_format(useOffset=False, style='plain')
for idx in range(1,4):  #for each orthogonal component of the position and velocity data
    plt.plot(posData[:, 0]*macros.NANO2SEC/P, posData[:, idx]/1000.,
             color=unitTestSupport.getLineColor(idx,3),
             label='$r_{BN,'+str(idx)+'}$') #plot(time, position(idx), color, label)
plt.legend(loc='lower right')
plt.xlabel('Time [orbits]')
plt.ylabel('Inertial Position [km]')


# draw orbit in perifocal frame
b = oe.a*np.sqrt(1-oe.e*oe.e)   #the semi-minor axis
p = oe.a*(1-oe.e*oe.e)          #the parameter, semilatus rectum
plt.figure(2,figsize=np.array((1.0, b/oe.a))*4.75,dpi=100)      #switch to figure 2 and set it up to fit the data nicely
plt.axis(np.array([-oe.rApoap, oe.rPeriap, -b, b])/1000*1.25)   #set the axis to handle the data nicely.
# draw the planet (just a colored circle on the graph)
fig = plt.gcf()         #make fig the handle for figure 2 now
ax = fig.gca()          #make ax the handle for the axis on figure 2 now
planetColor= '#559900'  #choose whatever you like
planetRadius = simIncludeGravity.gravBodyList[0].radEquator/1000    #set the radius of the circle in [km]
ax.add_artist(plt.Circle((0, 0), planetRadius, color=planetColor))  #use the artist functionality to draw the circle(planet)
# draw the actual orbit
rData=[] #an empty list for radius data
fData=[] #an empty list for orbital ???
for idx in range(0,len(posData)): #for every time step
    oeData = orbitalMotion.rv2elem(mu,posData[idx,1:4],velData[idx,1:4]) #convert r,v back to oe
    rData.append(oeData.rmag)
    fData.append(oeData.f + oeData.omega - oe.omega)
plt.plot(rData*np.cos(fData)/1000, rData*np.sin(fData)/1000
         ,color='#aa0000'
         ,linewidth = 3.0
         )
# draw the full osculating orbit from the initial conditions
fData = np.linspace(0,2*np.pi,100)
rData = []
for idx in range(0,len(fData)):
    rData.append(p/(1+oe.e*np.cos(fData[idx])))
plt.plot(rData*np.cos(fData)/1000, rData*np.sin(fData)/1000
         ,'--'
         , color='#555555'
         )
plt.xlabel('$i_e$ Cord. [km]')
plt.ylabel('$i_p$ Cord. [km]')
plt.grid()


plt.show()
    
# close the plots being saved off to avoid over-writing old and new figures
plt.close("all")
