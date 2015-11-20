import sys, os, inspect #Don't worry about this, standard stuff plus file discovery
import SimulationBaseClass
import random
random.seed(0x1badcad1)

class VariableDispersion:
 def __init__(self, varName, center=0.0, limits=(1.0/3.0,), generator=random.gauss):
    self.generator = generator
    self.generator
    self.center = center
    self.limits = limits
    self.varName = varName

class MonteCarloBaseClass:
 def __init__(self):
    self.simList = []
    self.varDisp = []
    self.randomizeSeeds = False
    self.executionModule = None
    self.simulationObject = None
    self.executionCount = 0
    self.retainSimulationData = False
 def setRandomizeSeeds(self, seedSet):
    self.randomizeSeeds = seedSet
 def setExecutionModule(self, newModule):
    self.executionModule = newModule
 def setSimulationObject(self, newObject):
    self.simulationObject = newObject
 def setExecutionCount(self, newCount):
    self.executionCount = newCount
 def setRetainSimulationData(self, retainData):
    self.retainSimulationData = retainData
 def addNewDispersion(self, varDisp):
    self.varDisp.append(varDisp)
 def executeSimulations(self):
    i=0
    previousSimulation = None
    while(i<self.executionCount):
        if(previousSimulation != None):
            previousSimulation.TotalSim.terminateSimulation()
        newSim = self.simulationObject()
        for disp in self.varDisp:
            nextValue = disp.generator(disp.center, disp.limits[0])
            execString = 'newSim.' + disp.varName + ' = ' + str(nextValue)
            exec(execString)
        self.executionModule(newSim)
        if(self.retainSimulationData):
            self.simList.append(newSim)
        previousSimulation = newSim
        i += 1
        print i

