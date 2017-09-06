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
#   IMU Unit Test
#   Purpose:  Test IMU functions
#   Author:  Scott Carnahan
#   Author: John Alcorn wrote the original script which was highly modified after modifications to the IMU sensor
#   Creation Date:  September 6, 2016
#   Revision Date:  August 17, 2017
#

# import pytest
import sys, os, inspect
import numpy as np
import pytest
import matplotlib.pyplot as plt

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
splitPath = path.split('Basilisk')
sys.path.append(splitPath[0]+'/Basilisk/modules')
sys.path.append(splitPath[0]+'/Basilisk/PythonModules')

import SimulationBaseClass
import unitTestSupport  # general support file with common unit test functions
import macros
import imu_sensor
import sim_model
import RigidBodyKinematics as rbk

np.random.seed(2000000)

# methods
def m33v3mult(M,v):
    output = [[0.],[0.],[0.]]
    np.reshape(v,(3,1))
    for i in range(0,3):
        for k in range(0,3):
            output[i] += M[i][k]*v[k]

    return np.reshape(np.array(output),(1,3))

def v3vTmult(v1,v2):
    output = [[0,0,0],[0,0,0],[0,0,0]]
    for i in range(0,3):
        for j in range(0,3):
            output[i][j] = v1[i] * v2[j]
    return output

def skew(vector):
    vector = np.array(vector)
    skew_symmetric = np.array([[0, -vector.item(2), vector.item(1)],
                     [vector.item(2), 0, -vector.item(0)],
                     [-vector.item(1), vector.item(0), 0]])
    return skew_symmetric

def listStack(vec,stopTime,unitProcRate):
    # returns a list duplicated the number of times needed to be consistent with module output
    return [vec] * int(stopTime/(float(unitProcRate)/float(macros.sec2nano(1))))

def myRand(n):
    # generates a uniformly distributed np array of size n and range +/- 1
    return np.random.rand(n)*2 - 1

def nextMRP(sigma,omega,dt):
    # propagate MRP from current sigma and omega and dt
    dsigma = 0.25*np.dot(rbk.BmatMRP(sigma),np.reshape(omega,[3,1]))
    return dsigma*dt + np.reshape(sigma,[3,1])

def setRandomWalk(self,senRotNoiseStd = 0.0,senTransNoiseStd = 0.0,errorBoundsGyro = [1e6] * 3,errorBoundsAccel = [1e6] * 3):
    # sets the random walk for IRU module
    PMatrixGyro = [0.0] * 3 * 3
    PMatrixGyro[0*3+0] = PMatrixGyro[1*3+1] = PMatrixGyro[2*3+2] = senRotNoiseStd
    PMatrixAccel = [0.0] * 3 * 3
    PMatrixAccel[0*3+0] = PMatrixAccel[1*3+1] = PMatrixAccel[2*3+2] = senTransNoiseStd
    self.PMatrixAccel = sim_model.DoubleVector(PMatrixAccel)
    self.walkBoundsAccel = sim_model.DoubleVector(errorBoundsAccel)
    self.PMatrixGyro = sim_model.DoubleVector(PMatrixGyro)
    self.walkBoundsGyro = sim_model.DoubleVector(errorBoundsGyro)

# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed

# The following 'parametrize' function decorator provides the parameters and expected results for each
#   of the multiple test runs for this test.
@pytest.mark.parametrize("show_plots, make_plots, testCase, stopTime, accuracy", [
    (False, False,'gyroIO', 0.25, 1e-6),
    # (False, False,'bias', 0.5, 1e-3),
    # (False, False,'noise', 2000., 1e-2),
    # (False, False,'discretization', 0.5, 1e-5),
    # (False, False,'saturation', 0.1, 1e-3),
    # (False, False,'CoM offset', 0.1, 1e-4),
    # (False, False,'misalignment', 0.5, 1e-5),
    # (False, False,'walk bounds', 2500., 0.0),
])

# provide a unique test method name, starting with test_
def test_unitSimIMU(show_plots, make_plots, testCase, stopTime, accuracy):
    # each test method requires a single assert method to be called
    [testResults, testMessage] = unitSimIMU(show_plots, make_plots, testCase, stopTime, accuracy)
    assert testResults < 1, testMessage


def unitSimIMU(show_plots, make_plots, testCase, stopTime, accuracy):
    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty array to store test log messages

    unitTaskName = "unitTask"  # arbitrary name (don't change)
    unitProcName = "TestProcess"  # arbitrary name (don't change)

    # initialize SimulationBaseClass
    unitSim = SimulationBaseClass.SimBaseClass()
    unitSim.TotalSim.terminateSimulation()

    # create the task and specify the integration update time
    unitProcRate_s = 0.0001
    unitProcRate = macros.sec2nano(unitProcRate_s)
    unitProc = unitSim.CreateNewProcess(unitProcName)
    unitTask = unitSim.CreateNewTask(unitTaskName, unitProcRate)
    unitProc.addTask(unitTask)

    # configure IMU
    ImuSensor = imu_sensor.ImuSensor()
    ImuSensor.ModelTag = "imusensor"
    ImuSensor.sensorPos_B = imu_sensor.DoubleVector([0.0, 0.0, 0.0]) #must be set by user - no default.
    setRandomWalk(ImuSensor)


    # StatePrevious = imu_sensor.SCPlusStatesSimMsg(StateCurrent) # For time differencing internal to IMU for truth values

    # get module output fields
    ImuSensorOutput = imu_sensor.IMUSensorIntMsg()
    fieldNames = list()
    for fieldName in dir(ImuSensorOutput):
        if fieldName.find('__') < 0 and fieldName.find('this') < 0:
            if(callable(getattr(ImuSensorOutput,fieldName))):
                continue
            fieldNames.append(fieldName)

    #Set-up the fake kinematics vectors
    rDotDot_CN = np.resize(np.array([1., 1., 1.]),(stopTime/unitProcRate_s+1,3)) #acceleration of center of mass wrt inertial frame
    rDot_CN = np.resize(np.array([0., 0., 0.]),(stopTime/unitProcRate_s+1,3)) #velocity of center of mass wrt inertial frame
    rDot_CN[0][:] = [0., 0., 0.] #initial value
    r_CN = np.resize(np.array([0., 0., 0.]),(stopTime/unitProcRate_s+1,3)) #position of center of mass wrt to inertial frame
    # rDot_CN[0][:] = [1., 1., 1.] #initial value
    rDotDot_BN = np.resize(np.array([1., 1., 1.]),(stopTime/unitProcRate_s+1,3)) # acceleration of body frame wrt to inertial
    # rDotDot_BN[0][:] = [1., 1., 1.] #initial value
    rDot_BN = np.resize(np.array([0., 0., 0.]),(stopTime/unitProcRate_s+1,3)) #velocity of body frame wrt to inertial
    # rdot_BN[0][:] = [1., 1., 1.] #initial value
    r_BN = np.resize(np.array([0., 0., 0.]),(stopTime/unitProcRate_s+1,3)) #position of body frame wrt to inertial
    # r_BN[0][:] = [1., 1., 1.] #initial value
    omegaDot = np.resize(np.array([0.1, 0.1, 0.1]),(stopTime/unitProcRate_s+1,3)) #angular acceleration of body frame wrt to inertial
    # omegaDot[0][:] = [1., 1., 1.] #initila value
    omega = np.resize(np.array([0.01, 0.01,0.01]),(stopTime/unitProcRate_s+1,3)) # angular rate of body frame wrt to inertial
    # omega[0][:] = [1., 1., 1.] #initial value
    sigmaDot = np.resize(np.array([0., 0., 0.]),(stopTime/unitProcRate_s+1,3)) #MRP derivative, body wrt to inertial
    # sigmaDot[0][:] = [1., 1., 1.] #initial value
    sigma = np.resize(np.array([0., 0., 0.]),(stopTime/unitProcRate_s+1,3)) # MRP body wrt to inertial
    # sigma[0][:] = [1., 1., 1.] #initial value
    rDotDot_SN = np.resize(np.array([0., 0., 0.]),(stopTime/unitProcRate_s+1,3)) #sensor sensed acceleration
    # rDotDot_SN[0][:] = [1., 1., 1.] #initial value
    rDot_SN = np.resize(np.array([0., 0., 0.]),(stopTime/unitProcRate_s+1,3)) #sensor accumulated DV
    # rDot_SN[0][:] = [1., 1., 1.] #initial value
    r_SN = np.resize(np.array([0., 0., 0.]),(stopTime/unitProcRate_s+1,3)) #sensor
    r_SB = np.array([1.,2.,3.]) #constant. sensor position wrt to body frame origin
    r_SC = np.array([1.,2.,3.]) #variable but untracked sensor position wrt to center of mass


    #Set initial conditions for fake kinematics vectors
    # Center of mass
    # rDotDot_CN = leaving blank for now to keep at zeros. Should be made to move slowly relative to B or oscillate about B
    # rDot_CN[0][i] = leaving blank for now to keep at zeros
    r_CN[0][i] = [10000., 10000., 10000.] # Some arbitrary location in space
    # Body Frame Origin
    # rDotDot_BN = leaving blank for now (no acceleration)
    r_SN[0][:] = r_SC + r_CN[0][:]


    # # configure spacecraft dummy message
    # StateCurrent = imu_sensor.SCPlusStatesSimMsg()
    # StateCurrent.sigma_BN = np.array([0., 0., 0.])
    # StateCurrent.omega_BN_B = np.array([0.,0.,0.]) #1 rpm around each axis
    # StateCurrent.nonConservativeAccelpntB_B = np.array([0., 0., 0.])
    # StateCurrent.omegaDot_BN_B = np.array([0., 0., 0.])
    # StateCurrent.TotalAccumDV_BN_B = np.array([0., 0., 0.])
    # massStateCurrent = imu_sensor.SCPlusMassPropsSimMsg()
    # massStateCurrent.c_B = [0., 0., 0.]

    # configure truth
    # trueVector = dict()
    # trueVector['AngVelPlatform'] = listStack(omega,stopTime,unitProcRate)
    # trueVector['AccelPlatform'] = listStack(accel,stopTime,unitProcRate)
    # trueVector['DRFramePlatform'] = listStack(np.asarray(omega)*unitProcRate_s,stopTime,unitProcRate)
    # trueVector['DVFramePlatform'] = listStack(np.asarray(accel)*unitProcRate_s,stopTime,unitProcRate)

    # omegaTrue = StateCurrent.omega_BN_B

    # add module to the task
    unitSim.AddModelToTask(unitTaskName, ImuSensor)

    # log module output message
    unitSim.TotalSim.logThisMessage(ImuSensor.OutputDataMsg, unitProcRate)

    # # configure spacecraft_mass_props message
    # unitSim.TotalSim.CreateNewMessage("TestProcess", ImuSensor.InputMassMsg, MassPropsData.getStructSize(), 2)
    # unitSim.TotalSim.WriteMessageData(ImuSensor.InputMassMsg, MassPropsData.getStructSize(), 0, MassPropsData)

    # configure inertial_state_output message
    # unitSim.TotalSim.CreateNewMessage("TestProcess", "inertial_state_output", 8*3*11, 2)
    # unitSim.TotalSim.WriteMessageData("inertial_state_output", 8*3*11, 0, StateCurrent)

    unitSim.InitializeSimulation()

    # loop through ExecuteSimulation() and propagate sigma, omega, DV
    dt = unitProcRate_s
    for i in range(1,int(stopTime/dt)+1):
        # Step through the sim
        unitSim.ConfigureStopTime(macros.sec2nano(unitProcRate_s*float(i)))
        unitSim.ExecuteSimulation()

        # linear kinematcs
        rDot_CN[i][:] = rDot_CN[i-1][:] + ((rDotDot_CN[i-1][:] + rDotDot_CN[i][:])/2)*dt
        r_CN[i][:] = r_CN[i-1][:] + ((rDot_CN[i-1][:] + rDot_CN[i][:])/2)*dt

        rDot_BN[i][:] = rDot_BN[i-1][:] + ((rDotDot_BN[i-1][:] + rDotDot_BN[i][:])/2)*dt
        r_BN[i][:] = r_BN[i-1][:] + ((rDot_BN[i-1][:] + rDot_BN[i][:])/2)*dt

        cDotDot = rDotDot_CN[i][:] - rDotDot_BN[i][:]
        cDot = rDot_CN[i][:] - rDot_BN[i][:]
        c = r_BN[i][:] - r_CN[i][:]

        # attitude kinematics
        omega[i][:] = omega[i-1][:] + ((omegaDot[i-1][:] + omegaDot[i][:])/2)*dt

        # iterate on sigma/sigmaDot
        sigmaDot[i][:] = sigmaDot[i-1][:]
        for j in range(0,5):
            sigma[i][:] = sigma[i-1][:] + ((sigmaDot[i-1][:]+sigmaDot[i][:])/2)*dt
            sigmaMag = np.linalg.norm(sigma[i][:])
            B1 = 1-sigmaMag**2
            BI = np.identity(3)
            sigmaTilde = skew(sigma[i][:])
            B2 = np.dot(2,sigmaTilde)
            B3 = np.dot(2,v3vTmult(sigma[i][:], sigma[i][:]))
            B = np.dot(B1, BI) + B2 + B3
            sigmaDot[i][:] = np.dot(0.25, m33v3mult(B,omega[i][:]))
        sigma[i][:] = sigma[i-1][:] + ((sigmaDot[i-1][:]+sigmaDot[i][:])/2)*dt

        # center of mass calculations
        cPrime = cDot - np.cross(omega[i][:], c)
        cPrimePrime = cDotDot - np.dot(2,np.cross(omega[i][:], cPrime)) - np.cross(omegaDot[i][:],c)-np.cross(omega[i][:],np.cross(omega[i][:],c))
        r_SC = r_BN[i][:] + r_SB - r_CN[i][:]

        # solving for sensor inertial states
        rDotDot_SN[i][:] = rDotDot_CN[i][:] - cPrimePrime - np.dot(2,np.cross(omega[i][:],cPrime)) + np.cross(omegaDot[i][:],r_SC) +np.cross(omega[i][:],np.cross(omega[i][:],r_SC))
        rDot_SN[i][:] = rDot_CN[i][:] - cPrime + np.cross(omega[i][:],  r_SC)
        rDot_SN_check = rDot_SN[i-1][:] + ((rDotDot_SN[i-1][:]+rDotDot_SN[i][:])/2)*dt #This is here to check the output of the "truth" code written here in python

        r_SN[i][:] = r_SN[i-1][:] + ((rDot_SN[i-1][:] + rDot_SN[i][:])/2)*dt #for a simple check of the "truth" code
        r_SN_simple = r_SC + r_CN[i][:] #for a simple check of the "truth" code

    return [testFailCount, ''.join(testMessages)]


# This statement below ensures that the unit test script can be run as a
# stand-along python script
# if __name__ == "__main__":
#     test_unitSimIMU(
#         True, # show_plots
#         False, # make_plots
#         'gyroIO' # testCase
#     )
