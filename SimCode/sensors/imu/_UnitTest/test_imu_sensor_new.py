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
import math

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

def findSigmaDot(sigma, omega):
    sigmaMag = np.linalg.norm(sigma)
    B1 = 1 - sigmaMag ** 2
    BI = np.identity(3)
    sigmaTilde = skew(sigma)
    B2 = np.dot(2, sigmaTilde)
    B3 = np.dot(2, v3vTmult(sigma, sigma))
    B = np.dot(B1, BI) + B2 + B3
    sigmaDot = np.dot(0.25, m33v3mult(B, omega))
    return sigmaDot

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

#The following tests are parameterized and run:
# clean - a little bit of every motion/displacement but no bias, noise, etc.
# bias - bias added to the clean signal.
# discretization - Signal is discretized
# saturation - signal doesn't move outside max/min
# walk bounds - random walk over time stays within bounds

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
    # as of right now, the accelerations here are non-conservative accelerations. No conservative accelerations are used in this test - SJKC
    # center of mass
    rDotDot_CN = np.resize(np.array([0., 0., 0.]),(stopTime/unitProcRate_s+1,3)) #acceleration of center of mass wrt inertial frame
    rDotDot_CB = np.resize(np.array([0., 0., 0.]),(stopTime/unitProcRate_s+1,3)) #acceleration of center of mass wrt body frame
    rDot_CN = np.resize(np.array([0., 0., 0.]),(stopTime/unitProcRate_s+1,3)) #velocity of center of mass wrt inertial frame
    r_CN = np.resize(np.array([0., 0., 0.]),(stopTime/unitProcRate_s+1,3)) #position of center of mass wrt to inertial frame
    # body frame
    rDotDot_BN = np.resize(np.array([0., 0., 0.]),(stopTime/unitProcRate_s+1,3)) # acceleration of body frame wrt to inertial
    rDot_BN = np.resize(np.array([0., 0., 0.]),(stopTime/unitProcRate_s+1,3)) #velocity of body frame wrt to inertial
    r_BN = np.resize(np.array([0., 0., 0.]),(stopTime/unitProcRate_s+1,3)) #position of body frame wrt to inertial
    omegaDot = np.resize(np.array([0., 0., 0.]),(stopTime/unitProcRate_s+1,3)) #angular acceleration of body frame wrt to inertial
    omega = np.resize(np.array([0., 0., 0.]),(stopTime/unitProcRate_s+1,3)) # angular rate of body frame wrt to inertial
    sigmaDot = np.resize(np.array([0., 0., 0.]),(stopTime/unitProcRate_s+1,3)) #MRP derivative, body wrt to inertial
    sigma = np.resize(np.array([0., 0., 0.]),(stopTime/unitProcRate_s+1,3)) # MRP body wrt to inertial
    # sensor
    rDotDot_SN = np.resize(np.array([0., 0., 0.]),(stopTime/unitProcRate_s+1,3)) #sensor sensed acceleration
    rDot_SN = np.resize(np.array([0., 0., 0.]),(stopTime/unitProcRate_s+1,3)) #sensor accumulated DV
    rDot_SN_P = np.resize(np.array([0., 0., 0.]),(stopTime/unitProcRate_s+1,3)) #sensor accumulated DV in sensor platform frame coordinates
    r_SN = np.resize(np.array([0., 0., 0.]),(stopTime/unitProcRate_s+1,3)) #sensor position in body frame coordinates


    #Set initial conditions for fake kinematics vectors
    # Acceleration vectors
    dataRows= np.shape(rDotDot_CN)[0]
    for i in range(0, dataRows): #loops through acceleration vectors so that each element can be set individually (or, hopefully, as a function)
        rDotDot_BN[i][0] = 1.
        rDotDot_BN[i][1] = 1.
        rDotDot_BN[i][2] = 1.
        rDotDot_CB[i][0] = 0.01*math.sin(2*np.pi/25.)
        rDotDot_CB[i][1] = 0.01*math.sin(2*np.pi/25.)
        rDotDot_CB[i][2] = 0.01*math.sin(2*np.pi/25.)
        rDotDot_CN[i][0] = rDotDot_BN[i][0] + rDotDot_CB[i][0]
        rDotDot_CN[i][1] = rDotDot_BN[i][1] + rDotDot_CB[i][1]
        rDotDot_CN[i][2] = rDotDot_BN[i][2] + rDotDot_CB[i][2]
        omegaDot[i][0] = 0.01
        omegaDot[i][1] = 0.02
        omegaDot[i][2] = 0.03
    # Center of Mass
    rDot_CN[0][:] = [0.5, 0.4, 0.3]
    r_CN[0][:] = [10000., 10000., 10000.] # Some arbitrary location in space
    # Body Frame Origin
    rDot_BN[0][:] = [0.5, 0.4, 0.3]
    r_BN[0][:] = [9999.75, 10000., 10000.] # leaves r_CN[0][i] with some offset
    # Body Rotation
    # omegaDot[0][:] = [0., 0., 0.] #unlabeled omega is omegaDot_BN_B
    omega[0][:] = [0.1, 0.2, 0.3] #omega_BN_B
    sigma[0][:] = [0., 0., 0.25] # note that unlabeled sigma is sigma_BN
    sigmaDot[0][:] = findSigmaDot(sigma[0][:], omega[0][:]) # sigmaDot_BN
    # Sensor linear states (note that these initial conditions must be solved as functions of another initial conditions to maintain consistency
    r_SB = np.array([0., 0., 0.5]) #constant. sensor position wrt to body frame origin
    cDotDot = rDotDot_CN[0][:] - rDotDot_BN[0][:]
    cDot = rDot_CN[0][:] - rDot_BN[0][:]
    c =  r_CN[0][:] - r_BN[0][:]
    cPrime = cDot - np.cross(omega[0][:], c)
    cPrimePrime = cDotDot - np.dot(2, np.cross(omega[0][:], cPrime)) - np.cross(omegaDot[0][:], c) - np.cross(omega[0][:], np.cross(omega[0][:], c))
    r_SC = r_BN[0][:] + r_SB - r_CN[0][:]
    rDotDot_SN[0][:] = rDotDot_CN[0][:] - cPrimePrime - np.dot(2, np.cross(omega[0][:], cPrime)) + np.cross(omegaDot[0][:], r_SC) + np.cross(omega[0][:], np.cross(omega[0][:], r_SC))
    rDot_SN[0][:] = rDot_CN[0][:] - cPrime + np.cross(omega[0][:], r_SC)
    r_SN[0][:] = r_SB + r_BN[0][:]

    #Sensor Setup
    ImuSensor = imu_sensor.ImuSensor()
    ImuSensor.ModelTag = "imusensor"
    ImuSensor.sensorPos_B = imu_sensor.DoubleVector(r_SB) #must be set by user - no default. check if this works by giving an array - SJKC
    yaw = 0.61 #should be given as parameter [rad]
    pitch = 0.33  # [rad]
    roll = 0.75 # [rad]
    dcm_PB = rbk.euler3212C([yaw,pitch,roll]) #done separately as a check
    ImuSensor.setBodyToPlatformDCM(yaw, pitch, roll) # done separately as a check
    senRotNoiseStd = 0.0
    senTransNoiseStd = 0.0
    errorBoundsGyro = [0.0] * 3
    errorBoundsAccel = [0.0] * 3
    setRandomWalk(ImuSensor, senRotNoiseStd, senTransNoiseStd, errorBoundsGyro, errorBoundsAccel)
    ImuSensor.gyroLSB = 0.
    ImuSensor.accelLSB = 0.
    ImuSensor.senRotBias = [0.0] * 3
    ImuSensor.senTransBias = [0.0] * 3
    ImuSensor.senTransMax = 1000.
    ImuSensor.senRotMax = 1000.

    # Set-up the sensor output truth vectors
    rDotDot_SN_P = np.resize(np.array([0., 0., 0.]), (stopTime / unitProcRate_s + 1, 3))  # sensor sensed acceleration in sensor platform frame coordinates
    rDotDot_SN_P[0][:] = m33v3mult(dcm_PB, rDotDot_SN[0][:])
    DVAccum_P = np.resize(np.array([0., 0., 0.]), (stopTime / unitProcRate_s + 1, 3))  # sensor accumulated delta V ouput in the platform frame
    stepPRV_P = np.resize(np.array([0., 0., 0.]), (stopTime / unitProcRate_s + 1, 3))  # principal rotatation vector from time i-1 to time i in platform frame coordinates
    omega_P = np.resize(np.array([0., 0., 0.]), (stopTime / unitProcRate_s + 1, 3))  # angular rate omega_BN_P = omega_PN_P
    omega_P[0][:] = m33v3mult(dcm_PB, omega[0][:])

    # configure spacecraft dummy message
    StateCurrent = imu_sensor.SCPlusStatesSimMsg()
    StateCurrent.sigma_BN = sigma[0][:]
    StateCurrent.omega_BN_B = omega[0][:] #1 rpm around each axis
    StateCurrent.nonConservativeAccelpntB_B = rDotDot_BN[0][:]
    StateCurrent.omegaDot_BN_B = omegaDot[0][:]
    StateCurrent.TotalAccumDV_BN_B = np.array([0., 0., 0.])

    # add module to the task
    unitSim.AddModelToTask(unitTaskName, ImuSensor)

    # configure MassPropsData - should be able to remove and remove mass message from IMU code - SJKC
    MassPropsData = imu_sensor.SCPlusMassPropsSimMsg()
    MassPropsData.massSC = -97.9
    MassPropsData.c_B = [0,0,0]
    MassPropsData.ISC_PntB_B = [[100.0, 0.0, 0.0], [0.0, 100.0, 0.0], [0.0, 0.0, 100.0]]

    # configure spacecraft_mass_props message
    unitSim.TotalSim.CreateNewMessage(unitProcName, ImuSensor.InputMassMsg, MassPropsData.getStructSize(), 2)
    unitSim.TotalSim.WriteMessageData(ImuSensor.InputMassMsg, MassPropsData.getStructSize(), 0, MassPropsData)

    # configure inertial_state_output message
    unitSim.TotalSim.CreateNewMessage(unitProcName, "inertial_state_output", 8*3*11, 2)
    unitSim.TotalSim.WriteMessageData("inertial_state_output", 8*3*11, 0, StateCurrent)

    # log module output message
    unitSim.TotalSim.logThisMessage(ImuSensor.OutputDataMsg, unitProcRate)

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
        c = r_CN[i][:] - r_BN[i][:]

        # attitude kinematics
        omega[i][:] = omega[i-1][:] + ((omegaDot[i-1][:] + omegaDot[i][:])/2)*dt

        # iterate on sigma/sigmaDot
        sigmaDot[i][:] = sigmaDot[i-1][:]
        for j in range(0,5): #Seems to converge after a few iterations
            sigma[i][:] = sigma[i-1][:] + ((sigmaDot[i-1][:]+sigmaDot[i][:])/2)*dt
            sigmaDot[i][:] = findSigmaDot(sigma[i][:],omega[i][:])
        sigma[i][:] = sigma[i-1][:] + ((sigmaDot[i-1][:]+sigmaDot[i][:])/2)*dt
        # if np.linalg.norm(sigma[i][:])  > 1.:
        #     print "hey"
        #     sigma[i][:] = np.dot(np.dot(-1, sigma[i][:]),  1/(sigma[i][0]**2 + sigma[i][1]**2 + sigma[i][2]**2))

        # center of mass calculations
        cPrime = cDot - np.cross(omega[i][:], c)
        cPrimePrime = cDotDot - np.dot(2,np.cross(omega[i][:], cPrime)) - np.cross(omegaDot[i][:],c)-np.cross(omega[i][:],np.cross(omega[i][:],c))
        r_SC = r_BN[i][:] + r_SB - r_CN[i][:]

        # solving for sensor inertial states
        rDotDot_SN[i][:] = rDotDot_CN[i][:] - cPrimePrime - np.dot(2,np.cross(omega[i][:],cPrime)) + np.cross(omegaDot[i][:],r_SC) +np.cross(omega[i][:],np.cross(omega[i][:],r_SC))
        rDot_SN[i][:] = rDot_CN[i][:] - cPrime + np.cross(omega[i][:],  r_SC)
        rDot_SN_check = rDot_SN[i-1][:] + ((rDotDot_SN[i-1][:]+rDotDot_SN[i][:])/2)*dt #This is here to check the output of the "truth" code written here in python if desired
        r_SN[i][:] = r_SN[i-1][:] + ((rDot_SN[i-1][:] + rDot_SN[i][:])/2)*dt #for a simple check of the "truth" code
        # r_SN_simple = r_SC + r_CN[i][:] #for a simple check of the "truth" code if desired.

        # Now create outputs which are (supposed to be) equivalent to the IMU output
        # linear acceleration (non-conservative) in platform frame
        rDotDot_SN_P[i][:] = m33v3mult(dcm_PB, rDotDot_SN[i][:]) #This should match trueValues.AccelPlatform
        # accumulated delta v (non-conservative) in platform frame
        DVAccum_P[i][:] = m33v3mult(dcm_PB, rDot_SN[i][:]-rDot_SN[i-1][:])
        # find PRV between before and now
        sigma_NB_2 = np.dot(-1, sigma[i][:]) #sigma from B to N in B frame coordinates at time 2
        sigma_NB_1 = np.dot(-1, sigma[i-1][:])
        sigma_21 = rbk.subMRP(sigma_NB_2, sigma_NB_1)
        if np.linalg.norm(sigma_21) != 0.: #MRP2PRV divides by zero and gives a warning if the attitude hasn't changed.
            stepPRV = rbk.MRP2PRV(sigma_21)
        else:
            stepPRV = [0., 0., 0.]
        stepPRV_P[i][:] = m33v3mult(dcm_PB, stepPRV)
        # angular rate in platform frame
        omega_P[i][:] = m33v3mult(dcm_PB, omega[i][:])

        #Now update spacecraft states for the IMU:
        StateCurrent = imu_sensor.SCPlusStatesSimMsg()
        StateCurrent.sigma_BN = sigma[i][:]
        StateCurrent.omega_BN_B = omega[i][:]  # 1 rpm around each axis
        StateCurrent.nonConservativeAccelpntB_B = rDotDot_BN[i][:]
        StateCurrent.omegaDot_BN_B = omegaDot[i][:]
        StateCurrent.TotalAccumDV_BN_B = rDot_BN[i][:] - rDot_BN[0][:]
        unitSim.TotalSim.WriteMessageData("inertial_state_output", 8 * 3 * 11, unitSim.TotalSim.CurrentNanos, StateCurrent)

        # DV_BN_B = StateCurrent.TotalAccumDV_BN_B - TotalAccumDV_BN_B_prev
        # print DV_BN_B

    DRout       = unitSim.pullMessageLogData(ImuSensor.OutputDataMsg + '.' + "DRFramePlatform", range(3))
    omegaOut    = unitSim.pullMessageLogData(ImuSensor.OutputDataMsg + '.' + "AngVelPlatform", range(3)) #checks
    rDotDotOut  = unitSim.pullMessageLogData(ImuSensor.OutputDataMsg + '.' + "AccelPlatform", range(3))
    DVout       = unitSim.pullMessageLogData(ImuSensor.OutputDataMsg + '.' + "DVFramePlatform", range(3))

    # test outputs
    for i in range(2,len(stepPRV_P)-1):
        if not unitTestSupport.isArrayEqualRelative(DRout[i][:], stepPRV_P[i+1][:], 3, 1e-4):
            print "fail", i
            print stepPRV_P[i+1][:], DRout[i][:], sigma[i][:], sigma[i+1][:]
        # # else:
        # #     print "pass"
        # if not unitTestSupport.isArrayEqualRelative(omegaOut[i][:], omega_P[i+1][:], 3, 1e-4):
        #     print "fail"
        # # else:
        # #     print "pass"
        # if not unitTestSupport.isArrayEqualRelative(DVout[i][:], DVAccum_P[i + 1][:], 3, 1e-4):
        #     print "fail"
        # # else:
        # #     print "pass"
        # if not unitTestSupport.isArrayEqualRelative(rDotDotOut[i][:], rDotDot_SN_P[i + 1][:], 3, 1e-4):
        #     print "fail"
        # # else:
        # #     print "pass"

    # print DVout, DVAccum_P
    # print DVAccum_P, DVout
    return [testFailCount, ''.join(testMessages)]

# This statement below ensures that the unit test script can be run as a
# stand-along python script
# if __name__ == "__main__":
#     test_unitSimIMU(
#         True, # show_plots
#         False, # make_plots
#         'gyroIO' # testCase
#     )
