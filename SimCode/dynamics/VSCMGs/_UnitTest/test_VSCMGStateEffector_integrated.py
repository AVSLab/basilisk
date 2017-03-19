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
import sys, os, inspect
import numpy as np
import pytest
import math

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
splitPath = path.split('SimCode')
sys.path.append(splitPath[0] + '/modules')
sys.path.append(splitPath[0] + '/PythonModules')

import SimulationBaseClass
import unitTestSupport  # general support file with common unit test functions
import matplotlib as mpl
import matplotlib.pyplot as plt
import spacecraftPlus
import sim_model
import macros
import gravityEffector
import spice_interface
import VSCMGStateEffector

mpl.rc("figure", figsize=(5.75,4))

rpm2rad = 2.*math.pi/60.

def defaultVSCMG():
    VSCMG = VSCMGStateEffector.VSCMGConfigSimMsg()
    VSCMG.typeName = 'Honeywell_HR16'
    VSCMG.rGB_S = [[0.],[0.],[0.]]
    VSCMG.gsHat0_S = [[0.],[0.],[0.]]
    VSCMG.gtHat0_S = [[0.],[0.],[0.]]
    VSCMG.ggHat_S = [[0.],[0.],[0.]]
    VSCMG.u_s_max = -1
    VSCMG.u_s_min = -1
    VSCMG.u_s_f = 0.
    VSCMG.wheelLinearFrictionRatio = -1
    VSCMG.u_g_current = 0.
    VSCMG.u_g_max = -1
    VSCMG.u_g_min = -1
    VSCMG.u_g_f = 0.
    VSCMG.gimbalLinearFrictionRatio = -1
    VSCMG.Omega = 0.
    VSCMG.gamma = 0.
    VSCMG.gammaDot = 0.
    VSCMG.Omega_max = 6000. * rpm2rad
    VSCMG.gammaDot_max = -1
    VSCMG.IW1 = 100./VSCMG.Omega_max # 0.159154943092
    VSCMG.IW2 = 0.5*VSCMG.IW1 # 0.0795774715459
    VSCMG.IW3 = 0.5*VSCMG.IW1 # 0.0795774715459
    VSCMG.IG1 = 0.1
    VSCMG.IG2 = 0.2
    VSCMG.IG3 = 0.3
    VSCMG.U_s = 4.8e-06 * 1e4
    VSCMG.U_d = 1.54e-06 * 1e4
    VSCMG.l = 0.01
    VSCMG.L = 0.1
    VSCMG.rGcG_G = [[0.0001],[-0.02],[0.1]]
    VSCMG.massW = 6.
    VSCMG.massG = 6.
    VSCMG.VSCMGModel = 0
    return VSCMG

def computeFFT(y,dt):
    Fs = 1.0/dt  # sampling rate
    Ts = dt # sampling interval
    n = len(y) # length of the signal
    k = np.arange(n)
    T = n/Fs
    frq = k/T # two sides frequency range
    frq = frq[range(n/2)] # one side frequency range
    Y = np.fft.fft(y)/n # fft computing and normalization
    Y = Y[range(n/2)]
    Y = abs(Y)
    return [frq,Y]

def findPeaks(Y,maxfind):
        peakIdxs = np.r_[True, Y[1:] > Y[:-1]] & np.r_[Y[:-1] > Y[1:], True]
        peakIdxs[0] = False
        peakIdxs[-1] = False

        peakIdxs = np.array(np.where(peakIdxs==True))[0]
        threshold = np.sort(Y[peakIdxs])[-maxfind]
        peakIdxs = peakIdxs[np.where(Y[peakIdxs] >= threshold)[0]]

        return peakIdxs


@pytest.mark.parametrize("useFlag, testCase", [
    (False,'BalancedWheels'),
    (False,'JitterSimple'),
    (False,'JitterFullyCoupled'),
])

# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail() # need to update how the RW states are defined
# provide a unique test method name, starting with test_
def test_VSCMGIntegratedTest(show_plots,useFlag,testCase):
    [testResults, testMessage] = VSCMGIntegratedTest(show_plots,useFlag,testCase)
    assert testResults < 1, testMessage

def VSCMGIntegratedTest(show_plots,useFlag,testCase):
    # The __tracebackhide__ setting influences pytest showing of tracebacks:
    # the mrp_steering_tracking() function will not be shown unless the
    # --fulltrace command line option is specified.
    __tracebackhide__ = True

    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty list to store test log messages

    scObject = spacecraftPlus.SpacecraftPlus()
    scObject.ModelTag = "spacecraftBody"

    unitTaskName = "unitTask"  # arbitrary name (don't change)
    unitProcessName = "TestProcess"  # arbitrary name (don't change)
    rwCommandName = "vscmg_cmds"

    # Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()
    unitTestSim.TotalSim.terminateSimulation()

    # Create test thread
    if testCase == 'JitterFullyCoupled':
        dt = 0.00001
        duration = 1.
    else:
        dt = 0.001
        duration = 1.
    testProcessRate = macros.sec2nano(dt)  # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # add RW devices

    VSCMGs = []

    ang = 54.75 * np.pi/180

    VSCMGs.append(defaultVSCMG())
    VSCMGs[0].gsHat0_S = [[1.0], [0.0], [0.0]]
    VSCMGs[0].gtHat0_S = [[0.0], [1.0], [0.0]]
    VSCMGs[0].ggHat_S = [[0.0], [0.0], [1.0]]
    VSCMGs[0].Omega = 2000 * rpm2rad
    VSCMGs[0].gamma = 0.
    VSCMGs[0].gammaDot = 0.06
    VSCMGs[0].rGB_S = [[0.1], [0.002], [-0.02]]

    VSCMGs.append(defaultVSCMG())
    VSCMGs[1].gsHat0_S = [[0.0], [1.0], [0.0]]
    VSCMGs[1].ggHat_S = [[math.cos(ang)], [0.0], [math.sin(ang)]]
    VSCMGs[1].gtHat0_S = np.cross(np.array([math.cos(ang), 0.0, math.sin(ang)]),np.array([0.0, 1.0, 0.0]))
    VSCMGs[1].Omega =  350 * rpm2rad
    VSCMGs[1].gamma = 0.
    VSCMGs[1].gammaDot = 0.011
    VSCMGs[1].rGB_S = [[0.0], [-0.05], [0.0]]

    VSCMGs.append(defaultVSCMG())
    VSCMGs[2].gsHat0_S = [[0.0], [-1.0], [0.0]]
    VSCMGs[2].ggHat_S = [[-math.cos(ang)], [0.0], [math.sin(ang)]]
    VSCMGs[2].gtHat0_S = np.cross(np.array([-math.cos(ang), 0.0, math.sin(ang)]),np.array([0.0, -1.0, 0.0]))
    VSCMGs[2].Omega = -900 * rpm2rad
    VSCMGs[2].gamma = 0.
    VSCMGs[2].gammaDot = -0.003
    VSCMGs[2].rGB_S = [[-0.1], [0.05], [0.05]]

    if testCase == 'BalancedWheels':
        VSCMGModel = 0
    elif testCase == 'JitterSimple':
        VSCMGModel = 1
    elif testCase == 'JitterFullyCoupled':
        VSCMGModel = 2

    for VSCMG in VSCMGs:
        VSCMG.VSCMGModel = VSCMGModel

    if testCase == 'JitterFullyCoupled':
        VSCMGs = [VSCMGs[0],VSCMGs[2]]

    N = len(VSCMGs)

    # create RW object container and tie to spacecraft object
    rwStateEffector = VSCMGStateEffector.VSCMGStateEffector()
    rwStateEffector.ModelTag = "VSCMGs"
    for item in VSCMGs:
        rwStateEffector.AddVSCMG(item)
    scObject.addStateEffector(rwStateEffector)

    # set RW torque command
    cmdArray = VSCMGStateEffector.VSCMGArrayTorqueIntMsg()
    cmdArray.wheelTorque = [0.001, 0.005, -0.009] # [Nm]
    cmdArray.gimbalTorque = [0.008, -0.0015, -0.006] # [Nm]
    unitTestSupport.setMessage(unitTestSim.TotalSim,
                               unitProcessName,
                               rwCommandName,
                               cmdArray)

    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, rwStateEffector)
    unitTestSim.AddModelToTask(unitTaskName, scObject)

    if testCase != 'JitterFullyCoupled':
        unitTestSim.earthGravBody = gravityEffector.GravBodyData()
        unitTestSim.earthGravBody.bodyInMsgName = "earth_planet_data"
        unitTestSim.earthGravBody.outputMsgName = "earth_display_frame_data"
        unitTestSim.earthGravBody.mu = 0.3986004415E+15 # meters!
        unitTestSim.earthGravBody.isCentralBody = True
        unitTestSim.earthGravBody.useSphericalHarmParams = False

        earthEphemData = spice_interface.SpicePlanetStateSimMsg()
        earthEphemData.J2000Current = 0.0
        earthEphemData.PositionVector = [0.0, 0.0, 0.0]
        earthEphemData.VelocityVector = [0.0, 0.0, 0.0]
        earthEphemData.J20002Pfix = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
        earthEphemData.J20002Pfix_dot = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
        earthEphemData.PlanetName = "earth"

        scObject.gravField.gravBodies = spacecraftPlus.GravBodyVector([unitTestSim.earthGravBody])

        msgSize = earthEphemData.getStructSize()
        unitTestSim.TotalSim.CreateNewMessage(unitProcessName,
            unitTestSim.earthGravBody.bodyInMsgName, msgSize, 2)
        unitTestSim.TotalSim.WriteMessageData(unitTestSim.earthGravBody.bodyInMsgName, msgSize, 0, earthEphemData)

    unitTestSim.InitializeSimulation()

    # log data
    unitTestSim.TotalSim.logThisMessage(scObject.scStateOutMsgName, testProcessRate)
    unitTestSim.TotalSim.logThisMessage(rwStateEffector.OutputDataString, testProcessRate)

    unitTestSim.AddVariableForLogging(scObject.ModelTag + ".totOrbAngMomPntN_N", testProcessRate, 0, 2, 'double')
    unitTestSim.AddVariableForLogging(scObject.ModelTag + ".totRotAngMomPntC_N", testProcessRate, 0, 2, 'double')
    unitTestSim.AddVariableForLogging(scObject.ModelTag + ".totRotEnergy", testProcessRate, 0, 0, 'double')
    unitTestSim.AddVariableForLogging(scObject.ModelTag + ".totOrbKinEnergy", testProcessRate, 0, 0, 'double')

    posRef = scObject.dynManager.getStateObject("hubPosition")
    velRef = scObject.dynManager.getStateObject("hubVelocity")
    sigmaRef = scObject.dynManager.getStateObject("hubSigma")
    omegaRef = scObject.dynManager.getStateObject("hubOmega")

    if testCase == 'JitterFullyCoupled':
        posRef.setState([[0.0],	[0.0],	[0.0]])
        velRef.setState([[0.0],	[0.0],	[0.0]])
    else:
        posRef.setState([[-4020338.690396649],	[7490566.741852513],	[5248299.211589362]])
        velRef.setState([[-5199.77710904224],	[-3436.681645356935],	[1041.576797498721]])
    sigmaRef.setState([[0.0], [0.0], [0.0]])
    omegaRef.setState([[0.08], [0.01], [0.0]])

    scObject.hub.mHub = 750.0
    scObject.hub.r_BcB_B = [[-0.0002], [0.0001], [0.1]]
    scObject.hub.IHubPntBc_B = [[900.0, 0.0, 0.0], [0.0, 800.0, 0.0], [0.0, 0.0, 600.0]]

    stopTime = duration
    unitTestSim.ConfigureStopTime(macros.sec2nano(stopTime))
    unitTestSim.ExecuteSimulation()

    orbAngMom_N = unitTestSim.GetLogVariableData(scObject.ModelTag + ".totOrbAngMomPntN_N")
    rotAngMom_N = unitTestSim.GetLogVariableData(scObject.ModelTag + ".totRotAngMomPntC_N")
    rotEnergy = unitTestSim.GetLogVariableData(scObject.ModelTag + ".totRotEnergy")
    orbKinEnergy = unitTestSim.GetLogVariableData(scObject.ModelTag + ".totOrbKinEnergy")

    wheelSpeeds = unitTestSim.pullMessageLogData(rwStateEffector.OutputDataString + "." + "wheelSpeeds",range(3))
    gimbalAngles = unitTestSim.pullMessageLogData(rwStateEffector.OutputDataString + "." + "gimbalAngles",range(3))
    gimbalRates = unitTestSim.pullMessageLogData(rwStateEffector.OutputDataString + "." + "gimbalRates",range(3))
    sigmaData = unitTestSim.pullMessageLogData(scObject.scStateOutMsgName+'.sigma_BN',range(3))
    omegaData = unitTestSim.pullMessageLogData(scObject.scStateOutMsgName+'.omega_BN_B',range(3))

    dataPos = posRef.getState()
    dataSigma = sigmaRef.getState()
    dataPos = [[stopTime, dataPos[0][0], dataPos[1][0], dataPos[2][0]]]
    dataSigma = [[stopTime, dataSigma[0][0], dataSigma[1][0], dataSigma[2][0]]]


    if testCase == 'BalancedWheels':
        truePos = [
            [-4025537.663530937, 7487128.5629007, 5249339.739717924]
        ]

        trueSigma = [
            [0.01857774589897812, 0.0021098814589502733, -0.001194065770317139]
        ]

    elif testCase == 'JitterSimple':
        truePos = [
            [-4025537.6587910117, 7487128.563118205, 5249339.749664459]
        ]

        trueSigma = [
            [0.018774477186285467, 0.0018376842577357564, -0.00023633044221463834]
        ]

    elif testCase == 'JitterFullyCoupled':
        truePos = [
            [0.0009998791949221823, 0.003085053664784031, 0.008772780702050606]
        ]

        trueSigma = [
            [0.028007665602731092, 0.007826213565073164, -0.0011225237462873503]
        ]



    initialOrbAngMom_N = [
                [orbAngMom_N[0,1], orbAngMom_N[0,2], orbAngMom_N[0,3]]
                ]

    finalOrbAngMom = [
                [orbAngMom_N[-1,0], orbAngMom_N[-1,1], orbAngMom_N[-1,2], orbAngMom_N[-1,3]]
                 ]

    initialRotAngMom_N = [
                [rotAngMom_N[0,1], rotAngMom_N[0,2], rotAngMom_N[0,3]]
                ]

    finalRotAngMom = [
                [rotAngMom_N[-1,0], rotAngMom_N[-1,1], rotAngMom_N[-1,2], rotAngMom_N[-1,3]]
                 ]


    plt.figure()
    plt.plot(orbAngMom_N[:,0]*1e-9, orbAngMom_N[:,1] - orbAngMom_N[0,1], orbAngMom_N[:,0]*1e-9, orbAngMom_N[:,2] - orbAngMom_N[0,2], orbAngMom_N[:,0]*1e-9, orbAngMom_N[:,3] - orbAngMom_N[0,3])
    plt.title("Change in Orbital Angular Momentum")

    plt.figure()
    plt.plot(rotAngMom_N[:,0]*1e-9, rotAngMom_N[:,1] - rotAngMom_N[0,1], rotAngMom_N[:,0]*1e-9, rotAngMom_N[:,2] - rotAngMom_N[0,2], rotAngMom_N[:,0]*1e-9, rotAngMom_N[:,3] - rotAngMom_N[0,3])
    plt.title("Change in Rotational Angular Momentum")

    # plt.figure()
    # plt.plot(orbKinEnergy[:,0]*1e-9, orbKinEnergy[:,1] - orbKinEnergy[0,1])
    # plt.title("Change in Orbital Kinetic Energy")

    # plt.figure()
    # plt.plot(rotEnergy[:,0]*1e-9, rotEnergy[:,1] - rotEnergy[0,1])
    # plt.title("Change in Rotational Energy")

    # plt.figure()
    # for i in range(1,N+1):
    #     plt.subplot(4,1,i)
    #     plt.plot(wheelSpeeds[:,0]*1.0E-9, wheelSpeeds[:,i] / (2.0 * math.pi) * 60, label='RWA' + str(i))
    #     plt.xlabel('Time (s)')
    #     plt.ylabel(r'RW' + str(i) + r' $\Omega$ (RPM)')

    # plt.figure()
    # for i in range(1,N+1):
    #     plt.subplot(4,1,i)
    #     plt.plot(gimbalAngles[:,0]*1.0E-9, gimbalAngles[:,i], label=str(i))
    #     plt.xlabel('Time (s)')
    #     plt.ylabel(r'$\gamma_'+str(i)+'$ (rad)')

    # plt.figure()
    # for i in range(1,N+1):
    #     plt.subplot(4,1,i)
    #     plt.plot(gimbalRates[:,0]*1.0E-9, gimbalRates[:,i] * 180/np.pi, label=str(i))
    #     plt.xlabel('Time (s)')
    #     plt.ylabel(r'$\dot{\gamma}_'+str(i)+'$ (d/s)')

    # plt.figure()
    # for i in range(1,N+1):
    #     plt.subplot(4,1,i)
    #     plt.plot(sigmaData[:,0]*1.0E-9, sigmaData[:,i], label='MRP' + str(i))
    #     plt.xlabel('Time (s)')
    #     plt.ylabel(r'MRP b' + str(i))

    # plt.figure()
    # for i in range(1,N+1):
    #     plt.subplot(4,1,i)
    #     plt.plot(omegaData[:,0]*1.0E-9, omegaData[:,i] * 180/math.pi, label='omega' + str(i))
    #     plt.xlabel('Time (s)')
    #     plt.ylabel(r'b' + str(i) + r' $\omega$ (d/s)')

    if testCase != 'BalancedWheels':
        istart = int(.01/dt)
        sigmaDataCut = sigmaData[istart:,:]
        thetaData = np.empty([len(sigmaDataCut[:,0]),2])
        thetaData[:,0] = sigmaDataCut[:,0]
        for i in range(0,len(thetaData[:,0])):
            thetaData[i,1] = 4*np.arctan(np.linalg.norm(sigmaDataCut[i,1:]))

        if testCase == 'JitterSimple':
            fitOrd = 11
        else:
            fitOrd = 9

        thetaFit = np.empty([len(sigmaDataCut[:,0]),2])
        thetaFit[:,0] = thetaData[:,0]
        p = np.polyfit(thetaData[:,0]*1e-9,thetaData[:,1],fitOrd)
        thetaFit[:,1] = np.polyval(p,thetaFit[:,0]*1e-9)

        plt.figure()
        plt.plot(thetaData[:,0]*1e-9, thetaData[:,1]-thetaFit[:,1])
        plt.title("Principle Angle Fit")
        plt.xlabel('Time (s)')
        plt.ylabel(r'$\theta$ (deg)')

        [frq,Y] = computeFFT(thetaData[:,1]-thetaFit[:,1],dt)
        peakIdxs = findPeaks(Y,N)
        wheelSpeeds_data = np.array(frq[peakIdxs])*60.
        wheelSpeeds_true = np.sort(abs(np.array([VSCMG.Omega/rpm2rad for VSCMG in VSCMGs])))

        fig, ax = plt.subplots(2,1)
        ax[0].plot(thetaFit[:,0]*1e-9,thetaData[:,1]-thetaFit[:,1])
        ax[0].set_xlabel('Time')
        ax[0].set_ylabel('Amplitude')
        ax[1].plot(frq,abs(Y),'r')
        ax[1].set_xlabel('Freq (Hz)')
        ax[1].set_ylabel('Magnitude')
        ax[1].plot(frq[peakIdxs],Y[peakIdxs],'bo')
        plt.xlim((0,VSCMGs[0].Omega_max/rpm2rad/60.))

        # plt.figure()
        # plt.plot(thetaData[:,0]*1e-9, thetaData[:,1])
        # plt.title("Principle Angle")
        # plt.xlabel('Time (s)')
        # plt.ylabel(r'$\theta$ (deg)')

    if show_plots == True:
        plt.show()


    accuracy = 1e-8
    for i in range(0,len(truePos)):
        # check a vector values
        print 'Position'
        print truePos
        print dataPos
        if not unitTestSupport.isArrayEqualRelative(dataPos[i],truePos[i],3,accuracy):
            testFailCount += 1
            testMessages.append("FAILED: Reaction Wheel Integrated Test failed pos unit test")

    for i in range(0,len(trueSigma)):
        # check a vector values
        print 'Attitude'
        print trueSigma
        print dataSigma
        if not unitTestSupport.isArrayEqualRelative(dataSigma[i],trueSigma[i],3,accuracy):
            testFailCount += 1
            testMessages.append("FAILED: Reaction Wheel Integrated Test failed attitude unit test")

    if testCase == 'BalancedWheels' or testCase == 'JitterFullyCoupled':
        for i in range(0,len(initialOrbAngMom_N)):
            # check a vector values
            print 'Orbital Angular Momentum'
            print initialOrbAngMom_N
            print finalOrbAngMom
            if not unitTestSupport.isArrayEqualRelative(finalOrbAngMom[i],initialOrbAngMom_N[i],3,accuracy):
                testFailCount += 1
                testMessages.append("FAILED: Reaction Wheel Integrated Test failed orbital angular momentum unit test")

        for i in range(0,len(initialRotAngMom_N)):
            # check a vector values
            print 'Rotational Angular Momentum'
            print initialRotAngMom_N
            print finalRotAngMom
            if not unitTestSupport.isArrayEqualRelative(finalRotAngMom[i],initialRotAngMom_N[i],3,accuracy):
                testFailCount += 1
                testMessages.append("FAILED: Reaction Wheel Integrated Test failed rotational angular momentum unit test")

    if testCase == 'JitterSimple' or testCase == 'JitterFullyCoupled':
        print wheelSpeeds_true
        print wheelSpeeds_data
        for i in range(N):
            # check a vector values
            if not abs(wheelSpeeds_data[i]-wheelSpeeds_true[i])/wheelSpeeds_true[i] < .09:
                testFailCount += 1
                testMessages.append("FAILED: Reaction Wheel Integrated Test failed jitter unit test")

    if testFailCount == 0:
        print "PASSED: " + " Reaction Wheel Integrated Sim Test"

    assert testFailCount < 1, testMessages

    # return fail count and join into a single string all messages in the list
    # testMessage
    return [testFailCount, ''.join(testMessages)]

if __name__ == "__main__":
    VSCMGIntegratedTest(True,False,'JitterFullyCoupled')