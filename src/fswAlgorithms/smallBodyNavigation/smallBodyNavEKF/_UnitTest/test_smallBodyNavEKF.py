# 
#  ISC License
# 
#  Copyright (c) 2021, Autonomous Vehicle Systems Lab, University of Colorado Boulder
# 
#  Permission to use, copy, modify, and/or distribute this software for any
#  purpose with or without fee is hereby granted, provided that the above
#  copyright notice and this permission notice appear in all copies.
# 
#  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
#  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
#  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
#  ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
#  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
#  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
#  OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
# 
# 

import numpy as np
from Basilisk.architecture import messaging
from Basilisk.fswAlgorithms import smallBodyNavEKF
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros, orbitalMotion
from Basilisk.utilities import unitTestSupport
from matplotlib import pyplot as plt


def test_smallBodyNavEKF(show_plots):
    r"""
    **Validation Test Description**

    This unit test checks that the filter converges to a constant state estimate under the presence of static measurements.
    No reaction wheels or thrusters are used, but a message for each is created and connected to avoid warnings.

    **Test Parameters**

    Args:
        :param show_plots: flag if plots should be shown.
    """
    [testResults, testMessage] = smallBodyNavEKFTestFunction(show_plots)
    assert testResults < 1, testMessage


def smallBodyNavEKFTestFunction(show_plots):
    """Test method"""
    testFailCount = 0
    testMessages = []
    unitTaskName = "unitTask"
    unitProcessName = "TestProcess"

    unitTestSim = SimulationBaseClass.SimBaseClass()
    testProcessRate = macros.sec2nano(0.5)
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # setup module to be tested
    module = smallBodyNavEKF.SmallBodyNavEKF()
    module.ModelTag = "smallBodyNavEKFTag"
    unitTestSim.AddModelToTask(unitTaskName, module)

    # Set the filter parameters (sc area, mass, gravitational constants, etc.)
    module.A_sc = 1.  # Surface area of the spacecraft, m^2
    module.M_sc = 100.  # Mass of the spacecraft, kg
    I = [82.12, 0.0, 0.0, 0.0, 98.40, 0.0, 0.0, 0.0, 121.0]
    module.IHubPntC_B = unitTestSupport.np2EigenMatrix3d(I)  # sc inertia
    module.IWheelPntC_B = ((50./6000.0*macros.RPM)*np.identity(3)).tolist()  # wheel inertia
    module.mu_ast = 5.2  # Gravitational constant of the asteroid
    module.Q = (0.1*np.identity(18)).tolist()  # Process Noise
    module.R = (0.1*np.identity(18)).tolist()  # Measurement Noise

    bennu_radius = 1.355887692*orbitalMotion.AU*1000.0  # meters
    bennu_velocity = np.sqrt(orbitalMotion.MU_SUN*(1000.**3)/bennu_radius) # m/s, assumes circular orbit

    x_0 = [2010., 1510., 1010., 0., 2., 0., 0.14, 0., 0., 0., 0., 0., 0.13, 0., 0., 0., 0., 0.]
    module.x_hat_k = unitTestSupport.np2EigenVectorXd(x_0)
    module.P_k = (1000.*np.identity(18)).tolist()

    # Configure blank module input messages
    navTransInMsgData = messaging.NavTransMsgPayload()
    navTransInMsgData.r_BN_N = [bennu_radius + 1000., 1000., 1000.]
    navTransInMsgData.v_BN_N = [0., bennu_velocity+1., 0.]
    navTransInMsg = messaging.NavTransMsg().write(navTransInMsgData)

    navAttInMsgData = messaging.NavAttMsgPayload()
    navAttInMsgData.sigma_BN = [0.1, 0.0, 0.0]
    navAttInMsgData.omega_BN_B = [0.0, 0.0, 0.0]
    navAttInMsg = messaging.NavAttMsg().write(navAttInMsgData)

    asteroidEphemerisInMsgData = messaging.EphemerisMsgPayload()
    asteroidEphemerisInMsgData.r_BdyZero_N = [bennu_radius, 0., 0.]
    asteroidEphemerisInMsgData.v_BdyZero_N = [0., bennu_velocity, 0.]
    asteroidEphemerisInMsgData.sigma_BN = [0.1, 0.0, 0.0]
    asteroidEphemerisInMsgData.omega_BN_B = [0.0, 0.0, 0.0]
    asteroidEphemerisInMsg = messaging.EphemerisMsg().write(asteroidEphemerisInMsgData)

    sunEphemerisInMsgData = messaging.EphemerisMsgPayload()
    sunEphemerisInMsg = messaging.EphemerisMsg().write(sunEphemerisInMsgData)

    rwConfigLogInMsgData = messaging.RWConfigLogMsgPayload()
    rwConfigLogInMsgData.Js = 50./6000.0*macros.RPM
    rwConfigLogInMsg = messaging.RWConfigLogMsg().write(rwConfigLogInMsgData)

    THROutputInMsgData = messaging.THROutputMsgPayload()
    THROutputInMsg = messaging.THROutputMsg().write(THROutputInMsgData)

    # subscribe input messages to module
    module.navTransInMsg.subscribeTo(navTransInMsg)
    module.navAttInMsg.subscribeTo(navAttInMsg)
    module.asteroidEphemerisInMsg.subscribeTo(asteroidEphemerisInMsg)
    module.sunEphemerisInMsg.subscribeTo(sunEphemerisInMsg)
    module.addRWToFilter(rwConfigLogInMsg)
    module.addThrusterToFilter(THROutputInMsg)

    # setup output message recorder objects
    navTransOutMsgRec = module.navTransOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, navTransOutMsgRec)
    navAttOutMsgRec = module.navAttOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, navAttOutMsgRec)
    smallBodyNavOutMsgRec = module.smallBodyNavOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, smallBodyNavOutMsgRec)
    smallBodyNavOutMsgRecC = module.smallBodyNavOutMsgC.recorder()
    unitTestSim.AddModelToTask(unitTaskName, smallBodyNavOutMsgRecC)
    asteroidEphemerisOutMsgRec = module.asteroidEphemerisOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, asteroidEphemerisOutMsgRec)


    unitTestSim.InitializeSimulation()
    unitTestSim.ConfigureStopTime(macros.sec2nano(10.5))
    unitTestSim.ExecuteSimulation()

    x_hat = smallBodyNavOutMsgRec.state
    x_hat_c_wrapped = smallBodyNavOutMsgRecC.state
    covar = smallBodyNavOutMsgRec.covar
    true_x_hat = np.array([[1000.1, 1000., 1000., 0., 1.0, 0., 0.1, 0., 0., 0., 0., 0., 0.1, 0., 0., 0., 0., 0.]])

    # print(x_hat[0,:])
    # print(x_hat_c_wrapped)

    testFailCount, testMessages = unitTestSupport.compareArray(
        true_x_hat, np.array([x_hat[0,:]]), 0.1, "x_hat",
        testFailCount, testMessages)

    testFailCount, testMessages = unitTestSupport.compareArray(
        true_x_hat, np.array([x_hat_c_wrapped[0,:]]), 0.1, "x_hat_c_wrapped",
        testFailCount, testMessages)

    plt.figure(1)
    plt.clf()
    plt.figure(1, figsize=(7, 5), dpi=80, facecolor='w', edgecolor='k')
    plt.ticklabel_format(useOffset=False)
    plt.plot(navTransOutMsgRec.times() * 1.0E-9, x_hat[:,0], label='x-pos')
    plt.plot(navTransOutMsgRec.times() * 1.0E-9, x_hat[:,1], label='y-pos')
    plt.plot(navTransOutMsgRec.times() * 1.0E-9, x_hat[:,2], label='z-pos')
    plt.legend(loc='upper left')
    plt.xlabel('Time (s)')
    plt.ylabel('r_BO_O (m)')
    plt.title('Estimated Relative Spacecraft Position')

    plt.figure(2)
    plt.clf()
    plt.figure(2, figsize=(7, 5), dpi=80, facecolor='w', edgecolor='k')
    plt.plot(navTransOutMsgRec.times() * 1.0E-9, x_hat[:,3], label='x-vel')
    plt.plot(navTransOutMsgRec.times() * 1.0E-9, x_hat[:,4], label='y-vel')
    plt.plot(navTransOutMsgRec.times() * 1.0E-9, x_hat[:,5], label='z-vel')
    plt.legend(loc='upper left')
    plt.xlabel('Time (s)')
    plt.ylabel('v_BO_O (m/s)')
    plt.title('Estimated Spacecraft Velocity')

    plt.figure(3)
    plt.clf()
    plt.figure(3, figsize=(7, 5), dpi=80, facecolor='w', edgecolor='k')
    plt.plot(navTransOutMsgRec.times() * 1.0E-9, x_hat[:,12], label='s1')
    plt.plot(navTransOutMsgRec.times() * 1.0E-9, x_hat[:,13], label='s2')
    plt.plot(navTransOutMsgRec.times() * 1.0E-9, x_hat[:,14], label='s3')
    plt.legend(loc='upper left')
    plt.xlabel('Time (s)')
    plt.ylabel('sigma_BN (rad)')
    plt.title('Estimated Spacecraft Attitude')

    plt.figure(4)
    plt.clf()
    plt.figure(4, figsize=(7, 5), dpi=80, facecolor='w', edgecolor='k')
    plt.plot(navTransOutMsgRec.times() * 1.0E-9, x_hat[:,15], label='omega1')
    plt.plot(navTransOutMsgRec.times() * 1.0E-9, x_hat[:,16], label='omega2')
    plt.plot(navTransOutMsgRec.times() * 1.0E-9, x_hat[:,17], label='omega3')
    plt.legend(loc='upper left')
    plt.xlabel('Time (s)')
    plt.ylabel('omega_BN_B (rad/s)')
    plt.title('Estimated Spacecraft Rate')

    plt.figure(5)
    plt.clf()
    plt.figure(5, figsize=(7, 5), dpi=80, facecolor='w', edgecolor='k')
    plt.plot(navTransOutMsgRec.times() * 1.0E-9, x_hat[:,6], label='s1')
    plt.plot(navTransOutMsgRec.times() * 1.0E-9, x_hat[:,7], label='s2')
    plt.plot(navTransOutMsgRec.times() * 1.0E-9, x_hat[:,8], label='s3')
    plt.legend(loc='upper left')
    plt.xlabel('Time (s)')
    plt.ylabel('sigma_AN (rad)')
    plt.title('Estimated Asteroid Attitude')

    plt.figure(6)
    plt.clf()
    plt.figure(6, figsize=(7, 5), dpi=80, facecolor='w', edgecolor='k')
    plt.plot(navTransOutMsgRec.times() * 1.0E-9, x_hat[:,9], label='omega1')
    plt.plot(navTransOutMsgRec.times() * 1.0E-9, x_hat[:,10], label='omega2')
    plt.plot(navTransOutMsgRec.times() * 1.0E-9, x_hat[:,11], label='omega3')
    plt.legend(loc='upper left')
    plt.xlabel('Time (s)')
    plt.ylabel('omega_AN_A (rad/s)')
    plt.title('Estimated Asteroid Rate')

    if show_plots:
        plt.show()



    if testFailCount == 0:
        print("PASSED: " + module.ModelTag)
    else:
        print(testMessages)

    return [testFailCount, "".join(testMessages)]


if __name__ == "__main__":
    test_smallBodyNavEKF(True)


