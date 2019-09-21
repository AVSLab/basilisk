#
#   Unit Test Script
#   Module Name:        pixelLineConverter.py
#   Creation Date:      May 16, 2019
#   Author:             Thibaud Teil
#

from Basilisk.utilities import SimulationBaseClass, unitTestSupport, macros
from Basilisk.fswAlgorithms.horizonOpNav import horizonOpNav
from Basilisk.utilities import RigidBodyKinematics as rbk

import os, inspect
import numpy as np
filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

def back_substitution(A, b):
    n = b.size
    x = np.zeros_like(b)

    if A[-1, -1] == 0:
        raise ValueError

    x[-1] = b[-1]/ A[-1, -1]
    for i in range(n-2, -1, -1):
        sum=0
        for j in range(i, n):
            sum += A[i, j]*x[j]
        x[i] = (b[i] - sum)/A[i,i]
    return x


def test_horizonOpNav():
    [testResults, testMessage] = horizonOpNav_methods()
    assert testResults < 1, testMessage
    [testResults, testMessage] = horizonOpNav_update()
    assert testResults < 1, testMessage

def horizonOpNav_methods():
    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty array to store test log messages
    ###################################################################################
    ## Testing QR decomp
    ###################################################################################
    Hinput = np.array([[1,2,3],[1,20,3],[3,0,1],[2,1,0],[20,-1, -5],[0,10,-5]])
    numStates = np.shape(Hinput)[0]
    # Fill in the variables for the test
    Qin = horizonOpNav.new_doubleArray(3 * numStates)
    Rin = horizonOpNav.new_doubleArray(3 * 3)
    Hin = horizonOpNav.new_doubleArray(numStates * 3)
    for j in range(numStates*3):
        horizonOpNav.doubleArray_setitem(Qin, j, 0)
    for j in range(3 * 3):
        horizonOpNav.doubleArray_setitem(Rin, j, 0)
    for j in range(numStates * 3):
        horizonOpNav.doubleArray_setitem(Hin, j, Hinput.flatten().tolist()[j])
    horizonOpNav.QRDecomp(Hin, numStates, Qin, Rin)

    Qout = []
    for j in range(3 * numStates):
        Qout.append(horizonOpNav.doubleArray_getitem(Qin, j))
    Rout = []
    for j in range(3 * 3):
        Rout.append(horizonOpNav.doubleArray_getitem(Rin, j))

    q,r = np.linalg.qr(Hinput)

    Rpy = np.zeros([3,3])
    Qpy = np.zeros([numStates, 3])
    for i in range(0,3):
        Qpy[:,i] = Hinput[:,i]
        for j in range(i):
            Rpy[j,i] = np.dot(Qpy[:,j], Hinput[:,i])
            Qpy[:,i]= Qpy[:,i] - Rpy[j,i]*Qpy[:,j]
        Rpy[i,i] = np.linalg.norm(Qpy[:,i])
        Qpy[:,i] = 1 / Rpy[i,i] *  Qpy[:,i]


    Qtest = np.array(Qout).reshape([numStates,3])
    Rtest = np.array(Rout).reshape(3, 3)
    errorNorm1 = np.linalg.norm(Qpy - Qtest)
    errorNorm2 = np.linalg.norm(Rpy - Rtest)
    if (errorNorm1 > 1.0E-10):
        print(errorNorm1, "QR decomp")
        testFailCount += 1
        testMessages.append("QR decomp Failure in Q" + "\n")
    if (errorNorm2 > 1.0E-10):
        print(errorNorm2, "QR decomp")
        testFailCount += 1
        testMessages.append("QR decomp Failure in R" + "\n")
    errorNorm1 = np.linalg.norm(q + Qtest)
    errorNorm2 = np.linalg.norm(r[:3,:3] + Rtest)
    if (errorNorm1 > 1.0E-10):
        print(errorNorm1, "QR decomp")
        testFailCount += 1
        testMessages.append("QR decomp Failure in Q" + "\n")
    if (errorNorm2 > 1.0E-10):
        print(errorNorm2, "QR decomp")
        testFailCount += 1
        testMessages.append("QR decomp Failure in R" + "\n")

    ###################################################################################
    ## Testing Back Sub
    ###################################################################################
    V = np.ones(3)
    nIn = horizonOpNav.new_doubleArray(3)
    VIn = horizonOpNav.new_doubleArray(3)
    RIn = horizonOpNav.new_doubleArray(numStates*3)
    for i in range(3):
        horizonOpNav.doubleArray_setitem(nIn, i, 0.0)
    for i in range(3*3):
        horizonOpNav.doubleArray_setitem(RIn, i, r.flatten().tolist()[i])
    for i in range(3):
        horizonOpNav.doubleArray_setitem(VIn, i, V.flatten().tolist()[i])

    horizonOpNav.BackSub(RIn, VIn, 3, nIn)
    BackSubOut = []
    for i in range(3):
        BackSubOut.append(horizonOpNav.doubleArray_getitem(nIn, i))

    exp = back_substitution(r[:3,:3], V)

    BackSubOut = np.array(BackSubOut)
    errorNorm = np.linalg.norm(exp - BackSubOut)
    if(errorNorm > 1.0E-10):
        print(errorNorm, "BackSub")
        testFailCount += 1
        testMessages.append("BackSub Failure " + "\n")

###################################################################################
## Testing dynamics matrix computation
###################################################################################
def horizonOpNav_update():
    # Create a sim module as an empty container
    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty array to store test log messages
    unitTaskName = "unitTask"  # arbitrary name (don't change)
    unitProcessName = "TestProcess"  # arbitrary name (don't change)
    unitTestSim = SimulationBaseClass.SimBaseClass()

    # This is needed if multiple unit test scripts are run
    # This create a fresh and consistent simulation environment for each test run
    unitTestSim.TotalSim.terminateSimulation()

    # Create test thread
    testProcessRate = macros.sec2nano(0.5)  # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))  # Add a new task to the process

    # Construct the ephemNavConverter module
    # Set the names for the input messages
    pixelLine = pixelLineConverter.PixelLineConvertData()  # Create a config struct
    pixelLine.circlesInMsgName = "circles_name"
    pixelLine.cameraConfigMsgName = "camera_config_name"
    pixelLine.attInMsgName = "nav_att_name"
    # ephemNavConfig.outputState = simFswInterfaceMessages.NavTransIntMsg()

    # This calls the algContain to setup the selfInit, crossInit, update, and reset
    pixelLineWrap = unitTestSim.setModelDataWrap(pixelLine)
    pixelLineWrap.ModelTag = "pixelLineConverter"

    # Add the module to the task
    unitTestSim.AddModelToTask(unitTaskName, pixelLineWrap, pixelLine)

    # These are example points for fitting used from an image processing algorithm
    inputPoints = [226., 113., 227., 113., 223., 114., 224., 114., 225., 114., 219.,
       115., 220., 115., 221., 115., 222., 115., 215., 116., 216., 116.,
       217., 116., 218., 116., 212., 117., 213., 117., 214., 117., 209.,
       118., 210., 118., 211., 118., 205., 119., 206., 119., 207., 119.,
       208., 119., 204., 120., 205., 120., 201., 121., 202., 121., 203.,
       121., 199., 122., 200., 122., 197., 123., 198., 123., 195., 124.,
       196., 124., 193., 125., 194., 125., 191., 126., 192., 126., 189.,
       127., 190., 127., 187., 128., 188., 128., 185., 129., 186., 129.,
       183., 130., 184., 130., 181., 131., 182., 131., 180., 132., 181.,
       132., 178., 133., 179., 133., 177., 134., 178., 134., 175., 135.,
       176., 135., 174., 136., 175., 136., 172., 137., 173., 137., 171.,
       138., 172., 138., 170., 139., 171., 139., 168., 140., 169., 140.,
       167., 141., 168., 141., 166., 142., 167., 142., 164., 143., 165.,
       143., 163., 144., 164., 144., 162., 145., 163., 145., 161., 146.,
       162., 146., 160., 147., 161., 147., 159., 148., 160., 148., 158.,
       149., 159., 149., 156., 150., 157., 150., 155., 151., 156., 151.,
       154., 152., 155., 152., 153., 153., 154., 153., 153., 154., 152.,
       155., 151., 156., 152., 156., 150., 157., 151., 157., 149., 158.,
       150., 158., 148., 159., 149., 159., 147., 160., 148., 160., 146.,
       161., 147., 161., 145., 162., 146., 162., 145., 163., 144., 164.,
       143., 165., 144., 165., 142., 166., 143., 166., 142., 167., 141.,
       168., 140., 169., 141., 169., 139., 170., 140., 170., 139., 171.,
       138., 172., 137., 173., 138., 173., 137., 174., 136., 175., 135.,
       176., 136., 176., 135., 177., 134., 178., 133., 179., 134., 179.,
       133., 180., 132., 181., 132., 182., 131., 183., 131., 184., 130.,
       185., 129., 186., 130., 186., 129., 187., 128., 188., 128., 189.,
       127., 190., 127., 191., 126., 192., 126., 193., 125., 194., 125.,
       195., 125., 196., 124., 197., 124., 198., 123., 199., 123., 200.,
       122., 201., 122., 202., 122., 203., 121., 204., 120., 205., 121.,
       205., 120., 206., 120., 207., 120., 208., 119., 209., 119., 210.,
       119., 211., 118., 212., 118., 213., 118., 214., 117., 215., 117.,
       216., 117., 217., 117., 218., 116., 219., 116., 220., 116., 221.,
       116., 222., 115., 223., 115., 224., 115., 225., 115., 226., 114.,
       227., 114., 228., 114., 229., 114., 230., 114., 231., 114., 232.,
       113., 233., 113., 234., 113., 235., 113., 236., 113., 237., 113.,
       238., 113., 239., 112., 240., 112., 241., 112., 242., 112., 243.,
       112., 244., 112., 245., 112., 246., 112., 247., 112., 248., 112.,
       249., 112., 250., 112., 251., 112., 252., 112., 253., 112., 254.,
       111., 255., 111., 256., 112., 257., 112., 258., 112., 259., 112.,
       260., 112., 261., 112., 262., 112., 263., 112., 264., 112., 265.,
       112., 266., 112., 267., 112., 268., 112., 269., 112., 270., 112.,
       271., 113., 272., 113., 273., 113., 274., 113., 275., 113., 276.,
       113., 277., 113., 278., 114., 279., 114., 280., 114., 281., 114.,
       282., 114., 283., 114., 284., 115., 285., 115., 286., 115., 287.,
       115., 288., 116., 289., 116., 290., 116., 291., 116., 292., 117.,
       293., 117., 294., 117., 295., 117., 296., 118., 297., 118., 298.,
       118., 299., 119., 300., 119., 301., 119., 302., 120., 303., 120.,
       304., 120., 305., 121., 306., 121., 307., 122., 308., 122., 309.,
       122., 310., 123., 311., 123., 312., 124., 313., 124., 314., 125.,
       315., 125., 316., 125., 317., 126., 318., 126., 319., 127., 320.,
       127., 321., 128., 322., 128., 323., 129., 324., 129., 325., 130.,
       325., 130., 326., 131., 327., 131., 328., 132., 329., 132., 330.,
       133., 331., 133., 332., 134., 332., 134., 333., 135., 334., 135.,
       335., 136., 335., 136., 336., 137., 337., 137., 338., 138., 338.,
       138., 339., 139., 340., 139., 341., 140., 341., 140., 342., 141.,
       342., 141., 343., 142., 344., 142., 345., 143., 345., 143., 346.,
       144., 346., 144., 347., 145., 348., 145., 349., 146., 349., 146.,
       350., 147., 350., 147., 351., 148., 351., 148., 352., 149., 352.,
       149., 353., 150., 353., 150., 354., 151., 354., 151., 355., 152.,
       356., 152., 357., 153., 357., 153., 358., 154., 358., 154., 359.,
       155., 359., 155., 360., 156., 360., 156., 361., 157., 361., 158.,
       362., 159., 362., 159., 363., 160., 363., 160., 364., 161., 364.,
       161., 365., 162., 365., 162., 366., 163., 366., 163., 367., 164.,
       367., 164., 368., 165., 368., 166., 369., 167., 369., 167., 370.,
       168., 370., 168., 371., 169., 371., 169., 372., 170., 372., 171.,
       373., 172., 373., 172., 374., 173., 374., 174., 375., 175., 375.,
       175., 376., 176., 376., 177., 377., 178., 377., 178., 378., 179.,
       378., 180., 379., 181., 379., 181., 380., 182., 380., 183., 381.,
       184., 381., 185., 382., 186., 382., 187., 383., 188., 383., 188.,
       384., 189., 384., 190., 385., 191., 385., 192., 386.]

    # Create the input messages.
    inputCamera = pixelLineConverter.CameraConfigMsg()
    inputCircles = pixelLineConverter.CirclesOpNavMsg()
    inputAtt = pixelLineConverter.NavAttIntMsg()

    # Set camera
    inputCamera.focalLength = 1.
    inputCamera.sensorSize = [10, 10] # In mm
    inputCamera.resolution = [512, 512]
    inputCamera.sigma_CB = [1.,0.3,0.1]
    unitTestSupport.setMessage(unitTestSim.TotalSim, unitProcessName, pixelLine.cameraConfigMsgName, inputCamera)

    # Set circles
    inputCircles.circlesCenters = [152, 251]
    inputCircles.circlesRadii = [75]
    inputCircles.uncertainty = [0.5, 0., 0., 0., 0.5, 0., 0., 0., 1.]
    inputCircles.timeTag = 12345
    unitTestSupport.setMessage(unitTestSim.TotalSim, unitProcessName, pixelLine.circlesInMsgName, inputCircles)

    # Set attitude
    inputAtt.sigma_BN = [0.6, 1., 0.1]
    unitTestSupport.setMessage(unitTestSim.TotalSim, unitProcessName, pixelLine.attInMsgName, inputAtt)

    # Set module for Mars
    pixelLine.planetTarget = 2
    pixelLine.opNavOutMsgName = "output_nav_msg"
    unitTestSim.TotalSim.logThisMessage(pixelLine.opNavOutMsgName)

    # Initialize the simulation
    unitTestSim.InitializeSimulation()
    # The result isn't going to change with more time. The module will continue to produce the same result
    unitTestSim.ConfigureStopTime(testProcessRate)  # seconds to stop simulation
    unitTestSim.ExecuteSimulation()

    # Truth Vlaues
    planet = {}
    camera = {}
    planet["name"] = "Mars"
    planet["diameter"] = 3396.19 * 2  # km

    camera["focal"] = inputCamera.focalLength  # m
    camera["pixelSizeX"] = inputCamera.sensorSize[0]/inputCamera.resolution[0] * 1E-3  # m
    camera["pixelSizeY"] = inputCamera.sensorSize[1]/inputCamera.resolution[1] * 1E-3  # m

    state = [inputCircles.circlesCenters[0], inputCircles.circlesCenters[1], inputCircles.circlesRadii[0]]

    r_Cexp = mapState(state, planet, camera)
    covar_Cexp = mapCovar(inputCircles.uncertainty, state[2], planet, camera)

    dcm_CB = rbk.MRP2C(inputCamera.sigma_CB)
    dcm_BN = rbk.MRP2C(inputAtt.sigma_BN)

    dcm_NC = np.dot(dcm_CB, dcm_BN).T

    r_Nexp = np.dot(dcm_NC, r_Cexp)
    covar_Nexp = np.dot(dcm_NC, np.dot(covar_Cexp, dcm_NC.T)).flatten()
    timTagExp = inputCircles.timeTag

    posErr = 1e-10
    covarErr = 1e-10
    unitTestSupport.writeTeXSnippet("toleranceValuePos", str(posErr), path)
    unitTestSupport.writeTeXSnippet("toleranceValueVel", str(covarErr), path)

    outputR = unitTestSim.pullMessageLogData(pixelLine.opNavOutMsgName + '.r_BN_N',  list(range(3)))
    outputCovar = unitTestSim.pullMessageLogData(pixelLine.opNavOutMsgName + '.covar_N',  list(range(9)))
    outputTime = unitTestSim.pullMessageLogData(pixelLine.opNavOutMsgName + '.timeTag')
    #
    #
    for i in range(len(outputR[-1, 1:])):
        if np.abs(r_Nexp[i] - outputR[-1, i+1]) > 1E-10 and np.isnan(outputR.any()):
            testFailCount += 1
            testMessages.append("FAILED: Position Check in pixelLine")

    for i in range(len(outputCovar[-1, 1:])):
        if np.abs((covar_Nexp[i] - outputCovar[-1, i+1])) > 1E-10 and np.isnan(outputTime.any()):
            testFailCount += 1
            testMessages.append("FAILED: Covar Check in pixelLine")

    if np.abs((timTagExp - outputTime[-1, 1])/timTagExp) > 1E-10 and np.isnan(outputTime.any()):
        testFailCount += 1
        testMessages.append("FAILED: Time Check in pixelLine")
    #
    #   print out success message if no error were found
    snippentName = "passFail"
    if testFailCount == 0:
        colorText = 'ForestGreen'
        print("PASSED: " + pixelLineWrap.ModelTag)
        passedText = r'\textcolor{' + colorText + '}{' + "PASSED" + '}'
    else:
        colorText = 'Red'
        print("Failed: " + pixelLineWrap.ModelTag)
        passedText = r'\textcolor{' + colorText + '}{' + "Failed" + '}'
    unitTestSupport.writeTeXSnippet(snippentName, passedText, path)


    return [testFailCount, ''.join(testMessages)]


if __name__ == '__main__':
    horizonOpNav_methods()
    # horizonOpNav_update()
