
def TestNrlmsiseAtmosphere():
    '''
    Verifies that the loaded model outputs are correctly transferred to the input struct and that the model output matches
    the test values included with NRLMSISE-00.h
    :param atmoModel:
    :return: testFailCount, testMessages
    '''

    testFailCount = 0
    testMessages = []

    #testValidation = np.loadtxt(open("test_outputs.txt","rb"), delimiter=",")

    # Create simulation variable names
    simTaskName = "simTask"
    simProcessName = "simProcess"

    #  Create a sim module as an empty container
    scSim = SimulationBaseClass.SimBaseClass()
    scSim.TotalSim.terminateSimulation()

    #  create the simulation process
    dynProcess = scSim.CreateNewProcess(simProcessName)

    # create the dynamics task and specify the integration update time
    simulationTimeStep = macros.sec2nano(10.)
    dynProcess.addTask(scSim.CreateNewTask(simTaskName, simulationTimeStep))

    msisModelVec = []
    scModelVec = []

    tmpAtmosphere = atmosphere.Atmosphere()

    sw_msg_names = [
        "ap_24_0", "ap_3_0", "ap_3_-3","ap_3_-6","ap_3_-9",
        "ap_3_-12","ap_3_-15","ap_3_-18","ap_3_-21","ap_3_-24",
        "ap_3_-27", "ap_3_-30","ap_3_-33","ap_3_-36","ap_3_-39",
        "ap_3_-42", "ap_3_-45", "ap_3_-48","ap_3_-51","ap_3_-54",
        "ap_3_-57","f107_1944_0","f107_24_-24"
    ]


    for swName in sw_msg_names:
        msgName = swName
        msgData = atmosphere.SwDataSimMsg()
        msgData.dataValue=0.
        unitTestSupport.setMessage(scSim.TotalSim, simProcessName, msgName, msgData)


    r_earth = 6378.137e3
    scObject = spacecraftPlus.SpacecraftPlus()
    scObject.ModelTag = "spacecraftBody"
    scObject.hub.r_CN_NInit = [[400.0e3+r_earth], [0], [0]]
    scObject.hub.v_CN_NInit = [[7000.0], [0], [0]]
    # clear prior gravitational body and SPICE setup definitions
    gravFactory = simIncludeGravBody.gravBodyFactory()

    planet = gravFactory.createEarth()

    planet.isCentralBody = True          # ensure this is the central gravitational body
    mu = planet.mu
    # attach gravity model to spaceCraftPlus
    scObject.gravField.gravBodies = spacecraftPlus.GravBodyVector(gravFactory.gravBodies.values())

    scSim.AddModelToTask(simTaskName, scObject)

    testValidation = np.loadtxt('unit_test_outputs.csv',delimiter=',')

    simulationTime = 2.

    #
    #   Setup data logging before the simulation is initialized
    #
    numDataPoints = 2
    samplingTime = simulationTime / (numDataPoints-1)


    scSim.InitializeSimulation()
    scSim.ConfigureStopTime(simulationTime)
    scSim.ExecuteSimulation()


    for ind in range(0,17):
        tmpAtmosphere = atmosphere.Atmosphere()
        tmpAtmosphere.setEnvType('nrlmsise00')
        tmpAtmosphere.addSpacecraftToModel(scObject.scStateOutMsgName)
        tmpAtmosphere.ModelTag = "msis_atmo_"+str(ind)
        msisModelVec.append(tmpAtmosphere)

        scSim.AddModelToTask(simTaskName, tmpAtmosphere)

        msisModelVec[-1].doy = 172
        msisModelVec[-1].year = 0
        msisModelVec[-1].sec = 29000
        msisModelVec[-1].alt = 400
        msisModelVec[-1].g_lat = 60
        msisModelVec[-1].g_long = -70
        msisModelVec[-1].lst = 16
        msisModelVec[-1].f107A = 150
        msisModelVec[-1].f107 = 150
        msisModelVec[-1].ap = 4
        for ap_ind in range(0,7):
            msisModelVec[-1].aph.a[ap_ind] = 0.0

    msisModelVec[1].doy = 81
    msisModelVec[2].sec = 75000
    msisModelVec[2].alt = 1000
    msisModelVec[3].alt = 100
    msisModelVec[10].alt = 0
    msisModelVec[11].alt = 10
    msisModelVec[12].alt = 30
    msisModelVec[13].alt = 50
    msisModelVec[14].alt = 70
    msisModelVec[16].alt = 100
    msisModelVec[4].g_lat = 0
    msisModelVec[5].g_long = 0
    msisModelVec[6].lst = 4
    msisModelVec[7].f107A = 70
    msisModelVec[8].f107 = 180
    msisModelVec[9].ap = 40


    for ind in range(0,15):
        msisModelVec[ind].updateLocalAtmo(0)

    for ind in range(15,17):
        msisModelVec[ind].msisFlags.switches[9] = 1
        msisModelVec[ind].updateLocalAtmo(0)

    for ind in range(0,17):
        if msisModelVec[ind].msisOutput.d != testValidation[ind, 0:9] :
            testFailCount += 1
            testMessages.append("FAILED: NRLMSISE-00 atmosphere missed check for case "+str(ind))
        if msisModelVec[ind].msisOutput.t != testValidation[ind,9:]:
            testFailCount += 1
            testMessages.append("FAILED: Nrlmsise incorrectly computed temperature for model run " + str(ind))

    return testFailCount, testMessages