
# ISC License
#
# Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
#
# Permission to use, copy, modify, and/or distribute this software for any
# purpose with or without fee is hereby granted, provided that the above
# copyright notice and this permission notice appear in all copies.
#
# THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
# WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
# ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
# WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
# ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
# OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.


import inspect
import os

import pytest

#
# Spice Unit Test
#
# Purpose:  Test the proper function of the Spice Ephemeris module.
#           Proper function is tested by comparing Spice Ephemeris to
#           JPL Horizons Database for different planets and times of year
# Author:   Thibaud Teil
# Creation Date:  Dec. 20, 2016
#

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
from Basilisk import __path__
bskPath = __path__[0]


import datetime
from Basilisk.utilities import unitTestSupport
from Basilisk.utilities import SimulationBaseClass
import numpy
from Basilisk.simulation import spiceInterface
from Basilisk.utilities import macros
import matplotlib.pyplot as plt


# Class in order to plot using data across the different parameterized scenarios
class DataStore:
    def __init__(self):
        self.Date = []  # replace these with appropriate containers for the data to be stored for plotting
        self.MarsPosErr = []
        self.EarthPosErr = []
        self.SunPosErr = []

    def plotData(self):
        fig1 = plt.figure(1)
        rect = fig1.patch
        rect.set_facecolor('white')

        plt.xticks(numpy.arange(len(self.Date)), self.Date)
        plt.plot(self.MarsPosErr, 'r', label='Mars')
        plt.plot(self.EarthPosErr, 'bs', label='Earth')
        plt.plot(self.SunPosErr, 'yo', label='Sun')

        plt.rc('font', size=50)
        plt.legend(loc='upper left')
        fig1.autofmt_xdate()
        plt.xlabel('Date of test')
        plt.ylabel('Position Error [m]')
        plt.show()

    def giveData(self):
        plt.figure(1)
        plt.close(1)
        fig1 = plt.figure(1, figsize=(7, 5), dpi=80, facecolor='w', edgecolor='k')

        plt.xticks(numpy.arange(len(self.Date)), self.Date)
        plt.plot(self.MarsPosErr, 'r', label='Mars')
        plt.plot(self.EarthPosErr, 'b', label='Earth')
        plt.plot(self.SunPosErr, 'y', label='Sun')

        plt.legend(loc='upper left')
        fig1.autofmt_xdate()
        plt.xlabel('Date of test')
        plt.ylabel('Position Error [m]')

        return plt


# Py.test fixture in order to plot
@pytest.fixture(scope="module")
def testPlottingFixture(show_plots):
    dataStore = DataStore()
    yield dataStore

    plt = dataStore.giveData()
    unitTestSupport.writeFigureLaTeX('EphemMars', 'Ephemeris Error on Mars', plt, 'height=0.7\\textwidth, keepaspectratio', path)
    plt.ylim(0, 2E-2)
    unitTestSupport.writeFigureLaTeX('EphemEarth', 'Ephemeris Error on Earth', plt, 'height=0.7\\textwidth, keepaspectratio', path)
    plt.ylim(0, 5E-6)
    unitTestSupport.writeFigureLaTeX('EphemSun', 'Ephemeris Error on Sun', plt, 'height=0.7\\textwidth, keepaspectratio', path)
    if show_plots:
        dataStore.plotData()


# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail(True)

# The following 'parametrize' function decorator provides the parameters and expected results for each
#   of the multiple test runs for this test.
@pytest.mark.parametrize("DateSpice, DatePlot , MarsTruthPos , EarthTruthPos, SunTruthPos, useMsg", [
    ("2015 February 10, 00:00:00.0 TDB", "02/10/15" ,[2.049283795042291E+08, 4.654550957513031E+07, 1.580778617009296E+07] ,[-1.137790671899544E+08, 8.569008401822130E+07, 3.712507705247846E+07],[4.480338216752146E+05, -7.947764237588293E+04, -5.745748832696378E+04], False),
    ("2015 February 20, 00:00:00.0 TDB", "02/20/15" ,[ 1.997780190793433E+08, 6.636509140613769E+07, 2.503734390917690E+07], [-1.286636391244711E+08, 6.611156946652703E+07, 2.863736192793162E+07],[4.535258225415821E+05, -7.180260790174162E+04, -5.428151563919459E+04], False),
    ("2015 April 10, 00:00:00.0 TDB", "04/10/15" ,[1.468463828343225E+08, 1.502909016357645E+08, 6.495986693265321E+07],[-1.406808744055921E+08, -4.614996219219251E+07, -2.003047222725225E+07],[4.786772771370058E+05, -3.283487082146838E+04, -3.809690999538498E+04], False),
    ("2015 April 20, 00:00:00.0 TDB", "04/20/15" ,[1.313859463489376E+08, 1.637422864185919E+08, 7.154683454681394E+07], [-1.304372112275012E+08, -6.769916175223185E+07, -2.937179538983412E+07],[4.834188130324496E+05, -2.461799304268214E+04, -3.467083541217462E+04], False),
    ("2015 June 10, 00:00:00.0 TDB", "06/10/15" , [3.777078276008123E+07, 2.080046702252371E+08, 9.437503998071109E+07],[-2.946784585692780E+07, -1.365779215199542E+08, -5.923299652516938E+07],[5.052070933145228E+05, 1.837038638578682E+04, -1.665575509449521E+04], False),
    ("2015 June 20, 00:00:00.0 TDB", "06/20/15" , [1.778745850655047E+07, 2.116712756672253E+08, 9.659608128589308E+07], [-4.338053580692466E+06, -1.393743714818930E+08, -6.044500073550620E+07],[5.090137386469169E+05, 2.694272566092012E+04, -1.304862690440150E+04], False),
    ("2015 August 10, 00:00:00.0 TDB", "08/10/15" , [-8.302866300591002E+07, 2.053384243312354E+08, 9.641192179982041E+07],[1.111973130587749E+08, -9.507060674145325E+07, -4.123957889640842E+07],[5.263951240980130E+05, 7.113788303899391E+04, 5.601415938789949E+03], False),
    ("2015 August 20, 00:00:00.0 TDB", "08/20/15" , [-1.015602120938274E+08, 1.994877113707506E+08, 9.422840996376510E+07], [1.267319535735967E+08, -7.664279872369213E+07, -3.325170492059625E+07],[5.294368386862668E+05, 7.989635630604146E+04, 9.307380417148834E+03], False),
    ("2015 October 10, 00:00:00.0 TDB", "10/10/15" , [-1.828944464171771E+08, 1.498117608528323E+08, 7.363786404040858E+07],[1.440636517249971E+08, 3.827074461651713E+07, 1.656440704503509E+07],[5.433268959250135E+05, 1.251717566539142E+05, 2.849999378507032E+04], False),
    ("2015 October 20, 00:00:00.0 TDB", "10/20/15" , [-1.955685835661002E+08, 1.367530082668284E+08, 6.799006120628108E+07],[1.343849818314204E+08, 6.019977403127116E+07, 2.607024615553362E+07],[5.457339294611338E+05, 1.342148834831663E+05, 3.234185290692726E+04], False),
    ("2015 December 10, 00:00:00.0 TDB", "12/10/15" , [-2.392022492203017E+08, 5.847873056287902E+07, 3.326431285934674E+07],[3.277145427626925E+07, 1.320956053465003E+08, 5.723804900679157E+07],[5.559607335969181E+05, 1.813263443761486E+05, 5.242991145066972E+04], False),
    ("2015 December 20, 00:00:00.0 TDB", "12/20/15" , [-2.432532635321551E+08, 4.156075241419497E+07, 2.561357000250468E+07], [6.866134677340530E+06, 1.351080593806486E+08, 5.854460725992832E+07],[5.575179227151767E+05, 1.907337298309725E+05, 5.645553733620559E+04], False),
    ("2016 February 10, 00:00:00.0 TDB", "02/10/16" , [-2.385499316808322E+08, -4.818711325555268E+07, -1.567983931044450E+07],[-1.132504603146183E+08, 8.647183658089657E+07, 3.746046979137553E+07],[5.630027736619673E+05, 2.402590276126245E+05, 7.772804647512530E+04], False),
    ("2016 February 20, 00:00:00.0 TDB", "02/20/16" , [-2.326189626865872E+08, -6.493600278878165E+07,-2.352250994262638E+07], [-1.282197228963156E+08, 6.695033443521750E+07, 2.899822684259482E+07],[5.635403728358555E+05, 2.498445036007692E+05, 8.185973180152237E+04], False),
    ("2016 April 10, 00:00:00.0 TDB", "04/10/16" , [-1.795928090776869E+08, -1.395102817786797E+08, -5.916067235603344E+07],[-1.399773606371217E+08, -4.749126225182984E+07, -2.061331784067504E+07],[5.639699901756521E+05, 2.977755140828988E+05, 1.025609223621660E+05], False),
    ("2016 April 20, 00:00:00.0 TDB", "04/20/16" , [-1.646488222290798E+08, -1.517361128528306E+08, -6.517204439842616E+07], [-1.294310199316289E+08, -6.890085351639988E+07, -2.989515576444890E+07],[5.636348418836893E+05, 3.073681895822543E+05, 1.067117713233756E+05], False),
    ("2016 June 10, 00:00:00.0 TDB", "06/10/16" , [-7.085675304355654E+07, -1.933788289169757E+08, -8.680583098365544E+07],[-2.756178030784301E+07, -1.365892814324490E+08, -5.923888961370525E+07],[5.597493293268962E+05, 3.563312003229167E+05, 1.279448896916322E+05], False),
    ("2016 June 20, 00:00:00.0 TDB", "06/20/16" , [-4.994076874358515E+07, -1.967336617729592E+08, -8.890950206156498E+07], [-2.413995783152328E+06, -1.390828611356292E+08, -6.032062925759617E+07],[5.585605124166313E+05, 3.659402953599973E+05, 1.321213212542782E+05], False),
    ("2016 August 10, 00:00:00.0 TDB", "08/10/16" , [5.965895856067932E+07, -1.851251207911916E+08, -8.654489416392270E+07],[1.124873641861596E+08, -9.344410235679612E+07, -4.053621796412993E+07],[5.501477436262188E+05, 4.149525682073312E+05, 1.534983234437988E+05], False),
    ("2016 August 20, 00:00:00.0 TDB", "08/20/16" , [8.021899957229958E+07, -1.770523551156136E+08, -8.339737954004113E+07] , [1.277516713236801E+08, -7.484361731649198E+07, -3.247232896320245E+07],[5.480148786547601E+05, 4.245199573757614E+05, 1.576874582857746E+05], False),
    ("2016 October 10, 00:00:00.0 TDB", "10/10/16" , [1.674991232418756E+08, -1.090427183510760E+08, -5.456038490399034E+07],[ 1.434719792031415E+08, 4.029387436854774E+07, 1.744057129058395E+07],[5.348794087050703E+05, 4.727986075510736E+05, 1.788947974297997E+05], False),
    ("2016 October 20, 00:00:00.0 TDB", "10/20/16" , [1.797014954219412E+08, -9.133803506487390E+07, -4.676932170648168E+07] , [ 1.334883617893556E+08, 6.209917895944476E+07, 2.689423661839254E+07],[5.318931290166953E+05, 4.821605725626923E+05, 1.830201949404063E+05], False),
    ("2016 December 10, 00:00:00.0 TDB", "12/10/16" , [2.087370835407280E+08, 1.035903131428964E+07, -9.084001492689754E+05] ,[3.082308903715757E+07, 1.328026978502711E+08, 5.754559376449955E+07],[5.147792796720199E+05, 5.293951386498961E+05, 2.038816823549219E+05], False),
    ("2016 December 20, 00:00:00.0 TDB", "12/20/16" , [2.075411186038260E+08, 3.092173198610598E+07, 8.555251204561792E+06], [4.885421269793877E+06, 1.355125217382336E+08, 5.872015978003684E+07],[5.110736843893399E+05, 5.385860060393942E+05, 2.079481168492821E+05], True)
])



# provide a unique test method name, starting with test_
def test_unitSpice(testPlottingFixture, show_plots, DateSpice, DatePlot, MarsTruthPos, EarthTruthPos,
                   SunTruthPos, useMsg):
    """Module Unit Test"""
    # each test method requires a single assert method to be called
    [testResults, testMessage] = unitSpice(testPlottingFixture, show_plots, DateSpice, DatePlot, MarsTruthPos,
                                           EarthTruthPos, SunTruthPos, useMsg)
    assert testResults < 1, testMessage


# Run unit test
def unitSpice(testPlottingFixture, show_plots, DateSpice, DatePlot, MarsTruthPos, EarthTruthPos, SunTruthPos, useMsg):
    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty array to store test log messages

    # Create a sim module as an empty container
    unitTaskName = "unitTask"  # arbitrary name (don't change)
    unitProcessName = "TestProcess"  # arbitrary name (don't change)

    # Create a sim module as an empty container
    TotalSim = SimulationBaseClass.SimBaseClass()

    DynUnitTestProc = TotalSim.CreateNewProcess(unitProcessName)
    # create the dynamics task and specify the integration update time
    DynUnitTestProc.addTask(TotalSim.CreateNewTask(unitTaskName, macros.sec2nano(0.1)))

    # Initialize the spice modules that we are using.
    SpiceObject = spiceInterface.SpiceInterface()
    SpiceObject.ModelTag = "SpiceInterfaceData"
    SpiceObject.SPICEDataPath = bskPath + '/supportData/EphemerisData/'
    planetNames = ["earth", "mars barycenter", "sun"]
    SpiceObject.addPlanetNames(spiceInterface.StringVector(planetNames))
    SpiceObject.UTCCalInit = DateSpice

    if useMsg:      # in this case check that the planet frame names can be set as well
        planetFrames = []
        for planet in planetNames:
            planetFrames.append("IAU_" + planet)
        planetFrames[1] = ""    # testing that default IAU values are used here
        SpiceObject.planetFrames = spiceInterface.StringVector(planetFrames)

    TotalSim.AddModelToTask(unitTaskName, SpiceObject)

    if useMsg:
        epochMsg = unitTestSupport.timeStringToGregorianUTCMsg(DateSpice)
        SpiceObject.epochInMsg.subscribeTo(epochMsg)

        # The following value is set, but should not be used by the module.  This checks that the above
        # epoch message over-rules any info set in this variable.
        SpiceObject.UTCCalInit = "1990 February 10, 00:00:00.0 TDB"

    spiceObjectLog = SpiceObject.logger(["GPSSeconds", "J2000Current", "julianDateCurrent", "GPSWeek"])
    TotalSim.AddModelToTask(unitTaskName, spiceObjectLog)

    # Configure simulation
    TotalSim.ConfigureStopTime(int(60.0 * 1E9))

    # Execute simulation
    TotalSim.InitializeSimulation()
    TotalSim.ExecuteSimulation()

    # Get the logged variables (GPS seconds, Julian Date)
    DataGPSSec = unitTestSupport.addTimeColumn(spiceObjectLog.times(), spiceObjectLog.GPSSeconds)
    DataJD = unitTestSupport.addTimeColumn(spiceObjectLog.times(), spiceObjectLog.julianDateCurrent)

    # Get parametrized date from DatePlot
    year = "".join(("20", DatePlot[6:8]))
    date = datetime.datetime( int(year), int(DatePlot[0:2]), int(DatePlot[3:5]))

    testPlottingFixture.Date = DatePlot[0:8]

    # Get the GPS time
    date2 = datetime.datetime(1980, 0o1, 6)  # Start of GPS time
    timeDiff = date-date2  # Time in days since GPS time
    GPSTimeSeconds = timeDiff.days*86400

    # Get the Julian day
    JulianStartDate = date.toordinal()+1721424.5

    #
    # Begin testing module results to truth values
    #

    # Truth values
    # For Time delta check
    GPSRow = DataGPSSec[0, :]
    InitDiff = GPSRow[1] - GPSRow[0] * 1.0E-9
    i = 1

    # Compare the GPS seconds values
    AllowTolerance = 1E-6
    while i < DataGPSSec.shape[0]:
        if date.isoweekday() == 7:  # Skip test on Sundays, because it's the end of a GPS week (seconds go to zero)
            i += 1
        else:
            GPSRow = DataGPSSec[i, :]
            CurrDiff = GPSRow[1] - GPSRow[0] * 1.0E-9
            if abs(CurrDiff - InitDiff) > AllowTolerance:
                testFailCount += 1
                testMessages.append("FAILED: Time delta check failed with difference of: %(DiffVal)f \n"% \
                                {"DiffVal": CurrDiff - InitDiff})
            i += 1

    # Truth values
    # For absolute GPS time check
    if date > datetime.datetime(2015, 0o6, 30):  # Taking into account the extra leap second added on 6/30/2015
        GPSEndTime = GPSTimeSeconds + 17 + 60.0 - 68.184  # 17 GPS skip seconds passed that date
    else:
        GPSEndTime = GPSTimeSeconds + 16 + 60.0 - 67.184  # 16 GPS skip seconds before

    GPSWeek = int(GPSEndTime / (86400 * 7))
    GPSSecondAssumed = GPSEndTime - GPSWeek * 86400 * 7
    GPSSecDiff = abs(GPSRow[1] - GPSSecondAssumed)

    # TestResults['GPSAbsTimeCheck'] = True
    AllowTolerance = 1E-4
    if useMsg:
        AllowTolerance = 2E-2
    # Skip test days that are Sunday because of the end of a GPS week
    if date.isoweekday() != 7 and GPSSecDiff > AllowTolerance:
        testFailCount += 1
        testMessages.append("FAILED: Absolute GPS time check failed with difference of: %(DiffVal)f \n" % \
                            {"DiffVal": GPSSecDiff})

    # Truth values
    # For absolute Julian date time check
    if date>datetime.datetime(2015, 0o6, 30):  # Taking into account the extra leap second added on 6/30/2015
        JDEndTime = JulianStartDate + 0.0006944440 - 68.184 / 86400
    else:
        JDEndTime = JulianStartDate + 0.0006944440 - 67.184 / 86400

    # Simulated values
    JDEndSim = DataJD[i - 1, 1]
    JDTimeErrorAllow = 0.1 / (24.0 * 3600.0)
    if abs(JDEndSim - JDEndTime) > JDTimeErrorAllow:
        testFailCount += 1
        testMessages.append("FAILED: Absolute Julian Date time check failed with difference of: %(DiffVal)f \n" % \
                            {"DiffVal": abs(JDEndSim - JDEndTime)})

    # Truth values
    # For Mars position check
    MarsPosEnd = numpy.array(MarsTruthPos)
    MarsPosEnd = MarsPosEnd * 1000.0

    # Get Simulated values
    MarsPosVec = SpiceObject.planetStateOutMsgs[1].read().PositionVector

    # Compare Mars position values
    MarsPosArray = numpy.array([MarsPosVec[0], MarsPosVec[1], MarsPosVec[2]])
    MarsPosDiff = MarsPosArray - MarsPosEnd
    PosDiffNorm = numpy.linalg.norm(MarsPosDiff)

    # Plot Mars position values
    testPlottingFixture.MarsPosErr = PosDiffNorm

    # Test Mars position values
    PosErrTolerance = 250
    if useMsg:
        PosErrTolerance = 1000
    if (PosDiffNorm > PosErrTolerance):
        testFailCount += 1
        testMessages.append("FAILED: Mars position check failed with difference of: %(DiffVal)f \n" % \
                            {"DiffVal": PosDiffNorm})

    # Truth values
    # For Earth position check
    EarthPosEnd = numpy.array(EarthTruthPos)
    EarthPosEnd = EarthPosEnd * 1000.0

    # Simulated Earth position values
    EarthPosVec = SpiceObject.planetStateOutMsgs[0].read().PositionVector

    # Compare Earth position values
    EarthPosArray = numpy.array([EarthPosVec[0], EarthPosVec[1], EarthPosVec[2]])
    EarthPosDiff = EarthPosArray - EarthPosEnd
    PosDiffNorm = numpy.linalg.norm(EarthPosDiff)

    # Plot Earth position values
    testPlottingFixture.EarthPosErr = PosDiffNorm

    # TestResults['EarthPosCheck'] = True
    if (PosDiffNorm > PosErrTolerance):
        testFailCount += 1
        testMessages.append("FAILED: Earth position check failed with difference of: %(DiffVal)f \n" % \
                            {"DiffVal": PosDiffNorm})

    # Truth Sun position values
    SunPosEnd = numpy.array(SunTruthPos)
    SunPosEnd = SunPosEnd * 1000.0

    #Simulated Sun position values
    SunPosVec = SpiceObject.planetStateOutMsgs[2].read().PositionVector

    # Compare Sun position values
    SunPosArray = numpy.array([SunPosVec[0], SunPosVec[1], SunPosVec[2]])
    SunPosDiff = SunPosArray - SunPosEnd
    PosDiffNorm = numpy.linalg.norm(SunPosDiff)

    # plot Sun position values
    testPlottingFixture.SunPosErr = PosDiffNorm

    # Test Sun position values
    if (PosDiffNorm > PosErrTolerance):
        testFailCount += 1
        testMessages.append("FAILED: Sun position check failed with difference of: %(DiffVal)f \n" % \
                            {"DiffVal": PosDiffNorm})

    if date == datetime.datetime(2016,12,20): # Only test the false files on the last test

        # Test non existing directory
        SpiceObject.SPICEDataPath = "ADirectoryThatDoesntreallyexist"
        SpiceObject.SPICELoaded = False

        # TotalSim.ConfigureStopTime(int(1E9)) #Uncomment these 3 lines to test false directory
        # TotalSim.InitializeSimulation()
        # TotalSim.ExecuteSimulation()

        # Test overly long planet name
        SpiceObject.SPICEDataPath = ""
        SpiceObject.SPICELoaded = False
        # Uncomment these 4 lines to test false planet names
        # SpiceObject.addPlanetNames(spiceInterface.StringVector(["earth", "mars", "sun",
        #                                                     "thisisaplanetthatisntreallyanythingbutIneedthenametobesolongthatIhitaninvalidconditioninmycode"]))
        # TotalSim.ConfigureStopTime(int(1E9))
        # TotalSim.InitializeSimulation()
        # TotalSim.ExecuteSimulation()

    # print out success message if no error were found
    if testFailCount == 0:
        print(" \n PASSED ")

    # each test method requires a single assert method to be called
    # this check below just makes sure no sub-test failures were found
    return [testFailCount, ''.join(testMessages)]


# This statement below ensures that the unit test scrip can be run as a
# stand-along python script
#
if __name__ == "__main__":
    test_unitSpice(
                   testPlottingFixture,
                   True,  # show_plots
                   "2015 February 10, 00:00:00.00 TDB",
                   "02/10/15",
                   [2.049283795042291E+08, 4.654550957513031E+07, 1.580778617009296E+07],
                   [-1.137790671899544E+08, 8.569008401822130E+07, 3.712507705247846E+07],
                   [4.480338216752146E+05, -7.947764237588293E+04, -5.745748832696378E+04],
                   True   # useMsg
                   )
