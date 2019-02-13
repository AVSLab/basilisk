from test_unitSpice import unitSpice

DateSpice = "2015 February 10, 00:00:00.0 TDB"
DatePlot = "02/10/15"
MarsTruthPos = [2.049283795042291E+08, 4.654550957513031E+07, 1.580778617009296E+07]
EarthTruthPos = [-1.137790671899544E+08, 8.569008401822130E+07, 3.712507705247846E+07]
SunTruthPos = [4.480338216752146E+05, -7.947764237588293E+04, -5.745748832696378E+04]


if __name__ == "__main__":
    unitSpice(testPlottingFixture=None, show_plots=True, DateSpice=DateSpice, DatePlot=DatePlot,
              MarsTruthPos=MarsTruthPos, EarthTruthPos=EarthTruthPos, SunTruthPos=SunTruthPos)


