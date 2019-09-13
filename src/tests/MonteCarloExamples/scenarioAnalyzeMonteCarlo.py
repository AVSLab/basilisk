
import inspect
import os
from Basilisk.utilities.datashader_utilities import DS_Plot, curve_per_df_component, pull_and_format_df
from Basilisk.utilities.MonteCarlo.AnalysisBaseClass import AnalysisBaseClass
import Basilisk.utilities.macros as macros
from bokeh.palettes import Blues9, Reds9, Greens9, \
    Blues3, Reds3, Greens3, Oranges3, RdYlBu9


filename = inspect.getframeinfo(inspect.currentframe()).filename
fileNameString = os.path.basename(os.path.splitext(__file__)[0])
path = os.path.dirname(os.path.abspath(filename))
from Basilisk import __path__

bskPath = __path__[0]

def plotSuite(dataDir):

    plotList = []
    sigma_BR = pull_and_format_df(dataDir + "attErrorInertial3DMsg.sigma_BR.data")
    sigmaPlot = DS_Plot(sigma_BR, title="Attitude Error",
                        xAxisLabel='time [s]', yAxisLabel='Sigma_BR',
                        macro_x=macros.NANO2SEC,
                        labels = ['b1', 'b2', 'b3'], cmap=RdYlBu9,
                        plotFcn=curve_per_df_component)
    plotList.append(sigmaPlot)

    sigma_BR = pull_and_format_df(dataDir + "attErrorInertial3DMsg.omega_BR_B.data")
    sigmaPlot = DS_Plot(sigma_BR, title="Attitude Rate Error",
                        xAxisLabel='time [s]', yAxisLabel='omega_BR_B',
                        macro_x=macros.NANO2SEC, macro_y=macros.R2D,
                        labels = ['b1', 'b2', 'b3'], cmap=RdYlBu9,
                        plotFcn=curve_per_df_component)
    plotList.append(sigmaPlot)
    return plotList


def main():
    '''
    How to use this script:
    First call show_all_data = True to get a broad view of the data and find a time window to investigate closer
    Second call show_extreme_data = True to look at specific run cases within the window
    Third call show_optional_data = True to look at any extra data to determine why the extrema cases exist

    :param show_all_data: plot all MC runs for the plots specified in the plotSuite method
    :param show_extreme_data: call plotSuite method for user-defined number of extrema MC runs
    :param optional_plots: plots additional user-defined plots
    '''

    # TODO: Assert that the directory has data to make sure the data has been run initially
    # assert

    show_all_data = True
    show_extreme_data = False
    optional_plots = False

    plotList = []
    analysis = AnalysisBaseClass()
    analysis.dataDir = "/scenario_AttFeedbackMC"

    # save_as_static: save off static .html files of the plots generated into the staticDir directory (Note: This inhibits dynamic plotting!
    analysis.save_as_static = False
    analysis.staticDir = "/plots/"

    if show_all_data:
        plotList.extend(plotSuite(analysis.dataDir))

    if show_extreme_data:
        analysis.variableName = "solar_array_sun_bore.missAngle"
        analysis.variableDim = 1

        extremaRunNumbers = analysis.getExtremaRunIndices(numExtrema=1, window=[500 * 1E9, 550 * 1E9])

        analysis.extractSubsetOfRuns(runIdx=extremaRunNumbers)
        plotList.extend(plotSuite(analysis.dataDir + "/subset"))

    if optional_plots:
        # nominalRuns = analysis.getNominalRunIndices(50)
        # statPlots = analysis.generateStatPlots()

        shadowFactor = pull_and_format_df(analysis.dataDir + "/eclipse_data_0.shadowFactor.data", 1)
        shadowFactor = shadowFactor.dropna(axis=1)
        shadowFactorPlot = DS_Plot(shadowFactor, title="Optional Plots: Eclipse",
                                               xAxisLabel='time[s]', yAxisLabel='Eclipse Factor',
                                               macro_x=macros.NANO2SEC, macro_y=macros.R2D,
                                               cmap=RdYlBu9,
                                               plotFcn=curve_per_df_component)

        # plotList.extend([statPlots])
        plotList.extend([shadowFactorPlot])


