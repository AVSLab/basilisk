import glob
import os
import time

import numpy as np
import pandas as pd
from Basilisk.utilities import macros

try:
    import holoviews as hv
    from Basilisk.utilities.datashader_utilities import DS_Plot, curve_per_df_component
except:
    pass

class mcAnalysisBaseClass:
    def __init__(self):
        self.variableName = ""
        self.variableDim = 0
        self.dataDir = ""
        self.numExtrema = 0
        self.extremaRuns = []
        self.timeWindow = []
        self.data = None

    def pull_and_format_df(path, varIdxLen):
        df = pd.read_pickle(path)
        if len(np.unique(df.columns.codes[1])) is not varIdxLen:
            print("Warning: " + path + " not formatted correctly!")
            newMultIndex = pd.MultiIndex.from_product([df.columns.codes[0], range(varIdxLen)],
                                                      names=['runNum', 'varIdx'])
            indices = pd.Index([0, 1])  # Need multiple rows for curves
            df = df.reindex(columns=newMultIndex, index=indices)
        return df

    def getNominalRunIndices(self, maxNumber=50):
        """
        Find the specific MC run indices of the most nominal cases (by iteratively widdling away runs which
        have the largest std)

        :param maxNumber: the number of nominal runs to widdle down to.
        :return: list of run indices
        """
        if self.data is None:
            self.data = pd.read_pickle(self.dataDir + "/" + self.variableName + ".data")

        dataBar = self.data[np.abs(self.data - self.data.mean()) < 0.5 * self.data.std()]
        i = 5
        while len(dataBar.columns.codes[0]) > maxNumber * self.variableDim:
            i += 1
            cols_to_delete = dataBar.columns[dataBar.isnull().sum() / len(dataBar) > 1. / np.sqrt(i)]
            dataBar.drop(cols_to_delete, axis=1, inplace=True)

        print("Nominal runs are ", list(dict.fromkeys(dataBar.columns.codes[0].tolist())))
        return dataBar.columns.codes[0]

    def getExtremaRunIndices(self, numExtrema, window):
        """
        Determine the MC run indices of the most deviant values within a particular time window

        :param numExtrema: number of extreme runs to collect
        :param window: window of time to search for the extremes in
        :return: list of run indices
        """
        if self.data is None:
            self.data = pd.read_pickle(self.dataDir + "/" + self.variableName + ".data")
        times = self.data.index.tolist()

        # Find the closest indices to the time window requested
        indStart = min(range(len(times)), key=lambda i: abs(times[i] - window[0]))
        indEnd = min(range(len(times)), key=lambda i: abs(times[i] - window[1]))
        self.timeWindow = [indStart, indEnd]

        # Find outliers based on largest deviation off of the mean
        self.mean = self.data.mean(axis=1, level=1)
        self.diff = self.data.subtract(self.mean)
        self.diff = self.diff.abs()
        self.diff = self.diff.iloc[indStart:indEnd].max(axis=0)
        self.extremaRuns = self.diff.nlargest(numExtrema).index._codes[0]
        print("Extreme runs are ", list(dict.fromkeys(self.extremaRuns.tolist())))
        return self.extremaRuns

    def generateStatCurves(self):
        """
        Generate curves that represent the mean, median, and standard deviation of a particular variable.
        Not Tested.
        """
        if self.data is None:
            self.data = pd.read_pickle(self.dataDir + "/" + self.variableName + ".data")

        idx = pd.IndexSlice
        self.runs, self.varNum = self.data.columns.values[-1]
        self.runs += 1
        self.varNum += 1
        axesMean = []
        axesMedian = []
        axesStd = []
        for j in range(self.varNum):
            axisMean = self.data.loc[idx[:], idx[:, j]].mean(axis=1)
            axisMedian = self.data.loc[idx[:], idx[:, j]].median(axis=1)
            axisStd = self.data.loc[idx[:], idx[:, j]].std(axis=1)
            axesMean.append(axisMean)
            axesMedian.append(axisMedian)
            axesStd.append(axisStd)

        meanRun = pd.concat(axesMean, axis=1)
        medianRun = pd.concat(axesMedian, axis=1)
        stdRun = pd.concat(axesStd, axis=1)

        return [meanRun, medianRun, stdRun]

    def generateStatPlots(self):
        """
        Generate plots for the mean, median, and mode.
        Not Tested.
        :return: list of stats plots
        """
        if self.data is None:
            self.data = self.pull_and_format_df(self.dataDir + "/" + self.variableName + ".data", self.variableDim)

        meanRun, medianRun, stdRun = self.generateStatCurves()
        varIdxList = range(self.variableDim)
        varIdxListStr = str(varIdxList)

        meanRun.columns = pd.MultiIndex.from_product([['mean'], [0,1,2]], names=["stats", "varIdx"])
        medianRun.columns = pd.MultiIndex.from_product([['median'], [0,1,2]], names=["stats", "varIdx"])
        stdRun.columns = pd.MultiIndex.from_product([['std'], [0,1,2]], names=["stats", "varIdx"])

        meanRun_plot = DS_Plot(meanRun, title="Mean Plot: " + self.variableName,
                                            xAxisLabel='time[s]', yAxisLabel= self.variableName.split('.')[-1],
                                            macro_x=macros.NANO2SEC,
                                            labels=['1', '2', '3'],
                                            plotFcn=curve_per_df_component)

        medRun_plot = DS_Plot(medianRun, title="Median Plot: " + self.variableName,
                                            xAxisLabel='time[s]', yAxisLabel= self.variableName.split('.')[-1],
                                            macro_x=macros.NANO2SEC,
                                            labels=['1', '2', '3'],
                                            plotFcn=curve_per_df_component)

        stdRun_plot = DS_Plot(stdRun, title="Standard Dev Plot: " + self.variableName,
                                            xAxisLabel='time[s]', yAxisLabel= self.variableName.split('.')[-1],
                                            macro_x=macros.NANO2SEC,
                                            labels=['1', '2', '3'],
                                            plotFcn=curve_per_df_component)

        statRun_plots = []
        statRun_plots.append(meanRun_plot)
        statRun_plots.append(medRun_plot)
        statRun_plots.append(stdRun_plot)

        return statRun_plots

    def extractSubsetOfRuns(self, runIdx):
        """
        Create a separate folder in the data directory that contains the subset of data the user is looking to plot.
        If the ``/subset/`` directory already exists, check if it contains the data for the runs requested, if so skip.

        :param runIdx: list of run indices to extract
        :return: nothing
        """
        idx = pd.IndexSlice
        baseDir = self.dataDir

        # check if a subset directory exists, and if it already contains all runIdx requested
        if not os.path.exists(baseDir + "/subset/"):
            os.mkdir(baseDir + "/subset/")
        else:
            filePaths = glob.glob(baseDir + "/subset" + "/*.data")
            for filePath in filePaths:
                if "MonteCarlo.data" in filePath:
                    continue
                if "run" in filePath and "overrun" not in filePath:
                    continue
                df = pd.read_pickle(filePath)
                singleton = list(dict.fromkeys(np.array(df.columns.codes[0]).tolist()))
                singletonRuns = list(dict.fromkeys(np.sort(np.array(runIdx)).tolist()))
                if len(singleton) == len(singletonRuns):
                    if singletonRuns == singleton:
                        print("Subset directory already contains runIdx values. Skipping extraction")
                        return
                    else:
                        break

        # If no data in subset (or the wrong data), extract and save the right data.
        print("Populating Subset Directory with Dataframes for runs: " + str(runIdx))
        # shutil.rmtree(dataDir + "/subset/")
        filePaths = glob.glob(baseDir + "/*.data")
        for filePath in filePaths:
            if "MonteCarlo.data" in filePath:
                continue
            if "run" in filePath and "overrun" not in filePath:
                continue
            df = pd.read_pickle(filePath)
            dfSubSet = df.loc[idx[:], idx[runIdx, :]]
            varName = filePath.rsplit("/")
            pd.to_pickle(dfSubSet, baseDir + "/subset/" + varName[-1])
        print("Finished Populating Subset Directory")

    def renderPlots(self, plotList):
        """
        Render all plots in plotList and print information about time taken, percent complete, which plot, etc.

        :param plotList: List of plots to render
        :return: nothing.
        """
        hv.extension('bokeh')
        renderer = hv.renderer('bokeh').instance(mode='server')

        if self.save_as_static:
            print("Note: You requested to save static plots. This means no interactive python session will be generated.")
        print("Beginning the plotting")

        if not os.path.exists(self.dataDir + self.staticDir):
            os.mkdir(self.dataDir + self.staticDir)

        for i in range(len(plotList)):
            startTime = time.time()
            image, title = plotList[i].generateImage()
            try:
                if self.save_as_static:
                    # Save .html files of each of the plots into the static directory
                    hv.save(image, self.dataDir + self.staticDir + "/" + title + ".html")
                else:
                    renderer.server_doc(image)
                # Print information about the rendering process
                print("LOADED: " + title +"\t\t\t" +
                      "Percent Complete: " + str(round((i + 1) / len(plotList) * 100, 2)) + "% \t\t\t"
                      "Time Elapsed: " + str( round(time.time() - startTime)) + " [s]")
            except Exception as e:
                print("Couldn't Plot " + title)
                print(e)
