import glob
import os
import time

import numpy as np
import pandas as pd
import dask.dataframe as dd

from Basilisk.utilities import macros
from bokeh.plotting import figure
from bokeh.models import ColumnDataSource, HoverTool, ColorBar, LinearColorMapper, BasicTicker, PrintfTickFormatter, Tabs, TabPanel
from bokeh.transform import linear_cmap
from bokeh.palettes import Viridis256

try:
    import holoviews as hv
    from bokeh.layouts import column, row
    from bokeh.models import Slider
    from bokeh.plotting import figure
    from bokeh.models import ColumnDataSource
    from bokeh.plotting import curdoc
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

    @staticmethod
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
        If the ``/subset/`` directory already exists, check if it already contains all runIdx requested, if so skip.

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

    def create_bokeh_plots(self, figures):
        return figures  # Assuming figures are already Bokeh plots
    
    def generateBokehCurves(self):
        curves = []
        for df in self.data:
            source = ColumnDataSource(df)
            p = figure(width=800, height=400, title=self.title)
            p.line(x='x', y='y', source=source, line_width=2, color=self.cmap[0])  # Customize as needed
            curves.append(p)
        return curves

    def renderPlots(self, plotList):
        # Assuming plotList contains Bokeh figures
        layout = column(*plotList)
        curdoc().add_root(layout)

class MonteCarloPlotter(mcAnalysisBaseClass):
    def __init__(self, dataDir):
        super().__init__()
        self.dataDir = dataDir
        self.data = {}
        self.plots = {}
        self.plot_info = {}  # Dictionary to store plot information
        self.component_map = {0: 'x', 1: 'y', 2: 'z'}  # Mapping from component number to label

    def pull_and_format_df(self, path, varIdxLen):
        df = pd.read_pickle(path)
        if len(np.unique(df.columns.codes[1])) != varIdxLen:
            print(f"Warning: {path} not formatted correctly!")
            newMultIndex = pd.MultiIndex.from_product([df.columns.codes[0], range(varIdxLen)],
                                                      names=['runNum', 'varIdx'])
            indices = pd.Index([0, 1])  # Need multiple rows for curves
            df = df.reindex(columns=newMultIndex, index=indices)
        
        # Convert column names to strings and add a 'time' column in seconds
        df.columns = df.columns.map(lambda x: f"run{x[0]}_{x[1]}")
        df['time'] = df.index / 1e9  # Convert nanoseconds to seconds
        return df

    def load_data(self, variable_names):
        for var_name in variable_names:
            file_path = os.path.join(self.dataDir, f"{var_name}.data")
            if os.path.exists(file_path):
                self.data[var_name] = self.pull_and_format_df(file_path, 3)  # Assuming variableDim is 3
            else:
                print(f"Warning: {file_path} not found.")

    def generate_plots(self, components):
        self.plots = {}
        self.plot_info = {}  # Reset plot_info
        for var_name, df in self.data.items():
            for component in components:
                component_label = self.component_map.get(component, str(component))
                title = f"{var_name} - Component {component_label}"
                plot_df = df[['time'] + [col for col in df.columns if col.endswith(f"_{component}")]]
                self.plots[title] = plot_df
                
                # Store plot information
                y_label = f"{var_name.replace('attGuidMsg.', '')} - Component {component_label}"
                self.plot_info[title] = {'y_label': y_label}

    def downsample_data(self, df, target_size=10000):
        if len(df) <= target_size:
            return df
        
        step = len(df) // target_size
        return df.iloc[::step].reset_index(drop=True)

    def get_downsampled_plots(self, target_size=10000):
        return {title: self.downsample_data(df, target_size) for title, df in self.plots.items()}

    def get_plot_info(self):
        return self.plot_info

    def create_bokeh_plot(self, df, var_name, component):
        component_label = self.component_map.get(component, str(component))
        
        p = figure(width=1000, height=600, 
                   title=f"{var_name} Monte Carlo Results - Component {component_label.upper()}",
                   tools="pan,box_zoom,wheel_zoom,reset,save",
                   x_axis_label='Time [s]',
                   y_axis_label=f"{var_name.split('.')[-1]} - Component {component_label.upper()}",
                   background_fill_color="white")

        runs = sorted(df.columns.get_level_values('runNum').unique())
        num_runs = len(runs)
        
        color_palette = Viridis256
        
        color_mapper = LinearColorMapper(palette=color_palette, low=runs[0], high=runs[-1])

        source = ColumnDataSource(data=dict(
            xs=[df.index * macros.NANO2SEC for _ in runs],
            ys=[df.loc[:, (run, component)] for run in runs],
            run=runs
        ))

        p.multi_line(xs='xs', ys='ys', source=source, line_width=2.5, alpha=0.7,
                     color={'field': 'run', 'transform': color_mapper})

        p.add_tools(HoverTool(tooltips=[("Time", "$x{0.00} s"), ("Value", "$y{0.000}"), ("Run", "@run")]))

        ticker = BasicTicker(desired_num_ticks=min(10, num_runs))
        formatter = PrintfTickFormatter(format="%d")
        color_bar = ColorBar(
            color_mapper=color_mapper, 
            width=20, 
            location=(0,0), 
            title="Run Number",
            ticker=ticker,
            formatter=formatter
        )
        p.add_layout(color_bar, 'right')

        return p

    def render_plots(self):
        layout = column(*self.plots.values())
        curdoc().add_root(layout)