import warnings

import numpy as np

with warnings.catch_warnings():
    warnings.simplefilter("ignore", category=DeprecationWarning)
    import pandas as pd
    import datashader as ds
    import holoviews as hv
    from holoviews.operation.datashader import datashade, dynspread
    from holoviews.streams import RangeXY
    from datashader.colors import Sets1to3
from Basilisk.utilities import macros

def pull_and_format_df(path, varIdxLen):
    df = pd.read_pickle(path)
    if len(np.unique(df.columns.codes[1])) is not varIdxLen:
        print("Warning: " + path + " not formatted correctly!")
        newMultIndex = pd.MultiIndex.from_product([df.columns.codes[0], list(range(varIdxLen))], names=['runNum', 'varIdx'])
        indices = pd.Index([0,1]) # Need multiple rows for curves
        df = df.reindex(columns=newMultIndex, index=indices)
    return df

def curve_per_df_component(df):
    """
    Make a curve per component in the message dataframe (i.e. omega_BR_B[2] across all runs as a single curve)

    :param df:
    :return:
    """
    idx = pd.IndexSlice
    df = df.interpolate(method = "linear")
    df_list = []
    for i in np.unique(df.columns.codes[1]):
        # Select all of the component
        varIdx_df = df.loc[idx[:], idx[:, i]]

        # Inject NaNs at the end of the run so the curves don't wrap from t_f to t_0
        varIdx_df = varIdx_df.append(pd.Series(name=np.nan, dtype='float'))

        # Flatten values by column order
        time = np.tile(varIdx_df.index, len(varIdx_df.columns.codes[0]))  # Repeat time by number of runs
        varIdx_flat = varIdx_df.values.flatten('F')

        # Generate a curve for each component
        curve_df = pd.DataFrame(np.transpose([time, varIdx_flat]).tolist(), columns=['x', 'y'])
        df_list.append(curve_df)

    return df_list

def curve_per_df_column(df):
    """
    Divides the dataframe by column into format friendly for datashaders
    :return: List of single column dataframes
    """
    idx = pd.IndexSlice
    df_list = []
    for index in range(len(df.columns)):
        try:
            i = df.columns.codes[0][index] # Multi-Index level=0 index
            j = df.columns.codes[1][index] # Multi-Index level=1 index

            # Grab the desired x and y data
            xData = df.index.values # time [ns]
            yData = df.loc[idx[:], idx[i, j]].values # variable data
            runNum = np.repeat(i, len(xData))
        except:
            # Grab the desired x and y data
            xData = df.index.values  # time [ns]
            yData = df.loc[idx[:], idx[index]].values  # variable data
            runNum = np.repeat(index, len(xData))

        # Convert to two columns
        plotData = pd.DataFrame(np.transpose([xData, yData]).tolist(), columns=['x', 'y'])#, runNum]).tolist()
        df_list.append(plotData)
    return df_list




class DS_Plot():
    '''
    Object which stores data necessary to generate a bokeh image.
    '''

    def __init__(self, data, title='',
              yAxisLabel='', xAxisLabel='time [ns]',
              macro_y=1.0, macro_x=macros.NANO2SEC,
              cmap=Sets1to3,
              plotObjType=hv.Curve,
              labels=[],
              plotFcn=curve_per_df_component):
        if type(data) is not list:
            self.data = [data]
        else:
            self.data = data
        self.title = title
        self.yAxisLabel = yAxisLabel
        self.xAxisLabel = xAxisLabel
        self.macro_x = macro_x
        self.macro_y = macro_y
        self.plotObjType = plotObjType
        self.cmap = cmap
        #self.backend =
        self.labels = labels
        self.plotFcn = plotFcn


    def generateCurves(self):
        '''
        Generate hv.Curve or hv.Points from the provided dataframe(s)
        Scales the index and values using macro_x and macro_y
        Populates a dictionary with a unique identifier for each curve for curve coloring purposes
        :return: dict of hv.Curve or hv.Point objects
        '''
        count = 0
        curves = []
        missingData = []
        self.min = self.data[0].values.min()
        self.max = self.data[0].values.max()

        for i in range(len(self.data)):
            if self.min > self.data[0].values.min() : self.min = self.data[0].values.min()
            if self.max < self.data[0].values.max() : self.max = self.data[0].values.max()
            self.data[i] = self.data[i] * self.macro_y
            self.data[i].index = self.data[i].index * 1e-9

            # Seperate dataframe by component
            curveList = self.plotFcn(self.data[i])  # Only one component so it will be a single curve

            # Customize the individual component curves, points, other
            for curve_df in curveList:
                curve = self.plotObjType(curve_df)#.opts(framewise=True)
                curves.append(curve)
                count += 1

            if self.data[i].dropna().empty:
                missingData.append(True)
        # Label each curve with a unique identifier
        curves = {i: curves[i] for i in range(len(curves))}
        return curves, missingData


    def generateImage(self):
        '''
        Generate the image to be sent to the bokeh server. This includes
        1) generating curves from the dataframe or list of dataframes,
        2) overlaying those curves onto a single image, and
        3) populating various annotations and asethetic configurations
        :return: hv.DynImage()
        '''
        hv.extension('bokeh')
        # Overlay these curves
        curves, missingData = self.generateCurves()
        overlay = hv.NdOverlay(curves, kdims='k')#.opts(framewise=True)

        # Rasterize the plot using datashade()
        if np.sum(missingData) == len(self.data):
            image = hv.Text(0.5, 0.5, "All Data Missing")
        else:
            if self.min == self.max and self.min != np.nan:
                y_range = (self.min-0.1, self.max+0.1)
                image = dynspread(datashade(overlay, dynamic=True, streams=[RangeXY],
                                            aggregator=ds.count_cat('k'), color_key=self.cmap,
                                            y_range=y_range)).opts(framewise=True)
            else:
                image = dynspread(datashade(overlay, dynamic=True, streams=[RangeXY],
                                            aggregator=ds.count_cat('k'), color_key=self.cmap
                                           )).opts(framewise=True)

        image.opts(width=960, height=540)
        image.opts(tools=['hover'])
        image.opts(padding=0.05)
        image.opts(title=self.title, xlabel=self.xAxisLabel, ylabel=self.yAxisLabel)

        if not self.labels == []:
            color_key = [(name, color) for name, color in zip(self.labels, self.cmap)]
            legend = hv.NdOverlay({n: hv.Points([np.nan, np.nan], label=str(n)).opts(style=dict(color=c)) for n, c in color_key})
            image = image*legend

        return image, self.title

