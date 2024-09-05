import warnings

import numpy as np

with warnings.catch_warnings():
    warnings.simplefilter("ignore", category=DeprecationWarning)
    import pandas as pd
    import datashader as ds
    import holoviews as hv
    from holoviews.operation.datashader import datashade, dynspread, rasterize, spread
    from holoviews.streams import RangeXY
    from datashader.colors import Sets1to3
    from holoviews import opts
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

    :param df: DataFrame to process
    :return: List of DataFrames with curves
    """
    idx = pd.IndexSlice
    df = df.interpolate(method="linear")
    df_list = []

    for i in np.unique(df.columns.codes[1]):
        # Select all of the component
        varIdx_df = df.loc[idx[:], idx[:, i]]

        # Ensure that varIdx_df and new_row have compatible indices
        if isinstance(varIdx_df.index, pd.MultiIndex):
            varIdx_df.index = varIdx_df.index.to_flat_index()

        # Create new row with NaNs having the same length as varIdx_df columns
        new_row = pd.Series([np.nan] * len(varIdx_df.columns), index=varIdx_df.columns)

        # Concatenate DataFrames
        varIdx_df = pd.concat([varIdx_df, new_row.to_frame().T], ignore_index=False)

        # Flatten values by column order
        time = np.tile(varIdx_df.index[:-1], len(varIdx_df.columns.codes[0]))  # Repeat time by number of runs
        varIdx_flat = varIdx_df.values.flatten('F')[:-len(varIdx_df.columns.codes[0])]  # Flatten by column order

        # Generate a curve for each component
        curve_df = pd.DataFrame({'x': time, 'y': varIdx_flat})
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
        curves = {}
        missingData = []
        self.min = self.data[0].values.min()
        self.max = self.data[0].values.max()

        for i in range(len(self.data)):
            if self.min > self.data[i].values.min():
                self.min = self.data[i].values.min()
            if self.max < self.data[i].values.max():
                self.max = self.data[i].values.max()
            self.data[i] = self.data[i] * self.macro_y
            self.data[i].index = self.data[i].index * self.macro_x

            # Separate dataframe by component
            curveList = self.plotFcn(self.data[i])  # Only one component so it will be a single curve
            for curve_df in curveList:
                curves[f'Curve {count}'] = self.plotObjType(curve_df, kdims=['x'], vdims=['y']).opts(color=self.cmap[count % len(self.cmap)])
                count += 1

        return curves, missingData

    def generateImage(self):
        '''
        Generate the image to be rendered dynamically.
        :return: holoviews Element and title
        '''
        curves, _ = self.generateCurves()  # This should return a dictionary now
        overlay = hv.NdOverlay(curves, kdims='k')

        # Return the overlay as a holoviews element with interactive capabilities
        return overlay.opts(
            width=800, height=600, tools=['hover', 'pan', 'wheel_zoom', 'box_zoom', 'reset'],
            title=self.title, xlabel=self.xAxisLabel, ylabel=self.yAxisLabel
        ), self.title