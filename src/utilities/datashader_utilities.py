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
        varIdx_df = pd.concat([varIdx_df, pd.DataFrame([np.nan] * varIdx_df.shape[1], index=varIdx_df.columns).T])

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


def transform_dataframe(df_in, transforming_function):
    """
    Transforms the data in 'df_in' using the function specified in 'transforming_function'.
    E.g. if the data in 'df_in' are position vectors rvec and the 'transforming_function' is np.linalg.norm(rvec)
    then a dataframe with the norm of each position vector is returned.
    """
    num_runs = df_in.columns.levshape[0]
    num_comp = df_in.columns.levshape[1]
    num_comp_out = np.size(transforming_function(df_in.iloc[0][0].values))

    data = []
    for runNum in range(num_runs):
        result = df_in.iloc[:, num_comp * runNum:num_comp * runNum + num_comp].apply(transforming_function, axis=1, raw=True)
        if num_comp_out == 1:  # The result is one-dimensional of type Series
            data.append(result)
        else:  # The result is multi-dimensional of type dataframe
            for idx in range(num_comp_out):  # Unpack the dataframe into series
                data.append(pd.Series(result.iloc[:,idx]))
    newMultIndex = pd.MultiIndex.from_product([list(range(num_runs)), list(range(num_comp_out))], names=['runNum', 'varIdx'])
    return pd.DataFrame(data, index=newMultIndex).T


def extract_effector_df(data_path, num_eff):
    """
    Extracts the effector (RW, thruster, etc.) information for only those effectors that are used.
    E.g. the RW effector message consists by default of 36 RWs, but if only 4 are used, this function will return the
    states of only 4 RWs. The data path specifies where the data from the MC is located, for example
    '/mc_data/rw_speed_msg.wheelSpeeds.data'.
    """
    df_effector = pd.read_pickle(data_path)
    num_runs = df_effector.columns.levshape[0]
    num_eff_default = df_effector.columns.levshape[1]

    effector_list = []
    for runNum in range(num_runs):
        for effNum in range(num_eff):  # Unpack the dataframe into series
            effector_list.append(pd.Series(df_effector.iloc[:, num_eff_default * runNum + effNum]))
    newMultIndex = pd.MultiIndex.from_product([list(range(num_runs)), list(range(num_eff))], names=['runNum', 'varIdx'])
    return pd.DataFrame(effector_list, index=newMultIndex).T


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
            self.data[i].index = self.data[i].index * self.macro_x

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
            legend = hv.NdOverlay({n: hv.Points([np.nan, np.nan], label=str(n)).opts(color=c) for n, c in color_key})
            image = image*legend

        return image, self.title

