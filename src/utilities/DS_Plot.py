import itertools
import warnings

import numpy as np

with warnings.catch_warnings():
    warnings.simplefilter("ignore", category=DeprecationWarning)
    import datashader as ds
    import holoviews as hv
    from holoviews.operation.datashader import datashade, dynspread
    from holoviews.streams import RangeXY
    from datashader.colors import Sets1to3
from Basilisk.utilities import macros
from Basilisk.utilities.dataframe_utilities import curve_per_df_column

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
                 plotFcn=curve_per_df_column):
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
        self.cmap = itertools.cycle(cmap)
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
                curve = self.plotObjType(curve_df)
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
        image.opts(padding=0.05)
        image.opts(title=self.title, xlabel=self.xAxisLabel, ylabel=self.yAxisLabel)

        if not self.labels == []:
            color_key = [(name, color) for name, color in zip(self.labels, self.cmap)]
            legend = hv.NdOverlay({n: hv.Points([np.nan, np.nan], label=str(n)).opts(color=c) for n, c in color_key})
            image = image*legend

        return image, self.title
